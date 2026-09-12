using System.Buffers.Binary;
using ModbusLite;
using Xunit;

/// <summary>
/// Full request/response round-trips against an in-memory "slave":
/// the fake stream parses the client's MBAP frame like a real device
/// would and scripts a spec-correct response — zero network needed.
/// </summary>
public class ModbusClientTests
{
    /// <summary>A duplex stream that behaves like a Modbus TCP slave.</summary>
    private sealed class FakeSlaveStream : Stream
    {
        private readonly Queue<byte[]> _scripted = new();
        private readonly Func<byte, byte[], byte[]> _responder;
        private byte[]? _pending;

        public FakeSlaveStream(Func<byte, byte[], byte[]> responder) => _responder = responder;

        public ushort? LastTransactionId { get; private set; }
        public byte? LastUnitId { get; private set; }
        public byte[]? LastPdu { get; private set; }

        public override void Write(byte[] buffer, int offset, int count)
        {
            // parse the client frame exactly like a slave would
            ushort tid = BinaryPrimitives.ReadUInt16BigEndian(buffer.AsSpan(0));
            ushort pid = BinaryPrimitives.ReadUInt16BigEndian(buffer.AsSpan(2));
            ushort len = BinaryPrimitives.ReadUInt16BigEndian(buffer.AsSpan(4));
            byte unit = buffer[6];

            Assert.Equal(0, pid);                       // MBAP protocol id
            LastTransactionId = tid;
            LastUnitId = unit;
            LastPdu = buffer[7..(6 + len)];

            _pending = BuildResponse(tid, unit, _responder(buffer[7], LastPdu!));
        }

        private static byte[] BuildResponse(ushort tid, byte unit, byte[] pdu)
        {
            var frame = new byte[7 + pdu.Length];
            BinaryPrimitives.WriteUInt16BigEndian(frame.AsSpan(0), tid);
            BinaryPrimitives.WriteUInt16BigEndian(frame.AsSpan(2), 0);
            BinaryPrimitives.WriteUInt16BigEndian(frame.AsSpan(4), (ushort)(pdu.Length + 1));
            frame[6] = unit;
            pdu.CopyTo(frame, 7);
            return frame;
        }

        public override int Read(byte[] buffer, int offset, int count)
        {
            if (_pending is null) return 0;
            int n = Math.Min(count, _pending.Length);
            Array.Copy(_pending, 0, buffer, offset, n);
            _pending = _pending[n..];
            return n;
        }

        public override bool CanRead => true;
        public override bool CanSeek => false;
        public override bool CanWrite => true;
        public override long Length => throw new NotSupportedException();
        public override long Position { get => 0; set => throw new NotSupportedException(); }
        public override void Flush() { }
        public override long Seek(long o, SeekOrigin d) => throw new NotSupportedException();
        public override void SetLength(long v) => throw new NotSupportedException();
    }

    private static byte[] RegistersResponse(params ushort[] regs)
    {
        var pdu = new byte[2 + regs.Length * 2];
        pdu[0] = 0x03;
        pdu[1] = (byte)(regs.Length * 2);
        for (int i = 0; i < regs.Length; i++)
            BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(2 + i * 2), regs[i]);
        return pdu;
    }

    [Fact]
    public void BuildFrame_encodes_valid_mbap()
    {
        var frame = ModbusClient.BuildFrame(transactionId: 7, unitId: 1,
            pdu: new byte[] { 0x03, 0x00, 0x6B, 0x00, 0x03 });

        Assert.Equal(12, frame.Length);
        Assert.Equal(7, BinaryPrimitives.ReadUInt16BigEndian(frame.AsSpan(0)));
        Assert.Equal(0, BinaryPrimitives.ReadUInt16BigEndian(frame.AsSpan(2)));
        Assert.Equal(6, BinaryPrimitives.ReadUInt16BigEndian(frame.AsSpan(4)));   // unit(1)+pdu(5)
        Assert.Equal(1, frame[6]);
        Assert.Equal(new byte[] { 0x03, 0x00, 0x6B, 0x00, 0x03 }, frame[7..]);
    }

    [Fact]
    public void ReadHoldingRegisters_parses_big_endian_words()
    {
        ushort[] expected = { 0x02AB, 0x0000, 0x0064 };
        var stream = new FakeSlaveStream((fc, pdu) =>
        {
            Assert.Equal(0x03, fc);
            Assert.Equal(0x006B, BinaryPrimitives.ReadUInt16BigEndian(pdu.AsSpan(1)));  // start 107
            Assert.Equal(3, BinaryPrimitives.ReadUInt16BigEndian(pdu.AsSpan(3)));       // count
            return RegistersResponse(expected);
        });
        using var client = new ModbusClient(stream);

        var regs = client.ReadHoldingRegisters(start: 107, count: 3);

        Assert.Equal(expected, regs);
        Assert.Equal(1u, stream.LastTransactionId);       // ids start at 1 and increment
        Assert.Equal(1, stream.LastUnitId);
    }

    [Fact]
    public void ReadCoils_unpacks_bits_lsb_first()
    {
        var stream = new FakeSlaveStream((fc, pdu) =>
        {
            Assert.Equal(0x01, fc);
            // 10 coils requested → 2 bytes; 0xCD = 1100 1011 LSB-first = [1,1,0,1,0,0,1,1]
            return new byte[] { 0x01, 0x02, 0xCD, 0x01 };
        });
        using var client = new ModbusClient(stream);

        var coils = client.ReadCoils(start: 20, count: 10);

        var want = new[] { true, true, false, true, false, false, true, true, true, false };
        Assert.Equal(want, coils);
    }

    [Fact]
    public void Slave_exception_code_raises_ModbusException()
    {
        var stream = new FakeSlaveStream((fc, pdu) => new byte[] { (byte)(fc | 0x80), 0x02 });
        using var client = new ModbusClient(stream);

        var ex = Assert.Throws<ModbusException>(() => client.ReadHoldingRegisters(0, 5));
        Assert.Equal(0x02, ex.ExceptionCode);
        Assert.Contains("illegal data address", ex.Message);
    }

    [Fact]
    public void WriteSingleRegister_round_trips_value()
    {
        var stream = new FakeSlaveStream((fc, pdu) =>
        {
            Assert.Equal(0x06, fc);
            Assert.Equal(0x0003, BinaryPrimitives.ReadUInt16BigEndian(pdu.AsSpan(1)));
            Assert.Equal(0x000A, BinaryPrimitives.ReadUInt16BigEndian(pdu.AsSpan(3)));
            return pdu.ToArray();                       // echo per spec
        });
        using var client = new ModbusClient(stream);

        client.WriteSingleRegister(address: 3, value: 10);   // must not throw
    }

    [Fact]
    public void Transaction_ids_increment_and_are_verified()
    {
        var stream = new FakeSlaveStream((fc, pdu) =>
        {
            return RegistersResponse(1);
        });
        using var client = new ModbusClient(stream);

        client.ReadHoldingRegisters(0, 1);
        client.ReadHoldingRegisters(0, 1);
        Assert.Equal(2u, stream.LastTransactionId);
    }

    [Fact]
    public void Count_out_of_range_is_rejected_before_sending()
    {
        var stream = new FakeSlaveStream((fc, pdu) => throw new Exception("should not be called"));
        using var client = new ModbusClient(stream);

        Assert.Throws<ArgumentOutOfRangeException>(() => client.ReadHoldingRegisters(0, 126));
        Assert.Throws<ArgumentOutOfRangeException>(() => client.ReadCoils(0, 0));
    }
}
