using System.Buffers.Binary;

namespace ModbusLite;

/// <summary>Modbus function codes supported by this client.</summary>
public enum FunctionCode : byte
{
    ReadCoils = 0x01,
    ReadDiscreteInputs = 0x02,
    ReadHoldingRegisters = 0x03,
    ReadInputRegisters = 0x04,
    WriteSingleCoil = 0x05,
    WriteSingleRegister = 0x06,
}

/// <summary>Standard Modbus exception codes returned by slaves.</summary>
public class ModbusException : Exception
{
    public byte FunctionCode { get; }
    public byte ExceptionCode { get; }

    private static string Describe(byte code) => code switch
    {
        0x01 => "illegal function",
        0x02 => "illegal data address",
        0x03 => "illegal data value",
        0x04 => "slave device failure",
        0x06 => "slave device busy",
        _ => $"modbus exception 0x{code:X2}",
    };

    public ModbusException(byte functionCode, byte exceptionCode)
        : base($"FC {functionCode}: {Describe(exceptionCode)}")
    {
        FunctionCode = functionCode;
        ExceptionCode = exceptionCode;
    }
}

/// <summary>
/// Minimal Modbus TCP (MBAP) framing over any Stream — NetworkStream in
/// production, MemoryStream in tests. One transaction at a time by design:
/// industrial field loops are request/response and serialized anyway.
/// </summary>
public sealed class ModbusClient : IDisposable
{
    private readonly Stream _stream;
    private ushort _transactionId;
    private const int MbapHeaderSize = 7;
    private const int TimeoutMs = 3000;

    public byte UnitId { get; set; } = 1;

    public ModbusClient(Stream stream) => _stream = stream;

    public ModbusClient(string host, int port = 502)
        : this(TcpConnect(host, port)) { }

    private static Stream TcpConnect(string host, int port)
    {
        var tcp = new System.Net.Sockets.TcpClient { ReceiveTimeout = TimeoutMs, SendTimeout = TimeoutMs };
        tcp.ConnectAsync(host, port).AsTask().Wait(TimeoutMs);
        return tcp.GetStream();
    }

    // ------------------------------------------------------------------
    // Public API
    // ------------------------------------------------------------------

    /// <summary>Read 1–125 holding registers (FC3). Returns big-endian words.</summary>
    public ushort[] ReadHoldingRegisters(ushort start, ushort count)
        => ReadRegisters(FunctionCode.ReadHoldingRegisters, start, count);

    /// <summary>Read 1–125 input registers (FC4).</summary>
    public ushort[] ReadInputRegisters(ushort start, ushort count)
        => ReadRegisters(FunctionCode.ReadInputRegisters, start, count);

    /// <summary>Read 1–2000 coils (FC1). Returns one bool per coil.</summary>
    public bool[] ReadCoils(ushort start, ushort count) => ReadBits(FunctionCode.ReadCoils, start, count);

    /// <summary>Read 1–2000 discrete inputs (FC2).</summary>
    public bool[] ReadDiscreteInputs(ushort start, ushort count)
        => ReadBits(FunctionCode.ReadDiscreteInputs, start, count);

    /// <summary>Write one holding register (FC6).</summary>
    public void WriteSingleRegister(ushort address, ushort value)
    {
        var pdu = new byte[5];
        pdu[0] = (byte)FunctionCode.WriteSingleRegister;
        BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(1), address);
        BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(3), value);

        var response = Transact(pdu);
        ValidateEcho(response, pdu);
    }

    /// <summary>Write one coil (FC5). Non-zero value = ON per spec.</summary>
    public void WriteSingleCoil(ushort address, bool on)
    {
        var pdu = new byte[5];
        pdu[0] = (byte)FunctionCode.WriteSingleCoil;
        BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(1), address);
        BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(3), (ushort)(on ? 0xFF00 : 0x0000));

        var response = Transact(pdu);
        ValidateEcho(response, pdu);
    }

    // ------------------------------------------------------------------
    // Framing
    // ------------------------------------------------------------------

    private ushort[] ReadRegisters(FunctionCode fc, ushort start, ushort count)
    {
        ValidateCount(fc, count, 1, 125);
        var pdu = BuildReadPdu(fc, start, count);
        var response = Transact(pdu);

        if (response[0] != (byte)fc || response.Length < 2)
            throw new ModbusException(response[0], response.Length > 1 ? response[1] : (byte)0);

        var byteCount = response[1];
        if (byteCount != count * 2 || response.Length != 2 + byteCount)
            throw new InvalidOperationException($"malformed register response (byteCount={byteCount})");

        var result = new ushort[count];
        for (int i = 0; i < count; i++)
            result[i] = BinaryPrimitives.ReadUInt16BigEndian(response.AsSpan(2 + i * 2, 2));
        return result;
    }

    private bool[] ReadBits(FunctionCode fc, ushort start, ushort count)
    {
        ValidateCount(fc, count, 1, 2000);
        var pdu = BuildReadPdu(fc, start, count);
        var response = Transact(pdu);

        if (response[0] != (byte)fc || response.Length < 2)
            throw new ModbusException(response[0], response.Length > 1 ? response[1] : (byte)0);

        var byteCount = response[1];
        var expected = (count + 7) / 8;
        if (byteCount != expected || response.Length != 2 + byteCount)
            throw new InvalidOperationException($"malformed bit response (byteCount={byteCount})");

        var result = new bool[count];
        for (int i = 0; i < count; i++)
            result[i] = (response[2 + i / 8] & (1 << (i % 8))) != 0;
        return result;
    }

    private static byte[] BuildReadPdu(FunctionCode fc, ushort start, ushort count)
    {
        var pdu = new byte[5];
        pdu[0] = (byte)fc;
        BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(1), start);
        BinaryPrimitives.WriteUInt16BigEndian(pdu.AsSpan(3), count);
        return pdu;
    }

    /// <summary>
    /// Build MBAP frame, write it, read the response, strip the header and
    /// return the response PDU. Transaction IDs are echoed and verified.
    /// </summary>
    private byte[] Transact(byte[] pdu)
    {
        var tid = ++_transactionId;
        var frame = BuildFrame(tid, UnitId, pdu);
        _stream.Write(frame, 0, frame.Length);

        var header = ReadExactly(MbapHeaderSize);
        ushort responseTid = BinaryPrimitives.ReadUInt16BigEndian(header);
        ushort protocolId = BinaryPrimitives.ReadUInt16BigEndian(header.AsSpan(2));
        ushort length = BinaryPrimitives.ReadUInt16BigEndian(header.AsSpan(4));
        byte unitId = header[6];

        if (responseTid != tid)
            throw new InvalidOperationException($"transaction id mismatch (sent {tid}, got {responseTid})");
        if (protocolId != 0)
            throw new InvalidOperationException($"unexpected protocol id {protocolId}");
        if (length < 2 || length > 260)
            throw new InvalidOperationException($"implausible length {length}");

        var rest = ReadExactly(length - 1);
        if (unitId != UnitId)
            throw new InvalidOperationException($"unit id mismatch (sent {UnitId}, got {unitId})");

        var pduResponse = new byte[rest.Length];
        rest.CopyTo(pduResponse, 0);
        return pduResponse;
    }

    /// <summary>MBAP header (7B) + PDU. Exposed for tests and tooling.</summary>
    public static byte[] BuildFrame(ushort transactionId, byte unitId, byte[] pdu)
    {
        var frame = new byte[MbapHeaderSize + pdu.Length];
        BinaryPrimitives.WriteUInt16BigEndian(frame.AsSpan(0), transactionId);
        BinaryPrimitives.WriteUInt16BigEndian(frame.AsSpan(2), 0);            // protocol id
        BinaryPrimitives.WriteUInt16BigEndian(frame.AsSpan(4), (ushort)(pdu.Length + 1)); // unit + pdu
        frame[6] = unitId;
        pdu.CopyTo(frame, MbapHeaderSize);
        return frame;
    }

    private static void ValidateEcho(byte[] response, byte[] request)
    {
        if (response.Length != 5 || !response.AsSpan().SequenceEqual(request))
            throw new InvalidOperationException("write response did not echo the request");
    }

    private static void ValidateCount(FunctionCode fc, ushort count, int min, int max)
    {
        if (count < min || count > max)
            throw new ArgumentOutOfRangeException(nameof(count), $"{fc}: count must be {min}..{max}");
    }

    private byte[] ReadExactly(int n)
    {
        var buffer = new byte[n];
        int read = 0;
        while (read < n)
        {
            int chunk = _stream.Read(buffer, read, n - read);
            if (chunk == 0) throw new EndOfStreamException("connection closed mid-frame");
            read += chunk;
        }
        return buffer;
    }

    public void Dispose() => _stream.Dispose();
}
