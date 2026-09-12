# ModbusLite

<p align="center">
  <strong>Minimal Modbus TCP client for .NET 8 — read and write coils &amp;<br>
  registers with one clean class and zero dependencies.</strong>
</p>

<p align="center">
  .NET 8 · zero dependencies · xUnit-tested MBAP framing · field-ready
</p>

---

## Why

Industrial integrations shouldn't require a 2 MB library to flip a coil.
ModbusLite is a **single-class Modbus TCP (MBAP) client** built for HMI
tooling, SCADA glue code and test benches — where a compact, auditable
implementation beats a framework. It pairs naturally with my PLC/Ladder
background: this is the C# side of talking to real hardware.

## Features

- ✅ **FC 1/2/3/4/5/6** — read coils, discrete inputs, holding & input
  registers; write single coil/register
- 🔒 **Strict framing** — MBAP protocol-id and transaction-id verification,
  unit-id echo checks, plausibility limits on lengths
- 🧯 **Real error handling** — slave exception frames (FC | 0x80) surface as
  `ModbusException` with the spec's exception codes decoded
- 🔌 **Any `Stream`** — `NetworkStream` in production, `MemoryStream` in
  tests; the client never knows the difference
- 📏 **Big-endian by the book** — `BinaryPrimitives` throughout, no manual
  byte shuffling

## Quick start

```csharp
using ModbusLite;

using var client = new ModbusClient("192.168.1.50");   // port 502 default

ushort[] registers = client.ReadHoldingRegisters(start: 107, count: 3);
bool[]   inputs    = client.ReadDiscreteInputs(start: 0,   count: 8);

client.WriteSingleRegister(address: 3, value: 10);
client.WriteSingleCoil(address: 20, on: true);
```

## Testing

The entire request/response cycle runs against an **in-memory slave** —
a fake `Stream` that parses client frames like a real device and scripts
spec-correct answers:

```bash
dotnet test -c Release
```

7 tests cover MBAP encoding, big-endian register parsing, LSB-first coil
unpacking, exception frames, write echo verification and transaction-id
sequencing — with zero network access.

## Protocol notes

- One transaction in flight at a time — field loops are serialized by
  design; add your own lock only if multiple threads share a client.
- Timeouts are enforced at the socket layer (`ReceiveTimeout`/`SendTimeout`).
- FC 15/16 (multi-write) are intentionally omitted to keep the surface
  minimal — file an issue if you need them.

## License

MIT © Reza Bazdar (Godde3s)
