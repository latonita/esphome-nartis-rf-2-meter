**Table A — TAG → quantity, size, encoding**

| TAG | SRC-ID | Quantity | Size | Enc | ✓ |
|---|---|---|---|---|---|
| 0x00 | 0x0001 | Active energy import, **total** | 4 | bin LE | ✓ |
| 0x01 | 0x0002 | Active energy import, T1 | 4 | bin LE | ✓ |
| 0x02 | 0x0003 | Active energy import, T2 | 4 | bin LE | ✓ |
| 0x03 | 0x0004 | Active energy import, T3 | 4 | bin LE | (m) |
| 0x04 | 0x0005 | Active energy import, T4 | 4 | bin LE | (m) |
| 0x05 | 0x0011 | Active energy export, **total** | 4 | bin LE | ✓ |
| 0x06 | 0x0012 | Active energy export, T1 | 4 | bin LE | ✓ |
| 0x07 | 0x0013 | Active energy export, T2 | 4 | bin LE | ✓ |
| 0x08–0x13 | 0x0014–0x0035 | Reactive / other energy registers | 4 | bin LE | (m) |
| 0x14 | 0x0131 | Voltage (single-phase meter) | 4 | BCD LE | (m) |
| 0x15 | 0x0131 | **Voltage L1** | 4 | BCD LE | ✓ |
| 0x16 | 0x0132 | **Voltage L2** | 4 | BCD LE | ✓ |
| 0x17 | 0x0133 | **Voltage L3** | 4 | BCD LE | ✓ |
| 0x18–0x1A | 0x0141–0x0143 | Line voltage AB/BC/CA | 4 | BCD LE | (m) |
| 0x1B | 0x0E08 | Current (single-phase meter) | 4 | BCD LE | (m) |
| 0x1C | 0x0151 | Current, neutral | 4 | BCD LE | (m) |
| 0x1D | 0x0152 | **Current L1** | 4 | BCD LE | ✓ |
| 0x1E | 0x0153 | **Current L2** | 4 | BCD LE | ✓ |
| 0x1F | 0x0E07 | **Current L3** | 4 | BCD LE | ✓ |
| 0x20 | 0x00D1 | **Active power, total** | 4 | BCD LE | ✓ |
| 0x21 | 0x00D2 | Active power L1 | 4 | BCD LE | ✓ |
| 0x22 | 0x00D3 | Active power L2 | 4 | BCD LE | ✓ |
| 0x23 | 0x00D4 | Active power L3 | 4 | BCD LE | ✓ |
| 0x24 | 0x00F1 | **Reactive power, total** | 4 | BCD LE **signed** | ✓ |
| 0x25 | 0x00F2 | Reactive power L1 | 4 | BCD LE **signed** | ✓ |
| 0x26 | 0x00F3 | Reactive power L2 | 4 | BCD LE **signed** | ✓ |
| 0x27 | 0x00F4 | Reactive power L3 | 4 | BCD LE **signed** | ✓ |
| 0x28 | 0x0E00 | **Frequency** | 4 | BCD LE | ✓ |
| 0x29 | 0x2E07 | **Date/time** | 7 | BCD | ✓ |
| 0x2A | 0x4E83 | **Temperature** | 2 | bin LE signed | ✓ |
| 0x2C–0x2F | 0x0111–0x0114 | Power group (FW) / prev-period energy (manual — differs) | 4 | BCD | ⚠ |
| 0x30–0x33 | 0x0161–0x0164 | Power-factor group | 4 | BCD | (m) |
| 0x34–0x47 | 0x6Exx/0x6Fxx | Event / log counters | 4 | bin LE | (m) |
| 0x48–0x4F | 0x5Exx | Identity / config objects | 1/var | bin | (m) |

**Table B — TAG → scale & unit** (consumer multiplier: physical = decoded × scale)

| TAG(s) | Scale | Unit | note |
|---|---|---|---|
| 0x00–0x13 | ×0.001 | kWh (kvarh for reactive regs) | energy accumulators |
| 0x14–0x1A | ×0.1 | V | phase & line voltages |
| 0x1B–0x1F | ×0.01 | A | phase & neutral currents |
| 0x20–0x23 | ×10 ⚠ | W | active power (see ⚠) |
| 0x24–0x27 | ×10 ⚠ | var | reactive power, **signed** (see ⚠) |
| 0x28 | ×0.01 | Hz | frequency |
| 0x29 | — | date/time | 7-byte BCD, format below |
| 0x2A | ×0.1 | °C | temperature (signed) |
| 0x30–0x33 | ÷10 | (ratio) | power factor |
| 0x34–0x47 | ×1 | count | event/log counters |

Conventions:
- **Signed BCD** — how it's presented on the wire (reactive power 0x24–0x27):
  - 4-byte BCD, little-endian (byte[0] = least-significant digit pair).
  - The **sign lives in the top byte** (byte[3]): bit `0x80` set ⇒ **negative**; the rest of the digits are the magnitude.
  - Decode: `neg = byte[3] & 0x80; byte[3] &= 0x7F; value = bcd_decode(byte[3] byte[2] byte[1] byte[0]); if neg: value = -value`.
  - Example: `78 08 00 80` → byte[3]=0x80 (neg; magnitude high pair = 0x00) → digits `00 00 08 78` → 878 → **−878** → ×0.1 = **−87.8 var**.
  - (Temperature 0x2A is *binary* signed, not BCD — two's-complement `int16` LE.)
- **Date/time** (0x29, 7 B BCD): `sec, min, hour, weekday, day, month, year` — e.g. `06 39 13 03 02 09 26` = 2026-09-02 (Wed) 13:39:06.
- ⚠ **P/Q scale (×10)** is the manual's multiplier. On one meter a raw-object cross-check (P ≈ U·I·PF; P_total vs energy-rate) fit **×1 W / ×0.001 A** instead — some builds/CT-variants differ by 10×. **Verify per meter** with `ΣP_phase ≈ P_total` and `P ≈ U·I·PF`.
