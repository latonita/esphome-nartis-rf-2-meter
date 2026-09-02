**Sources**, in order of authority:

1. **`TAG_TABLE` in `d101_frame.cpp`** — width, encoding and scale. It is what the
   component actually decodes with, and its rows carry provenance. Where this file
   disagreed with the code on a format, the code won.
2. **`tags-nartis-en.md`** (translated from the vendor's `tags-nartis-ru.csv`) —
   what each TAG *means*, with DLMS class and OBIS code, for TAG `0x00`–`0x3F`.
   It says nothing about width, encoding or scale.
3. The display's manual and on-air captures — everything neither of the above
   settles, and the origin of the open scale questions at the end.

**Columns.** `Size` is bytes on the wire and is what makes a page walkable at all:
records carry no length field, so a wrong width turns every record after it into
garbage. `Scale` gives physical = decoded × scale, and affects the **log line
only** — published values stay raw and the YAML picks the unit with a `multiply`
filter. `SRC-ID` is the firmware's internal object id, useful only for correlating
captures; blank where we never had one. OBIS codes are in `tags-nartis-en.md`.

`✓` seen decoded on air · `(m)` from the vendor list or the manual, not yet
observed · `⚠` open question, see the notes below.

**A `—` in Size means the TAG is refused, not guessed.** `0x2B` and `0x40`–`0x4F`
have no row in `TAG_TABLE`, so a record carrying one aborts the walk rather than
being misframed. A YAML `bytes:` override is the only way to read one.

| TAG | SRC-ID | Quantity | Size | Enc | Scale | Unit | ✓ |
|---|---|---|---|---|---|---|---|
| 0x00 | 0x0001 | Active energy import (A+), **total** | 4 | bin LE | ×0.001 | kWh | ✓ |
| 0x01 | 0x0002 | Active energy import (A+), T1 | 4 | bin LE | ×0.001 | kWh | ✓ |
| 0x02 | 0x0003 | Active energy import (A+), T2 | 4 | bin LE | ×0.001 | kWh | ✓ |
| 0x03 | 0x0004 | Active energy import (A+), T3 | 4 | bin LE | ×0.001 | kWh | (m) |
| 0x04 | 0x0005 | Active energy import (A+), T4 | 4 | bin LE | ×0.001 | kWh | (m) |
| 0x05 | 0x0011 | Active energy export (A−), **total** | 4 | bin LE | ×0.001 | kWh | ✓ |
| 0x06 | 0x0012 | Active energy export (A−), T1 | 4 | bin LE | ×0.001 | kWh | ✓ |
| 0x07 | 0x0013 | Active energy export (A−), T2 | 4 | bin LE | ×0.001 | kWh | ✓ |
| 0x08 | 0x0014 | Active energy export (A−), T3 | 4 | bin LE | ×0.001 | kWh | (m) |
| 0x09 | 0x0015 | Active energy export (A−), T4 | 4 | bin LE | ×0.001 | kWh | (m) |
| 0x0A |  | Reactive energy import (R+), **total** | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x0B |  | Reactive energy import (R+), T1 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x0C |  | Reactive energy import (R+), T2 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x0D |  | Reactive energy import (R+), T3 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x0E |  | Reactive energy import (R+), T4 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x0F |  | Reactive energy export (R−), **total** | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x10 |  | Reactive energy export (R−), T1 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x11 |  | Reactive energy export (R−), T2 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x12 |  | Reactive energy export (R−), T3 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x13 | 0x0035 | Reactive energy export (R−), T4 | 4 | bin LE | ×0.001 | kvarh | (m) |
| 0x14 | 0x0131 | Voltage (single-phase meter) | 4 | BCD LE | ×0.1 | V | (m) |
| 0x15 | 0x0131 | **Voltage L1** (phase A) | 4 | BCD LE | ×0.1 | V | ✓ |
| 0x16 | 0x0132 | **Voltage L2** (phase B) | 4 | BCD LE | ×0.1 | V | ✓ |
| 0x17 | 0x0133 | **Voltage L3** (phase C) | 4 | BCD LE | ×0.1 | V | ✓ |
| 0x18 | 0x0141 | Line voltage AB | 4 | BCD LE | ×0.1 | V | (m) |
| 0x19 | 0x0142 | Line voltage BC | 4 | BCD LE | ×0.1 | V | (m) |
| 0x1A | 0x0143 | Line voltage CA | 4 | BCD LE | ×0.1 | V | (m) |
| 0x1B | 0x0E08 | Current (single-phase meter) | 4 | BCD LE | ×0.001 ⚠ | A | (m) |
| 0x1C | 0x0151 | Current, neutral | 4 | BCD LE | ×0.001 ⚠ | A | (m) |
| 0x1D | 0x0152 | **Current L1** (phase A) | 4 | BCD LE | ×0.001 ⚠ | A | ✓ |
| 0x1E | 0x0153 | **Current L2** (phase B) | 4 | BCD LE | ×0.001 ⚠ | A | ✓ |
| 0x1F | 0x0E07 | **Current L3** (phase C) | 4 | BCD LE | ×0.001 ⚠ | A | ✓ |
| 0x20 | 0x00D1 | **Active power (P+), total** | 4 | BCD LE **signed** | ×1 ⚠ | W | ✓ |
| 0x21 | 0x00D2 | Active power (P+) L1 | 4 | BCD LE **signed** | ×1 ⚠ | W | ✓ |
| 0x22 | 0x00D3 | Active power (P+) L2 | 4 | BCD LE **signed** | ×1 ⚠ | W | ✓ |
| 0x23 | 0x00D4 | Active power (P+) L3 | 4 | BCD LE **signed** | ×1 ⚠ | W | ✓ |
| 0x24 | 0x00F1 | **Reactive power (Q+), total** | 4 | BCD LE **signed** | ×1 ⚠ | var | ✓ |
| 0x25 | 0x00F2 | Reactive power (Q+) L1 | 4 | BCD LE **signed** | ×1 ⚠ | var | ✓ |
| 0x26 | 0x00F3 | Reactive power (Q+) L2 | 4 | BCD LE **signed** | ×1 ⚠ | var | ✓ |
| 0x27 | 0x00F4 | Reactive power (Q+) L3 | 4 | BCD LE **signed** | ×1 ⚠ | var | ✓ |
| 0x28 | 0x0E00 | **Frequency** | 4 | BCD LE | ×0.01 | Hz | ✓ |
| 0x29 | 0x2E07 | **Date/time** | 7 | BCD | — | date/time | ✓ |
| 0x2A | 0x4E83 | **Temperature** | 2 | bin LE signed | ×0.1 | °C | ✓ |
| 0x2B |  | LCD test — manual scroll mode only, not a value | — | — | — | — | ⚠ |
| 0x2C |  | Active energy import (A+), **total** — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x2D |  | Active energy import (A+), T1 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x2E |  | Active energy import (A+), T2 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x2F |  | Active energy import (A+), T3 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x30 |  | Active energy import (A+), T4 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x31 |  | Active energy export (A−), **total** — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x32 |  | Active energy export (A−), T1 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x33 |  | Active energy export (A−), T2 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x34 |  | Active energy export (A−), T3 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x35 |  | Active energy export (A−), T4 — end of period | 4 | BCD LE | ×0.001 | kWh | (m) |
| 0x36 |  | Reactive energy import (R+), **total** — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x37 |  | Reactive energy import (R+), T1 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x38 |  | Reactive energy import (R+), T2 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x39 |  | Reactive energy import (R+), T3 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x3A |  | Reactive energy import (R+), T4 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x3B |  | Reactive energy export (R−), **total** — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x3C |  | Reactive energy export (R−), T1 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x3D |  | Reactive energy export (R−), T2 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x3E |  | Reactive energy export (R−), T3 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |
| 0x3F |  | Reactive energy export (R−), T4 — end of period | 4 | BCD LE | ×0.001 | kvarh | (m) |


`0x2C`–`0x3F` is `0x00`–`0x13` again, frozen at the close of the last billing
period, in the same order — subtract `0x2C` from the TAG for the offset into that
block. In OBIS terms the two blocks differ only in the last field, `255` vs `101`.

Conventions:
- **Signed BCD** — the whole power group, `0x20`–`0x27`:
  - 4-byte BCD, little-endian (byte[0] = least-significant digit pair).
  - The **sign lives in the top byte** (byte[3]): bit `0x80` set ⇒ **negative**; the rest of the digits are the magnitude.
  - Decode: `neg = byte[3] & 0x80; byte[3] &= 0x7F; value = bcd_decode(byte[3] byte[2] byte[1] byte[0]); if neg: value = -value`.
  - Example: `78 08 00 80` → byte[3]=0x80 (neg; magnitude high pair = 0x00) → digits `00 00 08 78` → **−878 var**.
  - The sign bit overlaps the top BCD digit, so the magnitude caps at 8,000,000 counts.
  - (Temperature `0x2A` is *binary* signed, not BCD — two's-complement `int16` LE.)
- **Date/time** (`0x29`, 7 B BCD): `sec, min, hour, weekday, day, month, year` — e.g. `06 39 13 03 02 09 26` = 2026-09-02 (Wed) 13:39:06.
- **Energy is the one binary family.** `0x00`–`0x13` is `bin LE`; everything from
  `0x14` to `0x3F` is BCD apart from temperature. Note the asymmetry that follows:
  the end-of-period copies at `0x2C`–`0x3F` are BCD even though they are the same
  objects as the binary `0x00`–`0x13`. That is the manual's reading, not an
  observation — worth a second look the first time such a record shows up, though
  the width is 4 either way, so framing is safe regardless.

Open questions:
- ⚠ **Current and power scale — a joint factor of ten.** Voltage is settled at
  ×0.1 V, and `P ≈ U·I` then pins current and power *to each other*: either
  ×0.001 A with ×1 W, or ×0.01 A with ×10 W. Both are internally consistent, so
  `ΣP_phase ≈ P_total` cannot tell them apart — it holds either way. The manual
  says the second pair; a raw-object cross-check on one meter (P against the
  energy rate) fitted the first, which is what `TAG_TABLE` now uses. **Verify
  against a known load on your own meter** before trusting the absolute value;
  builds or CT variants may genuinely differ.
  - Cheapest way to settle it: read the same quantity from the fixed page
    DI `0xF102`, whose scales are independent (see `DI.md`). The ratio of the two
    raw values is the answer, directly.
- ⚠ **Active power has not been seen negative on air.** Every capture is import,
  so `0x20`–`0x23` being signed rests on the vendor list marking them "report with
  sign" exactly as it marks `0x24`–`0x27`. The sign bit is assumed to sit in the
  same place.
