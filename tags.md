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
garbage. `Scale` gives physical = decoded × scale, and it is **what an entity
publishes**: the component applies it, so a value arrives in the `Unit` column
already and a `multiply` filter would scale it twice. It is also what lets the
fixed blocks feed the same entity — see *Fixed-block sources* below. `SRC-ID` is the firmware's internal object id, useful only for correlating
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
| 0x0A |  | Reactive energy import (R+), **total** | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x0B |  | Reactive energy import (R+), T1 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x0C |  | Reactive energy import (R+), T2 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x0D |  | Reactive energy import (R+), T3 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x0E |  | Reactive energy import (R+), T4 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x0F |  | Reactive energy export (R−), **total** | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x10 |  | Reactive energy export (R−), T1 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x11 |  | Reactive energy export (R−), T2 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x12 |  | Reactive energy export (R−), T3 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x13 | 0x0035 | Reactive energy export (R−), T4 | 4 | bin LE | ×0.001 | kvarh | ✓ |
| 0x14 | 0x0131 | Voltage (single-phase meter) | 4 | BCD LE | ×0.1 | V | (m) |
| 0x15 | 0x0131 | **Voltage L1** (phase A) | 4 | BCD LE | ×0.1 | V | ✓ |
| 0x16 | 0x0132 | **Voltage L2** (phase B) | 4 | BCD LE | ×0.1 | V | ✓ |
| 0x17 | 0x0133 | **Voltage L3** (phase C) | 4 | BCD LE | ×0.1 | V | ✓ |
| 0x18 | 0x0141 | Line voltage AB | 4 | BCD LE | ×0.1 | V | (m) |
| 0x19 | 0x0142 | Line voltage BC | 4 | BCD LE | ×0.1 | V | (m) |
| 0x1A | 0x0143 | Line voltage CA | 4 | BCD LE | ×0.1 | V | (m) |
| 0x1B | 0x0E08 | Current (single-phase meter) | 4 | BCD LE | ×0.001 | A | (m) |
| 0x1C | 0x0151 | **Current L1** (phase A) ⚠ | 4 | BCD LE | ×0.001 | A | ✓ |
| 0x1D | 0x0152 | **Current L2** (phase B) ⚠ | 4 | BCD LE | ×0.001 | A | ✓ |
| 0x1E | 0x0153 | **Current L3** (phase C) ⚠ | 4 | BCD LE | ×0.001 | A | ✓ |
| 0x1F | 0x0E07 | Current, object 0x0E07 — not phase C ⚠ | 4 | BCD LE | ×0.001 | A | (m) |
| 0x20 | 0x00D1 | **Active power (P+), total** | 4 | BCD LE **signed** | ×1 | W | ✓ |
| 0x21 | 0x00D2 | Active power (P+) L1 | 4 | BCD LE **signed** | ×1 | W | ✓ |
| 0x22 | 0x00D3 | Active power (P+) L2 | 4 | BCD LE **signed** | ×1 | W | ✓ |
| 0x23 | 0x00D4 | Active power (P+) L3 | 4 | BCD LE **signed** | ×1 | W | ✓ |
| 0x24 | 0x00F1 | **Reactive power (Q+), total** | 4 | BCD LE **signed** | ×1 | var | ✓ |
| 0x25 | 0x00F2 | Reactive power (Q+) L1 | 4 | BCD LE **signed** | ×1 | var | ✓ |
| 0x26 | 0x00F3 | Reactive power (Q+) L2 | 4 | BCD LE **signed** | ×1 | var | ✓ |
| 0x27 | 0x00F4 | Reactive power (Q+) L3 | 4 | BCD LE **signed** | ×1 | var | ✓ |
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

**Fixed-block sources.** The two fixed blocks carry some of these quantities
positionally, in their own units, and the maps in `d101_frame.cpp` put each onto
the TAG that names it — so a `tag:` entity can be fed by either source, the scales
making the two agree. DI `0xF102` covers `0x15`–`0x17`, `0x1C`–`0x1E`,
`0x20`–`0x27` and `0x28`; DI `0xF101` covers `0x0A`–`0x13`. A list wins where both
carry a TAG, so in practice the blocks supply what no list sends: the **per-phase
power** (list B carries `0x20` and none of `0x21`–`0x27`) and **all the reactive
energy**, which the lists do not carry at all. See `fixed.md`.

Settled since the last revision:
- **Current and power scale — the factor of ten is resolved.** ×0.001 A with ×1 W,
  which is what `TAG_TABLE` uses. `ΣP_phase ≈ P_total` never could tell the two
  pairs apart, but DI `0xF102` can, its scales being independent: on a live
  three-phase meter the implied multiplier came out ×0.1003 V, ×0.001008 A and
  ×0.001005 A on the two phases that had not moved between the reads. The ×0.01 A
  / ×10 W pair from the manual would have needed a ten-fold miss. A second,
  independent check: with 234 V and 1.6/1.3/3.6 A the phases carry ≈1.5 kVA, and
  the meter reported 1405 W — ×10 W would have claimed 14 kW out of 1.5 kVA.

Open questions:
- ⚠ **The current group is one TAG lower than the vendor labels say.** The vendor
  list calls `0x1C` "neutral current" (OBIS 1.0.91.7.0) and puts L1/L2/L3 on
  `0x1D`–`0x1F`. The object ids disagree, and so does the meter: `0x1C`–`0x1E`
  hold objects 0x0151–0x0153, which are the phase currents DI `0xF102` reads, and
  in a live capture `0x1C` and `0x1D` matched that block's L1 and L2 to within
  0.8% while the rise in `P_total` over the same seconds was accounted for by L3
  alone. The table above follows the objects. **If you configured a current entity
  from an earlier revision of this file, it is one phase off.** What object `0x1F`
  actually is remains unknown — it is one of the 0x0E0x pair the single-phase rows
  use, not phase C.
- ⚠ **Active power has not been seen negative on air.** Every capture is import,
  so `0x20`–`0x23` being signed rests on the vendor list marking them "report with
  sign" exactly as it marks `0x24`–`0x27`. The sign bit is assumed to sit in the
  same place.
