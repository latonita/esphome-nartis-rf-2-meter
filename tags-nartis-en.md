# Nartis display TAG → OBIS map (English)

Translation of `tags-nartis-ru.csv`, the vendor's list of the item TAGs the
НАРТИС-Д101-2 display can be configured to show, with the DLMS/COSEM object each
one reads. Covers TAG `0x00`–`0x3F`.

This is the *catalogue* of what a TAG means. It says nothing about which TAGs a
given meter actually sends — that is the per-meter indication list (see `DI.md`) —
and nothing about wire width, encoding or scale (see `tags.md`).

**Columns**

| Column | Meaning |
| --- | --- |
| TAG | The 1-byte item TAG on the wire. |
| Class | DLMS/COSEM interface class of the object: **3** = Register, **8** = Clock, **1** = Data. |
| OBIS code | OBIS identifier of the object the TAG reads. |
| Attr | DLMS attribute index. `2` is the *value* attribute throughout. |
| Description | What the value is. |
| Note | The vendor's own qualifier. |

Abbreviations used in the source: **ПУ** (*прибор учёта*) = metering device, i.e.
the meter itself. Phases are named A/B/C in the vendor's table; elsewhere in this
project they are L1/L2/L3 — A = L1, B = L2, C = L3.

## Current values (TAG 0x00–0x2B)

| TAG | Class | OBIS code | Attr | Description | Note |
| --- | --- | --- | --- | --- | --- |
| 0x00 | 3 | 1.0.1.8.0.255 | 2 | Active energy, import (A+) — total across all tariffs | Present value, cumulative |
| 0x01 | 3 | 1.0.1.8.1.255 | 2 | Active energy, import (A+) — Tariff 1 | Present value, cumulative |
| 0x02 | 3 | 1.0.1.8.2.255 | 2 | Active energy, import (A+) — Tariff 2 | Present value, cumulative |
| 0x03 | 3 | 1.0.1.8.3.255 | 2 | Active energy, import (A+) — Tariff 3 | Present value, cumulative |
| 0x04 | 3 | 1.0.1.8.4.255 | 2 | Active energy, import (A+) — Tariff 4 | Present value, cumulative |
| 0x05 | 3 | 1.0.2.8.0.255 | 2 | Active energy, export (A−) — total across all tariffs | Present value, cumulative |
| 0x06 | 3 | 1.0.2.8.1.255 | 2 | Active energy, export (A−) — Tariff 1 | Present value, cumulative |
| 0x07 | 3 | 1.0.2.8.2.255 | 2 | Active energy, export (A−) — Tariff 2 | Present value, cumulative |
| 0x08 | 3 | 1.0.2.8.3.255 | 2 | Active energy, export (A−) — Tariff 3 | Present value, cumulative |
| 0x09 | 3 | 1.0.2.8.4.255 | 2 | Active energy, export (A−) — Tariff 4 | Present value, cumulative |
| 0x0A | 3 | 1.0.3.8.0.255 | 2 | Reactive energy, import (R+) — total across all tariffs | Present value, cumulative |
| 0x0B | 3 | 1.0.3.8.1.255 | 2 | Reactive energy, import (R+) — Tariff 1 | Present value, cumulative |
| 0x0C | 3 | 1.0.3.8.2.255 | 2 | Reactive energy, import (R+) — Tariff 2 | Present value, cumulative |
| 0x0D | 3 | 1.0.3.8.3.255 | 2 | Reactive energy, import (R+) — Tariff 3 | Present value, cumulative |
| 0x0E | 3 | 1.0.3.8.4.255 | 2 | Reactive energy, import (R+) — Tariff 4 | Present value, cumulative |
| 0x0F | 3 | 1.0.4.8.0.255 | 2 | Reactive energy, export (R−) — total across all tariffs | Present value, cumulative |
| 0x10 | 3 | 1.0.4.8.1.255 | 2 | Reactive energy, export (R−) — Tariff 1 | Present value, cumulative |
| 0x11 | 3 | 1.0.4.8.2.255 | 2 | Reactive energy, export (R−) — Tariff 2 | Present value, cumulative |
| 0x12 | 3 | 1.0.4.8.3.255 | 2 | Reactive energy, export (R−) — Tariff 3 | Present value, cumulative |
| 0x13 | 3 | 1.0.4.8.4.255 | 2 | Reactive energy, export (R−) — Tariff 4 | Present value, cumulative |
| 0x14 | 3 | 1.0.12.7.0.255 | 2 | Voltage | Single-phase meters |
| 0x15 | 3 | 1.0.32.7.0.255 | 2 | Phase A voltage | Three-phase meters |
| 0x16 | 3 | 1.0.52.7.0.255 | 2 | Phase B voltage | Three-phase meters |
| 0x17 | 3 | 1.0.72.7.0.255 | 2 | Phase C voltage | Three-phase meters |
| 0x18 | 3 | 1.0.124.7.0.255 | 2 | Line voltage AB | Three-phase meters |
| 0x19 | 3 | 1.0.125.7.0.255 | 2 | Line voltage BC | Three-phase meters |
| 0x1A | 3 | 1.0.126.7.0.255 | 2 | Line voltage CA | Three-phase meters |
| 0x1B | 3 | 1.0.11.7.0.255 | 2 | Current | Single-phase meters |
| 0x1C | 3 | 1.0.91.7.0.255 | 2 | Neutral current | Three-phase meters |
| 0x1D | 3 | 1.0.31.7.0.255 | 2 | Phase A current | Three-phase meters |
| 0x1E | 3 | 1.0.51.7.0.255 | 2 | Phase B current | Three-phase meters |
| 0x1F | 3 | 1.0.71.7.0.255 | 2 | Phase C current | Three-phase meters |
| 0x20 | 3 | 1.0.1.7.0.255 | 2 | Active power (P+), sum of phases | Report with sign |
| 0x21 | 3 | 1.0.21.7.0.255 | 2 | Active power (P+), phase A | Report with sign |
| 0x22 | 3 | 1.0.41.7.0.255 | 2 | Active power (P+), phase B | Report with sign |
| 0x23 | 3 | 1.0.61.7.0.255 | 2 | Active power (P+), phase C | Report with sign |
| 0x24 | 3 | 1.0.3.7.0.255 | 2 | Reactive power (Q+), sum of phases | Report with sign |
| 0x25 | 3 | 1.0.23.7.0.255 | 2 | Reactive power (Q+), phase A | Report with sign |
| 0x26 | 3 | 1.0.43.7.0.255 | 2 | Reactive power (Q+), phase B | Report with sign |
| 0x27 | 3 | 1.0.63.7.0.255 | 2 | Reactive power (Q+), phase C | Report with sign |
| 0x28 | 3 | 1.0.14.7.0.255 | 2 | Line frequency | |
| 0x29 | 8 | 0.0.1.0.0.255 | 2 | Date and time | |
| 0x2A | 3 | 0.0.96.9.0.255 | 2 | Temperature, °C | |
| 0x2B | 1 | 0.0.96.128.0.255 | 2 | LCD test | Manual scroll mode only |

## End-of-billing-period values (TAG 0x2C–0x3F)

The same energy registers again, frozen at the close of the last billing period.
The OBIS codes differ from the block above only in the final field: `101` instead
of `255`.

| TAG | Class | OBIS code | Attr | Description | Note |
| --- | --- | --- | --- | --- | --- |
| 0x2C | 3 | 1.0.1.8.0.101 | 2 | Active energy, import (A+) — total across all tariffs | As of the end of the last billing period |
| 0x2D | 3 | 1.0.1.8.1.101 | 2 | Active energy, import (A+) — Tariff 1 | As of the end of the last billing period |
| 0x2E | 3 | 1.0.1.8.2.101 | 2 | Active energy, import (A+) — Tariff 2 | As of the end of the last billing period |
| 0x2F | 3 | 1.0.1.8.3.101 | 2 | Active energy, import (A+) — Tariff 3 | As of the end of the last billing period |
| 0x30 | 3 | 1.0.1.8.4.101 | 2 | Active energy, import (A+) — Tariff 4 | As of the end of the last billing period |
| 0x31 | 3 | 1.0.2.8.0.101 | 2 | Active energy, export (A−) — total across all tariffs | As of the end of the last billing period |
| 0x32 | 3 | 1.0.2.8.1.101 | 2 | Active energy, export (A−) — Tariff 1 | As of the end of the last billing period |
| 0x33 | 3 | 1.0.2.8.2.101 | 2 | Active energy, export (A−) — Tariff 2 | As of the end of the last billing period |
| 0x34 | 3 | 1.0.2.8.3.101 | 2 | Active energy, export (A−) — Tariff 3 | As of the end of the last billing period |
| 0x35 | 3 | 1.0.2.8.4.101 | 2 | Active energy, export (A−) — Tariff 4 | As of the end of the last billing period |
| 0x36 | 3 | 1.0.3.8.0.101 | 2 | Reactive energy, import (R+) — total across all tariffs | As of the end of the last billing period |
| 0x37 | 3 | 1.0.3.8.1.101 | 2 | Reactive energy, import (R+) — Tariff 1 | As of the end of the last billing period |
| 0x38 | 3 | 1.0.3.8.2.101 | 2 | Reactive energy, import (R+) — Tariff 2 | As of the end of the last billing period |
| 0x39 | 3 | 1.0.3.8.3.101 | 2 | Reactive energy, import (R+) — Tariff 3 | As of the end of the last billing period |
| 0x3A | 3 | 1.0.3.8.4.101 | 2 | Reactive energy, import (R+) — Tariff 4 | As of the end of the last billing period |
| 0x3B | 3 | 1.0.4.8.0.101 | 2 | Reactive energy, export (R−) — total across all tariffs | As of the end of the last billing period |
| 0x3C | 3 | 1.0.4.8.1.101 | 2 | Reactive energy, export (R−) — Tariff 1 | As of the end of the last billing period |
| 0x3D | 3 | 1.0.4.8.2.101 | 2 | Reactive energy, export (R−) — Tariff 2 | As of the end of the last billing period |
| 0x3E | 3 | 1.0.4.8.3.101 | 2 | Reactive energy, export (R−) — Tariff 3 | As of the end of the last billing period |
| 0x3F | 3 | 1.0.4.8.4.101 | 2 | Reactive energy, export (R−) — Tariff 4 | As of the end of the last billing period |

## Editorial notes — not part of the source

Points where this table disagrees with what the project currently assumes. Left
here as findings only; no code has been changed on the strength of them.

- **TAG 0x2C–0x3F is one contiguous run of end-of-period energy registers**, 20 of
  them. `tags.md` marks `0x2C`–`0x2F` as conflicting ("power group / previous
  period, differs") and lists `0x30`–`0x33` as a power-factor group; this source
  says `0x30` is A+ Tariff 4 and `0x31`–`0x33` are A− total / T1 / T2. The `.101`
  OBIS suffix running unbroken from `0x2C` to `0x3F` supports the reading here.
- **Nothing above 0x3F appears in this source.** `tags.md` assigns `0x34`–`0x47`
  to event/log counters and `0x48`–`0x4F` to identity objects; per this table
  `0x34`–`0x3F` are energy registers, and `0x40`+ is simply not documented.
- **TAG 0x2B is the LCD test**, available only in manual scroll mode. That is why
  it carries no width in the project's TAG table — it is a display self-test, not
  a value.
- **The sign qualifier applies to active power too** (`0x20`–`0x23`, "report with
  sign"), not only to reactive power. The project decodes `0x20`–`0x23` as
  unsigned BCD and only `0x24`–`0x27` as signed.
