# Fixed blocks: DI `0xF101`, DI `0xF102`

Positional values, no TAGs. Layouts in `nartis_dlt645_f1xx.h`, TAG mapping in
`F101_MAP` / `F102_*_MAP` (`d101_frame.cpp`). Identified by DATA length:
F101 = 84 B, F102 = 63 B three-phase, 23 B single-phase.

`Scale`: physical = raw x scale. `TAG` = the item TAG it publishes as, `-` = not
published. All F102 values are 4-byte BCD LE; F101 is 4-byte binary LE.

## DI `0xF102`, three-phase (63 B)

| # | Value | Unit | Scale | TAG |
|---|---|---|---|---|
| 1 | Active power, total (signed) | W | x0.1 | `0x20` |
| 2 | Active power L1 (signed) | W | x0.1 | `0x21` |
| 3 | Active power L2 (signed) | W | x0.1 | `0x22` |
| 4 | Active power L3 (signed) | W | x0.1 | `0x23` |
| 5 | Reactive power, total (signed) | var | x0.1 | `0x24` |
| 6 | Reactive power L1 (signed) | var | x0.1 | `0x25` |
| 7 | Reactive power L2 (signed) | var | x0.1 | `0x26` |
| 8 | Reactive power L3 (signed) | var | x0.1 | `0x27` |
| 9 | Voltage L1 | V | x0.01 | `0x15` |
| 10 | Current L1 | A | x0.01 | `0x1C` |
| 11 | Voltage L2 | V | x0.01 | `0x16` |
| 12 | Current L2 | A | x0.01 | `0x1D` |
| 13 | Voltage L3 | V | x0.01 | `0x17` |
| 14 | Current L3 | A | x0.01 | `0x1E` |
| 15 | Frequency | Hz | x0.01 | `0x28` |

Preceded by the 2-byte DI echo and a `0x01` marker. U and I interleave per phase.

## DI `0xF102`, single-phase (23 B)

| # | Value | Unit | Scale | TAG |
|---|---|---|---|---|
| 1 | Active power (signed) | W | x0.1 | `0x20` |
| 2 | Reactive power (signed) | var | x0.1 | `0x24` |
| 3 | Voltage | V | **?** | `-` |
| 4 | Current (signed) | A | **?** | `-` |
| 5 | Frequency | Hz | **?** | `-` |

Preceded by the 2-byte DI echo and a `0x00` lead. Never captured. The firmware
pre-divides U, I and f, which would leave them at 0.1 V / 1 A / 1 Hz - too coarse
to be right, so those three are logged raw and not published.

## DI `0xF101` (84 B)

| # | Value | Unit | Scale | TAG |
|---|---|---|---|---|
| 1 | Reactive energy import (R+), total | kvarh | x0.001 | `0x0A` |
| 2 | Reactive energy import (R+), T1 | kvarh | x0.001 | `0x0B` |
| 3 | Reactive energy import (R+), T2 | kvarh | x0.001 | `0x0C` |
| 4 | Reactive energy import (R+), T3 | kvarh | x0.001 | `0x0D` |
| 5 | Reactive energy import (R+), T4 | kvarh | x0.001 | `0x0E` |
| 6 | Reactive energy import (R+), T5 | kvarh | x0.001 | `-` |
| 7 | Reactive energy import (R+), T6 | kvarh | x0.001 | `-` |
| 8 | Reactive energy import (R+), T7 | kvarh | x0.001 | `-` |
| 9 | Reactive energy import (R+), T8 | kvarh | x0.001 | `-` |
| 10 | Reactive energy export (R-), total | kvarh | x0.001 | `0x0F` |
| 11 | Reactive energy export (R-), T1 | kvarh | x0.001 | `0x10` |
| 12 | Reactive energy export (R-), T2 | kvarh | x0.001 | `0x11` |
| 13 | Reactive energy export (R-), T3 | kvarh | x0.001 | `0x12` |
| 14 | Reactive energy export (R-), T4 | kvarh | x0.001 | `0x13` |
| 15 | Reactive energy export (R-), T5 | kvarh | x0.001 | `-` |
| 16 | Reactive energy export (R-), T6 | kvarh | x0.001 | `-` |
| 17 | Reactive energy export (R-), T7 | kvarh | x0.001 | `-` |
| 18 | Reactive energy export (R-), T8 | kvarh | x0.001 | `-` |

Preceded by the 2-byte DI echo, followed by the 10-byte status block. Values are
binary LE, not BCD. Eight tariffs where the TAG list reaches four, so T5-T8 have
no TAG and appear only in the log.

This is the reactive-energy half of the meter: the lists carry active energy and
no reactive, this block the other way round.

The scale was measured, not assumed, and it needed two captures 76 minutes apart.
Group 0x30 advanced 153 counts; at x0.001 kvarh that is 121 var averaged over the
interval, against the 106 and 126 var DI `0xF102` measured at each end. The same
arithmetic on `TAG 0x00`, whose scale was already known, gives 1535 W against a
measured 1465 and 1526 W - the same ~3% excess, so the method holds and the load
was simply a little higher between samples.

Which group is which direction rests on the object numbering: the lists number
their energy families 0x0001 active import and 0x0011 active export, so 0x0021 and
0x0031 are the two reactive directions. Consistent with the rates - Q was negative
throughout, and group 0x30 was the one advancing while group 0x20 sat still - but
the import assignment has not been seen move.

## Status block (10 B, tail of F101)

| # | Field | Meaning | Observed |
|---|---|---|---|
| 1 | `zero0` | - | `0x01` |
| 2-5 | `flags[4]` | alarm / device state | `00 20 84 80` |
| 6 | `zero1` | - | `0x00` |
| 7 | `obj_3E1C` | low byte of object 0x3E1C | `0x01` |
| 8 | `temp_or_id` | internal temperature, C | `0x1A` = 26 |
| 9 | `comm_status` | communication status | `0x00` |
| 10 | `relay_status` | relay state, `0xFF` if absent | `0x03` |

A list status half carries these same 10 bytes plus a trailing `0x01`.
