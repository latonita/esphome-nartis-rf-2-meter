// nartis_dlt645_f1xx.h
// Wire layouts for NARTIS / Wasion DL/T 645 F101 & F102 fixed-block reads (C=0x01).
//
// Structs describe the DL/T645 DATA field AFTER the -0x33 descramble, including the
// 2-byte DI echo. Full frame is:  68 <addr*6> 68 C L <DATA+0x33> CS 16.
// So: descramble (subtract 0x33 from each of the L DATA bytes), then memcpy/overlay.
//
// Meters:
//   3ph = Wasion DTSD341-MB530 1-0(240411)   (NARTIS-I300)
//   1ph = Wasion DDSD101-Z860  1-0(240406)   (NARTIS 1-phase)

#ifndef NARTIS_DLT645_F1XX_H
#define NARTIS_DLT645_F1XX_H

#include <stdint.h>

#pragma pack(push, 1)

// -------- primitive wire types --------

// 4-byte packed BCD, little-endian byte order (least-significant BCD pair first).
// value = ((b[3]&0x7F)hi,lo)*1e6 + b[2]*1e4 + b[1]*1e2 + b[0], each byte = hi*10+lo.
// Signed quantities (reactive power; 1ph current): bit 0x80 of b[3] set => negative.
struct bcd32_le { uint8_t b[4]; };

// Raw little-endian binary 32-bit (energy accumulators, fmt 0x84).
typedef uint32_t u32le;

// -------- shared 10-byte status/identity block --------
// Appended by F101 (tail) and F104 (head). Field meanings are assumptions.
// On 3ph, temp_or_id = obj 0x5EC0 (internal °C);
// on 1ph the corresponding object is 0x5EB8.
struct dlt645_status10 {
    uint8_t  zero0;        // always 0x00
    uint8_t  flags[4];     // alarm/device-state flag bytes
    uint8_t  zero1;        // always 0x00
    uint8_t  obj_3E1C;     // low byte of obj 0x3E1C
    uint8_t  temp_or_id;   // 3ph: internal temperature °C (obj 0x5EC0); 1ph: obj 0x5EB8
    uint8_t  comm_status;  // comms status  (FUN 0x08027d70 arg 0x10)
    uint8_t  relay_status; // relay state (0xFF if relay object absent / state > 3)
};

// ============================================================
//  F101 — energy accumulators + status   (DI = 0xF101)
//  Layout is IDENTICAL on 1ph and 3ph (values differ; 1ph phases are single).
//  Two energy group-objects (R+/R-), each expanded to 9 raw-LE u32 = [total, T1..T8].
//  Total DATA length L = 84 (payload 0x52 = 82, + 2-byte DI).
// ============================================================
struct nartis_f101 {
    uint8_t  di[2];             // DI echo: 01 F1  (LE = 0xF101)
    u32le    group20[9];        // obj 0x20 R+ energy group: [total, T1..T8], raw LE
    u32le    group30[9];        // obj 0x30 R- energy group: [total, T1..T8], raw LE
    dlt645_status10 status;     // shared 10-byte status block
};
typedef nartis_f101 f101_1ph;   // same layout on 1-phase...
typedef nartis_f101 f101_3ph;   // ...and 3-phase.

// ============================================================
//  F102 — live instantaneous P/Q/U/I/frequency   (DI = 0xF102)
//  DIFFERENT between 1ph and 3ph.
// ============================================================

// ---- 3-phase: DI(2) | marker 0x01 | 15 x BCD-4 ----
// Total DATA length L = 63 (payload 0x3D = 61, + 2-byte DI).
// Scales: P x0.1 W, Q x0.1 var (SIGNED), U x0.01 V, I x0.01 A, f x0.01 Hz.
// Note U/I are interleaved per phase; P and Q are grouped (total first).
struct f102_3ph {
    uint8_t  di[2];        // 02 F1
    uint8_t  marker;       // 0x01
    bcd32_le p_total;      // obj 0xC1  active power total     x0.1 W
    bcd32_le p_l1;         // obj 0xC2  active power L1        x0.1 W
    bcd32_le p_l2;         // obj 0xC3  active power L2        x0.1 W
    bcd32_le p_l3;         // obj 0xC4  active power L3        x0.1 W
    bcd32_le q_total;      // obj 0xF1  reactive total SIGNED x0.1 var
    bcd32_le q_l1;         // obj 0xF2  reactive L1   SIGNED  x0.1 var
    bcd32_le q_l2;         // obj 0xF3  reactive L2   SIGNED  x0.1 var
    bcd32_le q_l3;         // obj 0xF4  reactive L3   SIGNED  x0.1 var
    bcd32_le u_l1;         // obj 0x131 voltage L1           x0.01 V
    bcd32_le i_l1;         // obj 0x151 current L1           x0.01 A
    bcd32_le u_l2;         // obj 0x132 voltage L2           x0.01 V
    bcd32_le i_l2;         // obj 0x152 current L2           x0.01 A
    bcd32_le u_l3;         // obj 0x133 voltage L3           x0.01 V
    bcd32_le i_l3;         // obj 0x153 current L3           x0.01 A
    bcd32_le freq;         // obj 0xE00 frequency            x0.01 Hz
};

// ---- 1-phase: DI(2) | lead 0x00 | 5 x BCD-4 ----
// Total DATA length L = 23 (payload 0x15 = 21, + 2-byte DI).
// Firmware pre-scales before BCD: U = obj/10, I = obj/100 (signed), f = obj/100.
// P and Q are appended raw (native object units, no divide).
struct f102_1ph {
    uint8_t  di[2];        // 02 F1
    uint8_t  lead;         // 0x00
    bcd32_le p;            // obj 0x00C1 active power   (raw BCD, native units)
    bcd32_le q;            // obj 0x00F1 reactive power (raw BCD; SIGNED via 0x80)
    bcd32_le u;            // obj 0x0131 voltage        (firmware value/10)
    bcd32_le i;            // obj 0x0141 current SIGNED  (firmware value/100)
    bcd32_le freq;         // obj 0x0E00 frequency       (firmware value/100)
};

#pragma pack(pop)

// -------- size checks --------
#if defined(__cplusplus) && __cplusplus >= 201103L
static_assert(sizeof(bcd32_le)        ==  4, "bcd32_le");
static_assert(sizeof(dlt645_status10) == 10, "status10");
static_assert(sizeof(nartis_f101)     == 84, "f101");
static_assert(sizeof(f102_3ph)        == 63, "f102_3ph");
static_assert(sizeof(f102_1ph)        == 23, "f102_1ph");
#endif

// -------- decode helpers --------

// Decode a 4-byte LE packed-BCD field to unsigned integer (ignores sign bit).
static inline uint32_t bcd32_value(const bcd32_le* v) {
    uint32_t out = 0, mul = 1;
    for (int i = 0; i < 4; ++i) {
        uint8_t byte = v->b[i];
        if (i == 3) byte &= 0x7F;            // strip sign bit from MSB
        out += ((byte >> 4) * 10u + (byte & 0x0F)) * mul;
        mul *= 100u;
    }
    return out;
}

// Signed variant: bit 0x80 of the MSB byte = negative (sign-magnitude BCD).
static inline int32_t bcd32_signed(const bcd32_le* v) {
    int32_t m = (int32_t)bcd32_value(v);
    return (v->b[3] & 0x80) ? -m : m;
}

#endif // NARTIS_DLT645_F1XX_H
