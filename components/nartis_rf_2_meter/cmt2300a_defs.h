/*
 * CMT2300A Register Definitions - d101-2 / 443.9 MHz profile.
 *
 * Register banks and radio config are the authoritative RFPDK export for
 * 443.9 MHz / GFSK / 1.2 kbps, +20 dBm: narrow (~4 kHz) TX deviation and a
 * separate wide (~28 kHz) RX profile centred on the meter's reply.
 *
 * Register addresses/names mirror the CMOSTEK datasheet and cmt2300a_defs.h.
 */

#pragma once

#include <cstdint>

namespace esphome::nartis_rf_2_meter {

/* ================================================================
 * Register bank base addresses / sizes
 * ================================================================ */
static constexpr uint8_t CMT_BANK_ADDR = 0x00;
static constexpr uint8_t CMT_BANK_SIZE = 12;
static constexpr uint8_t SYSTEM_BANK_ADDR = 0x0C;
static constexpr uint8_t SYSTEM_BANK_SIZE = 12;
static constexpr uint8_t FREQUENCY_BANK_ADDR = 0x18;  // = REG_RF1 (first frequency register)
static constexpr uint8_t FREQUENCY_BANK_SIZE = 8;     // [RX-LO 4B][TX 4B]
static constexpr uint8_t DATA_RATE_BANK_ADDR = 0x20;
static constexpr uint8_t DATA_RATE_BANK_SIZE = 24;
static constexpr uint8_t BASEBAND_BANK_ADDR = 0x38;
static constexpr uint8_t BASEBAND_BANK_SIZE = 29;
static constexpr uint8_t TX_BANK_ADDR = 0x55;
static constexpr uint8_t TX_BANK_SIZE = 11;

/* ================================================================
 * Register addresses
 * ================================================================ */
static constexpr uint8_t REG_CMT4 = 0x03;
static constexpr uint8_t REG_CMT10 = 0x09;
static constexpr uint8_t REG_SYS2 = 0x0D;   // PLL/XO trim cluster - selects TX-half of freq bank
static constexpr uint8_t REG_SYS11 = 0x16;  // FIFO merge config
static constexpr uint8_t REG_PKT5 = 0x3C;   // SYNC_SIZE / SYNC_TOL
static constexpr uint8_t REG_PKT10 = 0x41;  // SYNC_VALUE bytes: PKT10..PKT13 (0x41..0x44)
static constexpr uint8_t REG_PKT12 = 0x43;  // SYNC_VALUE<55:48>
static constexpr uint8_t REG_PKT13 = 0x44;  // SYNC_VALUE<63:56>
static constexpr uint8_t REG_PKT14 = 0x45;  // PKT_TYPE / PAYLOAD_BIT_ORDER / PAYLOAD_LENG<10:8>
static constexpr uint8_t REG_PKT15 = 0x46;  // PAYLOAD_LENG<7:0>
static constexpr uint8_t REG_PKT29 = 0x54;  // FIFO threshold

static constexpr uint8_t REG_MODE_CTL = 0x60;
static constexpr uint8_t REG_MODE_STA = 0x61;
static constexpr uint8_t REG_IO_SEL = 0x65;
static constexpr uint8_t REG_INT2_CTL = 0x67;
static constexpr uint8_t REG_INT_EN = 0x68;
static constexpr uint8_t REG_FIFO_CTL = 0x69;
static constexpr uint8_t REG_INT_CLR1 = 0x6A;
static constexpr uint8_t REG_INT_CLR2 = 0x6B;
static constexpr uint8_t REG_FIFO_CLR = 0x6C;
static constexpr uint8_t REG_INT_FLAG = 0x6D;
static constexpr uint8_t REG_FIFO_FLAG = 0x6E;
static constexpr uint8_t REG_RSSI_DBM = 0x70;

static constexpr uint8_t REG_SOFT_RST = 0x7F;
static constexpr uint8_t SOFT_RST_VALUE = 0xFF;

/* ================================================================
 * Chip mode - GO commands (write to REG_MODE_CTL)
 * ================================================================ */
static constexpr uint8_t GO_STBY = 0x02;
static constexpr uint8_t GO_RFS = 0x04;
static constexpr uint8_t GO_RX = 0x08;
static constexpr uint8_t GO_TFS = 0x20;
static constexpr uint8_t GO_TX = 0x40;

/* ================================================================
 * Chip status - read from REG_MODE_STA
 * ================================================================ */
static constexpr uint8_t MASK_CHIP_MODE_STA = 0x0F;
static constexpr uint8_t STA_STBY = 0x02;
static constexpr uint8_t STA_RFS = 0x03;
static constexpr uint8_t STA_TFS = 0x04;
static constexpr uint8_t STA_RX = 0x05;
static constexpr uint8_t STA_TX = 0x06;

/* INT_CLR1 (0x6A) */
static constexpr uint8_t CLR1_TX_DONE_FLG = 0x08;

/* FIFO_CLR (0x6C) */
static constexpr uint8_t FIFO_RESTORE = 0x04;
static constexpr uint8_t FIFO_CLR_RX = 0x02;
static constexpr uint8_t FIFO_CLR_TX = 0x01;

/* FIFO_FLAG (0x6E) */
static constexpr uint8_t FIFO_RX_NMTY = 0x20;
static constexpr uint8_t FIFO_TX_TH = 0x01;  // 1 = unread TX bytes > threshold; 0 = room to refill

/* Chunked TX (frames larger than the merged FIFO) */
static constexpr uint8_t TX_REFILL_CHUNK = 15;  // bytes per TX_FIFO_TH refill

/* FIFO_CTL (0x69) */
static constexpr uint8_t MASK_FIFO_MERGE_EN = 0x02;
static constexpr uint8_t MASK_FIFO_RX_TX_SEL = 0x04;
static constexpr uint8_t MASK_SPI_FIFO_RD_WR_SEL = 0x01;

/* IO_SEL (0x65) - GPIO3 function */
static constexpr uint8_t MASK_GPIO3_SEL = 0x30;
static constexpr uint8_t GPIO3_SEL_INT2 = 0x20;

/* INT2_CTL (0x67) */
static constexpr uint8_t MASK_INT2_SEL = 0x1F;
static constexpr uint8_t MASK_INT_POLAR = 0x20;  // 0 = active-high
static constexpr uint8_t INT_SEL_RX_FIFO_TH = 0x0C;

/* SYS11 (0x16) - low 5 bits select FIFO/RSSI mode */
static constexpr uint8_t FIFO_MERGE_VALUE = 0x12;  // 64-B merged FIFO (packet mode)

/* PKT29 (0x54) - FIFO threshold */
static constexpr uint8_t MASK_FIFO_TH = 0x7F;
static constexpr uint8_t FIFO_TH_VALUE = 0x0F;  // 15 bytes

/* FIFO size when merged */
static constexpr uint8_t FIFO_SIZE_MERGED = 64;

/* Timing */
static constexpr uint32_t RESET_DELAY_MS = 20;
static constexpr uint32_t STATE_POLL_TIMEOUT_MS = 20;
static constexpr uint32_t STATE_POLL_INTERVAL_US = 100;

/* RX-half frequency-code step: 1 code ~= 6.199 Hz (used by set_rx_center). */
static constexpr float RX_CODE_HZ = 6.199f;

/* ================================================================
 * Frequency-bank computation (CMOSTEK AN199).
 *
 * Every meter channel (435.5-451.7 MHz; see frequency_from_address_) lies in
 * the 420-510 MHz PLL band, so the divider / VCO-bank selection AND all modem
 * banks (incl. AFC_OVF_TH) are constant across channels - only FREQ_RX_N/K and
 * FREQ_TX_N/K change. Verified byte-for-byte against RFPDK exports at
 * 435.5 / 439.7 / 443.9 / 447.4 / 451.7 MHz.
 *
 *   FREQ_LO_tx = f_rf ;   FREQ_LO_rx = f_rf + XTAL/92 (superhet IF)
 *   word = floor(FREQ_LO * DIVIDER / XTAL * 2^20) ;  N = word>>20 ;  K = word & 0xFFFFF
 * Bank layout: 0x18 RX_N | 0x19-0x1A RX_K[15:0] | 0x1B [PALDO|DIVX|RX_K[19:16]]
 *              0x1C TX_N | 0x1D-0x1E TX_K[15:0] | 0x1F [FSK_SWT|VCO_BANK|TX_K[19:16]]
 * ================================================================ */
static constexpr uint32_t XTAL_HZ = 26000000u;
static constexpr uint32_t FREQ_DIVIDER = 4u;          // 420-510 MHz band
static constexpr uint32_t FREQ_IF_HZ = XTAL_HZ / 92;  // RX LO offset above RF (282608 Hz)
static constexpr uint8_t FREQ_VCO_BANK = 0x1;         // <2:0>, 420-510 MHz
static constexpr uint8_t FREQ_DIVX_CODE = 0x1;        // <2:0>, 420-510 MHz
static constexpr uint8_t FREQ_PALDO_SEL = 0x0;        // TX < 500 MHz
static constexpr uint8_t FREQ_FSK_SWT = 0x0;          // RFPDK-fixed bit (0x1F bit7), freq-independent

/* ================================================================
 * d101-2 / 443.9 MHz register banks (RFPDK export, from the proven test app).
 * ================================================================ */
// clang-format off
static constexpr uint8_t CMT_BANK[12] = {
    0x00, 0x66, 0xEC, 0x1D, 0xF0, 0x80, 0x14, 0x08, 0x91, 0x02, 0x02, 0xD0
};
static constexpr uint8_t SYSTEM_BANK[12] = {
    0xAE, 0xE0, 0x35, 0x00, 0x00, 0xF4, 0x10, 0xE2, 0x42, 0x20, 0x00, 0x81
};
// TX profile: 1.2 kbps + narrow ~4 kHz deviation (loaded by base/tx init).
static constexpr uint8_t DATA_RATE_TX_BANK[24] = {
    0x19, 0x0C, 0x00, 0xBB, 0xC8, 0x9B, 0x0A, 0x0B, 0x9F, 0x39, 0x29, 0x29,
    0xC0, 0xA2, 0x54, 0x53, 0x00, 0x00, 0xB4, 0x00, 0x00, 0x01, 0x00, 0x00
};
// RX profile: matched to the meter's reply - 25 kHz deviation, 1.2 kbps, AFC on
// (loaded by rx init; TX stays narrow). RFPDK export 2026-07-24 with Rx Xtal Tol
// tightened to 5 ppm so Auto-Select picks a narrower channel filter: the reply is
// a very high modulation-index signal (+-25 kHz dev at 1.2 kbps, h~=41) and arrives
// weak, so a tighter filter cuts noise -> fewer discriminator spurious edges -> the
// counting CDR stops slipping bits. Diff vs the prior 28 kHz/20 ppm bank: reg 0x24
// E2->CA, 0x26 11->0F (dev 28->25 kHz), 0x27 0B->02 (bandwidth). Same baud, freq bank.
static constexpr uint8_t DATA_RATE_RX_BANK[24] = {
    0x19, 0x0C, 0x10, 0xBB, 0xCA, 0xDE, 0x0F, 0x02, 0xDF, 0x26, 0x29, 0x29,
    0xC0, 0xA2, 0x54, 0x53, 0x00, 0x00, 0xB4, 0x00, 0x00, 0x01, 0x00, 0x00
};
// Preamble 10x 0x55, sync 4B (value F6 55 55 55), fixed-length, no CRC/whiten.
// TX blends the HW sync to 0x55 and carries the real 98 f3 in the payload; RX
// reprograms the sync to the 2-byte 19 CF (= 98 f3 on air) - see the HAL.
static constexpr uint8_t BASEBAND_BANK[29] = {
    0x2A, 0x0A, 0x00, 0x55, 0x06, 0x00, 0x00, 0x00, 0x00, 0xF6, 0x55, 0x55, 0x55, 0x10, 0xFF, 0x00,
    0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x60, 0xFF, 0x00, 0x00, 0x1F, 0x10
};
static constexpr uint8_t TX_BANK[11] = {  // +20 dBm (4 kHz dev ramp)
    0x50, 0x85, 0x02, 0x00, 0x86, 0xD0, 0x00, 0x8A, 0x18, 0x3F, 0x7F
};
// FREQUENCY bank = [RX-LO 4B][TX 4B]. Reference value for 443.900 MHz (RFPDK
// export). The bank is now computed per channel at runtime (see compute_freq_bank_
// / AN199 above); this constant is the k=12 anchor the formula reproduces exactly.
static constexpr uint8_t FREQ_443M9[8] = {
    0x44, 0x60, 0x5F, 0x15,  0x44, 0x4A, 0xAD, 0x14
};
// clang-format on

}  // namespace esphome::nartis_rf_2_meter
