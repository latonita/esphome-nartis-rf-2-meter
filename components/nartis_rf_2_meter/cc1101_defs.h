/*
 * TI CC1101 register definitions - RF433-2 (d101-2) profile.
 *
 * Names and numbering follow the CC1101 datasheet (SWRS061I). This header is
 * deliberately standalone: ESPHome ships a `cc1101` component with an equivalent
 * table, but that one is an OOK-oriented driver in its own namespace and is not a
 * dependency of this project, so the layout is mirrored here rather than imported.
 *
 * Everything that depends on a user-settable parameter (carrier, deviation, RX
 * bandwidth, output power, the FOC carrier-sense gate) is solved at runtime in
 * cc1101_hal.cpp from the datasheet formulas - only the fixed part of the profile
 * lives in the table below, so there is exactly one place where a magic number can
 * be wrong.
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_2_meter {

/* ================================================================
 * Crystal
 * ================================================================ */

/* Every CC1101 register formula is referred to the crystal. 26 MHz is what the
 * common E07/CC1101 modules fit, and it is also what the CMT2300A bridge runs, so
 * the two backends share the same reference and the same rounding behaviour. A
 * 27 MHz module would need this changed and every derived value re-checked. */
static constexpr uint32_t CC1101_XTAL_HZ = 26000000u;

/// Synthesiser step, Fxosc / 2^16 = 396.728 Hz. Only used for logging the residual.
static constexpr float CC1101_FREQ_STEP_HZ = 26000000.0f / 65536.0f;

/* ================================================================
 * SPI header byte
 * ================================================================ */

/* The first byte of every access is R/W | BURST | address<5:0>. Status registers
 * (0x30-0x3D) collide with the command strobes and must be read with BURST set,
 * otherwise the chip executes a strobe instead of returning the register. */
static constexpr uint8_t CC1101_HDR_WRITE = 0x00;
static constexpr uint8_t CC1101_HDR_READ = 0x80;
static constexpr uint8_t CC1101_HDR_BURST = 0x40;

/* ================================================================
 * Configuration registers (0x00-0x2E)
 * ================================================================ */
static constexpr uint8_t CC1101_REG_IOCFG2 = 0x00;    // GDO2 pin configuration
static constexpr uint8_t CC1101_REG_IOCFG1 = 0x01;    // GDO1 pin configuration (shared with MISO)
static constexpr uint8_t CC1101_REG_IOCFG0 = 0x02;    // GDO0 pin configuration
static constexpr uint8_t CC1101_REG_FIFOTHR = 0x03;   // RX/TX FIFO thresholds
static constexpr uint8_t CC1101_REG_SYNC1 = 0x04;     // Sync word, high byte
static constexpr uint8_t CC1101_REG_SYNC0 = 0x05;     // Sync word, low byte
static constexpr uint8_t CC1101_REG_PKTLEN = 0x06;    // Packet length (fixed mode)
static constexpr uint8_t CC1101_REG_PKTCTRL1 = 0x07;  // Packet automation control
static constexpr uint8_t CC1101_REG_PKTCTRL0 = 0x08;  // Packet automation control
static constexpr uint8_t CC1101_REG_ADDR = 0x09;      // Device address
static constexpr uint8_t CC1101_REG_CHANNR = 0x0A;    // Channel number
static constexpr uint8_t CC1101_REG_FSCTRL1 = 0x0B;   // Synthesiser control (IF)
static constexpr uint8_t CC1101_REG_FSCTRL0 = 0x0C;   // Synthesiser control (freq offset)
static constexpr uint8_t CC1101_REG_FREQ2 = 0x0D;     // Carrier word <23:16>
static constexpr uint8_t CC1101_REG_FREQ1 = 0x0E;     // Carrier word <15:8>
static constexpr uint8_t CC1101_REG_FREQ0 = 0x0F;     // Carrier word <7:0>
static constexpr uint8_t CC1101_REG_MDMCFG4 = 0x10;   // CHANBW_E/M + DRATE_E
static constexpr uint8_t CC1101_REG_MDMCFG3 = 0x11;   // DRATE_M
static constexpr uint8_t CC1101_REG_MDMCFG2 = 0x12;   // Modulation / Manchester / sync mode
static constexpr uint8_t CC1101_REG_MDMCFG1 = 0x13;   // FEC / preamble length / channel spacing exp
static constexpr uint8_t CC1101_REG_MDMCFG0 = 0x14;   // Channel spacing mantissa
static constexpr uint8_t CC1101_REG_DEVIATN = 0x15;   // DEVIATION_E/M
static constexpr uint8_t CC1101_REG_MCSM2 = 0x16;     // RX timeout
static constexpr uint8_t CC1101_REG_MCSM1 = 0x17;     // CCA / RXOFF / TXOFF
static constexpr uint8_t CC1101_REG_MCSM0 = 0x18;     // Auto-calibration / PO timeout
static constexpr uint8_t CC1101_REG_FOCCFG = 0x19;    // Frequency offset compensation
static constexpr uint8_t CC1101_REG_BSCFG = 0x1A;     // Bit synchronisation
static constexpr uint8_t CC1101_REG_AGCCTRL2 = 0x1B;  // AGC control
static constexpr uint8_t CC1101_REG_AGCCTRL1 = 0x1C;  // AGC control
static constexpr uint8_t CC1101_REG_AGCCTRL0 = 0x1D;  // AGC control
static constexpr uint8_t CC1101_REG_WOREVT1 = 0x1E;   // Wake-on-radio event, high
static constexpr uint8_t CC1101_REG_WOREVT0 = 0x1F;   // Wake-on-radio event, low
static constexpr uint8_t CC1101_REG_WORCTRL = 0x20;   // Wake-on-radio control
static constexpr uint8_t CC1101_REG_FREND1 = 0x21;    // RX front end
static constexpr uint8_t CC1101_REG_FREND0 = 0x22;    // TX front end (PA_POWER -> PATABLE index)
static constexpr uint8_t CC1101_REG_FSCAL3 = 0x23;    // Synthesiser calibration
static constexpr uint8_t CC1101_REG_FSCAL2 = 0x24;
static constexpr uint8_t CC1101_REG_FSCAL1 = 0x25;
static constexpr uint8_t CC1101_REG_FSCAL0 = 0x26;
static constexpr uint8_t CC1101_REG_RCCTRL1 = 0x27;  // RC oscillator configuration
static constexpr uint8_t CC1101_REG_RCCTRL0 = 0x28;
static constexpr uint8_t CC1101_REG_FSTEST = 0x29;   // Test registers - datasheet says do not write
static constexpr uint8_t CC1101_REG_PTEST = 0x2A;
static constexpr uint8_t CC1101_REG_AGCTEST = 0x2B;
static constexpr uint8_t CC1101_REG_TEST2 = 0x2C;  // Must track the RX filter bandwidth
static constexpr uint8_t CC1101_REG_TEST1 = 0x2D;
static constexpr uint8_t CC1101_REG_TEST0 = 0x2E;  // VCO selection calibration

/* ================================================================
 * Status registers (0x30-0x3D) - read-only, burst bit mandatory
 * ================================================================ */
static constexpr uint8_t CC1101_REG_PARTNUM = 0x30;    // 0x00 on a genuine CC1101
static constexpr uint8_t CC1101_REG_VERSION = 0x31;    // 0x14 on current die revisions
static constexpr uint8_t CC1101_REG_FREQEST = 0x32;    // Frequency offset estimate from FOC
static constexpr uint8_t CC1101_REG_LQI = 0x33;        // Link quality
static constexpr uint8_t CC1101_REG_RSSI = 0x34;       // RSSI, 2's complement half-dB
static constexpr uint8_t CC1101_REG_MARCSTATE = 0x35;  // Main radio control state machine
static constexpr uint8_t CC1101_REG_WORTIME1 = 0x36;
static constexpr uint8_t CC1101_REG_WORTIME0 = 0x37;
static constexpr uint8_t CC1101_REG_PKTSTATUS = 0x38;  // GDO levels, CS, CRC_OK
static constexpr uint8_t CC1101_REG_VCO_VC_DAC = 0x39;
static constexpr uint8_t CC1101_REG_TXBYTES = 0x3A;  // <6:0> count, <7> underflow
static constexpr uint8_t CC1101_REG_RXBYTES = 0x3B;  // <6:0> count, <7> overflow
static constexpr uint8_t CC1101_REG_RCCTRL1_STATUS = 0x3C;
static constexpr uint8_t CC1101_REG_RCCTRL0_STATUS = 0x3D;

/* Multi-byte registers: PATABLE auto-increments through its 8 entries on a burst
 * access and wraps; the FIFO is one address that reads RX and writes TX. */
static constexpr uint8_t CC1101_REG_PATABLE = 0x3E;
static constexpr uint8_t CC1101_REG_FIFO = 0x3F;

static constexpr uint8_t CC1101_PARTNUM_EXPECTED = 0x00;
static constexpr uint8_t CC1101_VERSION_EXPECTED = 0x14;

static constexpr size_t CC1101_FIFO_SIZE = 64;
static constexpr size_t CC1101_PATABLE_SIZE = 8;

/* The largest NUM_RXBYTES that is a real count, not a bus fault. The RX FIFO is 64
 * bytes, but the receive chain also holds one byte in a prefetch register ahead of
 * it, and RXBYTES counts that byte too - so 65 is a documented, legitimate state and
 * NOT evidence of anything wrong. TI's errata note SWRZ020E ("RX FIFO problem")
 * shows exactly this: it is at 65, with the next byte, that the FIFO overflows, and
 * its workaround is to read n - 1 bytes while the chip is still receiving, which is
 * what drain_rx() does unconditionally anyway. With this profile - APPEND_STATUS = 0
 * in PKTCTRL1, CRC_EN = 0 in PKTCTRL0, FEC_EN = 0 in MDMCFG1 - there are no status,
 * CRC or FEC bytes in flight to make the accounting any different. A count ABOVE 65
 * cannot be produced by the part at all, so that one really is a bus fault. */
static constexpr uint8_t CC1101_RXBYTES_MAX_VALID = (uint8_t) (CC1101_FIFO_SIZE + 1);

static constexpr uint8_t CC1101_TXBYTES_UNDERFLOW = 0x80;
static constexpr uint8_t CC1101_RXBYTES_OVERFLOW = 0x80;
static constexpr uint8_t CC1101_FIFO_COUNT_MASK = 0x7F;

/* ================================================================
 * Command strobes (0x30-0x3D with the burst bit clear)
 * ================================================================ */
static constexpr uint8_t CC1101_STROBE_SRES = 0x30;    // Reset chip
static constexpr uint8_t CC1101_STROBE_SFSTXON = 0x31;  // Calibrate and hold the synthesiser
static constexpr uint8_t CC1101_STROBE_SXOFF = 0x32;   // Crystal off
static constexpr uint8_t CC1101_STROBE_SCAL = 0x33;    // Calibrate synthesiser, then IDLE
static constexpr uint8_t CC1101_STROBE_SRX = 0x34;     // Enable RX
static constexpr uint8_t CC1101_STROBE_STX = 0x35;     // Enable TX
static constexpr uint8_t CC1101_STROBE_SIDLE = 0x36;   // Leave RX/TX
static constexpr uint8_t CC1101_STROBE_SWOR = 0x38;    // Wake-on-radio
static constexpr uint8_t CC1101_STROBE_SPWD = 0x39;    // Power down on CS high
static constexpr uint8_t CC1101_STROBE_SFRX = 0x3A;    // Flush RX FIFO (IDLE or overflow only)
static constexpr uint8_t CC1101_STROBE_SFTX = 0x3B;    // Flush TX FIFO (IDLE or underflow only)
static constexpr uint8_t CC1101_STROBE_SWORRST = 0x3C;
static constexpr uint8_t CC1101_STROBE_SNOP = 0x3D;  // No-op: the cheapest way to fetch the status byte

/* ================================================================
 * Status byte (returned on MISO during every header byte)
 * ================================================================ */
static constexpr uint8_t CC1101_STATUS_CHIP_RDYN = 0x80;
static constexpr uint8_t CC1101_STATUS_STATE_MASK = 0x70;
static constexpr uint8_t CC1101_STATUS_FIFO_MASK = 0x0F;

static constexpr uint8_t CC1101_STATE_IDLE = 0;
static constexpr uint8_t CC1101_STATE_RX = 1;
static constexpr uint8_t CC1101_STATE_TX = 2;
static constexpr uint8_t CC1101_STATE_FSTXON = 3;
static constexpr uint8_t CC1101_STATE_CALIBRATE = 4;
static constexpr uint8_t CC1101_STATE_SETTLING = 5;
static constexpr uint8_t CC1101_STATE_RXFIFO_OVERFLOW = 6;
static constexpr uint8_t CC1101_STATE_TXFIFO_UNDERFLOW = 7;

/* ================================================================
 * MARCSTATE (0x35) - the fine-grained state, <4:0>
 * ================================================================ */
static constexpr uint8_t CC1101_MARCSTATE_MASK = 0x1F;
static constexpr uint8_t CC1101_MARC_SLEEP = 0x00;
static constexpr uint8_t CC1101_MARC_IDLE = 0x01;
static constexpr uint8_t CC1101_MARC_XOFF = 0x02;
static constexpr uint8_t CC1101_MARC_MANCAL = 0x05;
static constexpr uint8_t CC1101_MARC_FS_LOCK = 0x0A;
static constexpr uint8_t CC1101_MARC_ENDCAL = 0x0C;
static constexpr uint8_t CC1101_MARC_RX = 0x0D;
static constexpr uint8_t CC1101_MARC_RX_END = 0x0E;
static constexpr uint8_t CC1101_MARC_RXFIFO_OVERFLOW = 0x11;
static constexpr uint8_t CC1101_MARC_FSTXON = 0x12;
static constexpr uint8_t CC1101_MARC_TX = 0x13;
static constexpr uint8_t CC1101_MARC_TX_END = 0x14;
static constexpr uint8_t CC1101_MARC_TXFIFO_UNDERFLOW = 0x16;

/* ================================================================
 * Field encodings used by the driver
 * ================================================================ */

/* PKTCTRL0<1:0>. The RF433-2 length byte is the first byte after sync, but it is
 * bit-reversed on air, so the CC1101's own variable-length engine would read the
 * wrong value. Infinite length lets the engine stream forever and the component
 * bounds the frame by the LEN byte itself; fixed length is used only for TX, where
 * we know exactly how many bytes we are sending. */
static constexpr uint8_t CC1101_PKTCTRL0_LENGTH_FIXED = 0x00;
static constexpr uint8_t CC1101_PKTCTRL0_LENGTH_VARIABLE = 0x01;
static constexpr uint8_t CC1101_PKTCTRL0_LENGTH_INFINITE = 0x02;

/* PKTCTRL0 with whitening off (bit 6), normal FIFO packet format (bits 5:4 = 00)
 * and the PHY CRC off (bit 2) - RF433-2 has none of the three; the frame's
 * CRC-16/X.25 is checked by the transport layer. */
static constexpr uint8_t CC1101_PKTCTRL0_TX = CC1101_PKTCTRL0_LENGTH_FIXED;
static constexpr uint8_t CC1101_PKTCTRL0_RX = CC1101_PKTCTRL0_LENGTH_INFINITE;

/* The sync word as the CC1101 must hold it. RF433-2 is LSB-first on air and the
 * CC1101 shifts MSB-first with no bit-order control, so the logical 98 F3 goes in
 * bit-reversed: reverse8(0x98) = 0x19, reverse8(0xF3) = 0xCF. The CMT2300A backend
 * reaches the same two bytes from the other direction - it keeps its sync registers
 * in the on-air domain and they read 19 CF there too (see Cmt2300aHal::init_rx) -
 * which is independent corroboration rather than a shared assumption. */
static constexpr uint8_t CC1101_SYNC1_ON_AIR = 0x19;
static constexpr uint8_t CC1101_SYNC0_ON_AIR = 0xCF;

/// The logical sync word as it arrives at transmit(), per the RadioHal contract.
static constexpr uint8_t RF433_2_SYNC_HI = 0x98;
static constexpr uint8_t RF433_2_SYNC_LO = 0xF3;

/* MDMCFG3: data rate mantissa. R = (256 + DRATE_M) * 2^DRATE_E * Fxosc / 2^28.
 * Target 1200 baud needs (256 + M) * 2^E = 1200 * 2^28 / 26e6 = 12389.3, which only
 * E = 5 can express with M in [0, 255]: M = 12389.3/32 - 256 = 131.2 -> 131.
 * (256 + 131) * 32 * 26e6 / 2^28 = 1199.49 baud, i.e. -0.04 % - four orders of
 * magnitude inside what the bit synchroniser tolerates over a ~90 byte frame.
 * The rate is a fixed property of the PHY, so unlike bandwidth and deviation it is
 * not exposed as a tunable. */
static constexpr uint8_t CC1101_DRATE_E = 5;
static constexpr uint8_t CC1101_DRATE_M = 131;

/* The same rate as an integer, in milli-baud: (256 + 131) * 2^5 * 26e6 / 2^28 =
 * 1199.484... baud. Kept here so transmit()'s airtime arithmetic uses the rate the
 * chip is actually programmed with rather than the nominal 1200, and so that the two
 * cannot drift apart if DRATE_E/M are ever retuned. Integer milli-baud also keeps
 * that arithmetic free of floating point on a path that runs per transmission. */
static constexpr uint32_t CC1101_DRATE_MILLIBAUD = 1199484u;

/* What the packet engine puts on the air ahead of the payload, which airtime has to
 * account for: NUM_PREAMBLE = 101 in MDMCFG1 below is 12 preamble bytes, and the
 * sync word in SYNC1/SYNC0 is 2 more. These are the register settings restated as
 * numbers, so MDMCFG1 and the timeout cannot disagree about the preamble length. */
static constexpr uint32_t CC1101_PREAMBLE_BYTES = 12u;
static constexpr uint32_t CC1101_SYNC_BYTES = 2u;

/* FOCCFG. Everything but FOC_BS_CS_GATE is fixed: FOC_PRE_K = 10 (3K before sync),
 * FOC_POST_K = 1 (K/2 after), FOC_LIMIT = 10 (+-BW/4), i.e. 0x16. FOC_BS_CS_GATE is
 * bit 5, so setting it gives 0x36 and changes nothing else - see the long note at
 * Cc1101Hal::set_foc_cs_gate() for why that bit is a bring-up knob rather than a
 * decision this driver can make from the datasheet. */
static constexpr uint8_t CC1101_FOCCFG_BASE = 0x16;
static constexpr uint8_t CC1101_FOCCFG_BS_CS_GATE = 0x20;

/* TEST2/TEST1 are not "do not write" registers like FSTEST/PTEST/AGCTEST: TI gives
 * two characterised pairs for them and says which to use, keyed on the RX channel
 * filter bandwidth. 0x88/0x31 is the pair for bandwidths ABOVE 101 kHz and 0x81/0x35
 * the pair at or below it (CC1101 datasheet SWRS061, the TEST2/TEST1 register
 * descriptions, and what SmartRF Studio emits either side of that bandwidth). The
 * driver picks between them from the solved bandwidth in Cc1101Hal::solve_params_(),
 * so the whole 58-812 kHz range set_rx_bandwidth_hz() accepts stays characterised
 * instead of only the lower half of it. */
static constexpr uint8_t CC1101_TEST2_NARROW_BW = 0x81;
static constexpr uint8_t CC1101_TEST1_NARROW_BW = 0x35;
static constexpr uint8_t CC1101_TEST2_WIDE_BW = 0x88;
static constexpr uint8_t CC1101_TEST1_WIDE_BW = 0x31;
/* The bandwidth that divides the two pairs above: at or below it the narrow pair,
 * strictly above it the wide pair. */
static constexpr uint32_t CC1101_TEST_REG_BW_THRESHOLD_HZ = 101000u;

/* ================================================================
 * Defaults for the YAML-settable knobs
 * ================================================================ */

/* Channel 12 (443.900 MHz) - the reference channel of the spec. The component
 * always calls set_frequency() with the channel derived from the meter serial, so
 * this only ever shows up if that call were skipped. */
static constexpr uint32_t CC1101_DEFAULT_FREQUENCY_HZ = 443900000u;

/* The SDR capture measured the meter replying a steady +1.10 kHz above the host
 * carrier (two independent crystals, not an intentional offset). That is well
 * inside the CC1101's frequency offset compensation range, which is configured for
 * +-BW/4 = +-20 kHz at the default bandwidth, so the default is to centre RX on the
 * TX carrier and let FOC pull it in. The knob exists for the case where FOC turns
 * out to mis-track on this h~41 waveform and the window wants nudging by hand. */
static constexpr int32_t CC1101_DEFAULT_RX_OFFSET_HZ = 0;

/* +10 dBm is the top of the CC1101's 433 MHz PATABLE. See the link-budget warning
 * in cc1101_hal.h: a real Nartis CIU transmits +20 dBm. */
static constexpr int8_t CC1101_DEFAULT_OUTPUT_POWER_DBM = 10;

/* Carson bandwidth for the reply is 2*(25.0 + 1.2) ~= 52 kHz, plus the +1.1 kHz
 * crystal offset and whatever the ESP board's own module drifts. 81.25 kHz
 * (CHANBW_E=3, CHANBW_M=1) leaves comfortable margin; the chip's narrowest setting
 * is 58.0 kHz (E=3, M=3), which only just clears Carson and leaves none. */
static constexpr uint32_t CC1101_DEFAULT_RX_BANDWIDTH_HZ = 81250u;

/* Both directions are wide: the 2026-08-31 SDR capture of a real CIU measured the
 * host transmitting +-24.72...24.74 kHz, the same as the meter's +-24.5 kHz reply.
 * The older "+-4 kHz TX" figure in the spec came from a bench register readout, not
 * from the air, and is superseded. */
static constexpr uint32_t CC1101_DEFAULT_DEVIATION_HZ = 25000u;

/* ================================================================
 * PATABLE - 433 MHz optimum settings (CC1101 datasheet, Table 39)
 * ================================================================ */

/* These are the TI-characterised PA settings for the 433 MHz band; using the 868 or
 * 915 MHz column here would give both the wrong power and worse spectral purity.
 * The table is sparse, so set_output_power_dbm() snaps to the nearest entry and
 * dump_config() reports what was actually programmed. */
struct Cc1101PaSetting {
  int8_t dbm;
  uint8_t pa;
};

/* `inline` rather than `static`: the lookup below is an inline function that takes
 * the table's address, so the table has to be one entity across translation units. */
inline constexpr Cc1101PaSetting CC1101_PA_TABLE_433[] = {
    {-30, 0x12}, {-20, 0x0E}, {-15, 0x1D}, {-10, 0x34}, {0, 0x60}, {5, 0x84}, {7, 0xC8}, {10, 0xC0},
};
inline constexpr size_t CC1101_PA_TABLE_433_SIZE = sizeof(CC1101_PA_TABLE_433) / sizeof(CC1101_PA_TABLE_433[0]);

/// Nearest 433 MHz PATABLE entry to `dbm`; writes back the dBm actually selected.
inline uint8_t cc1101_pa_setting_for_dbm(int8_t dbm, int8_t *actual_dbm) {
  size_t best = 0;
  int best_err = INT16_MAX;
  for (size_t i = 0; i < CC1101_PA_TABLE_433_SIZE; i++) {
    const int err = (CC1101_PA_TABLE_433[i].dbm > dbm) ? (CC1101_PA_TABLE_433[i].dbm - dbm)
                                                       : (dbm - CC1101_PA_TABLE_433[i].dbm);
    if (err < best_err) {
      best_err = err;
      best = i;
    }
  }
  if (actual_dbm != nullptr) {
    *actual_dbm = CC1101_PA_TABLE_433[best].dbm;
  }
  return CC1101_PA_TABLE_433[best].pa;
}

/* ================================================================
 * The fixed part of the register profile
 * ================================================================ */

/* Only registers whose value does not depend on a tunable live here. FREQ2/1/0,
 * MDMCFG4 (which carries CHANBW_E/M alongside DRATE_E), DEVIATN, FOCCFG (the
 * carrier-sense gate is a tunable) and TEST2/TEST1 (keyed on the RX bandwidth) are
 * computed in Cc1101Hal::write_config_() and written straight afterwards, so a value
 * appearing in this table is one nothing else can override.
 *
 * The baseline is the SmartRF Studio export for "433 MHz, 2-FSK, 1.2 kbps", which
 * is the configuration TI characterises for exactly this symbol rate; the entries
 * that differ from it are the ones RF433-2 forces, and each says why. */
struct Cc1101RegInit {
  uint8_t addr;
  uint8_t value;
};

static constexpr Cc1101RegInit CC1101_CONFIG[] = {
    // GDO2 is not wired on any board this supports; 0x2E is the high-impedance
    // setting, which keeps it from fighting anything on the module.
    {CC1101_REG_IOCFG2, 0x2E},
    // GDO1 IS the MISO pin. 0x2E (high impedance) is the reset value and the only
    // one that leaves SPI reads working - never program this to a signal source.
    {CC1101_REG_IOCFG1, 0x2E},
    // GDO0 = "asserts when sync word has been sent or received, de-asserts at the
    // end of the packet". In fixed-length TX that falling edge is an exact TX-done
    // marker, which is why gdo0 is worth wiring even though the driver polls
    // MARCSTATE when it is absent.
    {CC1101_REG_IOCFG0, 0x06},
    // FIFO_THR = 7 -> 33 bytes in TX / 32 bytes in RX. transmit() mirrors the same
    // split in software (refill once the TX FIFO has drained to <= 32 bytes), so
    // the threshold and the refill low-water mark cannot drift apart.
    {CC1101_REG_FIFOTHR, 0x07},
    {CC1101_REG_SYNC1, CC1101_SYNC1_ON_AIR},
    {CC1101_REG_SYNC0, CC1101_SYNC0_ON_AIR},
    // Rewritten per transmit(); 0xFF is simply a harmless resting value, since RX
    // runs in infinite-length mode where PKTLEN is not consulted.
    {CC1101_REG_PKTLEN, 0xFF},
    // PQT = 0 (no preamble-quality gate), no address check, no CRC autoflush and -
    // importantly - APPEND_STATUS = 0. Infinite-length mode never ends a packet, so
    // the two status bytes would never be appended anyway, and leaving the bit set
    // would corrupt the stream the moment the mode changed.
    {CC1101_REG_PKTCTRL1, 0x00},
    // Resting state is the RX shape: infinite length, no whitening, no PHY CRC.
    {CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_RX},
    {CC1101_REG_ADDR, 0x00},
    // The carrier is programmed into FREQ2/1/0 directly from set_frequency(), so
    // the channel-number offset must contribute nothing.
    {CC1101_REG_CHANNR, 0x00},
    // FREQ_IF = 6 -> 6 * Fxosc / 2^10 = 152.34 kHz. The IF has to sit above half the
    // channel bandwidth; 152 kHz clears 81.25/2 with room to spare and is what
    // SmartRF picks for any bandwidth up to ~100 kHz.
    {CC1101_REG_FSCTRL1, 0x06},
    // No static frequency offset - the RX offset knob moves the carrier word itself,
    // which keeps the two mechanisms from stacking.
    {CC1101_REG_FSCTRL0, 0x00},
    {CC1101_REG_MDMCFG3, CC1101_DRATE_M},
    // DEM_DCFILT_OFF = 0 (DC blocking on, required for anything but the very
    // highest rates), MOD_FORMAT = 000 = 2-FSK - the spec's reference is un-shaped
    // 2-FSK, not GFSK - MANCHESTER_EN = 0, SYNC_MODE = 010 = 16/16 sync bits.
    // 16-of-16 exact is deliberate: the CMT2300A backend's notes record that a
    // tolerant 2-byte window false-locked on noise ahead of the reply on ~10 % of
    // attempts, which leaves the real frame in the FIFO at the wrong bit phase.
    {CC1101_REG_MDMCFG2, 0x02},
    // FEC_EN = 0, NUM_PREAMBLE = 101 = 12 bytes, CHANSPC_E = 2. The spec puts a real
    // CIU at ~10 preamble bytes; the chip offers 8 or 12 and 12 is the safe side of
    // that. Those 12 bytes cost 80 ms of airtime at 1200 baud, which is why the TX
    // timeout is computed from CC1101_PREAMBLE_BYTES rather than guessed. Channel
    // spacing is inert because CHANNR = 0.
    {CC1101_REG_MDMCFG1, 0x52},
    {CC1101_REG_MDMCFG0, 0xF8},
    // RX_TIME = 111: no RX timeout. The component decides when an exchange has
    // failed; the radio must never drop out of RX on its own.
    {CC1101_REG_MCSM2, 0x07},
    // CCA_MODE = 00 (always transmit). The link is strict half-duplex request/reply
    // and the meter only speaks when spoken to, so clear-channel assessment could
    // only ever block a request we must send. RXOFF_MODE = 11 (stay in RX),
    // TXOFF_MODE = 00 (IDLE after the packet) - which is what makes MARCSTATE
    // returning to IDLE a reliable TX-done indication.
    {CC1101_REG_MCSM1, 0x0C},
    // FS_AUTOCAL = 01: calibrate on every IDLE -> RX/TX transition. transmit() and
    // begin_rx() reprogram FREQ between them, so an automatic recalibration on each
    // turn is exactly what is needed. PO_TIMEOUT = 10 (~150 us) is the datasheet's
    // recommendation for a crystal that is already running.
    {CC1101_REG_MCSM0, 0x18},
    // FOCCFG is deliberately NOT here: FOC_BS_CS_GATE is a YAML tunable, so the
    // register is assembled from CC1101_FOCCFG_BASE in Cc1101Hal::write_config_()
    // alongside MDMCFG4/DEVIATN/TEST2/TEST1.
    // Bit synchronisation: 2Ki/3Kp before sync, halved after, BS_LIMIT = 00 (no data
    // rate offset compensation). The programmed 1199.49 baud is 0.04 % from nominal,
    // far inside what the loop pulls in without help.
    {CC1101_REG_BSCFG, 0x6C},
    // AGC, straight from the 1.2 kbps 2-FSK export: 33 dB magnitude target, maximum
    // LNA and DVGA gain available, LNA priority, medium hysteresis, 16 dB filter.
    // The reply is weak, so nothing here is allowed to cap the gain.
    {CC1101_REG_AGCCTRL2, 0x03},
    {CC1101_REG_AGCCTRL1, 0x40},
    {CC1101_REG_AGCCTRL0, 0x91},
    // Wake-on-radio is unused; RC_PD = 1 powers down the RC oscillator it needs.
    {CC1101_REG_WORCTRL, 0xFB},
    {CC1101_REG_FREND1, 0x56},
    // PA_POWER = 0, so the PA always uses PATABLE[0] - which is the single byte
    // set_output_power_dbm() programmes. 2-FSK is constant envelope, so one entry is
    // all that is ever read.
    {CC1101_REG_FREND0, 0x10},
    // Calibration seeds for the 433 MHz band. FS_AUTOCAL overwrites FSCAL3<3:0>,
    // FSCAL2 and FSCAL1 on the first calibration, so these only have to be sane
    // starting points; FSCAL0 is band-dependent and is kept.
    {CC1101_REG_FSCAL3, 0xE9},
    {CC1101_REG_FSCAL2, 0x2A},
    {CC1101_REG_FSCAL1, 0x00},
    {CC1101_REG_FSCAL0, 0x1F},
    // TEST2/TEST1 are not here either: they are keyed on the RX filter bandwidth,
    // which is a YAML tunable, so solve_params_() picks the characterised pair and
    // write_config_() writes it.
    // VCO_SEL_CAL_EN = 0, the value for carriers below 861 MHz.
    {CC1101_REG_TEST0, 0x09},
};
static constexpr size_t CC1101_CONFIG_SIZE = sizeof(CC1101_CONFIG) / sizeof(CC1101_CONFIG[0]);

}  // namespace esphome::nartis_rf_2_meter
