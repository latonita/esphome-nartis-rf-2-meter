/*
 * TI CC1101 hardware abstraction layer - bit-bang 4-wire SPI.
 *
 * The second RadioHal backend for the RF433-2 (d101-2) link. Everything above the
 * radio is unchanged; this file only has to put the PHY of RF_PHY_SPEC_RF433-2.md
 * on the air: 2-FSK, 1200 bps NRZ, +-25 kHz deviation in both directions, LSB-first,
 * 0x55 preamble, sync 98 F3, no whitening / Manchester / FEC / PHY CRC.
 *
 * Differences from the CMT2300A backend that are worth knowing up front:
 *
 *   - The CC1101 has a proper preamble + sync engine, so none of the CMT's TX
 *     workarounds are needed: no 0x55 FIFO pad, and the 98 F3 sync is programmed
 *     into SYNC1/SYNC0 rather than carried at the head of the payload. transmit()
 *     therefore strips the leading 98 F3 from the logical frame it is handed.
 *   - The CC1101 shifts MSB-first and has no bit-order control, so every payload
 *     byte is reverse8()-ed on the way to the FIFO and back on the way out, and the
 *     sync registers hold 19 CF rather than 98 F3.
 *   - RX runs in infinite-length mode. The on-air LEN byte is bit-reversed, so the
 *     chip's own variable-length engine would read it wrong; the component bounds
 *     the frame by the LEN byte instead, exactly as the drain_rx() contract in
 *     radio_hal.h describes.
 *
 * Pins: SCLK, MISO (SO), MOSI (SI), CS (CSn) and an optional GDO0. The bus is
 * bit-banged rather than going through ESPHome's `spi` component, for the same
 * reason the CMT2300A backend does: the radio backends stay self-contained, so a
 * YAML that uses one of them pulls in nothing from the other and no shared bus
 * arbitration can interleave with the half-duplex request/reply timing.
 *
 * ---------------------------------------------------------------------------
 * RISK 1 - link budget. The CC1101 tops out at about +10 dBm at 433 MHz, while a
 * real Nartis CIU transmits +20 dBm. That is a ~10 dB deficit on the uplink: the
 * meter may simply not hear a request that the CIU gets answered. Nothing in this
 * driver can close that gap; it needs an external PA or a boosted module. See the
 * comment at set_output_power_dbm().
 *
 * RISK 2 - demodulating h~41. The documented failure mode of the CMT2300A backend
 * is that its counting clock-data-recovery slips bits on this waveform even at good
 * SNR, where an SDR decodes every frame cleanly (see the repo's RF RX diagnosis
 * notes). The CC1101's demodulator is a different design - a frequency
 * discriminator with a PLL-style bit synchroniser - and may well do better on the
 * same signal. If it does not, the documented escape hatch is the CC1101's
 * ASYNCHRONOUS serial mode (PKTCTRL0.PKT_FORMAT = 11), and only that one: it routes
 * the raw output of the frequency discriminator to GDO0 with no data decision at
 * all, which is what lets clock recovery be redone in software against the full bit
 * stream. Synchronous serial mode (PKT_FORMAT = 01) is NOT an alternative here - it
 * still clocks data out on the CC1101's own recovered symbol clock, so it carries
 * the chip's bit synchroniser with it and escapes nothing. The price of asynchronous
 * mode is that the MCU must oversample GDO0 itself, several times per 833 us bit.
 * That is a FUTURE OPTION and is deliberately NOT implemented here.
 * ---------------------------------------------------------------------------
 */

#pragma once

#include <cstddef>
#include <cstdint>

#include "cc1101_defs.h"
#include "radio_hal.h"

#include "esphome/core/gpio.h"

namespace esphome::nartis_rf_2_meter {

class Cc1101Hal : public RadioHal {
 public:
  const char *radio_name() const override { return "CC1101"; }

  /// Set GPIO pins. Must be called before init(). Not part of RadioHal: the
  /// bit-banged 4-wire bus is this chip's own business. `gdo0` is optional and may
  /// be nullptr - it only buys a faster, exact TX-done edge; without it the driver
  /// polls MARCSTATE and TXBYTES instead and behaves identically.
  void set_pins(esphome::InternalGPIOPin *sclk, esphome::InternalGPIOPin *miso, esphome::InternalGPIOPin *mosi,
                esphome::InternalGPIOPin *cs, esphome::InternalGPIOPin *gdo0);

  /// Where begin_rx() centres the receiver, in Hz relative to the TX carrier.
  /// Defaults to 0: the meter's measured +1.1 kHz offset is inside the chip's
  /// frequency offset compensation range, so there is normally nothing to correct.
  void set_rx_offset_hz(int32_t hz) { this->rx_offset_hz_ = hz; }

  /// Requested PA output power in dBm, snapped to the nearest 433 MHz PATABLE entry
  /// (-30, -20, -15, -10, 0, +5, +7, +10). Defaults to +10, the chip's maximum.
  ///
  /// RISK: a real Nartis CIU transmits +20 dBm. Even at +10 the uplink is ~10 dB
  /// short of the reference link budget, which is a standing candidate explanation
  /// for a meter that never answers. An external PA or a boosted module is the only
  /// fix; lowering this value makes it strictly worse.
  void set_output_power_dbm(int8_t dbm) { this->output_power_dbm_ = dbm; }

  /// RX channel-filter bandwidth in Hz, snapped to the nearest (CHANBW_E, CHANBW_M).
  /// Defaults to 81250 Hz. The reply needs ~52 kHz of Carson bandwidth plus crystal
  /// offset; the chip's narrowest setting is 58 kHz and its widest is 812 kHz.
  void set_rx_bandwidth_hz(uint32_t hz) { this->rx_bandwidth_hz_ = hz; }

  /// FSK deviation in Hz, snapped to the nearest (DEV_E, DEV_M). Defaults to 25000,
  /// which the chip rounds to +-25.39 kHz. Used for TX and RX alike - the link is
  /// symmetric, contrary to the superseded "+-4 kHz TX" note in the spec.
  void set_deviation_hz(uint32_t hz) { this->deviation_hz_ = hz; }

  /// FOCCFG.FOC_BS_CS_GATE - a HARDWARE BRING-UP KNOB, not a setting with a known
  /// right answer. It decides whether the frequency-offset and bit-synchronisation
  /// loops are allowed to run before carrier sense asserts:
  ///
  ///   false (default) -> FOCCFG = 0x16, ungated. Both loops track from the instant
  ///     RX is entered. Nothing of a weak preamble is wasted waiting for CS, which is
  ///     why it is the default - but the loops also spend the whole reply wait
  ///     tracking receiver noise, and can walk out to their limits (+-BW/4 for FOC)
  ///     before the real preamble ever arrives.
  ///   true -> FOCCFG = 0x36, gated. Both loops are frozen at their initial values
  ///     until carrier sense asserts, so they start from zero offset on the real
  ///     signal - at the cost of the preamble bytes burned while CS makes up its
  ///     mind, which on a reply that is already marginal may be the bytes that were
  ///     needed for lock.
  ///
  /// Which one wins on this h~41 waveform is not answerable from the datasheet, and
  /// the RX diagnosis notes make a drifting offset loop a live suspect, so it is
  /// exposed rather than compiled in. Everything else in FOCCFG is identical either
  /// way; see CC1101_FOCCFG_BASE.
  void set_foc_cs_gate(bool enabled) { this->foc_cs_gate_ = enabled; }

  /// Channel centre in Hz. Call before init(), which is what programs the
  /// synthesiser. Defaults to 443.9 MHz (channel k=12).
  void set_frequency(uint32_t freq_hz) override;

  /// Power-up reset, probe PARTNUM/VERSION, then write the whole register profile
  /// and PATABLE. False means the chip is unusable; the reason is logged.
  bool init() override;

  /// Transmit one logical d101-2 frame (98 f3 .. CRC). Strips the 98 f3 - the packet
  /// engine emits preamble and sync itself - bit-reverses the rest, and blocks in
  /// fixed-length mode until the packet is out or the frame's own airtime budget
  /// elapses (see tx_done_timeout_ms_()), refilling the FIFO for frames longer than
  /// its 64 bytes. Returns true only with positive evidence that the chip really
  /// entered TX. Parks the chip in IDLE on every exit path.
  bool transmit(const uint8_t *frame, size_t len) override;

  /// Start a verified receive epoch: SIDLE aborts whatever was in flight (and clears
  /// a wedged RXFIFO_OVERFLOW, which is where this chip latches that fault), then the
  /// RX carrier (channel + rx_offset_hz) and infinite-length mode are programmed, the
  /// RX FIFO is flushed and SRX is strobed and confirmed in MARCSTATE.
  bool begin_rx() override;

  /// Drain whatever the RX FIFO holds, bit-reversed back into the logical domain.
  /// Never takes the final byte while the chip is still receiving - a documented
  /// CC1101 FIFO hazard. Returns bytes appended and sets `status`; this backend can
  /// report an overflow and a chip that has left RX, both of which end the epoch.
  size_t drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) override;

  /// The most one drain_rx() call can append. RXBYTES counts the 64 FIFO bytes plus
  /// the one byte held in the receive chain's prefetch register, so its largest legal
  /// value is 65 (see CC1101_RXBYTES_MAX_VALID); a drain made while the chip is still
  /// receiving leaves one byte behind, which puts the peak at exactly 64. drain_rx()
  /// clamps to this same constant on the one path that could otherwise exceed it - a
  /// 65-byte count read after the chip has already left RX - so the number here is
  /// the true bound and not an estimate.
  size_t max_drain_chunk() const override { return CC1101_FIFO_SIZE; }

  int8_t get_rssi_dbm() override;

  bool go_standby() override;

  /// Wiring and the PHY parameters actually programmed, for the component's
  /// dump_config(). Printed from solved member state rather than read back off the
  /// chip: ESPHome dumps a component even after mark_failed(), so this may run with
  /// nothing answering on the bus.
  void dump_config() override;

 private:
  /// Half a bit-bang SPI clock period. 2 us gives ~250 kHz, matching the CMT2300A
  /// backend; the CC1101 tolerates up to 6.5 MHz, so this is pure margin.
  static constexpr uint32_t SPI_HALF_PERIOD_US = 2;
  /// How long to wait for CHIP_RDYn (MISO low) after CS goes low.
  static constexpr uint32_t CHIP_RDY_TIMEOUT_US = 10000;
  /// How long the chip may take to come back from SRES.
  static constexpr uint32_t RESET_TIMEOUT_US = 50000;
  /// Status registers can change mid-read; this caps the re-read loop.
  static constexpr uint8_t STATUS_READ_RETRIES = 6;
  /// Largest frame transmit() will accept, bounded by PKTLEN being 8 bits.
  static constexpr size_t TX_FRAME_MAX = 255;
  /// Refill the TX FIFO once it has drained to this many bytes. Mirrors the
  /// FIFOTHR = 7 split (33 bytes in TX) programmed in CC1101_CONFIG.
  static constexpr uint8_t TX_REFILL_LOW_WATER = 32;
  /// Time allowed for the chip to report TX entry. Entry costs a calibration
  /// (~720 us) plus settling, and the shortest frame this PHY can send is still 14
  /// byte-times of preamble and sync, i.e. ~93 ms, so 20 ms is far too short for a
  /// packet to have come and gone inside it: failing to observe TX here really does
  /// mean TX was not entered.
  static constexpr uint32_t TX_ENTER_TIMEOUT_MS = 20;
  /// Slack added to a frame's computed airtime to get its TX-done deadline. It has
  /// to cover the IDLE->CALIBRATE->SETTLING->TX walk, every bit-banged FIFO refill,
  /// and whatever else ESPHome's loop is doing; 300 ms is more than an order of
  /// magnitude over the measured cost of all three and still fails fast.
  static constexpr uint32_t TX_DONE_MARGIN_MS = 300;
  /// Time allowed for a state-machine transition that is not on the air path.
  static constexpr uint32_t STATE_TIMEOUT_MS = 20;
  static constexpr uint32_t STATE_POLL_INTERVAL_US = 100;

  /// How long a frame of `payload_bytes` may take, preamble and sync included, at the
  /// programmed symbol rate, plus TX_DONE_MARGIN_MS. A fixed constant cannot work
  /// here: TX_FRAME_MAX is 255 bytes, which with 12 preamble and 2 sync bytes is
  /// 269 * 8 / 1199.484 = 1.79 s of airtime, so any one-size deadline short enough to
  /// be useful for a 20-byte request would guarantee a timeout on a long one -
  /// transmit() would be rejecting lengths it had just accepted.
  static uint32_t tx_done_timeout_ms_(size_t payload_bytes);

  // --- bit-banged 4-wire SPI (mode 0, MSB-first) ---
  uint8_t xfer_byte_(uint8_t out);
  bool wait_miso_low_(uint32_t timeout_us);
  /// Drive CS low and wait for CHIP_RDYn. Reports a readiness failure by raising
  /// spi_not_ready_, not by a return value - see the note on that member.
  void select_();
  void deselect_();
  uint8_t strobe_(uint8_t cmd);
  void write_reg_(uint8_t addr, uint8_t value);
  void write_burst_(uint8_t addr, const uint8_t *data, size_t len);
  uint8_t read_reg_(uint8_t addr);
  uint8_t read_burst_reg_once_(uint8_t addr);
  uint8_t read_status_reg_(uint8_t addr);
  void read_fifo_burst_(uint8_t *data, size_t len);

  static void pin_high(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(true); }
  static void pin_low(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(false); }
  static bool pin_read(esphome::ISRInternalGPIOPin &pin) { return pin.digital_read(); }

  /// Sticky "the chip was never ready" flag, raised by select_() whenever CHIP_RDYn
  /// (MISO low) did not appear inside CHIP_RDY_TIMEOUT_US. TI requires SO to be low
  /// before the header byte is clocked, because that is the chip saying its SPI
  /// interface and crystal are up; clocking a transaction anyway can return arbitrary
  /// register values, execute a strobe that was never intended, or drive a state
  /// transition that never happened. None of that is detectable downstream, so it
  /// must not be thrown away.
  ///
  /// It is a sticky flag rather than a checked return on every accessor on purpose:
  /// write_reg_() is called forty-odd times from write_config_() alone, and
  /// threading a status out of each one would bury the register profile in error
  /// handling for a fault that is all-or-nothing in practice - if the chip is not
  /// answering CHIP_RDYn, it is not answering it for any of them. Callers that care
  /// clear it before a sequence and test it after.
  bool spi_not_ready_{false};
  void clear_spi_error_() { this->spi_not_ready_ = false; }
  bool spi_error_() const { return this->spi_not_ready_; }

  uint8_t marcstate_();
  bool wait_marcstate_(uint8_t target, uint32_t timeout_ms);
  bool go_idle_();

  // --- configuration ---
  void solve_params_();
  /// FREQ = f * 2^16 / Fxosc and its inverse. Static and shared so program_frequency_()
  /// and dump_config() can never disagree about what was programmed.
  static uint32_t freq_word_for_(uint32_t hz);
  static uint32_t freq_hz_for_word_(uint32_t word);
  void program_frequency_(uint32_t hz);
  void write_config_();

  esphome::InternalGPIOPin *sclk_pin_{nullptr};
  esphome::InternalGPIOPin *miso_pin_{nullptr};
  esphome::InternalGPIOPin *mosi_pin_{nullptr};
  esphome::InternalGPIOPin *cs_pin_{nullptr};
  esphome::InternalGPIOPin *gdo0_pin_{nullptr};
  esphome::ISRInternalGPIOPin sclk_;
  esphome::ISRInternalGPIOPin miso_;
  esphome::ISRInternalGPIOPin mosi_;
  esphome::ISRInternalGPIOPin cs_;
  esphome::ISRInternalGPIOPin gdo0_;

  // --- requested parameters (YAML) ---
  uint32_t freq_hz_{CC1101_DEFAULT_FREQUENCY_HZ};
  int32_t rx_offset_hz_{CC1101_DEFAULT_RX_OFFSET_HZ};
  int8_t output_power_dbm_{CC1101_DEFAULT_OUTPUT_POWER_DBM};
  uint32_t rx_bandwidth_hz_{CC1101_DEFAULT_RX_BANDWIDTH_HZ};
  uint32_t deviation_hz_{CC1101_DEFAULT_DEVIATION_HZ};
  bool foc_cs_gate_{false};

  // --- solved register fields and the values they actually produce, so
  //     dump_config() can print what was programmed rather than what was asked for ---
  uint8_t chanbw_e_{0};
  uint8_t chanbw_m_{0};
  uint32_t rx_bandwidth_actual_hz_{0};
  uint8_t dev_e_{0};
  uint8_t dev_m_{0};
  uint32_t deviation_actual_hz_{0};
  uint8_t pa_value_{0};
  int8_t output_power_actual_dbm_{0};
  /// The characterised TEST2/TEST1 pair for rx_bandwidth_actual_hz_. Solved rather
  /// than tabled so dump_config() can print the bytes that were actually written.
  uint8_t test2_{CC1101_TEST2_NARROW_BW};
  uint8_t test1_{CC1101_TEST1_NARROW_BW};

  /// FOCCFG as the gate knob resolves it. Derived on demand rather than cached in a
  /// member: dump_config() runs even when init() never did, and a cached copy would
  /// then print the default while the setter said otherwise.
  uint8_t foccfg_value_() const {
    return (uint8_t) (CC1101_FOCCFG_BASE | (this->foc_cs_gate_ ? CC1101_FOCCFG_BS_CS_GATE : 0u));
  }
};

}  // namespace esphome::nartis_rf_2_meter
