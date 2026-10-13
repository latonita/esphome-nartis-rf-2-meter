/*
 * CMT2300A hardware abstraction layer - bit-bang 3-wire SPI.
 *
 * Asymmetric channel:
 *   - TX: narrow ~4 kHz deviation, HW sync blended to 0x55, the real 98 f3 carried
 *     in the FIFO payload behind a 0x55 pad, bytes bit-reversed (LSB-first on air).
 *   - RX: wide deviation centred on the reply, 2-byte sync 19 CF, chip does the
 *     bit-reversal (PAYLOAD_BIT_ORDER=1), fixed-length capture drained in 15-byte
 *     chunks off the GPIO3/INT2 = RX_FIFO_TH line.
 *
 * Pins: SDIO (bidir data), SCLK, CSB (register CS), FCSB (FIFO CS), GPIO3 (INT2).
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "cmt2300a_defs.h"
#include "radio_hal.h"

#include "esphome/core/gpio.h"

namespace esphome::nartis_rf_2_meter {

class Cmt2300aHal : public RadioHal {
 public:
  const char *radio_name() const override { return "CMT2300A"; }

  /// Set GPIO pins. Must be called before init(). Not part of RadioHal: the
  /// bit-banged 3-wire bus is this chip's own business.
  void set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                esphome::InternalGPIOPin *gpio3);

  /// Where begin_rx() parks the RX-half LO, in frequency codes (1 code ~= 6.199 Hz)
  /// above the channel centre. The default is the value proven on hardware, so this
  /// only needs setting when a board or a meter wants the window nudged.
  void set_rx_center_offset(int off_codes) { this->rx_center_offset_ = off_codes; }

  /// The same knob in Hz, which is the unit the CC1101 backend takes and the unit a
  /// YAML author thinks in. Converted to codes via RX_CODE_HZ, rounded to nearest so
  /// that the proven 758 codes = 4699 Hz round-trips back to exactly 758.
  ///
  /// The codes setter above is kept alongside it rather than replaced: its 758
  /// default is a hardware-proven value, and quietly reinterpreting an existing
  /// `rx_center_offset: 758` as 758 Hz would move the RX window by 4.7 kHz.
  void set_rx_offset_hz(int32_t hz);

  /// RF channel in Hz; recomputes the frequency bank via AN199. Call before init(),
  /// which is what writes it. Defaults to 443.9 MHz.
  void set_frequency(uint32_t freq_hz) override;

  /// Soft reset, write the six register banks and the common post-config patches.
  bool init() override;

  /// Read the product ID (0x66 for CMT2300A) - a wiring sanity check.
  bool is_chip_connected();

  /// Transmit one d101-2 frame (98 f3 .. CRC). Applies the TX profile, pads,
  /// bit-reverses, fills the FIFO and blocks until TX_DONE or ~600 ms. Parks the
  /// chip in standby afterwards, whether the frame went out or not.
  bool transmit(const uint8_t *frame, size_t len) override;

  /// Enter RX centred at rf_freq_hz_ + rx_center_offset_ * 6.199 Hz. init_rx() is
  /// what satisfies the begin_rx() epoch contract: GO_STBY aborts whatever was in
  /// flight, FIFO_CLR flushes the RX FIFO, INT_CLR1/INT_CLR2 clear every latched
  /// flag; this then confirms the RFS and RX transitions before returning true.
  bool begin_rx() override;

  /// Drain full 15-byte chunks while the threshold line is asserted, up to
  /// MAX_DRAIN_CHUNKS of them. Returns bytes appended and sets `status`.
  size_t drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) override;

  /// This backend only ever reads whole threshold-sized chunks of FIFO_TH_VALUE (15)
  /// bytes, never a partial one - but drain_rx() loops over up to MAX_DRAIN_CHUNKS of
  /// them in a single call, so one call can append 75 bytes, not 15. Reporting 15
  /// here understated the bound by a factor of five: nothing could overrun, because
  /// the drain loop stops on the buf_size it is handed, but the component's
  /// "the buffer is full" end-of-frame fallback reasons from this number and was
  /// reasoning from a false one. Draining behaviour is untouched - this backend is
  /// proven on hardware; only the figure it advertises is corrected.
  size_t max_drain_chunk() const override { return (size_t) FIFO_TH_VALUE * MAX_DRAIN_CHUNKS; }

  int8_t get_rssi_dbm() override;

  bool go_standby() override;

  /// Wiring and RX window, for the component's dump_config(). Stored configuration
  /// only - no SPI traffic, because ESPHome dumps a component even after it has been
  /// marked failed, i.e. possibly with no chip on the other end of the bus.
  void dump_config() override;

 private:
  /// Chunks one drain_rx() call may take. The cap bounds how long a single poll can
  /// hold the ESPHome loop while the FIFO keeps refilling.
  static constexpr uint8_t MAX_DRAIN_CHUNKS = 5;

  // --- SPI primitives ---
  void spi_delay();
  void spi_send_byte(uint8_t b);
  uint8_t spi_recv_byte();
  void spi_write_reg(uint8_t addr, uint8_t val);
  uint8_t spi_read_reg(uint8_t addr);
  void update_reg(uint8_t addr, uint8_t mask, uint8_t val);
  void write_bank(uint8_t start_addr, const uint8_t *data, size_t len);
  void write_fifo(const uint8_t *data, size_t len);
  void read_fifo(uint8_t *data, size_t len);

  void sdio_set_output();
  void sdio_set_input();
  static void pin_high(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(true); }
  static void pin_low(esphome::ISRInternalGPIOPin &pin) { pin.digital_write(false); }
  static bool pin_read(esphome::ISRInternalGPIOPin &pin) { return pin.digital_read(); }

  uint8_t get_state();
  bool wait_for_state(uint8_t target_state, uint32_t timeout_ms = STATE_POLL_TIMEOUT_MS);

  // --- Radio config (mirrors the test app's radio_init_base/tx/rx) ---
  void init_tx();               // narrow TX profile, sync blended to 0x55, FIFO -> TX
  void init_rx(int off_codes);  // wide RX profile, sync 19 CF, chip bit-order, FIFO -> RX
  void set_rx_center(int off_codes);  // shift the RX-half LO by off_codes (TX-half untouched)
  void compute_freq_bank_();          // fill freq_bank_ from rf_freq_hz_ (AN199)

  esphome::InternalGPIOPin *sdio_pin_{nullptr};
  esphome::InternalGPIOPin *sclk_pin_{nullptr};
  esphome::InternalGPIOPin *csb_pin_{nullptr};
  esphome::InternalGPIOPin *fcsb_pin_{nullptr};
  esphome::InternalGPIOPin *gpio3_pin_{nullptr};
  esphome::ISRInternalGPIOPin sdio_;
  esphome::ISRInternalGPIOPin sclk_;
  esphome::ISRInternalGPIOPin csb_;
  esphome::ISRInternalGPIOPin fcsb_;
  esphome::ISRInternalGPIOPin gpio3_;

  // Per-channel frequency: rf_freq_hz_ -> freq_bank_ (8 bytes: [RX-LO 4B][TX-LO 4B]).
  uint32_t rf_freq_hz_{443900000};
  uint8_t freq_bank_[FREQUENCY_BANK_SIZE]{};
  int rx_center_offset_{758};
};

}  // namespace esphome::nartis_rf_2_meter
