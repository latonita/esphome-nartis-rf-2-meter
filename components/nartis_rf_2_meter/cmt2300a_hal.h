/*
 * CMT2300A Hardware Abstraction Layer - d101-2 / 443.9 MHz.
 *
 * Bit-bang 3-wire SPI driver for the CMT2300A. Fixed 443.9 MHz. Asymmetric channel:
 *   - TX: narrow ~4 kHz deviation, HW sync blended to 0x55, real 98 f3 carried in
 *     the FIFO payload behind a 0x55 pad, bytes bit-reversed (LSB-first on air).
 *   - RX: wide ~28 kHz deviation centred on the reply, 2-byte sync 19 CF, chip
 *     does the bit-reversal (PAYLOAD_BIT_ORDER=1), fixed-length capture, drained
 *     in 15-byte chunks off the GPIO3/INT2 = RX_FIFO_TH line.
 *
 * Pins: SDIO (bidir data), SCLK, CSB (register CS), FCSB (FIFO CS), GPIO3 (INT2).
 */

#pragma once

#include <cstdint>
#include <cstddef>
#include "cmt2300a_defs.h"

#include "esphome/core/gpio.h"

namespace esphome::nartis_rf_2_meter {

class Cmt2300aHal {
 public:
  Cmt2300aHal() = default;
  ~Cmt2300aHal();

  /// Set GPIO pins. Must be called before init().
  void set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                esphome::InternalGPIOPin *gpio3);

  /// Set the RF channel frequency (Hz) and (re)compute the 8-byte frequency bank
  /// via the AN199 formula. Call before init() (init() writes the computed bank).
  /// Defaults to 443.9 MHz if never called.
  void set_frequency(uint32_t freq_hz);

  /// Base init: soft reset, write the 6 register banks (computed frequency +
  /// narrow-TX resting profile) and the common post-config patches. Returns true
  /// on success.
  bool init();

  /// Read product ID (0x66 for CMT2300A) - wiring/comms sanity check.
  bool is_chip_connected();

  /// Transmit one d101-2 frame (starts with 98 f3, ends with its CRC). Applies the
  /// TX profile, prepends a 0x55 pad, bit-reverses to LSB-first on air, fills the
  /// FIFO, and blocks until TX_DONE or ~600 ms. Returns true on TX_DONE. Parks the
  /// chip in standby afterwards.
  bool transmit(const uint8_t *frame, size_t len);

  /// Enter RX centred at (rf_freq_hz_ + off_codes * 6.199 Hz): applies the wide RX
  /// profile + 2-byte sync + chip bit-order, then RFS -> RX. Returns true if RX
  /// was entered.
  bool begin_rx(int off_codes);

  /// True while >= FIFO threshold (15) unread bytes sit in the RX FIFO
  /// (GPIO3 = INT2 = RX_FIFO_TH, active-high).
  bool rx_fifo_threshold();

  /// Drain full 15-byte chunks from the RX FIFO while the threshold line is
  /// asserted, up to buf_size / max_chunks. Returns bytes appended.
  size_t drain_rx(uint8_t *buf, size_t buf_size, uint8_t max_chunks = 5);

  /// Read RSSI in dBm (signed).
  int8_t get_rssi_dbm();

  bool go_standby();

  /// LSB<->MSB bit reversal of one byte (LSB-first on-air conversion).
  static uint8_t reverse8(uint8_t b);

 private:
  // --- SPI primitives ---
  void spi_delay();
  void spi_send_byte(uint8_t b);
  uint8_t spi_recv_byte();
  void spi_write_reg(uint8_t addr, uint8_t val);
  uint8_t spi_read_reg(uint8_t addr);
  void write_reg(uint8_t addr, uint8_t val) { this->spi_write_reg(addr, val); }
  uint8_t read_reg(uint8_t addr) { return this->spi_read_reg(addr); }
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

  bool initialized_{false};

  // Per-channel frequency: rf_freq_hz_ -> freq_bank_ (8 bytes: [RX-LO 4B][TX-LO 4B]).
  uint32_t rf_freq_hz_{443900000};
  uint8_t freq_bank_[FREQUENCY_BANK_SIZE]{};
};

}  // namespace esphome::nartis_rf_2_meter
