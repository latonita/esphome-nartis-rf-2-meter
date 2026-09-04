/*
 * CMT2300A HAL - d101-2 / 443.9 MHz.
 */

#include "cmt2300a_hal.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"  // esphome::delay / delayMicroseconds / millis

#include <cinttypes>

namespace esphome::nartis_rf_2_meter {

static const char *const TAG = "cmt2300a_hal";

static constexpr size_t TX_PAD = 6;  // 0x55 pad: the chip drops ~2-3 leading FIFO bytes at TX start

Cmt2300aHal::~Cmt2300aHal() = default;

// ---------------- bit-banged 3-wire SPI (MSB-first) ----------------
void Cmt2300aHal::spi_delay() { esphome::delayMicroseconds(2); }
void Cmt2300aHal::sdio_set_output() { this->sdio_.pin_mode(gpio::FLAG_OUTPUT); }
void Cmt2300aHal::sdio_set_input() { this->sdio_.pin_mode(gpio::FLAG_INPUT); }

uint8_t Cmt2300aHal::reverse8(uint8_t b) {
  b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
  b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
  b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
  return b;
}

void Cmt2300aHal::spi_send_byte(uint8_t b) {
  this->sdio_set_output();
  for (int8_t i = 7; i >= 0; i--) {
    pin_low(this->sclk_);
    this->sdio_.digital_write((b >> i) & 1);
    esphome::delayMicroseconds(2);
    pin_high(this->sclk_);
    esphome::delayMicroseconds(2);
  }
  pin_low(this->sclk_);
}

uint8_t Cmt2300aHal::spi_recv_byte() {
  uint8_t b = 0;
  this->sdio_set_input();
  for (int8_t i = 7; i >= 0; i--) {
    pin_low(this->sclk_);
    esphome::delayMicroseconds(2);
    pin_high(this->sclk_);
    if (pin_read(this->sdio_))
      b |= (1 << i);
    esphome::delayMicroseconds(2);
  }
  pin_low(this->sclk_);
  return b;
}

void Cmt2300aHal::spi_write_reg(uint8_t addr, uint8_t val) {
  pin_low(this->csb_);
  this->spi_delay();
  this->spi_send_byte(addr & 0x7F);
  this->spi_send_byte(val);
  pin_high(this->csb_);
  this->spi_delay();
}

uint8_t Cmt2300aHal::spi_read_reg(uint8_t addr) {
  pin_low(this->csb_);
  this->spi_delay();
  this->spi_send_byte(addr | 0x80);
  uint8_t v = this->spi_recv_byte();
  pin_high(this->csb_);
  this->spi_delay();
  return v;
}

void Cmt2300aHal::update_reg(uint8_t addr, uint8_t mask, uint8_t val) {
  uint8_t r = this->spi_read_reg(addr);
  r = (r & ~mask) | (val & mask);
  this->spi_write_reg(addr, r);
}

void Cmt2300aHal::write_bank(uint8_t start_addr, const uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++)
    this->spi_write_reg(start_addr + i, data[i]);
}

void Cmt2300aHal::write_fifo(const uint8_t *data, size_t len) {
  this->sdio_set_output();
  for (size_t i = 0; i < len; i++) {
    pin_low(this->fcsb_);
    this->spi_delay();
    this->spi_send_byte(data[i]);
    pin_high(this->fcsb_);
    this->spi_delay();
    this->spi_delay();
    this->spi_delay();
    this->spi_delay();
  }
}

void Cmt2300aHal::read_fifo(uint8_t *data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    pin_low(this->fcsb_);
    this->spi_delay();
    data[i] = this->spi_recv_byte();
    pin_high(this->fcsb_);
    this->spi_delay();
    this->spi_delay();
    this->spi_delay();
    this->spi_delay();
  }
}

void Cmt2300aHal::set_pins(esphome::InternalGPIOPin *sdio, esphome::InternalGPIOPin *sclk,
                           esphome::InternalGPIOPin *csb, esphome::InternalGPIOPin *fcsb,
                           esphome::InternalGPIOPin *gpio3) {
  this->sdio_pin_ = sdio;
  this->sclk_pin_ = sclk;
  this->csb_pin_ = csb;
  this->fcsb_pin_ = fcsb;
  this->gpio3_pin_ = gpio3;
  this->sdio_ = sdio->to_isr();
  this->sclk_ = sclk->to_isr();
  this->csb_ = csb->to_isr();
  this->fcsb_ = fcsb->to_isr();
  this->gpio3_ = gpio3->to_isr();
}

uint8_t Cmt2300aHal::get_state() { return this->spi_read_reg(REG_MODE_STA) & MASK_CHIP_MODE_STA; }

bool Cmt2300aHal::wait_for_state(uint8_t target_state, uint32_t timeout_ms) {
  uint32_t start = esphome::millis();
  while (esphome::millis() - start < timeout_ms) {
    if (this->get_state() == target_state)
      return true;
    esphome::delayMicroseconds(STATE_POLL_INTERVAL_US);
  }
  return false;
}

bool Cmt2300aHal::go_standby() {
  this->spi_write_reg(REG_MODE_CTL, GO_STBY);
  return this->wait_for_state(STA_STBY);
}

// ================= BASE INIT (once at startup) =================
bool Cmt2300aHal::init() {
  this->sclk_pin_->setup();
  this->csb_pin_->setup();
  this->fcsb_pin_->setup();
  this->sdio_pin_->setup();
  this->gpio3_pin_->setup();
  this->sclk_.pin_mode(gpio::FLAG_OUTPUT);
  this->csb_.pin_mode(gpio::FLAG_OUTPUT);
  this->fcsb_.pin_mode(gpio::FLAG_OUTPUT);
  this->sdio_.pin_mode(gpio::FLAG_OUTPUT);
  this->gpio3_.pin_mode(gpio::FLAG_INPUT);

  pin_high(this->csb_);
  pin_high(this->fcsb_);
  pin_low(this->sclk_);

  this->spi_write_reg(REG_SOFT_RST, SOFT_RST_VALUE);
  esphome::delay(RESET_DELAY_MS);

  this->spi_write_reg(REG_MODE_CTL, GO_STBY);
  if (!this->wait_for_state(STA_STBY)) {
    ESP_LOGE(TAG, "not in STANDBY after reset");
    return false;
  }

  if (!this->is_chip_connected()) {
    ESP_LOGE(TAG, "CMT2300A not detected - check wiring");
    return false;
  }

  // Register banks (freq + narrow ~4 kHz data-rate = resting/TX profile).
  this->compute_freq_bank_();  // ensure freq_bank_ reflects rf_freq_hz_ (default 443.9)
  this->write_bank(CMT_BANK_ADDR, CMT_BANK, CMT_BANK_SIZE);
  this->write_bank(SYSTEM_BANK_ADDR, SYSTEM_BANK, SYSTEM_BANK_SIZE);
  this->write_bank(FREQUENCY_BANK_ADDR, this->freq_bank_, FREQUENCY_BANK_SIZE);
  this->write_bank(DATA_RATE_BANK_ADDR, DATA_RATE_TX_BANK, DATA_RATE_BANK_SIZE);
  this->write_bank(BASEBAND_BANK_ADDR, BASEBAND_BANK, BASEBAND_BANK_SIZE);
  this->write_bank(TX_BANK_ADDR, TX_BANK, TX_BANK_SIZE);
  this->update_reg(REG_CMT10, 0x07, 0x02);

  // Common post-config patches:
  this->update_reg(REG_SYS2, 0xE0, 0x00);                                 // select TX-half of the freq bank
  this->update_reg(REG_SYS11, 0x1F, FIFO_MERGE_VALUE);                    // 64-B merged FIFO (packet mode)
  this->update_reg(REG_FIFO_CTL, MASK_FIFO_MERGE_EN, MASK_FIFO_MERGE_EN);
  this->update_reg(REG_PKT29, MASK_FIFO_TH, FIFO_TH_VALUE);               // FIFO threshold = 15 bytes
  this->spi_write_reg(REG_INT_EN, 0x39);                                  // PKT_DONE|PREAM_OK|SYNC_OK|TX_DONE
  this->update_reg(REG_IO_SEL, MASK_GPIO3_SEL, GPIO3_SEL_INT2);           // GPIO3 pad = INT2
  this->update_reg(REG_INT2_CTL, MASK_INT_POLAR, 0x00);                   // INT2 active-high

  this->initialized_ = true;
  ESP_LOGI(TAG, "CMT2300A initialized (%.3f MHz)", this->rf_freq_hz_ / 1e6f);
  return true;
}

void Cmt2300aHal::compute_freq_bank_() { freq_bank_from_hz(this->rf_freq_hz_, this->freq_bank_); }

void Cmt2300aHal::set_frequency(uint32_t freq_hz) {
  this->rf_freq_hz_ = freq_hz;
  this->compute_freq_bank_();
}

bool Cmt2300aHal::is_chip_connected() {
  uint8_t id = this->spi_read_reg(0x01);  // CMT2 product ID
  ESP_LOGD(TAG, "Product ID: 0x%02X (expected 0x66)", id);
  return id == 0x66;
}

// ================= TX INIT (before each transmit) =================
// Narrow ~4 kHz profile, un-shift RX-half, blend the HW sync to 0x55 (the real
// 98 f3 rides in the FIFO payload behind the pad), FIFO -> TX-write.
void Cmt2300aHal::init_tx() {
  this->spi_write_reg(REG_MODE_CTL, GO_STBY);
  this->wait_for_state(STA_STBY);
  this->set_rx_center(0);
  this->write_bank(DATA_RATE_BANK_ADDR, DATA_RATE_TX_BANK, DATA_RATE_BANK_SIZE);
  this->spi_write_reg(REG_PKT5, 0x06);  // SYNC_SIZE=4, TOL=0 (blended sync)
  this->spi_write_reg(REG_PKT10 + 0, 0x55);
  this->spi_write_reg(REG_PKT10 + 1, 0x55);
  this->spi_write_reg(REG_PKT10 + 2, 0x55);
  this->spi_write_reg(REG_PKT10 + 3, 0x55);
  this->update_reg(REG_FIFO_CTL, MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL,
                   MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL);
}

// ================= RX INIT (before each receive) =================
// 3-byte sync 55 19 CF (= the meter's last preamble byte plus 98 f3), chip bit-order
// flip, wide ~25 kHz profile centred on the reply, FIFO -> RX on RX_FIFO_TH.
void Cmt2300aHal::init_rx(int off_codes) {
  this->spi_write_reg(REG_MODE_CTL, GO_STBY);
  this->wait_for_state(STA_STBY);
  // Sync taken verbatim from the original display: PKT5=0x04 with 0x44=55 0x43=19
  // 0x42=CF, never rewritten between TX and RX. The bytes are held in the on-air
  // (bit-reversed) domain, so 55 19 CF on air is AA 98 F3 in frame terms - that
  // leading AA is the meter's last preamble byte, and folding it in narrows the
  // match window 256x. SYNC_TOL stays 0: tolerance only widened the window, and the
  // detector was locking onto noise ahead of the reply on ~10% of attempts, which
  // leaves the reply in the FIFO at a wrong bit phase. The FIFO still starts at the
  // LEN byte, so the frame layer is unaffected.
  this->spi_write_reg(REG_PKT5, 0x04);       // SYNC_TOL=0, SYNC_SIZE=2 (3 bytes)
  this->spi_write_reg(REG_PKT13, 0x55);      // SYNC_VALUE<63:56> = 1st byte on air (frame 0xAA)
  this->spi_write_reg(REG_PKT12, 0x19);      // SYNC_VALUE<55:48> = 2nd byte on air (frame 0x98)
  this->spi_write_reg(REG_PKT11, 0xCF);      // SYNC_VALUE<47:40> = 3rd byte on air (frame 0xf3)
  // Fixed RX length 180, matching the original display. PAYLOAD_LENG spans both
  // registers - PKT14<6:4> carries bits 10:8 - so those must be cleared too, or the
  // ceiling reads 0x1B4 = 436. Bit 1 is PAYLOAD_BIT_ORDER and stays set. The ceiling
  // MUST be re-set on every RX: transmit() leaves the packet engine at the exact TX
  // length, and an un-restored ceiling truncates every incoming frame.
  this->spi_write_reg(REG_PKT14, 0x02);      // PAYLOAD_BIT_ORDER=1, PAYLOAD_LENG<10:8>=0
  this->spi_write_reg(REG_PKT15, 0xB4);      // fixed length 180 (was 0x1FF = 511)
  this->update_reg(REG_INT2_CTL, MASK_INT2_SEL, INT_SEL_RX_FIFO_TH);
  this->update_reg(REG_FIFO_CTL, MASK_FIFO_RX_TX_SEL | MASK_SPI_FIFO_RD_WR_SEL, 0x00);  // FIFO -> RX read
  this->spi_write_reg(REG_FIFO_CLR, FIFO_RESTORE | FIFO_CLR_RX | FIFO_CLR_TX);
  this->spi_write_reg(REG_INT_CLR1, 0x0F);
  this->spi_write_reg(REG_INT_CLR2, 0x1F);
  this->write_bank(DATA_RATE_BANK_ADDR, DATA_RATE_RX_BANK, DATA_RATE_BANK_SIZE);  // wide RX profile
  this->set_rx_center(off_codes);
}

// RX-half LO = computed RX code (this channel) + off_codes. RX uses only the
// RX-half (0x18-0x1B); the TX-half (0x1C-0x1F) is left untouched. off_codes is
// small, so it only nudges FREQ_RX_K without disturbing the band bits in 0x1B.
void Cmt2300aHal::set_rx_center(int off_codes) {
  uint32_t base = (uint32_t) this->freq_bank_[1] | ((uint32_t) this->freq_bank_[2] << 8) |
                  ((uint32_t) this->freq_bank_[3] << 16);
  uint32_t code = (uint32_t) ((int32_t) base + off_codes);
  this->spi_write_reg(FREQUENCY_BANK_ADDR + 0, this->freq_bank_[0]);
  this->spi_write_reg(FREQUENCY_BANK_ADDR + 1, code & 0xFF);
  this->spi_write_reg(FREQUENCY_BANK_ADDR + 2, (code >> 8) & 0xFF);
  this->spi_write_reg(FREQUENCY_BANK_ADDR + 3, (code >> 16) & 0xFF);
}

bool Cmt2300aHal::transmit(const uint8_t *frame, size_t len) {
  uint8_t fifo[128];
  if (len + TX_PAD > sizeof(fifo)) {
    ESP_LOGW(TAG, "TX frame too big (%zu)", len);
    return false;
  }
  for (size_t i = 0; i < TX_PAD; i++)
    fifo[i] = 0x55;  // sacrificial pad (blends into preamble)
  for (size_t i = 0; i < len; i++)
    fifo[TX_PAD + i] = reverse8(frame[i]);  // LSB-first on air
  size_t tot = len + TX_PAD;

  this->init_tx();

  // Fixed length (PKT_TYPE=0), clear TX_DONE, reset FIFO.
  this->spi_write_reg(REG_PKT14, (uint8_t) (((tot >> 8) & 0x07) << 4));
  this->spi_write_reg(REG_PKT15, (uint8_t) (tot & 0xFF));
  this->spi_write_reg(REG_INT_CLR1, 0x0C);
  this->spi_write_reg(REG_FIFO_CLR, FIFO_RESTORE | FIFO_CLR_RX | FIFO_CLR_TX);

  // Fill the first FIFO-load, then refill as it drains: a frame can exceed the
  // 64-byte merged FIFO, and writing it all at once would truncate it on air.
  size_t written = (tot > FIFO_SIZE_MERGED) ? FIFO_SIZE_MERGED : tot;
  this->write_fifo(fifo, written);

  this->spi_write_reg(REG_MODE_CTL, GO_TFS);
  if (!this->wait_for_state(STA_TFS)) {
    ESP_LOGW(TAG, "TX: no TFS");
    return false;
  }
  this->spi_write_reg(REG_MODE_CTL, GO_TX);
  if (!this->wait_for_state(STA_TX))
    ESP_LOGW(TAG, "TX: state not confirmed");

  const bool chunked = tot > FIFO_SIZE_MERGED;
  uint32_t t0 = esphome::millis();
  while (esphome::millis() - t0 < 900) {
    if (this->spi_read_reg(REG_INT_CLR1) & CLR1_TX_DONE_FLG) {
      this->go_standby();
      const uint32_t dur = esphome::millis() - t0;
      if (written < tot)
        ESP_LOGW(TAG, "TX_DONE but only %zu/%zu written - frame truncated on air", written, tot);
      // For a chunked (>FIFO) frame, log timing + fill so we can tell a clean send
      // (dur ~ airtime, written==tot) from a FIFO underrun (dur too short).
      if (chunked)
        ESP_LOGVV(TAG, "TX done (chunked): %zu/%zu written, %" PRIu32 " ms", written, tot, dur);
      return true;
    }
    // Refill when the TX FIFO has drained to/below threshold (FIFO_TX_TH == 0).
    if (written < tot && (this->spi_read_reg(REG_FIFO_FLAG) & FIFO_TX_TH) == 0) {
      const size_t rem = tot - written;
      const size_t chunk = (rem > TX_REFILL_CHUNK) ? TX_REFILL_CHUNK : rem;
      this->write_fifo(fifo + written, chunk);
      written += chunk;
    }
    esphome::delayMicroseconds(50);
    esphome::yield();
  }
  ESP_LOGV(TAG, "TX_DONE timeout (%zu/%zu written, MODE_STA=0x%02X FIFO_FLAG=0x%02X)", written, tot,
           this->spi_read_reg(REG_MODE_STA), this->spi_read_reg(REG_FIFO_FLAG));
  this->go_standby();
  return false;
}

bool Cmt2300aHal::begin_rx(int off_codes) {
  this->init_rx(off_codes);
  this->spi_write_reg(REG_MODE_CTL, GO_RFS);
  if (!this->wait_for_state(STA_RFS)) {
    ESP_LOGW(TAG, "RX: no RFS");
    return false;
  }
  this->spi_write_reg(REG_MODE_CTL, GO_RX);
  if (!this->wait_for_state(STA_RX)) {
    ESP_LOGW(TAG, "RX: no RX");
    return false;
  }
  return true;
}

bool Cmt2300aHal::rx_fifo_threshold() { return pin_read(this->gpio3_); }

size_t Cmt2300aHal::drain_rx(uint8_t *buf, size_t buf_size, uint8_t max_chunks) {
  size_t total = 0;
  uint8_t chunks = 0;
  while (chunks < max_chunks && total + FIFO_TH_VALUE <= buf_size && pin_read(this->gpio3_)) {
    this->read_fifo(buf + total, FIFO_TH_VALUE);
    total += FIFO_TH_VALUE;
    chunks++;
  }
  return total;
}

int8_t Cmt2300aHal::get_rssi_dbm() { return (int8_t) ((int) this->spi_read_reg(REG_RSSI_DBM) - 128); }

}  // namespace esphome::nartis_rf_2_meter
