/*
 * CC1101 HAL - RF433-2 / d101-2, 435-452 MHz.
 */

#include "cc1101_hal.h"

#include "esphome/core/log.h"
#include "esphome/core/hal.h"  // esphome::delay / delayMicroseconds / micros / millis / yield

#include <cinttypes>

namespace esphome::nartis_rf_2_meter {

static const char *const TAG = "cc1101_hal";

// ---------------- bit-banged 4-wire SPI (mode 0, MSB-first) ----------------
//
// The CC1101 clocks MOSI in on the rising edge of SCLK and drives MISO on the
// falling edge, i.e. CPOL = 0, CPHA = 0, so one bit is: SCLK low, present the
// output bit, SCLK high, sample the input bit. ESPHome's `spi` component is
// deliberately not used - the CMT2300A backend is self-contained for the same
// reason, and keeping both that way means a YAML that selects one radio pulls in
// nothing belonging to the other, and no shared-bus arbitration can land in the
// middle of a half-duplex request/reply turn.

uint8_t Cc1101Hal::xfer_byte_(uint8_t out) {
  uint8_t in = 0;
  for (int8_t i = 7; i >= 0; i--) {
    pin_low(this->sclk_);
    this->mosi_.digital_write(((out >> i) & 1) != 0);
    esphome::delayMicroseconds(SPI_HALF_PERIOD_US);
    pin_high(this->sclk_);
    in = (uint8_t) (in << 1);
    if (pin_read(this->miso_)) {
      in |= 1;
    }
    esphome::delayMicroseconds(SPI_HALF_PERIOD_US);
  }
  pin_low(this->sclk_);
  return in;
}

// CHIP_RDYn. The chip holds MISO high while the crystal is still starting up or a
// reset is in progress; the datasheet requires waiting for it to fall before the
// first bit of any access, not just the first access after power-up.
bool Cc1101Hal::wait_miso_low_(uint32_t timeout_us) {
  const uint32_t t0 = esphome::micros();
  while (pin_read(this->miso_)) {
    if (esphome::micros() - t0 > timeout_us) {
      return false;
    }
  }
  return true;
}

// Every access starts here, so this is the one place that can observe CHIP_RDYn
// failing - and TI requires SO to be low before the header byte is clocked, because
// that low is the chip saying its SPI interface and crystal are up. Clocking the
// transaction anyway, which is what this code used to do once the 10 ms wait expired,
// can return arbitrary register values, execute a strobe that was never intended, or
// report a state transition that never happened.
//
// The failure is latched in spi_not_ready_ rather than returned. A bool here would be
// ignored at every one of the call sites below - threading a status out of
// write_reg_() would bury write_config_()'s forty-odd register writes in error
// handling for a fault that is all-or-nothing in practice - and an ignored return is
// exactly how this failure went missing in the first place. With no return value
// there is nothing to forget to check; init() and drain_rx() clear the flag before a
// sequence and test it afterwards.
void Cc1101Hal::select_() {
  pin_low(this->cs_);
  if (!this->wait_miso_low_(CHIP_RDY_TIMEOUT_US)) {
    this->spi_not_ready_ = true;
  }
}

void Cc1101Hal::deselect_() {
  pin_high(this->cs_);
  esphome::delayMicroseconds(SPI_HALF_PERIOD_US);
}

// A strobe is a bare header byte; what comes back on MISO during it is the chip
// status byte (CHIP_RDYn, state, FIFO bytes available), which is the cheapest
// state read there is.
uint8_t Cc1101Hal::strobe_(uint8_t cmd) {
  this->select_();
  const uint8_t status = this->xfer_byte_(cmd);
  this->deselect_();
  return status;
}

void Cc1101Hal::write_reg_(uint8_t addr, uint8_t value) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_WRITE));
  this->xfer_byte_(value);
  this->deselect_();
}

void Cc1101Hal::write_burst_(uint8_t addr, const uint8_t *data, size_t len) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_WRITE | CC1101_HDR_BURST));
  for (size_t i = 0; i < len; i++) {
    this->xfer_byte_(data[i]);
  }
  this->deselect_();
}

uint8_t Cc1101Hal::read_reg_(uint8_t addr) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_READ));
  const uint8_t value = this->xfer_byte_(0x00);
  this->deselect_();
  return value;
}

// Status registers live at 0x30-0x3D, which is exactly the command-strobe range.
// Without the burst bit the chip would execute a strobe instead of returning the
// register, so a read of anything in that range MUST set it.
uint8_t Cc1101Hal::read_burst_reg_once_(uint8_t addr) {
  this->select_();
  this->xfer_byte_((uint8_t) (addr | CC1101_HDR_READ | CC1101_HDR_BURST));
  const uint8_t value = this->xfer_byte_(0x00);
  this->deselect_();
  return value;
}

// CC1101 errata, "Register read with SPI": a status register that updates while it
// is being shifted out can return a byte that was never a valid value - the classic
// symptom is a bogus RXBYTES count that makes the driver read past the end of the
// FIFO. The documented workaround is to read until two consecutive reads agree.
uint8_t Cc1101Hal::read_status_reg_(uint8_t addr) {
  uint8_t previous = this->read_burst_reg_once_(addr);
  for (uint8_t i = 0; i < STATUS_READ_RETRIES; i++) {
    const uint8_t current = this->read_burst_reg_once_(addr);
    if (current == previous) {
      return current;
    }
    previous = current;
  }
  // Six disagreements in a row means something is genuinely churning (a FIFO filling
  // fast, or a wiring problem); the last value is still the freshest one we have.
  ESP_LOGV(TAG, "status register 0x%02X never settled, using 0x%02X", addr, previous);
  return previous;
}

void Cc1101Hal::read_fifo_burst_(uint8_t *data, size_t len) {
  this->select_();
  this->xfer_byte_((uint8_t) (CC1101_REG_FIFO | CC1101_HDR_READ | CC1101_HDR_BURST));
  for (size_t i = 0; i < len; i++) {
    data[i] = this->xfer_byte_(0x00);
  }
  this->deselect_();
}

// ---------------- state machine ----------------

uint8_t Cc1101Hal::marcstate_() { return this->read_status_reg_(CC1101_REG_MARCSTATE) & CC1101_MARCSTATE_MASK; }

bool Cc1101Hal::wait_marcstate_(uint8_t target, uint32_t timeout_ms) {
  const uint32_t start = esphome::millis();
  while (esphome::millis() - start < timeout_ms) {
    if (this->marcstate_() == target) {
      return true;
    }
    esphome::delayMicroseconds(STATE_POLL_INTERVAL_US);
  }
  return false;
}

bool Cc1101Hal::go_idle_() {
  this->strobe_(CC1101_STROBE_SIDLE);
  return this->wait_marcstate_(CC1101_MARC_IDLE, STATE_TIMEOUT_MS);
}

// IDLE is the lowest state that keeps the whole register profile alive. SPWD is
// lower still, but it stops the crystal, needs a CS-toggle wake-up dance and a full
// recalibration on the way out - all of which buys nothing in a bridge that wakes
// again a few seconds later.
bool Cc1101Hal::go_standby() { return this->go_idle_(); }

// ---------------- pins ----------------

void Cc1101Hal::set_pins(esphome::InternalGPIOPin *sclk, esphome::InternalGPIOPin *miso,
                         esphome::InternalGPIOPin *mosi, esphome::InternalGPIOPin *cs,
                         esphome::InternalGPIOPin *gdo0) {
  this->sclk_pin_ = sclk;
  this->miso_pin_ = miso;
  this->mosi_pin_ = mosi;
  this->cs_pin_ = cs;
  this->gdo0_pin_ = gdo0;
  this->sclk_ = sclk->to_isr();
  this->miso_ = miso->to_isr();
  this->mosi_ = mosi->to_isr();
  this->cs_ = cs->to_isr();
  // gdo0 is optional: every path that touches gdo0_ is guarded on gdo0_pin_.
  if (gdo0 != nullptr) {
    this->gdo0_ = gdo0->to_isr();
  }
}

// ---------------- parameter solving ----------------

void Cc1101Hal::set_frequency(uint32_t freq_hz) { this->freq_hz_ = freq_hz; }

// FREQ = f_carrier * 2^16 / Fxosc, one step being Fxosc / 2^16 = 396.728 Hz. This is
// computed rather than tabled: the channel comes from the meter's serial number and
// the RX offset is a free parameter, so there is no fixed set of carriers to table.
uint32_t Cc1101Hal::freq_word_for_(uint32_t hz) {
  return (uint32_t) ((((uint64_t) hz << 16) + (CC1101_XTAL_HZ / 2)) / CC1101_XTAL_HZ);
}

uint32_t Cc1101Hal::freq_hz_for_word_(uint32_t word) {
  return (uint32_t) ((((uint64_t) word * CC1101_XTAL_HZ) + (1u << 15)) >> 16);
}

void Cc1101Hal::program_frequency_(uint32_t hz) {
  const uint32_t word = freq_word_for_(hz);
  // FREQ2<7:6> are always zero; the synthesiser word is 22 bits wide.
  this->write_reg_(CC1101_REG_FREQ2, (uint8_t) ((word >> 16) & 0x3F));
  this->write_reg_(CC1101_REG_FREQ1, (uint8_t) ((word >> 8) & 0xFF));
  this->write_reg_(CC1101_REG_FREQ0, (uint8_t) (word & 0xFF));
}

void Cc1101Hal::solve_params_() {
  // Deviation: f_dev = (Fxosc / 2^17) * (8 + DEV_M) * 2^DEV_E, with DEVIATION_E and
  // DEVIATION_M both 3 bits. Searching all 64 combinations costs nothing at boot and
  // leaves no room for an arithmetic slip. At the 25 kHz default it lands on
  // DEV_E = 4, DEV_M = 0 -> 25.39 kHz; the neighbouring DEV_E = 3, DEV_M = 7 gives
  // 23.80 kHz, which is further from both the request and the +-24.73 kHz the SDR
  // capture measured off a real CIU.
  uint32_t best_err = UINT32_MAX;
  for (uint8_t e = 0; e < 8; e++) {
    for (uint8_t m = 0; m < 8; m++) {
      const uint32_t f = (uint32_t) ((((uint64_t) CC1101_XTAL_HZ * (8u + m)) << e) >> 17);
      const uint32_t err = (f > this->deviation_hz_) ? (f - this->deviation_hz_) : (this->deviation_hz_ - f);
      if (err < best_err) {
        best_err = err;
        this->dev_e_ = e;
        this->dev_m_ = m;
        this->deviation_actual_hz_ = f;
      }
    }
  }

  // RX channel filter: BW = Fxosc / (8 * (4 + CHANBW_M) * 2^CHANBW_E), both fields
  // 2 bits, so 16 settings spanning 58.04 kHz (E=3, M=3 - the chip's floor) to
  // 812.5 kHz. The 81.25 kHz default lands on E = 3, M = 1: Carson bandwidth for the
  // reply is 2 * (25.0 + 1.2) ~= 52 kHz and the meter's carrier sits ~1.1 kHz off
  // ours, so the floor would leave essentially no margin.
  best_err = UINT32_MAX;
  for (uint8_t e = 0; e < 4; e++) {
    for (uint8_t m = 0; m < 4; m++) {
      const uint32_t bw = CC1101_XTAL_HZ / ((8u * (4u + m)) << e);
      const uint32_t err =
          (bw > this->rx_bandwidth_hz_) ? (bw - this->rx_bandwidth_hz_) : (this->rx_bandwidth_hz_ - bw);
      if (err < best_err) {
        best_err = err;
        this->chanbw_e_ = e;
        this->chanbw_m_ = m;
        this->rx_bandwidth_actual_hz_ = bw;
      }
    }
  }
  // TEST2/TEST1 are keyed on the RX filter bandwidth, and TI characterises exactly
  // two pairs: 0x88/0x31 strictly above 101 kHz and 0x81/0x35 at or below it. The
  // bandwidth is a YAML tunable spanning the chip's full 58...812 kHz range, so a
  // fixed pair would leave most of that range running registers specified for a
  // different filter - which is not a warning-level problem, it is simply the wrong
  // value, and the driver knows which one is right the moment the bandwidth is
  // solved. Picking it here rather than warning about it is the whole fix.
  if (this->rx_bandwidth_actual_hz_ > CC1101_TEST_REG_BW_THRESHOLD_HZ) {
    this->test2_ = CC1101_TEST2_WIDE_BW;
    this->test1_ = CC1101_TEST1_WIDE_BW;
  } else {
    this->test2_ = CC1101_TEST2_NARROW_BW;
    this->test1_ = CC1101_TEST1_NARROW_BW;
  }


  this->pa_value_ = cc1101_pa_setting_for_dbm(this->output_power_dbm_, &this->output_power_actual_dbm_);
  if (this->output_power_actual_dbm_ != this->output_power_dbm_) {
    ESP_LOGW(TAG, "output power %d dBm is not a 433 MHz PATABLE entry - using the nearest, %d dBm",
             this->output_power_dbm_, this->output_power_actual_dbm_);
  }
}

void Cc1101Hal::write_config_() {
  for (size_t i = 0; i < CC1101_CONFIG_SIZE; i++) {
    this->write_reg_(CC1101_CONFIG[i].addr, CC1101_CONFIG[i].value);
  }

  // MDMCFG4 packs the RX filter together with the data-rate exponent, so it can only
  // be written once both are known: CHANBW_E<7:6> | CHANBW_M<5:4> | DRATE_E<3:0>.
  this->write_reg_(CC1101_REG_MDMCFG4,
                   (uint8_t) ((this->chanbw_e_ << 6) | (this->chanbw_m_ << 4) | CC1101_DRATE_E));
  // DEVIATN: DEVIATION_E<6:4> | DEVIATION_M<2:0>. One value serves both directions -
  // the 2026-08-31 SDR capture showed a real CIU transmitting as wide as the meter
  // replies, so there is no TX/RX profile swap here of the kind the CMT2300A needs.
  this->write_reg_(CC1101_REG_DEVIATN, (uint8_t) ((this->dev_e_ << 4) | this->dev_m_));
  // TEST2/TEST1 track the RX filter bandwidth and FOCCFG carries a tunable bit, so
  // all three are written here rather than from CC1101_CONFIG. They are written
  // AFTER the table loop on purpose - the table no longer holds them, and keeping
  // the order "table first, solved values second" means a future entry slipping back
  // into the table could not silently win over the solved one.
  this->write_reg_(CC1101_REG_TEST2, this->test2_);
  this->write_reg_(CC1101_REG_TEST1, this->test1_);
  this->write_reg_(CC1101_REG_FOCCFG, this->foccfg_value_());
  this->program_frequency_(this->freq_hz_);

  // FREND0.PA_POWER is 0, so the 2-FSK PA only ever reads PATABLE[0]; the remaining
  // entries are zeroed so that a stray index could only mean "off", never
  // "unexpectedly loud". A burst write to 0x3E starts at index 0 and auto-increments.
  uint8_t patable[CC1101_PATABLE_SIZE] = {};
  patable[0] = this->pa_value_;
  this->write_burst_(CC1101_REG_PATABLE, patable, CC1101_PATABLE_SIZE);
}

// ================= INIT (once at startup) =================
bool Cc1101Hal::init() {
  if (this->sclk_pin_ == nullptr || this->miso_pin_ == nullptr || this->mosi_pin_ == nullptr ||
      this->cs_pin_ == nullptr) {
    ESP_LOGE(TAG, "SPI pins not set - set_pins() must be called before init()");
    return false;
  }

  this->sclk_pin_->setup();
  this->miso_pin_->setup();
  this->mosi_pin_->setup();
  this->cs_pin_->setup();
  this->sclk_.pin_mode(gpio::FLAG_OUTPUT);
  this->mosi_.pin_mode(gpio::FLAG_OUTPUT);
  this->cs_.pin_mode(gpio::FLAG_OUTPUT);
  this->miso_.pin_mode(gpio::FLAG_INPUT);
  if (this->gdo0_pin_ != nullptr) {
    this->gdo0_pin_->setup();
    this->gdo0_.pin_mode(gpio::FLAG_INPUT);
  }

  // Manual power-up reset, CC1101 datasheet 19.1.2. The timing matters: the chip
  // needs CSn high for at least 40 us after the short low pulse before it will
  // accept SRES, and MISO must be seen low (CHIP_RDYn) both before the strobe and
  // after it, the second time meaning the reset has finished.
  pin_high(this->cs_);
  pin_high(this->sclk_);
  pin_low(this->mosi_);
  esphome::delay(1);
  pin_low(this->cs_);
  esphome::delayMicroseconds(10);
  pin_high(this->cs_);
  esphome::delayMicroseconds(45);
  pin_low(this->cs_);
  if (!this->wait_miso_low_(CHIP_RDY_TIMEOUT_US)) {
    pin_high(this->cs_);
    ESP_LOGE(TAG, "CC1101 not responding: MISO never went low - check SPI wiring and 3V3 supply");
    return false;
  }
  this->xfer_byte_(CC1101_STROBE_SRES);
  if (!this->wait_miso_low_(RESET_TIMEOUT_US)) {
    pin_high(this->cs_);
    ESP_LOGE(TAG, "CC1101 did not complete SRES - MISO stuck high");
    return false;
  }
  pin_high(this->cs_);
  esphome::delay(1);

  // From here on everything goes through select_(), which raises spi_not_ready_ if
  // CHIP_RDYn never appears. Start from a clean slate so the checks below can only
  // be reporting on this init.
  this->clear_spi_error_();

  // Wiring sanity check. A genuine CC1101 answers PARTNUM 0x00 / VERSION 0x14, but
  // VERSION tracks the die revision (0x04 and 0x17 have both been seen) and clone
  // parts answer with their own numbers, so only an all-zero or all-ones VERSION -
  // which is what a floating or shorted MISO looks like - is treated as fatal.
  const uint8_t partnum = this->read_status_reg_(CC1101_REG_PARTNUM);
  const uint8_t version = this->read_status_reg_(CC1101_REG_VERSION);
  if (this->spi_error_()) {
    // CHIP_RDYn never went low for one of those reads, so whatever came back on MISO
    // was shifted out of a chip that had not said it was ready - it is not evidence
    // of anything, and judging PARTNUM/VERSION on it would turn a wiring or supply
    // fault into a confident "clone part, continuing".
    ESP_LOGE(TAG, "CC1101 never asserted CHIP_RDYn (MISO stayed high for %" PRIu32
                  " us) while probing - check SPI wiring, CS and the 3V3 supply",
             CHIP_RDY_TIMEOUT_US);
    return false;
  }
  ESP_LOGD(TAG, "PARTNUM: 0x%02X (expected 0x%02X), VERSION: 0x%02X (expected 0x%02X)", partnum,
           CC1101_PARTNUM_EXPECTED, version, CC1101_VERSION_EXPECTED);
  if (version == 0x00 || version == 0xFF) {
    ESP_LOGE(TAG,
             "CC1101 not detected (PARTNUM=0x%02X VERSION=0x%02X) - an all-zero or all-ones reply means "
             "nothing is driving MISO; check wiring and power",
             partnum, version);
    return false;
  }
  if (version != CC1101_VERSION_EXPECTED || partnum != CC1101_PARTNUM_EXPECTED) {
    ESP_LOGW(TAG, "unexpected PARTNUM/VERSION 0x%02X/0x%02X - die revision or clone part, continuing",
             partnum, version);
  }

  this->solve_params_();
  this->write_config_();

  // Read one register back. PARTNUM/VERSION only prove the chip can drive MISO;
  // this proves MOSI reached it too, which is the failure a half-wired module shows.
  const uint8_t sync1 = this->read_reg_(CC1101_REG_SYNC1);
  if (this->spi_error_()) {
    // Raised somewhere between the probe and here, i.e. during write_config_(). The
    // register profile was clocked at a chip that never said it was ready, so some
    // unknown subset of it did not land - and an unknown subset is a radio that may
    // transmit on the wrong frequency or at the wrong deviation. Fail loudly.
    ESP_LOGE(TAG, "CC1101 stopped asserting CHIP_RDYn while the register profile was being written - "
                  "the configuration is incomplete and cannot be trusted");
    return false;
  }
  if (sync1 != CC1101_SYNC1_ON_AIR) {
    ESP_LOGE(TAG, "register write-back failed: SYNC1 reads 0x%02X, wrote 0x%02X - check MOSI and CS",
             sync1, CC1101_SYNC1_ON_AIR);
    return false;
  }

  if (!this->go_idle_()) {
    ESP_LOGE(TAG, "CC1101 would not settle in IDLE after configuration (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }

  ESP_LOGI(TAG, "CC1101 initialized (%.3f MHz, %.2f kHz dev, %.2f kHz BW, %d dBm)", this->freq_hz_ / 1e6f,
           this->deviation_actual_hz_ / 1000.0f, this->rx_bandwidth_actual_hz_ / 1000.0f,
           this->output_power_actual_dbm_);
  return true;
}

// How long a frame of `payload_bytes` can legitimately take to get out, which is the
// only honest deadline for transmit(). The packet engine puts 12 preamble bytes and
// 2 sync bytes on the air ahead of the payload, and every bit of all of it is clocked
// at the programmed 1199.484 baud:
//
//   airtime_ms = (12 + 2 + payload) * 8 bits / 1199.484 baud * 1000
//
// For the 255-byte TX_FRAME_MAX that is 269 * 8 / 1199.484 = 1794 ms, which is why
// the fixed 900 ms this replaced could not work: transmit() accepted lengths above
// ~121 payload bytes and was then guaranteed to abort them, reporting a failure for
// a frame that was going out perfectly well. The arithmetic is integer milli-baud
// and rounds up, so the computed figure never lands under the true airtime.
uint32_t Cc1101Hal::tx_done_timeout_ms_(size_t payload_bytes) {
  const uint64_t bits = (uint64_t) (CC1101_PREAMBLE_BYTES + CC1101_SYNC_BYTES + payload_bytes) * 8u;
  const uint32_t airtime_ms =
      (uint32_t) ((bits * 1000000u + CC1101_DRATE_MILLIBAUD - 1u) / CC1101_DRATE_MILLIBAUD);
  return airtime_ms + TX_DONE_MARGIN_MS;
}

// ================= TX =================
bool Cc1101Hal::transmit(const uint8_t *frame, size_t len) {
  if (frame == nullptr || len == 0) {
    ESP_LOGW(TAG, "TX: empty frame");
    return false;
  }

  // The frame arrives logical and starting at the sync word (see radio_hal.h). The
  // CC1101's packet engine emits preamble and sync itself from MDMCFG1/SYNC1/SYNC0,
  // so sending the 98 f3 again would put it on the air twice and leave the meter's
  // receiver looking at a second sync where the length byte should be. Strip it -
  // but only if it is actually there, because transmitting a frame we do not
  // recognise verbatim is always better than transmitting it two bytes short.
  size_t skip = 0;
  if (len >= 2 && frame[0] == RF433_2_SYNC_HI && frame[1] == RF433_2_SYNC_LO) {
    skip = 2;
  } else {
    ESP_LOGW(TAG, "TX frame does not start with %02X %02X (got %02X %02X) - sending it whole",
             RF433_2_SYNC_HI, RF433_2_SYNC_LO, frame[0], (len >= 2) ? frame[1] : 0);
  }

  const size_t n = len - skip;
  if (n == 0 || n > TX_FRAME_MAX) {
    ESP_LOGW(TAG, "TX frame length %zu out of range after stripping sync", n);
    return false;
  }

  // RF433-2 is LSB-first on air and the CC1101 shifts MSB-first with no bit-order
  // control, so every byte is reversed on the way to the FIFO.
  uint8_t air[TX_FRAME_MAX];
  for (size_t i = 0; i < n; i++) {
    air[i] = RadioHal::reverse8(frame[skip + i]);
  }

  if (!this->go_idle_()) {
    ESP_LOGW(TAG, "TX: chip would not go IDLE (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }
  // Fixed-length mode for TX: we know exactly how many bytes we are sending, and it
  // gives the packet engine a real end-of-packet, which is what makes GDO0's falling
  // edge and the return to IDLE mean "sent".
  this->write_reg_(CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_TX);
  this->write_reg_(CC1101_REG_PKTLEN, (uint8_t) n);
  this->program_frequency_(this->freq_hz_);  // TX sits on the channel centre, no offset
  this->strobe_(CC1101_STROBE_SFTX);

  // A d101-2 envelope can run past the 64-byte FIFO, so the FIFO is primed with what
  // fits and refilled as it drains - writing the whole frame up front would simply
  // drop the tail, exactly as it would on the CMT2300A.
  size_t written = (n > CC1101_FIFO_SIZE) ? CC1101_FIFO_SIZE : n;
  this->write_burst_(CC1101_REG_FIFO, air, written);

  this->strobe_(CC1101_STROBE_STX);

  // `written` only ever means "loaded into the TX FIFO", never "sent on air", so it
  // cannot by itself distinguish a completed packet from an STX that was rejected or
  // ineffective and left the chip sitting in IDLE with a full FIFO. Both look like
  // "everything written and MARCSTATE == IDLE". Without positive evidence that the
  // chip really entered TX, the second case is reported as a successful transmission
  // and the component then burns its entire receive timeout waiting for a reply to a
  // request that never went out - the exact failure mode this project exists to
  // eliminate. So TX entry is tracked as a fact in its own right: observed here on
  // entry, or later in the loop from MARCSTATE, or from GDO0 asserting, which under
  // IOCFG0 = 0x06 means the sync word has physically gone out.
  bool tx_started = this->wait_marcstate_(CC1101_MARC_TX, TX_ENTER_TIMEOUT_MS);
  if (!tx_started) {
    // Tell "STX did nothing" apart from "we looked too slowly". The shortest frame
    // here is ~93 ms of air, so the packet cannot have come and gone inside the 20 ms
    // wait; if the chip is still in IDLE with every loaded byte still in the FIFO,
    // nothing has gone out and nothing will, and waiting out the full packet deadline
    // would only delay the retry.
    const uint8_t marc = this->marcstate_();
    const uint8_t in_fifo = this->read_status_reg_(CC1101_REG_TXBYTES) & CC1101_FIFO_COUNT_MASK;
    if (marc == CC1101_MARC_IDLE && in_fifo >= written) {
      ESP_LOGW(TAG, "TX: STX did not start a transmission (MARCSTATE=0x%02X, %u/%zu bytes still in the "
                    "FIFO) - nothing went on the air",
               marc, (unsigned) in_fifo, written);
      this->strobe_(CC1101_STROBE_SIDLE);
      this->strobe_(CC1101_STROBE_SFTX);
      this->go_idle_();
      return false;
    }
    ESP_LOGW(TAG, "TX: entry not confirmed within %" PRIu32 " ms (MARCSTATE=0x%02X) - watching for a late "
                  "start",
             TX_ENTER_TIMEOUT_MS, marc);
  }

  const bool chunked = n > CC1101_FIFO_SIZE;
  // Derived from this frame's own airtime rather than a one-size constant - see
  // tx_done_timeout_ms_(). `n` is the payload; preamble and sync are added there.
  const uint32_t done_timeout_ms = tx_done_timeout_ms_(n);
  const uint32_t t0 = esphome::millis();
  bool sync_seen = false;
  bool done = false;
  while (esphome::millis() - t0 < done_timeout_ms) {
    const uint8_t txbytes = this->read_status_reg_(CC1101_REG_TXBYTES);
    if ((txbytes & CC1101_TXBYTES_UNDERFLOW) != 0) {
      // The PA ran out of bytes mid-packet: whatever reached the air is truncated
      // and the chip is wedged in TXFIFO_UNDERFLOW until the FIFO is flushed.
      ESP_LOGW(TAG, "TX FIFO underflow after %zu/%zu bytes - frame truncated on air", written, n);
      this->strobe_(CC1101_STROBE_SIDLE);
      this->strobe_(CC1101_STROBE_SFTX);
      this->go_idle_();
      return false;
    }

    const uint8_t in_fifo = txbytes & CC1101_FIFO_COUNT_MASK;
    if (written < n && in_fifo <= TX_REFILL_LOW_WATER) {
      // Refill to the brim once the FIFO has drained past the threshold programmed
      // in FIFOTHR (33 bytes in TX). At 1200 baud a full FIFO is 426 ms of air, so
      // the loop has an enormous margin over the poll interval.
      const size_t room = CC1101_FIFO_SIZE - in_fifo;
      const size_t remaining = n - written;
      const size_t chunk = (remaining > room) ? room : remaining;
      this->write_burst_(CC1101_REG_FIFO, air + written, chunk);
      written += chunk;
      continue;
    }

    if (this->gdo0_pin_ != nullptr) {
      // IOCFG0 = 0x06: GDO0 rises when the sync word has gone out and falls at the
      // end of the packet. That falling edge is the exact TX-done marker; without
      // the pin the MARCSTATE check below gets there a poll or two later. A high
      // GDO0 is also the strongest possible evidence that TX started - the sync word
      // is already on the air - so it settles tx_started on its own.
      if (pin_read(this->gdo0_)) {
        sync_seen = true;
        tx_started = true;
      } else if (sync_seen) {
        done = true;
        break;
      }
    }

    // One MARCSTATE read serves both purposes: it is the late-start evidence when
    // entry was missed above, and the completion test once everything is written.
    const uint8_t marc = this->marcstate_();
    if (marc == CC1101_MARC_TX || marc == CC1101_MARC_TX_END) {
      tx_started = true;
    }
    // MCSM1.TXOFF_MODE = 00 parks the chip in IDLE when the packet is done, so
    // "everything written and back in IDLE" is an unambiguous completion - but only
    // once TX is known to have started, because an unstarted TX sits in IDLE too and
    // would otherwise satisfy this on the very first poll.
    if (tx_started && written >= n && marc == CC1101_MARC_IDLE) {
      done = true;
      break;
    }

    esphome::delayMicroseconds(200);
    esphome::yield();
  }

  const uint32_t dur = esphome::millis() - t0;
  if (!tx_started) {
    // The deadline passed without the chip ever being seen in TX or TX_END and
    // without GDO0 ever asserting. Whatever is in the FIFO was never modulated, so
    // this is a failed transmission and must be reported as one: a "success" here
    // would send the component into a receive epoch that cannot possibly produce a
    // reply. Flush the FIFO on the way out so the next attempt starts empty.
    ESP_LOGW(TAG, "TX never started: no TX state and no GDO0 sync in %" PRIu32 " ms (MARCSTATE=0x%02X) - "
                  "the frame did not go on the air",
             dur, this->marcstate_());
    this->strobe_(CC1101_STROBE_SIDLE);
    this->strobe_(CC1101_STROBE_SFTX);
    this->go_idle_();
    return false;
  }
  if (!done) {
    ESP_LOGV(TAG, "TX done timeout after %" PRIu32 " ms (%zu/%zu written, MARCSTATE=0x%02X TXBYTES=0x%02X)",
             dur, written, n, this->marcstate_(), this->read_status_reg_(CC1101_REG_TXBYTES));
    this->go_idle_();
    return false;
  }
  if (written < n) {
    // The packet ended with bytes still unsent, so what reached the air is a frame
    // whose CRC cannot check out and which the meter will never answer. That is a
    // failed transmission, not a qualified success: say so, and let the component
    // resend rather than spend an RX timeout waiting on it. (The CMT2300A backend
    // reports the same condition the same way - the two must not disagree about
    // what "transmit() returned true" means.)
    ESP_LOGW(TAG, "TX complete but only %zu/%zu written - frame truncated on air", written, n);
    this->go_idle_();
    return false;
  }
  // For a chunked (> FIFO) frame, log timing and fill so a clean send (dur ~ airtime,
  // written == n) can be told apart from a FIFO underrun (dur far too short).
  if (chunked) {
    ESP_LOGVV(TAG, "TX done (chunked): %zu/%zu written, %" PRIu32 " ms", written, n, dur);
  }
  this->go_idle_();
  return true;
}

// ================= RX =================
// This is the whole begin_rx() epoch contract from radio_hal.h: SIDLE aborts a TX or
// RX still in flight and is also the only way out of a latched RXFIFO_OVERFLOW,
// SFRX empties the FIFO so not one byte of a late or abandoned reply can be read as
// the head of the next one, and SRX is confirmed in MARCSTATE before returning true.
// There is no interrupt-flag register to clear on this chip - the GDO lines are
// level outputs of the current state, not latches - and the driver keeps no software
// receive buffer of its own, so the FIFO flush is the complete flush.
bool Cc1101Hal::begin_rx() {
  if (!this->go_idle_()) {
    ESP_LOGW(TAG, "RX: chip would not go IDLE (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }

  // RX may be centred away from the TX carrier. The default offset is 0 because the
  // chip's frequency offset compensation absorbs the meter's measured +1.1 kHz, but
  // the knob is here for the case where it does not.
  const int64_t rx_hz = (int64_t) this->freq_hz_ + (int64_t) this->rx_offset_hz_;
  this->program_frequency_((uint32_t) rx_hz);

  // Infinite packet length: the on-air LEN byte is bit-reversed, so the chip's
  // variable-length engine would read the wrong value and truncate the frame. The
  // component bounds the frame by the LEN byte itself (see drain_rx in radio_hal.h),
  // so all the packet engine has to do is find the sync word and never stop.
  // PKTLEN is restored to its resting value because transmit() leaves it at the
  // last TX length and it would otherwise be wrong the moment the mode changed.
  this->write_reg_(CC1101_REG_PKTCTRL0, CC1101_PKTCTRL0_RX);
  this->write_reg_(CC1101_REG_PKTLEN, 0xFF);

  this->strobe_(CC1101_STROBE_SFRX);
  this->strobe_(CC1101_STROBE_SRX);
  if (!this->wait_marcstate_(CC1101_MARC_RX, STATE_TIMEOUT_MS)) {
    ESP_LOGW(TAG, "RX: no RX state (MARCSTATE=0x%02X)", this->marcstate_());
    return false;
  }
  return true;
}

// MARCSTATE values that mean the receive epoch is over rather than merely between
// states. Entering RX walks through CALIBRATE, SETTLING and FS_LOCK, and RX_END is
// transient, so none of those is a fault; what is a fault is the chip sitting
// somewhere it cannot leave by itself - IDLE or SLEEP, meaning something took it out
// of RX, or any TX-side state, meaning it is not listening at all. RXFIFO_OVERFLOW
// is deliberately absent: drain_rx() handles and recovers that one itself.
static bool marcstate_is_dead_rx(uint8_t marc) {
  switch (marc) {
    case CC1101_MARC_SLEEP:
    case CC1101_MARC_IDLE:
    case CC1101_MARC_XOFF:
    case CC1101_MARC_FSTXON:
    case CC1101_MARC_TX:
    case CC1101_MARC_TX_END:
    case CC1101_MARC_TXFIFO_UNDERFLOW:
      return true;
    default:
      return false;
  }
}

size_t Cc1101Hal::drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) {
  status = RxStatus::RX_STATUS_NO_DATA;
  if (buf == nullptr || buf_size == 0) {
    return 0;
  }

  // Everything below is read over SPI, so start from a clean readiness flag: a
  // CHIP_RDYn timeout during this poll must surface as IO_ERROR rather than being
  // inherited from, or leaked into, another one.
  this->clear_spi_error_();

  const uint8_t rxbytes = this->read_status_reg_(CC1101_REG_RXBYTES);
  if (this->spi_error_()) {
    // The chip never signalled ready, so this byte came out of a shift register that
    // was not driven by a running SPI interface. It is not a FIFO count and must not
    // be used as one - acting on it would read an arbitrary number of bytes out of
    // the FIFO. IO_ERROR ends the epoch, which is the right answer: a radio that
    // will not answer CHIP_RDYn is not receiving either.
    ESP_LOGW(TAG, "RX: CC1101 did not assert CHIP_RDYn within %" PRIu32 " us - the SPI read is meaningless",
             CHIP_RDY_TIMEOUT_US);
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }
  if ((rxbytes & CC1101_RXBYTES_OVERFLOW) != 0) {
    // The chip stops receiving the moment the FIFO overflows and stays wedged in
    // RXFIFO_OVERFLOW until it is flushed from IDLE. Whatever is in there is a
    // truncated frame at best, so drop it and get back to listening. The caller is
    // told OVERFLOW rather than "no bytes": the reply is gone, and waiting out the
    // RX timeout for it would only delay the retry that might still succeed.
    ESP_LOGW(TAG, "RX FIFO overflow - flushing and restarting RX");
    this->strobe_(CC1101_STROBE_SIDLE);
    this->strobe_(CC1101_STROBE_SFRX);
    this->strobe_(CC1101_STROBE_SRX);
    status = RxStatus::RX_STATUS_OVERFLOW;
    return 0;
  }

  const uint8_t available = rxbytes & CC1101_FIFO_COUNT_MASK;
  if (available > CC1101_RXBYTES_MAX_VALID) {
    // 65 is a REAL count, not a fault: the receive chain holds one byte in a prefetch
    // register ahead of the 64-byte FIFO and NUM_RXBYTES counts it, which is why the
    // bound here is CC1101_RXBYTES_MAX_VALID (= 65) and not the FIFO depth. Rejecting
    // 65 is actively harmful - a loop() that ran late lets the FIFO reach exactly that
    // while the reply is still arriving, and calling it a bus failure throws away a
    // frame that was being received correctly. Only a count ABOVE 65 is a value the
    // part cannot produce; read_status_reg_() already re-reads until two agree, so two
    // reads agreeing on something impossible means the bus is wrong - a floating MISO
    // or a wire that has come off - not the documented errata race.
    ESP_LOGW(TAG, "RX: implausible RXBYTES 0x%02X (above the %u-byte FIFO-plus-prefetch maximum) - the SPI "
                  "read cannot be trusted",
             rxbytes, (unsigned) CC1101_RXBYTES_MAX_VALID);
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }

  const uint8_t marc = this->marcstate_();
  if (available == 0) {
    if (marcstate_is_dead_rx(marc)) {
      ESP_LOGW(TAG, "RX: chip left RX with an empty FIFO (MARCSTATE=0x%02X)", marc);
      status = RxStatus::RX_STATUS_NOT_IN_RX;
    }
    return 0;
  }

  // Documented CC1101 FIFO hazard (errata SWRZ020E): emptying the RX FIFO completely
  // while the demodulator is still writing into it can return a byte that was never
  // received, and the errata's own workaround is to read n - 1 bytes. While the chip
  // is in RX we therefore always leave the last byte behind - including at the 65
  // the prefetch register makes possible, where n - 1 is 64 and the byte left behind
  // is the prefetched one. Nothing is stranded by this: infinite-length mode keeps
  // demodulating noise after the reply, so more bytes always arrive behind the one
  // held back.
  const bool receiving = (marc == CC1101_MARC_RX);
  size_t take = receiving ? (size_t) (available - 1u) : (size_t) available;
  if (take > CC1101_FIFO_SIZE) {
    // Only reachable at available == 65 with the chip already out of RX. Capping at
    // the FIFO depth keeps max_drain_chunk() an exact bound rather than an estimate,
    // which is what the component's "buffer is full" fallback reasons from; the odd
    // byte is not lost, the next poll takes it (and the FIFO is no longer filling, so
    // there is no race to lose it to).
    take = CC1101_FIFO_SIZE;
  }
  if (take == 0) {
    return 0;
  }
  if (take > buf_size) {
    take = buf_size;
  }

  this->read_fifo_burst_(buf, take);
  if (this->spi_error_()) {
    // CHIP_RDYn dropped out between the count read and the burst, so the bytes just
    // clocked in are not the FIFO's contents. Appending them would feed the frame
    // parser garbage dressed up as a reply, which is worse than reporting nothing.
    ESP_LOGW(TAG, "RX: CC1101 stopped asserting CHIP_RDYn during the FIFO burst - discarding %zu bytes",
             take);
    status = RxStatus::RX_STATUS_IO_ERROR;
    return 0;
  }
  // Back into the logical MSB-first domain the rest of the component works in.
  for (size_t i = 0; i < take; i++) {
    buf[i] = RadioHal::reverse8(buf[i]);
  }
  // Bytes in hand are reported as DATA even if the chip has since left RX: they were
  // received, and the next poll is where the dead epoch gets noticed.
  status = RxStatus::RX_STATUS_DATA;
  return take;
}

// Datasheet section 17.3: the RSSI register is a 2's complement value in half-dB
// steps, offset by 74 dB at 433 MHz.
int8_t Cc1101Hal::get_rssi_dbm() {
  const int rssi_dec = (int) this->read_status_reg_(CC1101_REG_RSSI);
  const int dbm = (rssi_dec >= 128) ? (((rssi_dec - 256) / 2) - 74) : ((rssi_dec / 2) - 74);
  return (int8_t) dbm;
}

// Called from the component's dump_config() after its own lines, so the two-space
// indent makes these read as part of the same block. Everything printed here is the
// value the chip was actually programmed with rather than the value asked for in
// YAML, so the log is evidence of what went on the air.
void Cc1101Hal::dump_config() {
  LOG_PIN("  SCLK pin: ", this->sclk_pin_);
  LOG_PIN("  MISO pin: ", this->miso_pin_);
  LOG_PIN("  MOSI pin: ", this->mosi_pin_);
  LOG_PIN("  CS pin: ", this->cs_pin_);
  if (this->gdo0_pin_ != nullptr) {
    LOG_PIN("  GDO0 pin: ", this->gdo0_pin_);
  } else {
    ESP_LOGCONFIG(TAG, "  GDO0 pin: not used (TX completion polled from MARCSTATE)");
  }

  const uint32_t tx_word = freq_word_for_(this->freq_hz_);
  const uint32_t tx_hz = freq_hz_for_word_(tx_word);
  const uint32_t rx_word = freq_word_for_((uint32_t) ((int64_t) this->freq_hz_ + (int64_t) this->rx_offset_hz_));
  const uint32_t rx_hz = freq_hz_for_word_(rx_word);
  const float data_rate =
      (256.0f + (float) CC1101_DRATE_M) * (float) (1u << CC1101_DRATE_E) * (float) CC1101_XTAL_HZ / 268435456.0f;

  ESP_LOGCONFIG(TAG, "  TX carrier: %.4f MHz (FREQ=0x%06" PRIX32 ", %+d Hz off the %.3f MHz channel)",
                tx_hz / 1e6f, tx_word, (int) ((int32_t) tx_hz - (int32_t) this->freq_hz_),
                this->freq_hz_ / 1e6f);
  ESP_LOGCONFIG(TAG, "  RX carrier: %.4f MHz (FREQ=0x%06" PRIX32 ", requested offset %+" PRId32 " Hz)",
                rx_hz / 1e6f, rx_word, this->rx_offset_hz_);
  ESP_LOGCONFIG(TAG, "  Deviation: %.2f kHz (DEVIATN=0x%02X, DEV_E=%u DEV_M=%u)",
                this->deviation_actual_hz_ / 1000.0f, (unsigned) ((this->dev_e_ << 4) | this->dev_m_),
                (unsigned) this->dev_e_, (unsigned) this->dev_m_);
  // TEST2/TEST1 are printed next to the bandwidth they were chosen for, because that
  // is the only way to see at a glance that the pair matches the filter: 0x81/0x35 at
  // or below 101 kHz, 0x88/0x31 above it.
  ESP_LOGCONFIG(TAG, "  RX bandwidth: %.2f kHz (CHANBW_E=%u CHANBW_M=%u, TEST2=0x%02X TEST1=0x%02X)",
                this->rx_bandwidth_actual_hz_ / 1000.0f, (unsigned) this->chanbw_e_,
                (unsigned) this->chanbw_m_, this->test2_, this->test1_);
  // The FOC/bit-sync carrier-sense gate is the bring-up knob most likely to need
  // changing on real hardware, so the effective register value is on every boot log:
  // 0x16 means both loops track from the moment RX is entered, 0x36 means they stay
  // frozen until carrier sense asserts. See set_foc_cs_gate().
  ESP_LOGCONFIG(TAG, "  FOC/bit-sync carrier-sense gate: %s (FOCCFG=0x%02X)",
                this->foc_cs_gate_ ? "ON - loops frozen until carrier sense"
                                   : "OFF - loops track continuously",
                this->foccfg_value_());
  ESP_LOGCONFIG(TAG, "  Data rate: %.2f baud (DRATE_E=%u DRATE_M=%u), 2-FSK, sync %02X %02X on air",
                data_rate, (unsigned) CC1101_DRATE_E, (unsigned) CC1101_DRATE_M, CC1101_SYNC1_ON_AIR,
                CC1101_SYNC0_ON_AIR);
  // The deficit is worth printing on every boot: it is the standing candidate
  // explanation for a meter that hears a real CIU but not us.
  ESP_LOGCONFIG(TAG, "  Output power: %d dBm (PATABLE[0]=0x%02X) - a Nartis CIU transmits +20 dBm, so the "
                     "uplink is ~%d dB short of the reference link budget",
                this->output_power_actual_dbm_, this->pa_value_, 20 - this->output_power_actual_dbm_);
}

}  // namespace esphome::nartis_rf_2_meter
