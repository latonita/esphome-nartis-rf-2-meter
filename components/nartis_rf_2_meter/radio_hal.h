/*
 * Radio hardware abstraction for the d101-2 link.
 *
 * The RF433-2 PHY is transceiver-agnostic:
 * 2-FSK, 1200 bps NRZ, +-25 kHz deviation, LSB-first on air, 0x55 preamble,
 * sync 98 F3, no whitening/Manchester/FEC/PHY-CRC. Everything above the radio -
 * the DL/T 645 envelope, the 6-byte BCD serial, the CRC-16/X.25 - is radio
 * independent, so a backend only has to carry opaque frames.
 *
 * A frame crossing this interface is always in the LOGICAL domain, MSB-first,
 * starting at the sync word: `98 f3 LEN ... CRCL CRCH` on transmit. What comes
 * back from drain_rx() starts at the LEN byte: preamble and sync are the backend's
 * to consume or detect, by whatever means it has - a hardware packet engine on both
 * of today's backends, software correlation on a future one - and never reach the
 * caller. Bit-reversal to the on-air LSB-first domain is each backend's own
 * business.
 *
 * Backends: Cmt2300aHal (cmt2300a_hal.h), Cc1101Hal (cc1101_hal.h).
 */

#pragma once

#include <cstddef>
#include <cstdint>

namespace esphome::nartis_rf_2_meter {

/* ================================================================
 * Channel grid
 * ================================================================ */

/* Which channel a meter answers on is derived from its serial, not configured:
 *
 *   k = last three digits of the serial, mod 24
 *   f = CHANNEL_BASE_HZ + k * CHANNEL_STEP_HZ, plus CHANNEL_STEP_EXTRA_HZ once
 *       k > CHANNEL_STEP_BREAK
 *
 * so the k=18 -> 19 step is 800 kHz and every other one is 700 kHz. That break is
 * on-air evidence rather than a reading of the display's register writes.
 */
static constexpr uint32_t CHANNEL_BASE_HZ = 435500000u;
static constexpr uint32_t CHANNEL_STEP_HZ = 700000u;
static constexpr uint32_t CHANNEL_STEP_BREAK = 18u;
static constexpr uint32_t CHANNEL_STEP_EXTRA_HZ = 100000u;

/// Channel frequency (Hz) from the last three digits of the meter serial.
/// e.g. "...060" -> 60 % 24 = 12 -> 443.900 MHz; "...596" -> 20 -> 449.600 MHz.
/// Non-digit characters are skipped; a null pointer reads as serial 0.
inline uint32_t frequency_from_serial(const char *digits12) {
  uint32_t n3 = 0;
  if (digits12 != nullptr) {
    size_t len = 0;
    while (digits12[len] != '\0') {
      len++;
    }
    const size_t start = (len >= 3) ? (len - 3) : 0;
    for (size_t i = start; i < len; i++) {
      const char c = digits12[i];
      if (c >= '0' && c <= '9') {
        n3 = n3 * 10 + static_cast<uint32_t>(c - '0');
      }
    }
  }
  const uint32_t k = n3 % 24;
  return CHANNEL_BASE_HZ + k * CHANNEL_STEP_HZ + (k > CHANNEL_STEP_BREAK ? CHANNEL_STEP_EXTRA_HZ : 0u);
}

/* ================================================================
 * Receive status
 * ================================================================ */

/* Why drain_rx() reports a status instead of just a count: a bare byte count makes
 * "nothing has arrived yet" and "the receiver is broken" the same answer, 0, and the
 * component then waits out its whole rf_rx_timeout_ms_ and books the exchange as
 * "the meter did not answer". That is a misdiagnosis with a cost - it hides a radio
 * fault behind a link fault, and it burns a timeout per attempt where the attempt
 * was already lost. Every zero-byte return therefore says WHY it is zero. */
enum class RxStatus : uint8_t {
  RX_STATUS_DATA,       // bytes were appended
  RX_STATUS_NO_DATA,    // nothing has arrived yet; keep waiting
  RX_STATUS_OVERFLOW,   // RX FIFO overflowed; the backend recovered, the frame is lost
  RX_STATUS_IO_ERROR,   // the chip did not respond plausibly over SPI
  RX_STATUS_NOT_IN_RX,  // the radio is no longer receiving; the epoch is dead
};

class RadioHal {
 public:
  virtual ~RadioHal() = default;

  /// Chip name for logs, e.g. "CMT2300A". Never null.
  virtual const char *radio_name() const = 0;

  /// Channel centre in Hz. Must be called before init(), which is what programs
  /// the synthesiser. The RX centre may sit a backend-specific offset above this.
  virtual void set_frequency(uint32_t freq_hz) = 0;

  /// Reset, probe and configure the chip. False means the radio is unusable and
  /// the component marks itself failed; the backend logs the reason.
  virtual bool init() = 0;

  /// Transmit one frame, blocking until the chip reports the packet sent or an
  /// internal timeout expires. Parks the chip in standby either way - on every exit
  /// path, including the early ones.
  ///
  /// True means the whole frame went out. A frame the chip truncated is a failure
  /// like any other: the meter heard something whose CRC cannot check out and will
  /// never answer it, so the caller must be free to resend rather than spend a
  /// receive timeout waiting on it.
  virtual bool transmit(const uint8_t *frame, size_t len) = 0;

  /// Start a fresh, VERIFIED receive epoch on this channel. The component calls it
  /// once after each transmit, and everything drained afterwards must belong to that
  /// one reply. "Switch to RX" is not enough: a sibling project in this repo had a
  /// late reply to a request that had already timed out sitting in the FIFO, where
  /// it was drained as the head of the NEXT reply and desynced every request/reply
  /// pair after it. So an implementation must, in this order:
  ///
  ///   - abort any transmit or receive still in progress;
  ///   - flush the hardware RX FIFO and any buffer the backend keeps in software;
  ///   - clear the latched overflow, sync, FIFO-threshold and packet-done interrupt
  ///     flags, so the first one seen afterwards was raised by this epoch;
  ///   - confirm the chip actually reached RX, and return false if it did not.
  ///
  /// False means no epoch was started: the component must not then poll drain_rx().
  virtual bool begin_rx() = 0;

  /// Append whatever has been received to `buf`, never writing past `buf_size`, and
  /// return the number of bytes appended. `status` always says what the return value
  /// means - in particular WHY it is zero, since "nothing yet" and "this epoch is
  /// dead" demand opposite responses from the caller.
  ///
  /// Must return promptly rather than blocking for the packet to complete: the
  /// caller bounds the frame by its own LEN byte and polls from ESPHome's loop(). A
  /// backend that does know a packet has completed is free to say so through
  /// `status` - it just may not wait for it.
  ///
  /// No default argument on purpose: a default on a virtual binds to the static type
  /// of the expression, so an override could silently supply a different one.
  virtual size_t drain_rx(uint8_t *buf, size_t buf_size, RxStatus &status) = 0;

  /// The largest number of bytes one drain_rx() call can append. The component uses
  /// it to tell "the buffer can still grow" from "the buffer is full", which is its
  /// end-of-reception fallback when a frame's LEN byte is not believable.
  virtual size_t max_drain_chunk() const = 0;

  /// RSSI of the last reception, dBm.
  virtual int8_t get_rssi_dbm() = 0;

  /// Park in the lowest state that keeps the configuration. False = the chip did
  /// not confirm the transition.
  virtual bool go_standby() = 0;

  /// Backend-specific lines for the component's dump_config(), already indented
  /// to sit under its own header. ESPHome dumps a component's configuration even
  /// after mark_failed(), so this may well be called when init() returned false and
  /// the chip is unresponsive or absent: print stored configuration only, and do not
  /// talk to the chip from here.
  virtual void dump_config() = 0;

  /// Reverse the bits of a byte - the logical <-> on-air mapping for every
  /// backend, since RF433-2 is LSB-first on air.
  static uint8_t reverse8(uint8_t b) {
    b = (b & 0xF0) >> 4 | (b & 0x0F) << 4;
    b = (b & 0xCC) >> 2 | (b & 0x33) << 2;
    b = (b & 0xAA) >> 1 | (b & 0x55) << 1;
    return b;
  }
};

}  // namespace esphome::nartis_rf_2_meter
