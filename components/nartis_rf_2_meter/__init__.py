"""Nartis RF-2 meter - ESPHome external component.

Reads Nartis И100/И300/И500 meters (2024+, the "RF-433-2" / Д101-2 protocol) over
a CMT2300A 443 MHz radio by emulating the НАРТИС-Д101-2 display. The link carries
a DL/T 645-1997 frame with no encryption, no session and no password, so this
component talks to the meter on its own - no UART bridge and no dlms_cosem.

Only a fixed set of values is available: the meter holds two pre-defined
indication lists, configured by the utility with the vendor tool, and answers
four constant requests that read them. Entities therefore select a value by its
1-byte item TAG, or by a field of the status block - there is no way to ask for
something else.

Each list is read with two requests, and every cycle sends all four:

    DI 0xF202   list B, records         tagged values, as many as fit one frame
    DI 0xF203   list B, status half     the records left over, then the status block
    DI 0xF200   list A, records
    DI 0xF201   list A, status half

The order matters: a status half has to follow its own records half back to back,
because the leftover records come from a cursor the meter drops as soon as
anything else is asked. Both lists are read and their records merged, so a `tag`
entity does not care which list carried its value - list B has been seen to be a
superset of list A, but which list holds what is a per-meter setting.
"""

from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_FREQUENCY, CONF_ID

CODEOWNERS = ["@latonita"]
# binary_sensor is auto-loaded even though the platform is optional: the component
# header includes its type unconditionally, so the sources must always be present.
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]
MULTI_CONF = True

# --- Hub configuration keys -------------------------------------------------
# CMT2300A wiring (bit-bang 3-wire SPI + INT2 on the chip's GPIO3 pad).
CONF_PIN_SDIO = "pin_sdio"
CONF_PIN_SCLK = "pin_sclk"
CONF_PIN_CSB = "pin_csb"
CONF_PIN_FCSB = "pin_fcsb"
CONF_PIN_GPIO3 = "pin_gpio3"

CONF_REQUEST_GAP = "request_gap"
CONF_RF_RX_TIMEOUT = "rf_rx_timeout"
CONF_RF_RETRIES = "rf_retries"
CONF_RX_CENTER_OFFSET = "rx_center_offset"

CONF_PROBE = "probe"
CONF_DI = "di"
CONF_BODY = "body"

# --- Platform configuration keys -------------------------------------------
CONF_NARTIS_RF_2_METER_ID = "nartis_rf_2_meter_id"
CONF_TAG = "tag"
CONF_STATUS = "status"
CONF_BYTES = "bytes"

nartis_rf_2_meter_ns = cg.esphome_ns.namespace("nartis_rf_2_meter")
NartisRf2MeterComponent = nartis_rf_2_meter_ns.class_(
    "NartisRf2MeterComponent", cg.PollingComponent
)
StatusField = nartis_rf_2_meter_ns.enum("StatusField", is_class=True)

# Fields of the 11-byte status block that ends each list.
#
# Both names are UNCONFIRMED. They were inferred from byte positions in a single
# capture; the firmware's own layout of that block (see STATUS_BLOCK_SIZE in
# d101_frame.h) calls the same two bytes the relay/breaker state and an internal
# object, and describes the whole block as device state and alarm flags rather
# than measurements. The bytes read are unchanged, so an existing entity keeps
# reporting the same number - but prefer `raw` when working out what a meter
# actually sends.
#
# `temperature` used to be listed here and is gone: it named a byte that is not a
# temperature, and it referred to a C++ enumerator that never existed, so the
# option could not compile. Real temperature is `tag: 0x2A`.
STATUS_FIELDS = {
    "active_tariff": StatusField.ACTIVE_TARIFF,
    "tariff_count": StatusField.TARIFF_COUNT,
}
STATUS_FIELDS_TEXT = {
    **STATUS_FIELDS,
    # Hex dump of the whole block - the useful one, since most of it is
    # unexplained bit flags.
    "raw": StatusField.RAW,
}

# TAGs with a built-in width. Records carry no length field, so a TAG of unknown
# width cannot even be skipped - it destroys framing for the rest of the payload.
# The widths, encodings and scales all live in one table, TAG_TABLE in
# d101_frame.cpp; this mirrors only what the config layer has to validate.
#
#   0x00..0x13   energy accumulators                  4B binary LE
#   0x14..0x1A   voltages, phase and line-to-line     4B BCD LE
#   0x1B..0x1F   currents, single/neutral/per-phase   4B BCD LE
#   0x20..0x23   active power, total and per-phase    4B BCD LE
#   0x24..0x27   reactive power, total and per-phase  4B BCD LE, signed
#   0x28         mains frequency                      4B BCD LE
#   0x29         date and time                        7B BCD
#   0x2A         temperature                          2B binary LE, signed
#   0x2C..0x2F   contested group - see the table      4B BCD
#   0x30..0x33   power factor                         4B BCD
#   0x34..0x47   event and log counters               4B binary LE
#
# Without a built-in width, and so needing `bytes:` before they can be read:
# 0x2B (the LCD test, a command rather than a register) and 0x48..0x4F (identity
# and configuration objects, whose widths differ per object).
#
# Only some of these have been seen on this link; the component warns once per
# TAG when it publishes one taken from the vendor table rather than a capture.
TAG_CLOCK = 0x29
TAG_NO_WIDTH = {0x2B} | set(range(0x48, 0x50))
TAG_KNOWN = set(range(0x00, 0x50)) - TAG_NO_WIDTH
TAG_NUMERIC = TAG_KNOWN - {TAG_CLOCK}

# Any TAG in range is accepted: a meter configured with the vendor tool can send
# any of them. TAGs in TAG_NO_WIDTH have no built-in width and stay unavailable
# until `bytes:` declares one.
TAG_MIN = 0x00
TAG_MAX = 0x4F
# Widest item value the parser can hold (MAX_ITEM_WIDTH in d101_frame.h).
TAG_MAX_WIDTH = 9


def validate_tag(value):
    """An item TAG of a data-page response (0x00-0x4F)."""
    tag = cv.hex_int(value)
    if not TAG_MIN <= tag <= TAG_MAX:
        raise cv.Invalid(
            f"tag must be between 0x{TAG_MIN:02X} and 0x{TAG_MAX:02X}; got 0x{tag:02X}"
        )
    return tag


def validate_numeric_tag(value):
    """Same, but the clock is text-only so it is rejected on a numeric sensor."""
    tag = validate_tag(value)
    if tag == TAG_CLOCK:
        raise cv.Invalid(
            f"tag 0x{TAG_CLOCK:02X} is the meter clock - use a text_sensor for it"
        )
    return tag


def validate_tag_entity(config):
    """A TAG with no built-in width needs `bytes:` before it can be decoded."""
    tag = config.get(CONF_TAG)
    if tag is None or tag in TAG_KNOWN or CONF_BYTES in config:
        return config
    raise cv.Invalid(
        f"tag 0x{tag:02X} has no known value width, so it cannot be decoded. Add "
        f"`bytes:` with the width you observed in the log (the component prints "
        f"the raw payload whenever it meets an unrecognised TAG). Note that "
        f"declaring a TAG does not make the meter send it: the request is a fixed "
        f"frame and the meter replies with its own configured indication set.",
        path=[CONF_TAG],
    )


# Data identifiers the component polls every cycle, with the request body each one
# carries. Note the body length is per-DI, not fixed - 6 bytes for the pages, 1
# byte for 0xF201 - so the body of an unknown DI has to be guessed.
#
#   0xF200  energy registers + clock
#   0xF201  status block
#   0xF202  the same registers plus voltages, currents, power, frequency
#   0xF203  the tail of the 0xF202 record list, resumed from the meter's cursor
KNOWN_DI = {0xF200: 6, 0xF201: 1, 0xF202: 6, 0xF203: 6}

MAX_REQUEST_BODY = 8

PROBE_SCHEMA = cv.Schema(
    {
        # Any data identifier. Only 0xF200 and 0xF201 have ever been seen; the
        # meter may answer more, may refuse, or may stay silent.
        cv.Required(CONF_DI): cv.hex_int_range(min=0, max=0xFFFF),
        # Bytes that follow the DI. Defaults to a single 0x00, the shape DI 0xF201
        # uses, which is the most plausible "no filter" body for a new DI.
        cv.Optional(CONF_BODY, default=[0x00]): cv.All(
            cv.ensure_list(cv.hex_uint8_t), cv.Length(min=0, max=MAX_REQUEST_BODY)
        ),
    }
)


def validate_probes(value):
    """Warn where a probe restates a DI whose body is already known."""
    for probe in value:
        di = probe[CONF_DI]
        known = KNOWN_DI.get(di)
        if known is not None and len(probe[CONF_BODY]) != known:
            raise cv.Invalid(
                f"DI 0x{di:04X} is polled normally with a {known}-byte body; a probe "
                f"with a {len(probe[CONF_BODY])}-byte body would ask the same DI a "
                f"second time with a different body. Remove it, or probe a different DI.",
                path=[CONF_BODY],
            )
    return value


def validate_address(value):
    """Meter serial: the 12-digit number printed on the nameplate."""
    s = cv.string_strict(value)
    if not s.isdigit() or len(s) != 12:
        raise cv.Invalid(
            f"address must be exactly 12 digits (the meter serial, e.g. "
            f"'023240271060'); got {len(s)} characters '{s}'"
        )
    return s


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(NartisRf2MeterComponent),
        # CMT2300A wiring.
        cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
        # Meter serial: becomes the DL/T 645 address, and its last 3 digits select
        # the channel frequency.
        cv.Required(CONF_ADDRESS): validate_address,
        # Override the derived channel frequency. Only needed if the meter turns
        # out to sit somewhere other than the serial-derived channel.
        cv.Optional(CONF_FREQUENCY): cv.All(
            cv.frequency, cv.Range(min=430000000, max=460000000)
        ),
        # Pause between exchanges. It also separates a list's records half from
        # the status half that continues it, and the meter's cursor is what has to
        # survive that pause - so if a status half comes back with no leftover
        # records on a working link, a shorter gap is the first thing to try.
        #
        # Four exchanges per cycle at ~1 s each, so the whole cycle is roughly
        # 4 * (airtime + gap); keep update_interval well clear of that.
        cv.Optional(
            CONF_REQUEST_GAP, default="500ms"
        ): cv.positive_time_period_milliseconds,
        # How long to wait for a reply per on-air attempt. Measured on the reference
        # meter: a good reply completes a median of ~965 ms after the start of
        # transmit, and TX airtime alone is ~230 ms of that, so the reply lands
        # ~735 ms into the RX window with real spread above it. 1000 ms cut into
        # that spread; 1500 ms is the value proven in the field.
        cv.Optional(
            CONF_RF_RX_TIMEOUT, default="1800ms"
        ): cv.positive_time_period_milliseconds,
        # Retransmissions per exchange on no-reply or a bad frame. Total on-air
        # attempts = 1 + rf_retries.
        cv.Optional(CONF_RF_RETRIES, default=2): cv.int_range(min=0, max=10),
        # RX centre offset in CMT2300A frequency codes (1 code ~= 6.199 Hz); shifts
        # the RX-half LO onto the meter's reply carrier, which sits a few kHz above
        # our transmit frequency. The default is the value proven on hardware; tune
        # per install only if reception is poor.
        cv.Optional(CONF_RX_CENTER_OFFSET, default=758): cv.int_range(
            min=-4000, max=4000
        ),
        # Extra reads to try once per cycle, purely to see what comes back. Replies
        # are logged in full and drive no entity. Reads only - the DL/T 645 control
        # code is hard-wired, so no probe can turn into a write. That matters:
        # this link also carries a relay-close command.
        cv.Optional(CONF_PROBE): cv.All(
            cv.ensure_list(PROBE_SCHEMA), cv.Length(min=1, max=8), validate_probes
        ),
    }
    # The meter's own display syncs about once an hour. Polling far more often than
    # this buys little and risks colliding with that sync.
).extend(cv.polling_component_schema("300s"))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    for key, setter in (
        (CONF_PIN_SDIO, var.set_pin_sdio),
        (CONF_PIN_SCLK, var.set_pin_sclk),
        (CONF_PIN_CSB, var.set_pin_csb),
        (CONF_PIN_FCSB, var.set_pin_fcsb),
        (CONF_PIN_GPIO3, var.set_pin_gpio3),
    ):
        pin = await cg.gpio_pin_expression(config[key])
        cg.add(setter(pin))

    cg.add(var.set_address(config[CONF_ADDRESS]))
    if CONF_FREQUENCY in config:
        cg.add(var.set_frequency_override(int(config[CONF_FREQUENCY])))

    cg.add(var.set_request_gap_ms(config[CONF_REQUEST_GAP]))
    cg.add(var.set_rf_rx_timeout_ms(config[CONF_RF_RX_TIMEOUT]))
    cg.add(var.set_rf_retries(config[CONF_RF_RETRIES]))
    cg.add(var.set_rx_center_offset(config[CONF_RX_CENTER_OFFSET]))

    for probe in config.get(CONF_PROBE, []):
        cg.add(var.add_probe(probe[CONF_DI], probe[CONF_BODY]))
