"""Nartis RF-2 meter - ESPHome external component.

Reads Nartis И100/И300/И500 meters (2024+, the "RF-433-2" / Д101-2 protocol) over
a CMT2300A 443 MHz radio by emulating the НАРТИС-Д101-2 display. The link carries
a DL/T 645-1997 frame with no encryption, no session and no password, so this
component talks to the meter on its own - no UART bridge and no dlms_cosem.

Only a fixed set of values is available: the meter answers two constant requests
and returns whatever its configured indication set contains. Entities therefore
select a value by its 1-byte item TAG (DI 0xF200) or by a field of the status
block (DI 0xF201) - there is no way to ask for something else.
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

CONF_DATA_PAGE = "data_page"
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

# Fields of the 9-byte DI 0xF201 status block. `active_tariff` is confirmed
# against the observed tariff schedule; `tariff_count` and `temperature` are
# inferred from byte positions and should be treated as experimental.
STATUS_FIELDS = {
    "active_tariff": StatusField.ACTIVE_TARIFF,
    "tariff_count": StatusField.TARIFF_COUNT,
    "temperature": StatusField.TEMPERATURE,
}
STATUS_FIELDS_TEXT = {
    **STATUS_FIELDS,
    # Hex dump of the whole status block - for reporting the still-unexplained
    # bytes 0..4 and 6.
    "raw": StatusField.RAW,
}

# TAGs with a built-in width. Items carry no length field, so a TAG of unknown
# width cannot even be skipped - it destroys framing for the rest of the payload.
#
#   0x00..0x02   active energy import: sum, tariff 1, tariff 2  4B u32  observed
#   0x29         date and time                                  7B BCD  observed
#   0x14..0x1A   voltages, phase and line-to-line               2B u16  DLMS
#   0x1B..0x1F   currents, single-phase, neutral, per-phase     4B u32  DLMS
#   0x20..0x23   active power, sum and per-phase (signed)       4B i32  DLMS
#   0x24..0x27   reactive power, sum and per-phase (signed)     4B i32  assumed
#   0x28         mains frequency                                2B u16  DLMS
#   0x2A         temperature (signed)                           2B i16  DLMS
#   0x03..0x13   remaining tariffs, export, reactive energy     4B u32  assumed
#   0x2C..0x3F   the same registers at the last billing period  4B u32  assumed
#
# "DLMS" widths come from the data type the same meter reports for the same OBIS
# code over the DLMS-HDLC link. That mapping is validated by the energy
# registers, where DLMS says double-long-unsigned and D101-2 does send 4 bytes -
# but no DLMS-derived width has been seen on this link yet, so the component
# warns once per TAG when it publishes one.
#
# 0x2B (LCD test) is the only TAG left without a width.
TAG_CLOCK = 0x29
TAG_NO_WIDTH = {0x2B}
TAG_KNOWN = set(range(0x00, 0x40)) - TAG_NO_WIDTH
TAG_NUMERIC = TAG_KNOWN - {TAG_CLOCK}

# Any TAG is accepted, because the item index is a 6-bit field and a meter
# configured with the vendor tool can send any of them. TAGs outside TAG_KNOWN
# have no built-in width: they stay unavailable until `bytes:` declares one.
TAG_MIN = 0x00
TAG_MAX = 0x3F
# Widest item value the parser can hold (MAX_ITEM_WIDTH in d101_frame.h).
TAG_MAX_WIDTH = 9


def validate_tag(value):
    """An item TAG of the DI 0xF200 response (6-bit index, 0x00-0x3F)."""
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


# Data identifiers the display itself polls, with the request body each one
# carries. Note the body length is per-DI, not fixed - 6 bytes for 0xF200, 1 byte
# for 0xF201 - so the body of an unknown DI has to be guessed.
KNOWN_DI = {0xF200: 6, 0xF201: 1, 0xF202: 6}

# Pages the TAG entities can be read from.
#   0xF200  energy registers + clock                      answered by every meter
#   0xF202  the same plus voltages, currents, power, freq  a superset, where present
DATA_PAGES = {0xF200: "energy + clock", 0xF202: "energy + instantaneous"}
# Sentinel for "decide in setup() from the TAGs actually configured".
DATA_PAGE_AUTO = 0


def validate_data_page(value):
    """`auto`, or one of the pages known to exist."""
    if isinstance(value, str) and value.strip().lower() == "auto":
        return DATA_PAGE_AUTO
    di = cv.hex_int(value)
    if di not in DATA_PAGES:
        known = ", ".join(f"0x{d:04X} ({what})" for d, what in DATA_PAGES.items())
        raise cv.Invalid(
            f"data_page must be `auto` or one of {known}; got 0x{di:04X}. Only those "
            f"two data identifiers are known to return a parameter page - use `probe:` "
            f"to test another before relying on it."
        )
    return di
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
        # Pause between the energy and the status exchange.
        # Which page the `tag` entities are read from.
        #
        # `auto` (the default) picks it from the entities you configured: DI 0xF202
        # if any TAG only exists there - the instantaneous values - and DI 0xF200
        # otherwise. Only ever one page: 0xF202 also carries the energy registers
        # and the clock, so a mixed set never needs both, and polling both would
        # double the airtime for nothing.
        #
        # Set it explicitly to force one, e.g. if a meter turns out to answer
        # 0xF200 but not 0xF202. `probe:` is the way to find out which it answers.
        cv.Optional(CONF_DATA_PAGE, default="auto"): validate_data_page,
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

    cg.add(var.set_data_page(config[CONF_DATA_PAGE]))
    cg.add(var.set_request_gap_ms(config[CONF_REQUEST_GAP]))
    cg.add(var.set_rf_rx_timeout_ms(config[CONF_RF_RX_TIMEOUT]))
    cg.add(var.set_rf_retries(config[CONF_RF_RETRIES]))
    cg.add(var.set_rx_center_offset(config[CONF_RX_CENTER_OFFSET]))

    for probe in config.get(CONF_PROBE, []):
        cg.add(var.add_probe(probe[CONF_DI], probe[CONF_BODY]))
