"""Nartis RF-2 meter - ESPHome external component.

Reads Nartis И100/И300/И500 meters (2024+, the "RF-433-2" / Д101-2 protocol) over
a CMT2300A 443 MHz radio by emulating the НАРТИС-Д101-2 display. DL/T 645-1997
inside a radio envelope: no encryption, no session, no password.

The meter answers six constant requests. Which are sent is the `sources:` setting,
default list B alone:

    DI 0xF202/0xF203   list B, records + status half
    DI 0xF200/0xF201   list A, records + status half
    DI 0xF101/0xF102   fixed blocks - positional values, see fixed.md

A status half must follow its own records half back to back: the leftover records
come from a cursor the meter drops as soon as anything else is asked.

An entity selects a value by its 1-byte item TAG, or by a status-block field. Every
reply is folded into one value per TAG, the list winning where both sources carry
one, and published already scaled to the unit tags.md gives - so a `multiply`
filter would scale it twice. A TAG read with `bytes:` is the exception: unknown
unit, published raw.
"""

from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.const import CONF_ADDRESS, CONF_FREQUENCY, CONF_ID

CODEOWNERS = ["@latonita"]
# The component header includes binary_sensor's type unconditionally, so it has to
# be auto-loaded even though the platform is optional.
AUTO_LOAD = ["binary_sensor", "sensor", "text_sensor"]
MULTI_CONF = True

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

CONF_SOURCES = "sources"

CONF_PROBE = "probe"
CONF_DI = "di"
CONF_BODY = "body"

CONF_NARTIS_RF_2_METER_ID = "nartis_rf_2_meter_id"
CONF_TAG = "tag"
CONF_STATUS = "status"
CONF_BYTES = "bytes"

nartis_rf_2_meter_ns = cg.esphome_ns.namespace("nartis_rf_2_meter")
NartisRf2MeterComponent = nartis_rf_2_meter_ns.class_(
    "NartisRf2MeterComponent", cg.PollingComponent
)
StatusField = nartis_rf_2_meter_ns.enum("StatusField", is_class=True)

# Both names are UNCONFIRMED - inferred from byte positions in one capture, where the
# firmware layout calls the same bytes device state. Real temperature is `tag: 0x2A`.
STATUS_FIELDS = {
    "active_tariff": StatusField.ACTIVE_TARIFF,
    "tariff_count": StatusField.TARIFF_COUNT,
}
STATUS_FIELDS_TEXT = {
    **STATUS_FIELDS,
    "raw": StatusField.RAW,
}

# Mirrors TAG_TABLE in d101_frame.cpp, which is the one place widths, encodings and
# scales live. A TAG of unknown width destroys framing for the rest of the payload,
# since records carry no length field - hence `bytes:` before use.
TAG_CLOCK = 0x29
TAG_NO_WIDTH = {0x2B} | set(range(0x40, 0x50))
TAG_KNOWN = set(range(0x00, 0x50)) - TAG_NO_WIDTH
TAG_NUMERIC = TAG_KNOWN - {TAG_CLOCK}

TAG_MIN = 0x00
TAG_MAX = 0x4F
# MAX_ITEM_WIDTH in d101_frame.h.
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


# Each source costs two exchanges, ~1 s apiece. Which list holds what is set per
# meter with the vendor tool; list B was a superset of list A on the reference meter,
# hence the default. `fixed` publishes what no list carried - chiefly per-phase power.
SOURCE_LIST_A = "list_a"
SOURCE_LIST_B = "list_b"
SOURCE_FIXED = "fixed"
SOURCES = [SOURCE_LIST_A, SOURCE_LIST_B, SOURCE_FIXED]


def validate_sources(value):
    """A list of sources: at least one, each named at most once."""
    seen = []
    for item in value:
        if item in seen:
            raise cv.Invalid(f"source '{item}' is listed twice")
        seen.append(item)
    if not seen:
        raise cv.Invalid(
            f"at least one source is required; choose from {', '.join(SOURCES)}"
        )
    return seen


# Body length is per-DI, not fixed, so the body of an unknown DI has to be guessed.
KNOWN_DI = {0xF200: 6, 0xF201: 1, 0xF202: 6, 0xF203: 6, 0xF101: 6, 0xF102: 6}

MAX_REQUEST_BODY = 8

PROBE_SCHEMA = cv.Schema(
    {
        cv.Required(CONF_DI): cv.hex_int_range(min=0, max=0xFFFF),
        # Defaults to a single 0x00, the shape DI 0xF201 uses.
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
        cv.Required(CONF_PIN_SDIO): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_SCLK): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_CSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_FCSB): pins.internal_gpio_output_pin_schema,
        cv.Required(CONF_PIN_GPIO3): pins.internal_gpio_input_pin_schema,
        # Becomes the DL/T 645 address; its last 3 digits select the channel.
        cv.Required(CONF_ADDRESS): validate_address,
        # Only needed if the meter sits off its serial-derived channel.
        cv.Optional(CONF_FREQUENCY): cv.All(
            cv.frequency, cv.Range(min=430000000, max=460000000)
        ),
        # Also separates a records half from the status half that continues it, and
        # the meter's cursor has to survive it - if a status half comes back with no
        # leftover records on a working link, try a shorter gap first.
        cv.Optional(
            CONF_REQUEST_GAP, default="500ms"
        ): cv.positive_time_period_milliseconds,
        # Per on-air attempt. A good reply completes ~965 ms after transmit starts on
        # the reference meter, with real spread above it.
        cv.Optional(
            CONF_RF_RX_TIMEOUT, default="1800ms"
        ): cv.positive_time_period_milliseconds,
        # Total on-air attempts = 1 + rf_retries.
        cv.Optional(CONF_RF_RETRIES, default=2): cv.int_range(min=0, max=10),
        # CMT2300A frequency codes (1 code ~= 6.199 Hz); shifts the RX-half LO onto the
        # meter's reply carrier. The default is proven on hardware.
        cv.Optional(CONF_RX_CENTER_OFFSET, default=758): cv.int_range(
            min=-4000, max=4000
        ),
        cv.Optional(CONF_SOURCES, default=[SOURCE_LIST_B]): cv.All(
            cv.ensure_list(cv.one_of(*SOURCES, lower=True)), validate_sources
        ),
        # Extra reads once per cycle, logged in full and driving no entity. Reads only -
        # the control code is hard-wired, and this link also carries a relay command.
        cv.Optional(CONF_PROBE): cv.All(
            cv.ensure_list(PROBE_SCHEMA), cv.Length(min=1, max=8), validate_probes
        ),
    }
    # The meter's own display syncs about once an hour; polling much more often buys
    # little and risks colliding with the sync.
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

    sources = config[CONF_SOURCES]
    cg.add(
        var.set_sources(
            SOURCE_LIST_A in sources,
            SOURCE_LIST_B in sources,
            SOURCE_FIXED in sources,
        )
    )

    for probe in config.get(CONF_PROBE, []):
        cg.add(var.add_probe(probe[CONF_DI], probe[CONF_BODY]))
