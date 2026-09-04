"""Nartis RF-2 meter - numeric sensor platform.

Each entity selects either a value by its item `tag` - from whichever configured
source carried it - or a numeric field of the `status` block. Exactly one of the
two.

The published state is already scaled to the unit `tags.md` gives for that TAG, so
set `unit_of_measurement` to match and do not add a `multiply` filter. A `tag`
read with a `bytes:` override is the exception: its unit is unknown, so it is
published raw and a filter is the only way to scale it.
"""

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor

from . import (
    CONF_BYTES,
    CONF_NARTIS_RF_2_METER_ID,
    CONF_STATUS,
    CONF_TAG,
    STATUS_FIELDS,
    TAG_MAX_WIDTH,
    NartisRf2MeterComponent,
    StatusField,
    validate_numeric_tag,
    validate_tag_entity,
)

DEPENDENCIES = ["nartis_rf_2_meter"]

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema().extend(
        {
            cv.GenerateID(CONF_NARTIS_RF_2_METER_ID): cv.use_id(
                NartisRf2MeterComponent
            ),
            cv.Optional(CONF_TAG): validate_numeric_tag,
            cv.Optional(CONF_STATUS): cv.enum(STATUS_FIELDS, lower=True),
            # Value width for a TAG the decoder does not know; needs `tag`.
            cv.Optional(CONF_BYTES): cv.int_range(min=1, max=TAG_MAX_WIDTH),
        }
    ),
    cv.has_exactly_one_key(CONF_TAG, CONF_STATUS),
    validate_tag_entity,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NARTIS_RF_2_METER_ID])
    var = await sensor.new_sensor(config)

    tag = config.get(CONF_TAG, 0)
    field = config.get(CONF_STATUS, StatusField.NONE)

    cg.add(parent.register_sensor(var, tag, field, config.get(CONF_BYTES, 0)))
