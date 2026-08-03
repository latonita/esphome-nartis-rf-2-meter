"""Nartis RF-2 meter - numeric sensor platform.

Each entity selects either an energy register from DI 0xF200 by its item `tag`,
or a numeric field of the DI 0xF201 `status` block. Exactly one of the two.
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
            # Value width for a TAG the decoder does not know. Only meaningful
            # alongside `tag`.
            cv.Optional(CONF_BYTES): cv.int_range(min=1, max=TAG_MAX_WIDTH),
        }
    ),
    cv.has_exactly_one_key(CONF_TAG, CONF_STATUS),
    validate_tag_entity,
)


async def to_code(config):
    parent = await cg.get_variable(config[CONF_NARTIS_RF_2_METER_ID])
    var = await sensor.new_sensor(config)

    # tag is meaningless for a status field and vice versa; the C++ side keys off
    # the status enum being NONE.
    tag = config.get(CONF_TAG, 0)
    field = config.get(CONF_STATUS, StatusField.NONE)

    cg.add(parent.register_sensor(var, tag, field, config.get(CONF_BYTES, 0)))
