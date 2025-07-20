import esphome.codegen as cg
from esphome.components import binary_sensor, sensor
import esphome.config_validation as cv
from esphome.const import (
    CONF_BATTERY_LEVEL,
    CONF_ID,
    CONF_NAME,
    DEVICE_CLASS_BATTERY,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_WEIGHT,
    ICON_BATTERY,
    ICON_BLUETOOTH,
    ICON_SCALE,
    ICON_THERMOMETER,
    STATE_CLASS_MEASUREMENT,
    UNIT_CELSIUS,
    UNIT_KILOGRAM,
    UNIT_PERCENT,
)

DEPENDENCIES = ["binary_sensor", "sensor"]
AUTO_LOAD = ["binary_sensor", "sensor"]

CONF_SYNCING = "syncing"
CONF_WEIGHT = "weight"
CONF_TEMPERATURE = "temperature_sensor"
CONF_REF_TEMPERATURE = "reference_temperature_sensor"
CONF_STDDEV = "standard_deviation"
CONF_LED_PIN = "led_pin"

wii_balance_board_ns = cg.esphome_ns.namespace("wii_balance_board")
WiiBalanceBoard = wii_balance_board_ns.class_("WiiBalanceBoard", cg.Component)

CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(WiiBalanceBoard),
        cv.Optional(CONF_LED_PIN, default=-1): cv.int_,
        cv.Optional(
            CONF_TEMPERATURE,
            default={
                CONF_NAME: "Balance Board Temperature",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(
            CONF_REF_TEMPERATURE,
            default={
                CONF_NAME: "Balance Board Reference Temperature",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_CELSIUS,
            icon=ICON_THERMOMETER,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_TEMPERATURE,
        ),
        cv.Optional(
            CONF_BATTERY_LEVEL,
            default={
                CONF_NAME: "Balance Board Battery Level",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_PERCENT,
            icon=ICON_BATTERY,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_BATTERY,
        ),
        cv.Optional(
            CONF_WEIGHT,
            default={
                CONF_NAME: "Balance Board Weight",
            },
        ): sensor.sensor_schema(
            unit_of_measurement=UNIT_KILOGRAM,
            icon=ICON_SCALE,
            accuracy_decimals=1,
            state_class=STATE_CLASS_MEASUREMENT,
            device_class=DEVICE_CLASS_WEIGHT,
        ),
        cv.Optional(
            CONF_SYNCING,
            default={
                CONF_NAME: "Balance Board Syncing",
            },
        ): binary_sensor.binary_sensor_schema(
            icon=ICON_BLUETOOTH,
        ),
        cv.Optional(CONF_STDDEV, default=0.4): cv.float_range(0, 5),
    }
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])

    await cg.register_component(var, config)

    temperature_sensor = await sensor.new_sensor(config.get(CONF_TEMPERATURE))
    cg.add(var.set_temperature_sensor(temperature_sensor))

    reference_temperature_sensor = await sensor.new_sensor(
        config.get(CONF_REF_TEMPERATURE)
    )
    cg.add(var.set_reference_temperature_sensor(reference_temperature_sensor))

    battery_level = await sensor.new_sensor(config.get(CONF_BATTERY_LEVEL))
    cg.add(var.set_battery_level(battery_level))

    weight = await sensor.new_sensor(config.get(CONF_WEIGHT))
    cg.add(var.set_weight(weight))

    syncing = await binary_sensor.new_binary_sensor(config.get(CONF_SYNCING))
    cg.add(var.set_syncing(syncing))

    if led_pin := config.get(CONF_LED_PIN):
        cg.add(var.set_led_pin(led_pin))

    if stddev := config.get(CONF_STDDEV):
        cg.add(var.set_stddev(stddev))
