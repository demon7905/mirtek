"""
МИРТЕК-32-РУ / CC1101 external component for ESPHome.
Порт рабочего My_Mirtek_Demon.ino автора demon7905.
"""
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import spi, sensor, text_sensor, binary_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_VOLTAGE,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_FREQUENCY,
    DEVICE_CLASS_POWER_FACTOR,
    DEVICE_CLASS_TEMPERATURE,
    DEVICE_CLASS_CONNECTIVITY,
    DEVICE_CLASS_SAFETY,
    STATE_CLASS_MEASUREMENT,
    STATE_CLASS_TOTAL_INCREASING,
    UNIT_KILOWATT_HOURS,
    UNIT_KILOVOLT_AMPS_REACTIVE,
    UNIT_KILOVOLT_AMPS_REACTIVE_HOURS,
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_HERTZ,
    UNIT_CELSIUS,
    UNIT_PERCENT,
    UNIT_VOLT_AMPS,
)

DEPENDENCIES = ["spi", "sensor", "text_sensor", "binary_sensor"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

CONF_GDO0_PIN = "gdo0_pin"
CONF_METER_ADDRESS = "meter_address"

SENSOR_KEYS = [
    "energy_sum", "energy_t1", "energy_t2",
    "power_active", "power_reactive", "frequency", "power_factor",
    "voltage_1", "voltage_2", "voltage_3",
    "current_1", "current_2", "current_3",
    "power_a", "power_b", "power_c",
    "reactive_a", "reactive_b", "reactive_c",
    "apparent_a", "apparent_b", "apparent_c",
    "pf_a", "pf_b", "pf_c", "temperature",
]
TEXT_KEYS = [
    "tariff", "meter_type", "meter_date", "meter_time", "last_response",
]
BINARY_KEYS = ["three_phase", "cc1101_ok"]

mirtek_ns = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101 = mirtek_ns.class_("MirtekCC1101", cg.PollingComponent, spi.SPIDevice)


def sensor_schema(unit, decimals, device_class=None, state_class=STATE_CLASS_MEASUREMENT):
    kwargs = {
        "unit_of_measurement": unit,
        "accuracy_decimals": decimals,
        "state_class": state_class,
    }
    if device_class:
        kwargs["device_class"] = device_class
    return sensor.sensor_schema(**kwargs)


def energy_schema():
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        icon="mdi:flash",
    )


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MirtekCC1101),
            cv.Required(CONF_GDO0_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_METER_ADDRESS): cv.int_range(min=1, max=65000),

            cv.Optional("energy_sum"): energy_schema(),
            cv.Optional("energy_t1"): energy_schema(),
            cv.Optional("energy_t2"): energy_schema(),

            cv.Optional("power_active"): sensor_schema(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional("power_reactive"): sensor_schema(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional("frequency"): sensor_schema(UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY),
            cv.Optional("power_factor"): sensor_schema("", 3, DEVICE_CLASS_POWER_FACTOR),

            cv.Optional("voltage_1"): sensor_schema(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional("voltage_2"): sensor_schema(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional("voltage_3"): sensor_schema(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional("current_1"): sensor_schema(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            cv.Optional("current_2"): sensor_schema(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            cv.Optional("current_3"): sensor_schema(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),

            cv.Optional("power_a"): sensor_schema(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional("power_b"): sensor_schema(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional("power_c"): sensor_schema(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional("reactive_a"): sensor_schema(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional("reactive_b"): sensor_schema(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional("reactive_c"): sensor_schema(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional("apparent_a"): sensor_schema(UNIT_VOLT_AMPS, 0),
            cv.Optional("apparent_b"): sensor_schema(UNIT_VOLT_AMPS, 0),
            cv.Optional("apparent_c"): sensor_schema(UNIT_VOLT_AMPS, 0),
            cv.Optional("pf_a"): sensor_schema("", 3),
            cv.Optional("pf_b"): sensor_schema("", 3),
            cv.Optional("pf_c"): sensor_schema("", 3),
            cv.Optional("temperature"): sensor_schema(UNIT_CELSIUS, 0, DEVICE_CLASS_TEMPERATURE),

            cv.Optional("tariff"): text_sensor.text_sensor_schema(),
            cv.Optional("meter_type"): text_sensor.text_sensor_schema(),
            cv.Optional("meter_date"): text_sensor.text_sensor_schema(),
            cv.Optional("meter_time"): text_sensor.text_sensor_schema(),
            cv.Optional("last_response"): text_sensor.text_sensor_schema(),

            cv.Optional("three_phase"): binary_sensor.binary_sensor_schema(),
            cv.Optional("cc1101_ok"): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_CONNECTIVITY),
        }
    )
    .extend(spi.spi_device_schema(cs_pin_required=True))
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    gdo0 = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0))
    cg.add(var.set_meter_address(config[CONF_METER_ADDRESS]))

    for idx, key in enumerate(SENSOR_KEYS):
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(var.set_sensor(idx, sens))

    for idx, key in enumerate(TEXT_KEYS):
        if key in config:
            ts = await text_sensor.new_text_sensor(config[key])
            cg.add(var.set_text_sensor(idx, ts))

    for idx, key in enumerate(BINARY_KEYS):
        if key in config:
            bs = await binary_sensor.new_binary_sensor(config[key])
            cg.add(var.set_binary_sensor(idx, bs))
