import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import binary_sensor, sensor, text_sensor
from esphome.const import (
    CONF_ID,
    DEVICE_CLASS_CURRENT,
    DEVICE_CLASS_ENERGY,
    DEVICE_CLASS_POWER,
    DEVICE_CLASS_VOLTAGE,
    ICON_ALERT,
    ICON_BATTERY,
    ICON_CHIP,
    ICON_COUNTER,
    ICON_CURRENT_AC,
    ICON_FLASH,
    ICON_GAUGE,
    ICON_POWER,
    ICON_SIGNAL,
    ICON_THERMOMETER,
    ICON_TIMER,
    ICON_TRANSMISSION_TOWER,
    ICON_WIFI,
    UNIT_AMPERE,
    UNIT_CELSIUS,
    UNIT_HERTZ,
    UNIT_KILOWATT_HOURS,
    UNIT_PERCENT,
    UNIT_VOLT,
    UNIT_WATT,
)
from esphome.core import CORE
from esphome.components import spi

CODEOWNERS = ["@Alecseyyy"]

mirtek_ns = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101 = mirtek_ns.class_(
    "MirtekCC1101",
    cg.PollingComponent,
    spi.SPIDevice,
)

CONF_CS_PIN = "cs_pin"
CONF_GDO0_PIN = "gdo0_pin"
CONF_METER_ADDRESS = "meter_address"
CONF_FORCE_THREE_PHASE = "force_three_phase"

CONF_ENERGY_SUM = "energy_sum"
CONF_ENERGY_T1 = "energy_t1"
CONF_ENERGY_T2 = "energy_t2"
CONF_ENERGY_T3 = "energy_t3"
CONF_POWER_ACTIVE = "power_active"
CONF_POWER_REACTIVE = "power_reactive"
CONF_FREQUENCY = "frequency"
CONF_POWER_FACTOR = "power_factor"
CONF_VOLTAGE_1 = "voltage_1"
CONF_VOLTAGE_2 = "voltage_2"
CONF_VOLTAGE_3 = "voltage_3"
CONF_CURRENT_1 = "current_1"
CONF_CURRENT_2 = "current_2"
CONF_CURRENT_3 = "current_3"
CONF_POWER_A = "power_a"
CONF_POWER_B = "power_b"
CONF_POWER_C = "power_c"
CONF_REACTIVE_A = "reactive_a"
CONF_REACTIVE_B = "reactive_b"
CONF_REACTIVE_C = "reactive_c"
CONF_APPARENT_A = "apparent_a"
CONF_APPARENT_B = "apparent_b"
CONF_APPARENT_C = "apparent_c"
CONF_POWER_FACTOR_A = "power_factor_a"
CONF_POWER_FACTOR_B = "power_factor_b"
CONF_POWER_FACTOR_C = "power_factor_c"
CONF_TEMPERATURE = "temperature"

CONF_TARIFF = "tariff"
CONF_RELAY_STATE = "relay_state"
CONF_SEAL_STATE = "seal_state"
CONF_METER_TYPE = "meter_type"
CONF_METER_DATE = "meter_date"
CONF_METER_TIME = "meter_time"
CONF_STATUS = "status"
CONF_BATTERY_STATE = "battery_state"
CONF_BATTERY_PERCENT = "battery_percent"
CONF_BATTERY_VOLTAGE = "battery_voltage"
CONF_FIRMWARE_VERSION = "firmware_version"
CONF_DEVICE_ID = "device_id"
CONF_UPTIME = "uptime"
CONF_RSSI = "rssi"

CONF_THREE_PHASE = "three_phase"
CONF_RELAY_ON = "relay_on"
CONF_SEAL_OK = "seal_ok"
CONF_CC1101_OK = "cc1101_ok"

SENSOR_TYPES = {
    CONF_ENERGY_SUM: sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        device_class=DEVICE_CLASS_ENERGY,
        state_class="total_increasing",
        accuracy_decimals=2,
        icon=ICON_COUNTER,
    ),
    CONF_ENERGY_T1: sensor.sensor_schema(unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY, state_class="total_increasing", accuracy_decimals=2),
    CONF_ENERGY_T2: sensor.sensor_schema(unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY, state_class="total_increasing", accuracy_decimals=2),
    CONF_ENERGY_T3: sensor.sensor_schema(unit_of_measurement=UNIT_KILOWATT_HOURS, device_class=DEVICE_CLASS_ENERGY, state_class="total_increasing", accuracy_decimals=2),
    CONF_POWER_ACTIVE: sensor.sensor_schema(unit_of_measurement=UNIT_WATT, device_class=DEVICE_CLASS_POWER, accuracy_decimals=0, icon=ICON_POWER),
    CONF_POWER_REACTIVE: sensor.sensor_schema(unit_of_measurement="var", accuracy_decimals=0, icon=ICON_POWER),
    CONF_FREQUENCY: sensor.sensor_schema(unit_of_measurement=UNIT_HERTZ, accuracy_decimals=2),
    CONF_POWER_FACTOR: sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=3),
    CONF_VOLTAGE_1: sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, accuracy_decimals=2),
    CONF_VOLTAGE_2: sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, accuracy_decimals=2),
    CONF_VOLTAGE_3: sensor.sensor_schema(unit_of_measurement=UNIT_VOLT, device_class=DEVICE_CLASS_VOLTAGE, accuracy_decimals=2),
    CONF_CURRENT_1: sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, accuracy_decimals=3, icon=ICON_CURRENT_AC),
    CONF_CURRENT_2: sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, accuracy_decimals=3, icon=ICON_CURRENT_AC),
    CONF_CURRENT_3: sensor.sensor_schema(unit_of_measurement=UNIT_AMPERE, device_class=DEVICE_CLASS_CURRENT, accuracy_decimals=3, icon=ICON_CURRENT_AC),
    CONF_POWER_A: sensor.sensor_schema(unit_of_measurement=UNIT_WATT, device_class=DEVICE_CLASS_POWER, accuracy_decimals=0),
    CONF_POWER_B: sensor.sensor_schema(unit_of_measurement=UNIT_WATT, device_class=DEVICE_CLASS_POWER, accuracy_decimals=0),
    CONF_POWER_C: sensor.sensor_schema(unit_of_measurement=UNIT_WATT, device_class=DEVICE_CLASS_POWER, accuracy_decimals=0),
    CONF_REACTIVE_A: sensor.sensor_schema(unit_of_measurement="var", accuracy_decimals=3),
    CONF_REACTIVE_B: sensor.sensor_schema(unit_of_measurement="var", accuracy_decimals=3),
    CONF_REACTIVE_C: sensor.sensor_schema(unit_of_measurement="var", accuracy_decimals=3),
    CONF_APPARENT_A: sensor.sensor_schema(unit_of_measurement="VA", accuracy_decimals=0),
    CONF_APPARENT_B: sensor.sensor_schema(unit_of_measurement="VA", accuracy_decimals=0),
    CONF_APPARENT_C: sensor.sensor_schema(unit_of_measurement="VA", accuracy_decimals=0),
    CONF_POWER_FACTOR_A: sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=3),
    CONF_POWER_FACTOR_B: sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=3),
    CONF_POWER_FACTOR_C: sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=3),
    CONF_TEMPERATURE: sensor.sensor_schema(unit_of_measurement=UNIT_CELSIUS, accuracy_decimals=0, device_class="temperature", icon=ICON_THERMOMETER),
    CONF_BATTERY_PERCENT: sensor.sensor_schema(unit_of_measurement=UNIT_PERCENT, accuracy_decimals=0, icon=ICON_BATTERY),
    CONF_BATTERY_VOLTAGE: sensor.sensor_schema(unit_of_measurement="V", accuracy_decimals=3, icon=ICON_BATTERY),
    CONF_RSSI: sensor.sensor_schema(unit_of_measurement="dBm", accuracy_decimals=0, icon=ICON_SIGNAL),
}

TEXT_TYPES = {
    CONF_TARIFF: text_sensor.text_sensor_schema(icon=ICON_FLASH),
    CONF_RELAY_STATE: text_sensor.text_sensor_schema(icon=ICON_POWER),
    CONF_SEAL_STATE: text_sensor.text_sensor_schema(icon=ICON_ALERT),
    CONF_METER_TYPE: text_sensor.text_sensor_schema(icon=ICON_COUNTER),
    CONF_METER_DATE: text_sensor.text_sensor_schema(icon=ICON_TIMER),
    CONF_METER_TIME: text_sensor.text_sensor_schema(icon=ICON_TIMER),
    CONF_STATUS: text_sensor.text_sensor_schema(icon=ICON_SIGNAL),
    CONF_BATTERY_STATE: text_sensor.text_sensor_schema(icon=ICON_BATTERY),
    CONF_FIRMWARE_VERSION: text_sensor.text_sensor_schema(icon=ICON_CHIP),
    CONF_DEVICE_ID: text_sensor.text_sensor_schema(icon=ICON_CHIP),
    CONF_UPTIME: text_sensor.text_sensor_schema(icon=ICON_TIMER),
}

BINARY_TYPES = {
    CONF_THREE_PHASE: binary_sensor.binary_sensor_schema(),
    CONF_RELAY_ON: binary_sensor.binary_sensor_schema(device_class="power"),
    CONF_SEAL_OK: binary_sensor.binary_sensor_schema(),
    CONF_CC1101_OK: binary_sensor.binary_sensor_schema(device_class="connectivity"),
}

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MirtekCC1101),
            cv.Required(CONF_CS_PIN): spi.gpio_cs_pin_schema,
            cv.Required(CONF_GDO0_PIN): cv.internal_gpio_input_pin_schema,
            cv.Required(CONF_METER_ADDRESS): cv.positive_int,
            cv.Optional(CONF_FORCE_THREE_PHASE, default=False): cv.boolean,
            cv.Optional(CONF_ENERGY_SUM): SENSOR_TYPES[CONF_ENERGY_SUM],
            cv.Optional(CONF_ENERGY_T1): SENSOR_TYPES[CONF_ENERGY_T1],
            cv.Optional(CONF_ENERGY_T2): SENSOR_TYPES[CONF_ENERGY_T2],
            cv.Optional(CONF_ENERGY_T3): SENSOR_TYPES[CONF_ENERGY_T3],
            cv.Optional(CONF_POWER_ACTIVE): SENSOR_TYPES[CONF_POWER_ACTIVE],
            cv.Optional(CONF_POWER_REACTIVE): SENSOR_TYPES[CONF_POWER_REACTIVE],
            cv.Optional(CONF_FREQUENCY): SENSOR_TYPES[CONF_FREQUENCY],
            cv.Optional(CONF_POWER_FACTOR): SENSOR_TYPES[CONF_POWER_FACTOR],
            cv.Optional(CONF_VOLTAGE_1): SENSOR_TYPES[CONF_VOLTAGE_1],
            cv.Optional(CONF_VOLTAGE_2): SENSOR_TYPES[CONF_VOLTAGE_2],
            cv.Optional(CONF_VOLTAGE_3): SENSOR_TYPES[CONF_VOLTAGE_3],
            cv.Optional(CONF_CURRENT_1): SENSOR_TYPES[CONF_CURRENT_1],
            cv.Optional(CONF_CURRENT_2): SENSOR_TYPES[CONF_CURRENT_2],
            cv.Optional(CONF_CURRENT_3): SENSOR_TYPES[CONF_CURRENT_3],
            cv.Optional(CONF_POWER_A): SENSOR_TYPES[CONF_POWER_A],
            cv.Optional(CONF_POWER_B): SENSOR_TYPES[CONF_POWER_B],
            cv.Optional(CONF_POWER_C): SENSOR_TYPES[CONF_POWER_C],
            cv.Optional(CONF_REACTIVE_A): SENSOR_TYPES[CONF_REACTIVE_A],
            cv.Optional(CONF_REACTIVE_B): SENSOR_TYPES[CONF_REACTIVE_B],
            cv.Optional(CONF_REACTIVE_C): SENSOR_TYPES[CONF_REACTIVE_C],
            cv.Optional(CONF_APPARENT_A): SENSOR_TYPES[CONF_APPARENT_A],
            cv.Optional(CONF_APPARENT_B): SENSOR_TYPES[CONF_APPARENT_B],
            cv.Optional(CONF_APPARENT_C): SENSOR_TYPES[CONF_APPARENT_C],
            cv.Optional(CONF_POWER_FACTOR_A): SENSOR_TYPES[CONF_POWER_FACTOR_A],
            cv.Optional(CONF_POWER_FACTOR_B): SENSOR_TYPES[CONF_POWER_FACTOR_B],
            cv.Optional(CONF_POWER_FACTOR_C): SENSOR_TYPES[CONF_POWER_FACTOR_C],
            cv.Optional(CONF_TEMPERATURE): SENSOR_TYPES[CONF_TEMPERATURE],
            cv.Optional(CONF_BATTERY_PERCENT): SENSOR_TYPES[CONF_BATTERY_PERCENT],
            cv.Optional(CONF_BATTERY_VOLTAGE): SENSOR_TYPES[CONF_BATTERY_VOLTAGE],
            cv.Optional(CONF_RSSI): SENSOR_TYPES[CONF_RSSI],
            cv.Optional(CONF_TARIFF): TEXT_TYPES[CONF_TARIFF],
            cv.Optional(CONF_RELAY_STATE): TEXT_TYPES[CONF_RELAY_STATE],
            cv.Optional(CONF_SEAL_STATE): TEXT_TYPES[CONF_SEAL_STATE],
            cv.Optional(CONF_METER_TYPE): TEXT_TYPES[CONF_METER_TYPE],
            cv.Optional(CONF_METER_DATE): TEXT_TYPES[CONF_METER_DATE],
            cv.Optional(CONF_METER_TIME): TEXT_TYPES[CONF_METER_TIME],
            cv.Optional(CONF_STATUS): TEXT_TYPES[CONF_STATUS],
            cv.Optional(CONF_BATTERY_STATE): TEXT_TYPES[CONF_BATTERY_STATE],
            cv.Optional(CONF_FIRMWARE_VERSION): TEXT_TYPES[CONF_FIRMWARE_VERSION],
            cv.Optional(CONF_DEVICE_ID): TEXT_TYPES[CONF_DEVICE_ID],
            cv.Optional(CONF_UPTIME): TEXT_TYPES[CONF_UPTIME],
            cv.Optional(CONF_THREE_PHASE): BINARY_TYPES[CONF_THREE_PHASE],
            cv.Optional(CONF_RELAY_ON): BINARY_TYPES[CONF_RELAY_ON],
            cv.Optional(CONF_SEAL_OK): BINARY_TYPES[CONF_SEAL_OK],
            cv.Optional(CONF_CC1101_OK): BINARY_TYPES[CONF_CC1101_OK],
        }
    ).extend(cv.polling_component_schema("60s")).extend(spi.spi_device_schema("4MHz"))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    cs = await cg.gpio_pin_expression(config[CONF_CS_PIN])
    gdo0 = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_cs_pin(cs))
    cg.add(var.set_gdo0_pin(gdo0))
    cg.add(var.set_meter_address(config[CONF_METER_ADDRESS]))
    cg.add(var.set_force_three_phase(config[CONF_FORCE_THREE_PHASE]))

    for key in SENSOR_TYPES:
        if key in config:
            sens = await sensor.new_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(sens))

    for key in TEXT_TYPES:
        if key in config:
            ts = await text_sensor.new_text_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(ts))

    for key in BINARY_TYPES:
        if key in config:
            bs = await binary_sensor.new_binary_sensor(config[key])
            cg.add(getattr(var, f"set_{key}_sensor")(bs))
