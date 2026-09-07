"""
Mirtek CC1101 ESPHome External Component — МИРТЕК-32-РУ
=========================================================
Протокол и байтовые смещения взяты из проверенного рабочего скетча
My_Mirtek_Demon.ino (github.com/demon7905/mirtek), а не из документации
"Star v1.20" — у МИРТЕК-32-РУ формат ОТВЕТА счётчика отличается от
Mirtek STAR 104/304 (см. комментарии в mirtek_cc1101.h).

Архитектура компонента (SPIDevice, byte-stuffing, схема конфигурации)
взята за основу от https://github.com/Alecseyyy/ESPHome-Mirt-830
Совместимость: ESPHome 2026.8.x, ESP32, Arduino framework
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
    UNIT_VOLT,
    UNIT_AMPERE,
    UNIT_WATT,
    UNIT_HERTZ,
    UNIT_CELSIUS,
    UNIT_EMPTY,
    UNIT_KILOVOLT_AMPS_REACTIVE,
    UNIT_VOLT_AMPS,
    ICON_FLASH,
    ICON_THERMOMETER,
)

CODEOWNERS = []

DEPENDENCIES = ["spi", "sensor", "text_sensor", "binary_sensor"]
AUTO_LOAD = ["sensor", "text_sensor", "binary_sensor"]

# ── Ключи конфигурации ──────────────────────────────────────────────────────
CONF_GDO0_PIN = "gdo0_pin"
CONF_METER_ADDR = "meter_address"

# Энергия (нарастающим итогом)
CONF_S_SUM = "energy_sum"
CONF_S_T1 = "energy_t1"
CONF_S_T2 = "energy_t2"

# Мгновенные суммарные
CONF_S_KW = "power_active"
CONF_S_KVAR = "power_reactive"
CONF_S_FREQ = "frequency"
CONF_S_COS = "power_factor"

# Напряжение / ток
CONF_S_V1 = "voltage_1"
CONF_S_V2 = "voltage_2"
CONF_S_V3 = "voltage_3"
CONF_S_I1 = "current_1"
CONF_S_I2 = "current_2"
CONF_S_I3 = "current_3"

# По фазам (только 3-фазный счётчик, команда 0x2B/0x10)
CONF_S_PA = "power_a"
CONF_S_PB = "power_b"
CONF_S_PC = "power_c"
CONF_S_QA = "reactive_a"
CONF_S_QB = "reactive_b"
CONF_S_QC = "reactive_c"
CONF_S_SA = "apparent_a"
CONF_S_SB = "apparent_b"
CONF_S_SC = "apparent_c"
CONF_S_CA = "pf_a"
CONF_S_CB = "pf_b"
CONF_S_CC = "pf_c"
CONF_S_TEMP = "temperature"

# Текстовые датчики
CONF_T_TARIFF = "tariff"
CONF_T_RELAY = "relay_state"
CONF_T_SEAL = "seal_state"
CONF_T_TYPE = "meter_type"
CONF_T_DATE = "meter_date"
CONF_T_TIME = "meter_time"
CONF_T_STAT = "status"

# Бинарные датчики
CONF_B_3PH = "three_phase"
CONF_B_RELAY = "relay_on"
CONF_B_SEAL = "seal_ok"
CONF_B_CC = "cc1101_ok"

# ── C++ класс ────────────────────────────────────────────────────────────────
mirtek_ns = cg.esphome_ns.namespace("mirtek_cc1101")
MirtekCC1101 = mirtek_ns.class_("MirtekCC1101", cg.PollingComponent, spi.SPIDevice)


def _sensor(unit, decimals, device_class=None, state_class=STATE_CLASS_MEASUREMENT, icon=None):
    kwargs = dict(
        unit_of_measurement=unit,
        accuracy_decimals=decimals,
        state_class=state_class,
    )
    if device_class:
        kwargs["device_class"] = device_class
    if icon:
        kwargs["icon"] = icon
    return sensor.sensor_schema(**kwargs)


def _energy_sensor(icon=ICON_FLASH):
    return sensor.sensor_schema(
        unit_of_measurement=UNIT_KILOWATT_HOURS,
        accuracy_decimals=2,
        device_class=DEVICE_CLASS_ENERGY,
        state_class=STATE_CLASS_TOTAL_INCREASING,
        icon=icon,
    )


CONFIG_SCHEMA = (
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(MirtekCC1101),
            cv.Required(CONF_GDO0_PIN): pins.gpio_input_pin_schema,
            cv.Required(CONF_METER_ADDR): cv.int_range(min=1, max=65000),
            # ── Энергия (кВт·ч) ──────────────────────────────────────────────
            cv.Optional(CONF_S_SUM): _energy_sensor(),
            cv.Optional(CONF_S_T1): _energy_sensor(icon="mdi:weather-sunny"),
            cv.Optional(CONF_S_T2): _energy_sensor(icon="mdi:weather-night"),
            # ── Мгновенные суммарные ─────────────────────────────────────────
            cv.Optional(CONF_S_KW): _sensor(UNIT_WATT, 0, DEVICE_CLASS_POWER, icon=ICON_FLASH),
            cv.Optional(CONF_S_KVAR): _sensor(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional(CONF_S_FREQ): _sensor(UNIT_HERTZ, 2, DEVICE_CLASS_FREQUENCY),
            cv.Optional(CONF_S_COS): _sensor(UNIT_EMPTY, 3, DEVICE_CLASS_POWER_FACTOR),
            # ── Напряжение / ток ─────────────────────────────────────────────
            cv.Optional(CONF_S_V1): _sensor(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional(CONF_S_V2): _sensor(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional(CONF_S_V3): _sensor(UNIT_VOLT, 1, DEVICE_CLASS_VOLTAGE),
            cv.Optional(CONF_S_I1): _sensor(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            cv.Optional(CONF_S_I2): _sensor(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            cv.Optional(CONF_S_I3): _sensor(UNIT_AMPERE, 3, DEVICE_CLASS_CURRENT),
            # ── По фазам (только 3ф, команда 0x2B/0x10) ──────────────────────
            cv.Optional(CONF_S_PA): _sensor(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional(CONF_S_PB): _sensor(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional(CONF_S_PC): _sensor(UNIT_WATT, 0, DEVICE_CLASS_POWER),
            cv.Optional(CONF_S_QA): _sensor(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional(CONF_S_QB): _sensor(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional(CONF_S_QC): _sensor(UNIT_KILOVOLT_AMPS_REACTIVE, 3),
            cv.Optional(CONF_S_SA): _sensor(UNIT_VOLT_AMPS, 0),
            cv.Optional(CONF_S_SB): _sensor(UNIT_VOLT_AMPS, 0),
            cv.Optional(CONF_S_SC): _sensor(UNIT_VOLT_AMPS, 0),
            cv.Optional(CONF_S_CA): _sensor(UNIT_EMPTY, 3),
            cv.Optional(CONF_S_CB): _sensor(UNIT_EMPTY, 3),
            cv.Optional(CONF_S_CC): _sensor(UNIT_EMPTY, 3),
            cv.Optional(CONF_S_TEMP): _sensor(UNIT_CELSIUS, 0, DEVICE_CLASS_TEMPERATURE, icon=ICON_THERMOMETER),
            # ── Текстовые датчики ─────────────────────────────────────────────
            cv.Optional(CONF_T_TARIFF): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_T_RELAY): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_T_SEAL): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_T_TYPE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_T_DATE): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_T_TIME): text_sensor.text_sensor_schema(),
            cv.Optional(CONF_T_STAT): text_sensor.text_sensor_schema(),
            # ── Бинарные датчики ──────────────────────────────────────────────
            cv.Optional(CONF_B_3PH): binary_sensor.binary_sensor_schema(),
            cv.Optional(CONF_B_RELAY): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_POWER),
            cv.Optional(CONF_B_SEAL): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_SAFETY),
            cv.Optional(CONF_B_CC): binary_sensor.binary_sensor_schema(device_class=DEVICE_CLASS_CONNECTIVITY),
        }
    )
    .extend(spi.spi_device_schema(cs_pin_required=True))
    .extend(cv.polling_component_schema("300s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await spi.register_spi_device(var, config)

    gdo0 = await cg.gpio_pin_expression(config[CONF_GDO0_PIN])
    cg.add(var.set_gdo0_pin(gdo0))
    cg.add(var.set_meter_address(config[CONF_METER_ADDR]))

    # Порядок должен совпадать с enum SensorIdx в mirtek_cc1101.h
    SENSORS = [
        CONF_S_SUM, CONF_S_T1, CONF_S_T2,
        CONF_S_KW, CONF_S_KVAR, CONF_S_FREQ, CONF_S_COS,
        CONF_S_V1, CONF_S_V2, CONF_S_V3,
        CONF_S_I1, CONF_S_I2, CONF_S_I3,
        CONF_S_PA, CONF_S_PB, CONF_S_PC,
        CONF_S_QA, CONF_S_QB, CONF_S_QC,
        CONF_S_SA, CONF_S_SB, CONF_S_SC,
        CONF_S_CA, CONF_S_CB, CONF_S_CC,
        CONF_S_TEMP,
    ]
    for idx, key in enumerate(SENSORS):
        if sensor_config := config.get(key):
            sens = await sensor.new_sensor(sensor_config)
            cg.add(var.set_sensor(idx, sens))

    # Порядок должен совпадать с enum TextIdx в mirtek_cc1101.h
    TEXT = [
        CONF_T_TARIFF, CONF_T_RELAY, CONF_T_SEAL, CONF_T_TYPE,
        CONF_T_DATE, CONF_T_TIME, CONF_T_STAT,
    ]
    for idx, key in enumerate(TEXT):
        if ts_config := config.get(key):
            ts = await text_sensor.new_text_sensor(ts_config)
            cg.add(var.set_text_sensor(idx, ts))

    # Порядок должен совпадать с enum BinIdx в mirtek_cc1101.h
    BINARY = [CONF_B_3PH, CONF_B_RELAY, CONF_B_SEAL, CONF_B_CC]
    for idx, key in enumerate(BINARY):
        if bs_config := config.get(key):
            bs = await binary_sensor.new_binary_sensor(bs_config)
            cg.add(var.set_binary_sensor(idx, bs))
