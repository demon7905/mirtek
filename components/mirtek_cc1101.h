#pragma once

#include "esphome/core/component.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"

#include <array>
#include <cstdint>
#include <cstdio>
#include <vector>
#include <string>
#include <algorithm>

namespace esphome {
namespace mirtek_cc1101 {

class MirtekCC1101 : public PollingComponent, public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                                                      spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  void setup() override;
  void update() override;
  void dump_config() override;

  void set_cs_pin(GPIOPin *pin) { this->cs_pin_ = pin; }
  void set_gdo0_pin(GPIOPin *pin) { this->gdo0_pin_ = pin; }
  void set_meter_address(uint32_t value) { this->meter_address_ = static_cast<uint16_t>(value); }
  void set_force_three_phase(bool value) { this->force_three_phase_ = value; }

#define DECL_SENSOR(name) void set_##name##_sensor(sensor::Sensor *s) { this->name##_sensor_ = s; }
  DECL_SENSOR(energy_sum)
  DECL_SENSOR(energy_t1)
  DECL_SENSOR(energy_t2)
  DECL_SENSOR(energy_t3)
  DECL_SENSOR(power_active)
  DECL_SENSOR(power_reactive)
  DECL_SENSOR(frequency)
  DECL_SENSOR(power_factor)
  DECL_SENSOR(voltage_1)
  DECL_SENSOR(voltage_2)
  DECL_SENSOR(voltage_3)
  DECL_SENSOR(current_1)
  DECL_SENSOR(current_2)
  DECL_SENSOR(current_3)
  DECL_SENSOR(power_a)
  DECL_SENSOR(power_b)
  DECL_SENSOR(power_c)
  DECL_SENSOR(reactive_a)
  DECL_SENSOR(reactive_b)
  DECL_SENSOR(reactive_c)
  DECL_SENSOR(apparent_a)
  DECL_SENSOR(apparent_b)
  DECL_SENSOR(apparent_c)
  DECL_SENSOR(power_factor_a)
  DECL_SENSOR(power_factor_b)
  DECL_SENSOR(power_factor_c)
  DECL_SENSOR(temperature)
  DECL_SENSOR(battery_percent)
  DECL_SENSOR(battery_voltage)
  DECL_SENSOR(rssi)
#undef DECL_SENSOR

#define DECL_TEXT(name) void set_##name##_sensor(text_sensor::TextSensor *s) { this->name##_sensor_ = s; }
  DECL_TEXT(tariff)
  DECL_TEXT(relay_state)
  DECL_TEXT(seal_state)
  DECL_TEXT(meter_type)
  DECL_TEXT(meter_date)
  DECL_TEXT(meter_time)
  DECL_TEXT(status)
  DECL_TEXT(battery_state)
  DECL_TEXT(firmware_version)
  DECL_TEXT(device_id)
  DECL_TEXT(uptime)
#undef DECL_TEXT

#define DECL_BINARY(name) void set_##name##_sensor(binary_sensor::BinarySensor *s) { this->name##_sensor_ = s; }
  DECL_BINARY(three_phase)
  DECL_BINARY(relay_on)
  DECL_BINARY(seal_ok)
  DECL_BINARY(cc1101_ok)
#undef DECL_BINARY

  // Public actions for YAML template buttons.
  void poll_all();
  void relay_on();
  void relay_off();

 protected:
  static constexpr const char *TAG = "mirtek_cc1101";

  static constexpr std::array<uint8_t, 47> RF_CFG = {
      0x0D, 0x2E, 0x06, 0x4F, 0xD3, 0x91, 0x3C, 0x00, 0x41, 0x00, 0x16, 0x0F, 0x00, 0x10, 0x8B, 0x54,
      0xD9, 0x83, 0x13, 0xD2, 0xAA, 0x31, 0x07, 0x0C, 0x08, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87, 0x6B,
      0xF8, 0x56, 0x10, 0xE9, 0x2A, 0x00, 0x1F, 0x41, 0x00, 0x59, 0x59, 0x3F, 0x81, 0x35, 0x09};

  uint16_t meter_address_{0};
  bool force_three_phase_{false};
  bool three_phase_{false};

  GPIOPin *cs_pin_{nullptr};
  GPIOPin *gdo0_pin_{nullptr};

  sensor::Sensor *energy_sum_sensor_{nullptr};
  sensor::Sensor *energy_t1_sensor_{nullptr};
  sensor::Sensor *energy_t2_sensor_{nullptr};
  sensor::Sensor *energy_t3_sensor_{nullptr};
  sensor::Sensor *power_active_sensor_{nullptr};
  sensor::Sensor *power_reactive_sensor_{nullptr};
  sensor::Sensor *frequency_sensor_{nullptr};
  sensor::Sensor *power_factor_sensor_{nullptr};
  sensor::Sensor *voltage_1_sensor_{nullptr};
  sensor::Sensor *voltage_2_sensor_{nullptr};
  sensor::Sensor *voltage_3_sensor_{nullptr};
  sensor::Sensor *current_1_sensor_{nullptr};
  sensor::Sensor *current_2_sensor_{nullptr};
  sensor::Sensor *current_3_sensor_{nullptr};
  sensor::Sensor *power_a_sensor_{nullptr};
  sensor::Sensor *power_b_sensor_{nullptr};
  sensor::Sensor *power_c_sensor_{nullptr};
  sensor::Sensor *reactive_a_sensor_{nullptr};
  sensor::Sensor *reactive_b_sensor_{nullptr};
  sensor::Sensor *reactive_c_sensor_{nullptr};
  sensor::Sensor *apparent_a_sensor_{nullptr};
  sensor::Sensor *apparent_b_sensor_{nullptr};
  sensor::Sensor *apparent_c_sensor_{nullptr};
  sensor::Sensor *power_factor_a_sensor_{nullptr};
  sensor::Sensor *power_factor_b_sensor_{nullptr};
  sensor::Sensor *power_factor_c_sensor_{nullptr};
  sensor::Sensor *temperature_sensor_{nullptr};
  sensor::Sensor *battery_percent_sensor_{nullptr};
  sensor::Sensor *battery_voltage_sensor_{nullptr};
  sensor::Sensor *rssi_sensor_{nullptr};

  text_sensor::TextSensor *tariff_sensor_{nullptr};
  text_sensor::TextSensor *relay_state_sensor_{nullptr};
  text_sensor::TextSensor *seal_state_sensor_{nullptr};
  text_sensor::TextSensor *meter_type_sensor_{nullptr};
  text_sensor::TextSensor *meter_date_sensor_{nullptr};
  text_sensor::TextSensor *meter_time_sensor_{nullptr};
  text_sensor::TextSensor *status_sensor_{nullptr};
  text_sensor::TextSensor *battery_state_sensor_{nullptr};
  text_sensor::TextSensor *firmware_version_sensor_{nullptr};
  text_sensor::TextSensor *device_id_sensor_{nullptr};
  text_sensor::TextSensor *uptime_sensor_{nullptr};

  binary_sensor::BinarySensor *three_phase_sensor_{nullptr};
  binary_sensor::BinarySensor *relay_on_sensor_{nullptr};
  binary_sensor::BinarySensor *seal_ok_sensor_{nullptr};
  binary_sensor::BinarySensor *cc1101_ok_sensor_{nullptr};

  std::vector<uint8_t> rx_;

  static uint8_t crc8_(const uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
      crc ^= data[i];
      for (uint8_t b = 0; b < 8; b++) {
        crc = (crc & 0x80) ? static_cast<uint8_t>((crc << 1) ^ 0xA9) : static_cast<uint8_t>(crc << 1);
      }
    }
    return crc;
  }

  static std::vector<uint8_t> stuff_(const std::vector<uint8_t> &body) {
    std::vector<uint8_t> out;
    if (body.size() < 4) return out;
    out.reserve(body.size() + 8);
    out.push_back(body[0]);            // length placeholder
    out.push_back(body[1]);            // 0x73
    out.push_back(body[2]);            // 0x55
    for (size_t i = 3; i + 1 < body.size(); i++) {
      const uint8_t b = body[i];
      if (b == 0x55) {
        out.push_back(0x73);
        out.push_back(0x11);
      } else if (b == 0x73) {
        out.push_back(0x73);
        out.push_back(0x22);
      } else {
        out.push_back(b);
      }
    }
    out.push_back(body.back());        // terminator 0x55 unstuffed
    out[0] = static_cast<uint8_t>(out.size() - 1);
    return out;
  }

  static void destuff_(std::vector<uint8_t> &data) {
    std::vector<uint8_t> out;
    out.reserve(data.size());
    for (size_t i = 0; i < data.size(); i++) {
      if (data[i] == 0x73 && i + 1 < data.size()) {
        if (data[i + 1] == 0x11) {
          out.push_back(0x55);
          i++;
          continue;
        }
        if (data[i + 1] == 0x22) {
          out.push_back(0x73);
          i++;
          continue;
        }
      }
      out.push_back(data[i]);
    }
    data.swap(out);
  }

  bool cc_init_();
  void cc_strobe_(uint8_t command);
  uint8_t cc_read_(uint8_t addr);
  void cc_write_(uint8_t addr, uint8_t value);
  void cc_burst_write_(uint8_t addr, const uint8_t *data, size_t len);
  bool wait_gdo_high_(uint32_t timeout_ms);
  bool do_cmd_(uint8_t cmd, int c14, int c15, uint8_t expected_packets);

  static std::vector<uint8_t> make_short_(uint16_t address, uint8_t comm) {
    std::vector<uint8_t> body = {0, 0x73, 0x55, 0x20, 0x00,
                                 static_cast<uint8_t>(address & 0xFF), static_cast<uint8_t>(address >> 8),
                                 0xFE, 0xFF, comm, 0x00, 0x00, 0x00, 0x00, 0x00, 0x55};
    body[14] = crc8_(body.data() + 3, body.size() - 5);
    return stuff_(body);
  }

  static std::vector<uint8_t> make_long_(uint16_t address, uint8_t comm, uint8_t c14) {
    std::vector<uint8_t> body = {0, 0x73, 0x55, 0x21, 0x00,
                                 static_cast<uint8_t>(address & 0xFF), static_cast<uint8_t>(address >> 8),
                                 0xFE, 0xFF, comm, 0x00, 0x00, 0x00, 0x00, c14, 0x00, 0x55};
    body[15] = crc8_(body.data() + 3, body.size() - 5);
    return stuff_(body);
  }

  static std::vector<uint8_t> make_long2_(uint16_t address, uint8_t comm, uint8_t c14, uint8_t c15) {
    std::vector<uint8_t> body = {0, 0x73, 0x55, 0x22, 0x00,
                                 static_cast<uint8_t>(address & 0xFF), static_cast<uint8_t>(address >> 8),
                                 0xFE, 0xFF, comm, 0x00, 0x00, 0x00, 0x00, c14, c15, 0x00, 0x55};
    body[16] = crc8_(body.data() + 3, body.size() - 5);
    return stuff_(body);
  }

  bool check_common_(uint8_t cmd, size_t min_len, bool require_crc = true) const;
  uint8_t rx_crc_() const { return (rx_.size() >= 4) ? crc8_(rx_.data() + 2, rx_.size() - 4) : 0xFF; }
  bool addr_ok_() const {
    return rx_.size() > 7 && rx_[6] == static_cast<uint8_t>(meter_address_ & 0xFF) &&
           rx_[7] == static_cast<uint8_t>(meter_address_ >> 8);
  }

  void parse_datetime_();
  void parse_energy_();
  void parse_instant_();
  void parse_phase_();
  void parse_status_();

  static uint32_t u32le_(size_t i, const std::vector<uint8_t> &v) {
    return static_cast<uint32_t>(v[i]) | (static_cast<uint32_t>(v[i + 1]) << 8) |
           (static_cast<uint32_t>(v[i + 2]) << 16) | (static_cast<uint32_t>(v[i + 3]) << 24);
  }
  static uint32_t u24le_(size_t i, const std::vector<uint8_t> &v) {
    return static_cast<uint32_t>(v[i]) | (static_cast<uint32_t>(v[i + 1]) << 8) |
           (static_cast<uint32_t>(v[i + 2]) << 16);
  }
  static int16_t signed_mirtek16_(uint8_t lo, uint8_t hi) {
    if (hi >= 128) return static_cast<int16_t>(-(static_cast<int>(lo) | ((static_cast<int>(hi) - 128) << 8)));
    return static_cast<int16_t>(static_cast<int>(lo) | (static_cast<int>(hi) << 8));
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome

#ifdef MIRTEK_CC1101_IMPLEMENTATION
namespace esphome {
namespace mirtek_cc1101 {

void MirtekCC1101::setup() {
  this->spi_setup();
  if (this->gdo0_pin_) this->gdo0_pin_->setup();
  bool ok = this->cc_init_();
  if (this->cc1101_ok_sensor_) this->cc1101_ok_sensor_->publish_state(ok);
  ESP_LOGCONFIG(TAG, "Meter address: %u", this->meter_address_);
  ESP_LOGCONFIG(TAG, "CC1101 init: %s", ok ? "OK" : "ERROR");
  }

void MirtekCC1101::update() { this->poll_all(); }

void MirtekCC1101::dump_config() {
  ESP_LOGCONFIG(TAG, "Mirtek CC1101 (Demon protocol adapted to Alecseyyy transport):");
  ESP_LOGCONFIG(TAG, "  Address: %u", this->meter_address_);
  ESP_LOGCONFIG(TAG, "  Force 3-phase: %s", YESNO(this->force_three_phase_));
}

uint8_t MirtekCC1101::cc_read_(uint8_t addr) {
  this->enable();
  this->write_byte(addr | 0x80);
  uint8_t v = this->read_byte();
  this->disable();
  return v;
}

void MirtekCC1101::cc_write_(uint8_t addr, uint8_t value) {
  this->enable();
  this->write_byte(addr & 0x3F);
  this->write_byte(value);
  this->disable();
}

void MirtekCC1101::cc_burst_write_(uint8_t addr, const uint8_t *data, size_t len) {
  this->enable();
  this->write_byte(addr | 0x40);
  for (size_t i = 0; i < len; i++) this->write_byte(data[i]);
  this->disable();
}

void MirtekCC1101::cc_strobe_(uint8_t command) {
  this->enable();
  this->write_byte(command);
  this->disable();
}

bool MirtekCC1101::cc_init_() {
  cc_strobe_(0x30);  // SRES
  delay(2);
  const uint8_t version = cc_read_(0x31);
  if (version != 0x14 && version != 0x04) {
    ESP_LOGE(TAG, "Unexpected CC1101 version 0x%02X", version);
    return false;
  }
  cc_burst_write_(0x00, RF_CFG.data(), RF_CFG.size());
  cc_strobe_(0x33);  // SCAL
  delay(2);
  cc_strobe_(0x3A);  // SFRX
  cc_strobe_(0x3B);  // SFTX
  cc_strobe_(0x34);  // SRX
  delay(2);
  ESP_LOGI(TAG, "CC1101 version 0x%02X initialized", version);
  return true;
}

bool MirtekCC1101::wait_gdo_high_(uint32_t timeout_ms) {
  const uint32_t start = millis();
  while (millis() - start < timeout_ms) {
    if (this->gdo0_pin_ && this->gdo0_pin_->digital_read()) return true;
    yield();
  }
  return false;
}

bool MirtekCC1101::do_cmd_(uint8_t cmd, int c14, int c15, uint8_t expected_packets) {
  std::vector<uint8_t> tx;
  if (c15 >= 0)
    tx = make_long2_(meter_address_, cmd, static_cast<uint8_t>(c14), static_cast<uint8_t>(c15));
  else if (c14 >= 0)
    tx = make_long_(meter_address_, cmd, static_cast<uint8_t>(c14));
  else
    tx = make_short_(meter_address_, cmd);

  cc_strobe_(0x3B);  // SFTX
  this->enable();
  this->write_byte(0x7F | 0x00);  // TX FIFO via burst
  for (auto b : tx) this->write_byte(b);
  this->disable();
  cc_strobe_(0x35);  // STX

  std::vector<uint8_t> raw;
  raw.reserve(expected_packets * 64);
  bool got = false;

  for (uint8_t packet = 0; packet < expected_packets; packet++) {
    if (!wait_gdo_high_(1800)) break;
    this->enable();
    this->write_byte(0xBF);  // RX FIFO burst read
    const uint8_t count = this->read_byte();
    for (uint8_t i = 0; i + 1 < count; i++) raw.push_back(this->read_byte());  // skip length byte
    this->disable();
    got = true;
    cc_strobe_(0x3A);  // SFRX
    cc_strobe_(0x34);  // SRX
    delay(2);
  }

  cc_strobe_(0x34);
  rx_ = std::move(raw);
  destuff_(rx_);

  if (!got || rx_.size() < 2) return false;
  if (rx_[0] != 0x73 || rx_[1] != 0x55) return false;
  if (!addr_ok_()) return false;
  return true;
}

bool MirtekCC1101::check_common_(uint8_t cmd, size_t min_len, bool require_crc) const {
  if (rx_.size() < min_len) return false;
  if (rx_[0] != 0x73 || rx_[1] != 0x55) return false;
  if (rx_[6] != static_cast<uint8_t>(meter_address_ & 0xFF) || rx_[7] != static_cast<uint8_t>(meter_address_ >> 8)) return false;
  if (rx_[8] != cmd) return false;
  if (require_crc) {
    if (rx_.size() < 4) return false;
    const size_t crc_idx = rx_.size() - 2;
    if (rx_[crc_idx] != this->rx_crc_() || rx_[crc_idx + 1] != 0x55) return false;
  } else if (rx_.back() != 0x55) {
    return false;
  }
  return true;
}

void MirtekCC1101::parse_datetime_() {
  const bool ok = check_common_(0x1C, 22, true) && rx_[20] == this->rx_crc_() && rx_[21] == 0x55;
  if (!ok) return;

  three_phase_ = force_three_phase_ || (rx_[9] == 0xA8);
  if (three_phase_sensor_) three_phase_sensor_->publish_state(three_phase_);
  if (meter_type_sensor_) {
    if (rx_[9] == 0xA8) meter_type_sensor_->publish_state("Счетчик 3ф трансформаторный активно-реактивный");
    else if (rx_[9] == 0x98) meter_type_sensor_->publish_state("Счетчик 1ф 2х элементный активно-реактивный");
    else meter_type_sensor_->publish_state("Неизвестный тип");
  }

  char date_buf[16];
  char time_buf[16];
  std::snprintf(date_buf, sizeof(date_buf), "%02u.%02u.%u", rx_[17], rx_[18], rx_[19]);
  std::snprintf(time_buf, sizeof(time_buf), "%02u:%02u:%02u", rx_[15], rx_[14], rx_[13]);
  if (meter_date_sensor_) meter_date_sensor_->publish_state(date_buf);
  if (meter_time_sensor_) meter_time_sensor_->publish_state(time_buf);
}

void MirtekCC1101::parse_energy_() {
  if (!check_common_(0x05, 45, true) || rx_[43] != this->rx_crc_() || rx_[44] != 0x55) return;
  if (energy_sum_sensor_) energy_sum_sensor_->publish_state(static_cast<float>(u32le_(19, rx_)) / 100.0f);
  if (energy_t1_sensor_) energy_t1_sensor_->publish_state(static_cast<float>(u32le_(27, rx_)) / 100.0f);
  if (energy_t2_sensor_) energy_t2_sensor_->publish_state(static_cast<float>(u32le_(31, rx_)) / 100.0f);

  switch ((rx_[14] >> 2) & 0x03) {
    case 0: if (tariff_sensor_) tariff_sensor_->publish_state("День"); break;
    case 1: if (tariff_sensor_) tariff_sensor_->publish_state("Ночь"); break;
    case 2: if (tariff_sensor_) tariff_sensor_->publish_state("Полупик"); break;
    default: if (tariff_sensor_) tariff_sensor_->publish_state("Специальный"); break;
  }
}

void MirtekCC1101::parse_instant_() {
  if (three_phase_) {
    if (!check_common_(0x2B, 45, true) || rx_[43] != this->rx_crc_() || rx_[44] != 0x55) return;
    if (power_active_sensor_) power_active_sensor_->publish_state(static_cast<float>(u24le_(18, rx_))); // W, no /1000
    if (power_reactive_sensor_) {
      const int16_t signq = signed_mirtek16_(rx_[21], rx_[22]);
      const float q = (rx_[23] >= 128) ? static_cast<float>(-(static_cast<int>(rx_[21]) | ((static_cast<int>(rx_[23]) - 128) << 8))) / 1000.0f
                                       : static_cast<float>(rx_[21] | (rx_[22] << 8)) / 1000.0f;
      (void) signq;
      power_reactive_sensor_->publish_state(q);
    }
    if (frequency_sensor_) frequency_sensor_->publish_state(static_cast<float>(rx_[24] | (rx_[25] << 8)) / 100.0f);
    if (power_factor_sensor_) {
      const int16_t cf = signed_mirtek16_(rx_[26], rx_[27]);
      power_factor_sensor_->publish_state(static_cast<float>(cf) / 1000.0f);
    }
    if (voltage_1_sensor_) voltage_1_sensor_->publish_state(static_cast<float>(rx_[28] | (rx_[29] << 8)) / 100.0f);
    if (voltage_2_sensor_) voltage_2_sensor_->publish_state(static_cast<float>(rx_[30] | (rx_[31] << 8)) / 100.0f);
    if (voltage_3_sensor_) voltage_3_sensor_->publish_state(static_cast<float>(rx_[32] | (rx_[33] << 8)) / 100.0f);
    if (current_1_sensor_) current_1_sensor_->publish_state(static_cast<float>(u24le_(34, rx_)) / 1000.0f);
    if (current_2_sensor_) current_2_sensor_->publish_state(static_cast<float>(u24le_(37, rx_)) / 1000.0f);
    if (current_3_sensor_) current_3_sensor_->publish_state(static_cast<float>(u24le_(40, rx_)) / 1000.0f);
  } else {
    if (rx_.size() < 41 || rx_[0] != 0x73 || rx_[1] != 0x55 || !addr_ok_() || rx_[8] != 0x2B) return;
    bool ok = false;
    if (rx_.size() >= 44 && rx_[43] == 0x55) {
      ok = true;  // known newer 44-byte 1ph response where CRC calculation differs in original
    } else if (rx_.size() >= 43 && rx_[41] == this->rx_crc_() && rx_[42] == 0x55) {
      ok = true;
    }
    if (!ok) return;
    if (power_active_sensor_) power_active_sensor_->publish_state(static_cast<float>(rx_[18] | (rx_[19] << 8)));
    if (power_reactive_sensor_) {
      const float q = (rx_[21] >= 128) ? static_cast<float>(-(static_cast<int>(rx_[20]) | ((static_cast<int>(rx_[21]) - 128) << 8))) / 1000.0f
                                       : static_cast<float>(rx_[20] | (rx_[21] << 8)) / 1000.0f;
      power_reactive_sensor_->publish_state(q);
    }
    if (frequency_sensor_) frequency_sensor_->publish_state(static_cast<float>(rx_[22] | (rx_[23] << 8)) / 100.0f);
    if (power_factor_sensor_) power_factor_sensor_->publish_state(static_cast<float>(signed_mirtek16_(rx_[24], rx_[25])) / 1000.0f);
    if (voltage_1_sensor_) voltage_1_sensor_->publish_state(static_cast<float>(rx_[26] | (rx_[27] << 8)) / 100.0f);
    if (current_1_sensor_) current_1_sensor_->publish_state(static_cast<float>(u24le_(32, rx_)) / 1000.0f);
  }
}

void MirtekCC1101::parse_phase_() {
  if (!check_common_(0x2B, 45, true) || rx_[43] != this->rx_crc_() || rx_[44] != 0x55) return;
  auto signed16f = [&](size_t i) -> float {
    return static_cast<float>(signed_mirtek16_(rx_[i], rx_[i + 1])) / 1000.0f;
  };
  if (power_factor_a_sensor_) power_factor_a_sensor_->publish_state(signed16f(18));
  if (power_factor_b_sensor_) power_factor_b_sensor_->publish_state(signed16f(20));
  if (power_factor_c_sensor_) power_factor_c_sensor_->publish_state(signed16f(22));
  if (power_a_sensor_) power_a_sensor_->publish_state(static_cast<float>(rx_[24] | (rx_[25] << 8)));
  if (power_b_sensor_) power_b_sensor_->publish_state(static_cast<float>(rx_[26] | (rx_[27] << 8)));
  if (power_c_sensor_) power_c_sensor_->publish_state(static_cast<float>(rx_[28] | (rx_[29] << 8)));
  if (reactive_a_sensor_) reactive_a_sensor_->publish_state(signed16f(30));
  if (reactive_b_sensor_) reactive_b_sensor_->publish_state(signed16f(32));
  if (reactive_c_sensor_) reactive_c_sensor_->publish_state(signed16f(34));
  if (apparent_a_sensor_) apparent_a_sensor_->publish_state(static_cast<float>(rx_[36] | (rx_[37] << 8)));
  if (apparent_b_sensor_) apparent_b_sensor_->publish_state(static_cast<float>(rx_[38] | (rx_[39] << 8)));
  if (apparent_c_sensor_) apparent_c_sensor_->publish_state(static_cast<float>(rx_[40] | (rx_[41] << 8)));
  if (temperature_sensor_) {
    const float t = (rx_[42] >= 128) ? -static_cast<float>(rx_[42] - 128) : static_cast<float>(rx_[42]);
    temperature_sensor_->publish_state(t);
  }
}

void MirtekCC1101::parse_status_() {
  if (!check_common_(0x10, 34, true) || rx_[32] != this->rx_crc_() || rx_[33] != 0x55) return;
  std::string seal;
  switch (rx_[28]) {
    case 0: seal = "OK"; break;
    case 1: seal = "Вскрыта пломба клемника"; break;
    case 2: seal = "Вскрыта пломба корпуса"; break;
    case 3: seal = "Вскрыта пломба клемника+корпуса"; break;
    default: seal = "Неизвестно"; break;
  }
  if (seal_state_sensor_) seal_state_sensor_->publish_state(seal);
  const bool seal_ok = rx_[28] == 0;
  if (seal_ok_sensor_) seal_ok_sensor_->publish_state(seal_ok);

  const uint8_t rele_bits = (rx_[24] >> 2) & 0x03;
  if (rele_bits == 0 || rele_bits == 1) {
    const bool on = rele_bits == 0;
    if (relay_state_sensor_) relay_state_sensor_->publish_state(on ? "Вкл" : "Выкл");
    if (relay_on_sensor_) relay_on_sensor_->publish_state(on);
  }
  if (status_sensor_) status_sensor_->publish_state("Ok");
}

void MirtekCC1101::poll_all() {
  if (!do_cmd_(0x1C, -1, -1, 3)) {
    if (status_sensor_) status_sensor_->publish_state("Error: 1C");
    return;
  }
  parse_datetime_();

  if (!do_cmd_(0x05, 0x00, -1, 4)) {
    if (status_sensor_) status_sensor_->publish_state("Error: 05");
    return;
  }
  parse_energy_();

  if (!do_cmd_(0x2B, 0x00, -1, 4)) {
    if (status_sensor_) status_sensor_->publish_state("Error: 2B/00");
    return;
  }
  parse_instant_();

  if (three_phase_) {
    if (!do_cmd_(0x2B, 0x10, -1, 4)) {
      if (status_sensor_) status_sensor_->publish_state("Error: 2B/10");
      return;
    }
    parse_phase_();
  }

  if (!do_cmd_(0x10, -1, -1, 3)) {
    if (status_sensor_) status_sensor_->publish_state("Error: 10");
    return;
  }
  parse_status_();
}

void MirtekCC1101::relay_on() {
  if (do_cmd_(0x3A, 0x00, 0x00, 4)) {
    delay(500);
    this->poll_all();
  }
}

void MirtekCC1101::relay_off() {
  if (do_cmd_(0x3A, 0x00, 0x01, 4)) {
    delay(500);
    this->poll_all();
  }
}

}  // namespace mirtek_cc1101
}  // namespace esphome
#endif
