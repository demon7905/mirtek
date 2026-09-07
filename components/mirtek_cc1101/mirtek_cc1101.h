#pragma once

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include <vector>
#include <cstdio>
#include <cstring>
#include <algorithm>
#include <string>

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek";

// Exactly the CRC used by My_Mirtek_Demon.ino: polynomial 0xA9, MSB first.
static uint8_t mirtek_crc8(const uint8_t *data, size_t len) {
  uint8_t c = 0;
  for (size_t i = 0; i < len; i++) {
    uint8_t b = data[i];
    for (uint8_t j = 0; j < 8; j++) {
      c = ((b ^ c) & 0x80) ? static_cast<uint8_t>((c << 1) ^ 0xA9)
                           : static_cast<uint8_t>(c << 1);
      b <<= 1;
    }
  }
  return c;
}

// The proven 47-byte RF setup from My_Mirtek_Demon.ino.
static const uint8_t RF_CFG[47] = {
  0x0D, 0x2E, 0x06, 0x4F, 0xD3, 0x91, 0x3C, 0x00,
  0x41, 0x00, 0x16, 0x0F, 0x00, 0x10, 0x8B, 0x54,
  0xD9, 0x83, 0x13, 0xD2, 0xAA, 0x31, 0x07, 0x0C,
  0x08, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87, 0x6B,
  0xF8, 0x56, 0x10, 0xE9, 0x2A, 0x00, 0x1F, 0x41,
  0x00, 0x59, 0x59, 0x3F, 0x81, 0x35, 0x09
};

enum SensorIdx {
  SI_SUM = 0, SI_T1, SI_T2,
  SI_KW, SI_KVAR, SI_FREQ, SI_COS,
  SI_V1, SI_V2, SI_V3,
  SI_I1, SI_I2, SI_I3,
  SI_PA, SI_PB, SI_PC,
  SI_QA, SI_QB, SI_QC,
  SI_SA, SI_SB, SI_SC,
  SI_CA, SI_CB, SI_CC,
  SI_TEMP,
  SI_COUNT
};

enum TextIdx {
  TI_TARIFF = 0,
  TI_TYPE,
  TI_DATE,
  TI_TIME,
  TI_LAST_RESPONSE,
  TI_COUNT
};

enum BinIdx {
  BI_3PH = 0,
  BI_CC,
  BI_COUNT
};

static constexpr uint8_t CC_SRES = 0x30;
static constexpr uint8_t CC_SCAL = 0x33;
static constexpr uint8_t CC_SRX = 0x34;
static constexpr uint8_t CC_STX = 0x35;
static constexpr uint8_t CC_SIDLE = 0x36;
static constexpr uint8_t CC_SFRX = 0x3A;
static constexpr uint8_t CC_SFTX = 0x3B;
static constexpr uint8_t CC_TXFIFO = 0x3F;
static constexpr uint8_t CC_RXFIFO = 0x3F;
static constexpr uint8_t CC_RXBYTES = 0x3B;
static constexpr uint8_t CC_VERSION = 0x31;
static constexpr uint8_t CC_PA = 0x3E;

class MirtekCC1101 : public PollingComponent,
                     public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                           spi::CLOCK_POLARITY_LOW,
                                           spi::CLOCK_PHASE_LEADING,
                                           spi::DATA_RATE_4MHZ> {
 public:
  void set_gdo0_pin(GPIOPin *p) { gdo0_ = p; }
  void set_meter_address(int a) { address_ = static_cast<uint16_t>(a); }
  void set_sensor(int i, sensor::Sensor *s) {
    if (i >= 0 && i < SI_COUNT) sensors_[i] = s;
  }
  void set_text_sensor(int i, text_sensor::TextSensor *s) {
    if (i >= 0 && i < TI_COUNT) texts_[i] = s;
  }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s) {
    if (i >= 0 && i < BI_COUNT) bins_[i] = s;
  }

  void setup() override {
    this->spi_setup();
    if (gdo0_) gdo0_->setup();

    ESP_LOGI(TAG, "МИРТЕК-32-РУ: адрес=%u", address_);
    bool ok = cc_init_();
    publish_bin_(BI_CC, ok);
    if (!ok) {
      ESP_LOGE(TAG, "CC1101 не найден");
    } else {
      ESP_LOGI(TAG, "CC1101 готов; RF 433 MHz; протокол МИРТЕК");
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek-32-RU CC1101:");
    ESP_LOGCONFIG(TAG, "  Адрес счётчика: %u", address_);
    ESP_LOGCONFIG(TAG, "  Интервал опроса: %lu ms",
                  static_cast<unsigned long>(this->get_update_interval()));
    LOG_PIN("  GDO0: ", gdo0_);
  }

  void update() override {
    if (poll_active_) {
      ESP_LOGW(TAG, "Опрос уже выполняется, пропускаем новый UPDATE");
      return;
    }
    start_cycle_();
  }

  void loop() override {
    service_poll_();
  }

 protected:
  GPIOPin *gdo0_{nullptr};
  uint16_t address_{1};
  bool three_phase_{true};
  sensor::Sensor *sensors_[SI_COUNT]{};
  text_sensor::TextSensor *texts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bins_[BI_COUNT]{};

  // Working buffers mirror the Arduino implementation: one decoded frame up to 50 bytes.
  uint8_t tx_raw_[24]{};
  uint8_t raw_joined_[96]{};
  uint8_t result_[64]{};
  size_t raw_joined_len_{0};
  size_t result_len_{0};

  // Non-blocking poll state. The Arduino firmware waits in packetReceiver(),
  // but ESPHome must return from loop()/update() quickly to avoid the task WDT.
  bool poll_active_{false};
  uint8_t request_index_{0};
  uint8_t expected_packets_{0};
  uint8_t received_packets_{0};
  bool cycle_ok_{true};
  uint32_t request_deadline_{0};
  uint8_t rx_frame_[65]{};
  uint8_t rx_frame_len_{0};
  uint8_t rx_pos_{0};
  bool rx_started_{false};

  void publish_(int i, float v) {
    if (i >= 0 && i < SI_COUNT && sensors_[i]) sensors_[i]->publish_state(v);
  }
  void publish_txt_(int i, const std::string &v) {
    if (i >= 0 && i < TI_COUNT && texts_[i]) texts_[i]->publish_state(v);
  }
  void publish_bin_(int i, bool v) {
    if (i >= 0 && i < BI_COUNT && bins_[i]) bins_[i]->publish_state(v);
  }

  uint16_t u16_(size_t i) const {
    return static_cast<uint16_t>(result_[i]) |
           (static_cast<uint16_t>(result_[i + 1]) << 8);
  }
  uint32_t u24_(size_t i) const {
    return static_cast<uint32_t>(result_[i]) |
           (static_cast<uint32_t>(result_[i + 1]) << 8) |
           (static_cast<uint32_t>(result_[i + 2]) << 16);
  }
  uint32_t u32_(size_t i) const {
    return static_cast<uint32_t>(result_[i]) |
           (static_cast<uint32_t>(result_[i + 1]) << 8) |
           (static_cast<uint32_t>(result_[i + 2]) << 16) |
           (static_cast<uint32_t>(result_[i + 3]) << 24);
  }
  float signed16_mirtek_(size_t i, float div) const {
    bool neg = result_[i + 1] >= 128;
    uint16_t v = static_cast<uint16_t>(result_[i]) |
                 (static_cast<uint16_t>(result_[i + 1] & 0x7F) << 8);
    return (neg ? -1.0f : 1.0f) * (static_cast<float>(v) / div);
  }
  float signed24_mirtek_(size_t i, float div) const {
    bool neg = result_[i + 2] >= 128;
    uint32_t v = static_cast<uint32_t>(result_[i]) |
                 (static_cast<uint32_t>(result_[i + 1]) << 8) |
                 (static_cast<uint32_t>(result_[i + 2] & 0x7F) << 16);
    return (neg ? -1.0f : 1.0f) * (static_cast<float>(v) / div);
  }

  void enable_() { this->enable(); }
  void disable_() { this->disable(); }
  void strobe_(uint8_t cmd) {
    enable_();
    this->transfer_byte(cmd);
    disable_();
  }
  void write_reg_(uint8_t reg, uint8_t val) {
    enable_();
    this->transfer_byte(reg & 0x3F);
    this->transfer_byte(val);
    disable_();
  }
  void write_burst_(uint8_t reg, const uint8_t *data, size_t len) {
    enable_();
    this->transfer_byte((reg & 0x3F) | 0x40);
    for (size_t i = 0; i < len; i++) this->transfer_byte(data[i]);
    disable_();
  }
  uint8_t read_status_(uint8_t reg) {
    enable_();
    this->transfer_byte(0xC0 | (reg & 0x3F));
    uint8_t v = this->transfer_byte(0x00);
    disable_();
    return v;
  }

  bool cc_init_() {
    strobe_(CC_SRES);
    delay(10);
    write_burst_(0x00, RF_CFG, sizeof(RF_CFG));
    strobe_(CC_SCAL);
    delay(2);
    strobe_(CC_SFRX);
    strobe_(CC_SFTX);
    strobe_(CC_SRX);
    uint8_t ver = read_status_(CC_VERSION);
    ESP_LOGI(TAG, "CC1101 version=0x%02X", ver);
    return ver == 0x14 || ver == 0x04;
  }

  // Exact byte stuffing from My_Mirtek_Demon.ino.
  size_t stuff_packet_(const uint8_t *in, size_t n, uint8_t *out, size_t cap) {
    if (n < 4 || cap < n) return 0;
    size_t p = 0;
    out[p++] = in[0];
    out[p++] = in[1];
    out[p++] = in[2];
    for (size_t i = 3; i < n - 1; i++) {
      if (in[i] == 0x55) {
        if (p + 2 > cap) return 0;
        out[p++] = 0x73; out[p++] = 0x11;
      } else if (in[i] == 0x73) {
        if (p + 2 > cap) return 0;
        out[p++] = 0x73; out[p++] = 0x22;
      } else {
        if (p + 1 > cap) return 0;
        out[p++] = in[i];
      }
    }
    if (p + 1 > cap) return 0;
    out[p++] = in[n - 1];
    out[0] = static_cast<uint8_t>(p - 1);
    return p;
  }

  size_t build_request_(uint8_t command, int sub1, int sub2) {
    uint8_t data_len = (sub1 < 0) ? 0 : (sub2 < 0 ? 1 : 2);
    const uint8_t total_len = static_cast<uint8_t>(0x0F + data_len);

    tx_raw_[0] = total_len;
    tx_raw_[1] = 0x73;
    tx_raw_[2] = 0x55;
    tx_raw_[3] = static_cast<uint8_t>(0x20 + data_len);
    tx_raw_[4] = 0x00;
    tx_raw_[5] = static_cast<uint8_t>(address_ & 0xFF);
    tx_raw_[6] = static_cast<uint8_t>((address_ >> 8) & 0xFF);
    tx_raw_[7] = 0xFE;
    tx_raw_[8] = 0xFF;
    tx_raw_[9] = command;
    tx_raw_[10] = tx_raw_[11] = tx_raw_[12] = tx_raw_[13] = 0x00;

    size_t p = 14;
    if (sub1 >= 0) tx_raw_[p++] = static_cast<uint8_t>(sub1);
    if (sub2 >= 0) tx_raw_[p++] = static_cast<uint8_t>(sub2);
    tx_raw_[p] = mirtek_crc8(&tx_raw_[3], p - 3);
    tx_raw_[p + 1] = 0x55;
    return p + 2;
  }

  void send_packet_(size_t raw_len) {
    uint8_t tx[64]{};
    size_t tx_len = stuff_packet_(tx_raw_, raw_len, tx, sizeof(tx));
    if (!tx_len) {
      ESP_LOGE(TAG, "Не удалось сформировать TX packet");
      return;
    }

    // Exactly the sequence used by the known-working Arduino firmware.
    strobe_(CC_SCAL);
    delay(1);
    strobe_(CC_SFTX);
    strobe_(CC_SIDLE);
    write_reg_(CC_PA, 0xC4);

    enable_();
    this->transfer_byte(CC_TXFIFO | 0x40);
    for (size_t i = 0; i < tx_len; i++) this->transfer_byte(tx[i]);
    disable_();
    strobe_(CC_STX);

    ESP_LOGD(TAG, "TX len=%u", static_cast<unsigned>(tx_len));
    if (tx_len <= 64) {
      std::string hex;
      char tmp[4];
      for (size_t i = 1; i < tx_len; i++) {
        snprintf(tmp, sizeof(tmp), "%02X ", tx[i]);
        hex += tmp;
      }
      ESP_LOGVV(TAG, "TX RAW: %s", hex.c_str());
    }

    delay(2);
    strobe_(CC_SFRX);
    strobe_(CC_SRX);
  }

  bool read_fifo_byte_(uint8_t &v) {
    if ((read_status_(CC_RXBYTES) & 0x7F) == 0) return false;
    enable_();
    this->transfer_byte(CC_RXFIFO | 0x80 | 0x40);
    v = this->transfer_byte(0x00);
    disable_();
    return true;
  }

  void rearm_rx_() {
    strobe_(CC_SIDLE);
    strobe_(CC_SFRX);
    strobe_(CC_SFTX);
    strobe_(CC_SRX);
  }

  void dump_rx_packet_() {
    char hex[65 * 3 + 1]{};
    size_t p = 0;
    for (uint8_t i = 0; i < rx_frame_len_ && p + 3 < sizeof(hex); i++) {
      p += snprintf(hex + p, sizeof(hex) - p, "%02X ", rx_frame_[i]);
    }
    ESP_LOGVV(TAG, "RX RAW (%u): %s", rx_frame_len_, hex);
  }

  bool append_received_packet_() {
    // ELECHOUSE ReceiveData() gives buffer[0] as the packet length and returns
    // that length. The original Arduino code appends buffer[1]..buffer[len-1],
    // deliberately dropping the final byte of every RF sub-packet. Reproduce
    // that behavior exactly here.
    if (rx_frame_len_ < 3) return false;

    const size_t append_len = static_cast<size_t>(rx_frame_len_ - 1);
    if (raw_joined_len_ + append_len > sizeof(raw_joined_)) {
      ESP_LOGW(TAG, "Слишком большой объединённый RX буфер");
      return false;
    }
    for (size_t i = 1; i < rx_frame_len_; i++) {
      raw_joined_[raw_joined_len_++] = rx_frame_[i];
    }
    received_packets_++;
    ESP_LOGD(TAG, "RX подпакет %u/%u, %u байт", received_packets_,
             expected_packets_, rx_frame_len_ - 1);
    return true;
  }

  void reset_rx_packet_() {
    rx_frame_len_ = 0;
    rx_pos_ = 0;
    rx_started_ = false;
  }

  void service_rx_() {
    // Read the variable-length CC1101 packet incrementally, one or a few bytes
    // per ESPHome loop pass. No long blocking wait here.
    uint8_t available = read_status_(CC_RXBYTES) & 0x7F;
    if (available == 0) return;

    while (available > 0) {
      uint8_t b = 0;
      if (!read_fifo_byte_(b)) return;

      if (!rx_started_) {
        rx_started_ = true;
        rx_frame_[0] = b;
        rx_frame_len_ = 1;
        rx_pos_ = 1;
        if (b == 0 || b > 63) {
          ESP_LOGW(TAG, "Некорректная длина CC1101 RX: %u", b);
          reset_rx_packet_();
          rearm_rx_();
          return;
        }
      } else if (rx_pos_ < sizeof(rx_frame_)) {
        rx_frame_[rx_pos_++] = b;
        rx_frame_len_ = rx_pos_;
      } else {
        ESP_LOGW(TAG, "RX frame buffer переполнен");
        reset_rx_packet_();
        rearm_rx_();
        return;
      }

      available = read_status_(CC_RXBYTES) & 0x7F;
      if (rx_started_ && rx_pos_ >= static_cast<uint8_t>(rx_frame_[0] + 1)) {
        dump_rx_packet_();
        bool ok = append_received_packet_();
        reset_rx_packet_();
        rearm_rx_();
        if (!ok) {
          complete_request_(false);
          return;
        }
        if (received_packets_ >= expected_packets_) {
          complete_request_(true);
        }
        return;
      }
    }
  }

  bool verify_frame_(uint8_t command, size_t expected_len);

  void start_request_(uint8_t idx) {
    request_index_ = idx;
    received_packets_ = 0;
    raw_joined_len_ = 0;
    result_len_ = 0;
    reset_rx_packet_();

    uint8_t cmd = 0;
    int sub1 = -1;
    int sub2 = -1;
    expected_packets_ = 4;

    switch (request_index_) {
      case 0:
        cmd = 0x1C;
        expected_packets_ = 3;
        break;
      case 1:
        cmd = 0x05;
        sub1 = 0x00;
        expected_packets_ = 4;
        break;
      case 2:
        cmd = 0x2B;
        sub1 = 0x00;
        expected_packets_ = 4;
        break;
      case 3:
        cmd = 0x2B;
        sub1 = 0x10;
        expected_packets_ = 4;
        break;
      default:
        return;
    }

    size_t raw_len = build_request_(cmd, sub1, sub2);
    send_packet_(raw_len);
    request_deadline_ = millis() + 10000UL;
    poll_active_ = true;

    ESP_LOGI(TAG, "Запрос %u/4: cmd=0x%02X, ожидаю %u подпакета(ов)",
             static_cast<unsigned>(request_index_ + 1), cmd, expected_packets_);
  }

  void start_cycle_() {
    poll_active_ = false;
    cycle_ok_ = true;
    request_index_ = 0;
    ESP_LOGI(TAG, "=== Опрос МИРТЕК-32-РУ, адрес=%u ===", address_);
    start_request_(0);
  }

  void complete_request_(bool transport_ok) {
    if (!poll_active_) return;

    bool parsed_ok = false;
    if (transport_ok) {
      destuff_joined_();
      switch (request_index_) {
        case 0:
          parsed_ok = parse_datetime_();
          break;
        case 1:
          parsed_ok = parse_energy_();
          break;
        case 2:
          parsed_ok = parse_instant_3ph_();
          break;
        case 3:
          parsed_ok = parse_phase_power_();
          break;
        default:
          parsed_ok = false;
          break;
      }

      if (parsed_ok) {
        ESP_LOGI(TAG, "Запрос %u/4: OK", static_cast<unsigned>(request_index_ + 1));
      } else {
        ESP_LOGW(TAG, "Запрос %u/4: RX есть, но пакет не прошёл проверку/парсинг",
                 static_cast<unsigned>(request_index_ + 1));
      }
    } else {
      ESP_LOGW(TAG, "Запрос %u/4: таймаут/ошибка RX, подпакетов %u/%u",
               static_cast<unsigned>(request_index_ + 1), received_packets_, expected_packets_);
    }

    cycle_ok_ = cycle_ok_ && transport_ok && parsed_ok;
    poll_active_ = false;

    if (request_index_ < 3) {
      // Small gap between Mirtek requests, matching the Arduino's sequential
      // send/receive cycle while keeping ESPHome's loop responsive.
      request_index_++;
      poll_active_ = false;
      request_deadline_ = millis() + 100;
    } else {
      publish_txt_(TI_LAST_RESPONSE, cycle_ok_ ? "OK" : "PARTIAL");
      ESP_LOGI(TAG, "=== Опрос завершён: %s ===", cycle_ok_ ? "OK" : "PARTIAL");
    }
  }

  void service_poll_() {
    if (!poll_active_) {
      // request_index_ is advanced by complete_request_ and a short delay is
      // used as a non-blocking gap between sequential requests.
      if (request_index_ > 0 && request_index_ < 4 &&
          static_cast<int32_t>(millis() - request_deadline_) >= 0) {
        start_request_(request_index_);
      }
      return;
    }

    if (static_cast<int32_t>(millis() - request_deadline_) >= 0) {
      reset_rx_packet_();
      rearm_rx_();
      complete_request_(false);
      return;
    }

    service_rx_();
  }

  void destuff_joined_() {
    result_len_ = 0;
    for (size_t i = 0; i < raw_joined_len_ && result_len_ < sizeof(result_); i++) {
      uint8_t b = raw_joined_[i];
      if (b == 0x73 && i + 1 < raw_joined_len_) {
        uint8_t n = raw_joined_[i + 1];
        if (n == 0x11) {
          result_[result_len_++] = 0x55;
          i++;
          continue;
        }
        if (n == 0x22) {
          result_[result_len_++] = 0x73;
          i++;
          continue;
        }
      }
      result_[result_len_++] = b;
    }
  }

  bool verify_frame_(uint8_t command, size_t expected_len) {
    if (result_len_ < expected_len) return false;
    if (result_[0] != 0x73 || result_[1] != 0x55) return false;
    if (result_[6] != static_cast<uint8_t>(address_ & 0xFF) ||
        result_[7] != static_cast<uint8_t>((address_ >> 8) & 0xFF)) return false;
    if (result_[8] != command) return false;
    if (result_[result_len_ - 1] != 0x55) return false;

    uint8_t crc = mirtek_crc8(&result_[2], result_len_ - 4);
    if (crc != result_[result_len_ - 2]) {
      ESP_LOGW(TAG, "CRC ошибка cmd=0x%02X: calc=%02X recv=%02X", command, crc,
               result_[result_len_ - 2]);
      return false;
    }
    return true;
  }

  bool do_request_(uint8_t command, int sub1, int sub2, uint8_t packet_count) {
    size_t raw_len = build_request_(command, sub1, sub2);
    send_packet_(raw_len);

    raw_joined_len_ = 0;
    uint8_t pkt[64]{};
    uint8_t got = 0;
    uint32_t start = millis();

    while (millis() - start < 10000UL && got < packet_count) {
      size_t n = read_one_air_packet_(pkt, sizeof(pkt), 2500);
      if (!n) continue;
      // Arduino code deliberately skips the packet's length byte.
      for (size_t i = 0; i < n && raw_joined_len_ < sizeof(raw_joined_); i++) {
        raw_joined_[raw_joined_len_++] = pkt[i];
      }
      got++;
    }

    if (got != packet_count) {
      ESP_LOGW(TAG, "cmd=0x%02X: получено подпакетов %u/%u", command, got, packet_count);
      return false;
    }

    destuff_joined_();
    return result_len_ >= 4;
  }

  bool parse_datetime_() {
    // Exact indices from packetParser_1() in My_Mirtek_Demon.ino.
    if (!verify_frame_(0x1C, 22)) return false;

    char date[16];
    char time[16];
    snprintf(date, sizeof(date), "%02u-%02u-%02u", result_[17], result_[18], result_[19]);
    snprintf(time, sizeof(time), "%02u:%02u:%02u", result_[15], result_[14], result_[13]);
    publish_txt_(TI_DATE, date);
    publish_txt_(TI_TIME, time);

    switch (result_[9]) {
      case 0xA8:
        three_phase_ = true;
        publish_txt_(TI_TYPE, "3ф трансформаторный, активно-реактивный");
        break;
      case 0x98:
        three_phase_ = false;
        publish_txt_(TI_TYPE, "1ф, активно-реактивный");
        break;
      default:
        char unknown[16];
        snprintf(unknown, sizeof(unknown), "0x%02X", result_[9]);
        publish_txt_(TI_TYPE, unknown);
        break;
    }
    publish_bin_(BI_3PH, three_phase_);
    return true;
  }

  bool parse_energy_() {
    // Exact packetParser_2() positions in My_Mirtek_Demon.ino.
    if (!verify_frame_(0x05, 45)) return false;

    publish_(SI_SUM, static_cast<float>(u32_(19)) / 100.0f);
    publish_(SI_T1, static_cast<float>(u32_(27)) / 100.0f);
    publish_(SI_T2, static_cast<float>(u32_(31)) / 100.0f);

    const char *tariffs[] = {"День", "Ночь", "Полупик", "Специальный"};
    uint8_t t = static_cast<uint8_t>((result_[14] >> 2) & 0x03);
    publish_txt_(TI_TARIFF, tariffs[t]);
    return true;
  }

  bool parse_instant_3ph_() {
    // Exact packetParser_3() positions in My_Mirtek_Demon.ino.
    if (!verify_frame_(0x2B, 45)) return false;

    publish_(SI_KW, static_cast<float>(u24_(18)));
    publish_(SI_KVAR, signed24_mirtek_(21, 1000.0f));
    publish_(SI_FREQ, static_cast<float>(u16_(24)) / 100.0f);
    publish_(SI_COS, signed16_mirtek_(26, 1000.0f));
    publish_(SI_V1, static_cast<float>(u16_(28)) / 100.0f);
    publish_(SI_V2, static_cast<float>(u16_(30)) / 100.0f);
    publish_(SI_V3, static_cast<float>(u16_(32)) / 100.0f);
    publish_(SI_I1, static_cast<float>(u24_(34)) / 1000.0f);
    publish_(SI_I2, static_cast<float>(u24_(37)) / 1000.0f);
    publish_(SI_I3, static_cast<float>(u24_(40)) / 1000.0f);
    return true;
  }

  bool parse_phase_power_() {
    // Exact packetParser_4() positions in My_Mirtek_Demon.ino.
    if (!verify_frame_(0x2B, 45)) return false;

    publish_(SI_CA, signed16_mirtek_(18, 1000.0f));
    publish_(SI_CB, signed16_mirtek_(20, 1000.0f));
    publish_(SI_CC, signed16_mirtek_(22, 1000.0f));
    publish_(SI_PA, static_cast<float>(u16_(24)));
    publish_(SI_PB, static_cast<float>(u16_(26)));
    publish_(SI_PC, static_cast<float>(u16_(28)));
    publish_(SI_QA, signed16_mirtek_(30, 1000.0f));
    publish_(SI_QB, signed16_mirtek_(32, 1000.0f));
    publish_(SI_QC, signed16_mirtek_(34, 1000.0f));
    publish_(SI_SA, static_cast<float>(u16_(36)));
    publish_(SI_SB, static_cast<float>(u16_(38)));
    publish_(SI_SC, static_cast<float>(u16_(40)));

    float temp = (result_[42] >= 128) ? -static_cast<float>(result_[42] - 128)
                                      : static_cast<float>(result_[42]);
    publish_(SI_TEMP, temp);
    return true;
  }

};

}  // namespace mirtek_cc1101
}  // namespace esphome
