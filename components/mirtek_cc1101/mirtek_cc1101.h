#pragma once
// =============================================================================
//  Mirtek CC1101 ESPHome External Component — МИРТЕК-32-РУ
//  Совместимость: ESPHome 2026.8.x, ESP32, Arduino framework
//
//  ВАЖНО: это НЕ протокол "Star v1.20" (Mirtek STAR 104/304). Формат ЗАПРОСА
//  у этих счётчиков совпадает байт-в-байт, но формат ОТВЕТА — нет: у ответа
//  МИРТЕК-32-РУ на 2 байта длиннее заголовок перед адресом счётчика, из-за
//  чего смещения всех полей отличаются (не всегда ровно на 2 — где-то на 3,
//  см. дату/время). Все смещения ниже взяты не расчётом, а напрямую из
//  проверенного рабочего скетча My_Mirtek_Demon.ino
//  (https://github.com/demon7905/mirtek), byte-в-byte.
//
//  Архитектура компонента (SPIDevice, byte-stuffing, помпа TX/RX) взята за
//  основу от https://github.com/Alecseyyy/ESPHome-Mirt-830 — она не
//  протокол-специфична и не менялась. Изменено:
//   - формат ответа (смещения полей, делители, CRC-проверка) — везде
//     заменено на значения из My_Mirtek_Demon.ino
//   - в TX-последовательность добавлены SCAL (рекалибровка) и запись
//     PATABLE=0xC4 (мощность передачи) перед КАЖДОЙ отправкой — в оригинале
//     Alecseyyy этого не было, а в рабочем скетче это есть перед каждым
//     packetSender()
//   - команда 0x3A (реле) переделана на 2-байтовый субкоманд-формат
//     (sub1=0x00, sub2=0x00/0x01) и 4 ожидаемых пакета, как в рабочем
//     скетче (у Alecseyyy было 1 байт-субкоманда и 2 пакета — это формат
//     Star, а не 32-РУ)
//   - добавлена команда 0x04 (сброс электронных пломб) — в Star-варианте
//     её не было вовсе
//   - убраны непроверенные для 32-РУ поля (T3/T4 тарифы, реактивная
//     энергия по тарифам, батарея, серийный №, версия ПО, абонент) —
//     рабочий скетч их не читает, точные смещения для этой модели неизвестны
// =============================================================================

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

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek32ru";

// ─── CRC-8, полином 0xA9 (совпадает с CRC8.h из рабочего скетча) ────────────
static uint8_t crc8_mirtek(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    for (int j = 0; j < 8; j++) {
      c = ((b ^ c) & 0x80) ? (uint8_t) ((c << 1) ^ 0xA9) : (uint8_t) (c << 1);
      b <<= 1;
    }
  }
  return c;
}

// ─── CC1101 команды/регистры ─────────────────────────────────────────────────
static const uint8_t CC_SRES = 0x30;    // Software reset
static const uint8_t CC_SCAL = 0x33;    // Calibrate frequency synthesizer
static const uint8_t CC_SRX = 0x34;     // Enable RX
static const uint8_t CC_STX = 0x35;     // Enable TX
static const uint8_t CC_SIDLE = 0x36;   // Exit RX/TX
static const uint8_t CC_SFRX = 0x3A;    // Flush RX FIFO
static const uint8_t CC_SFTX = 0x3B;    // Flush TX FIFO
static const uint8_t CC_PATABLE = 0x3E; // Output power table
static const uint8_t CC_BURST = 0x40;   // Burst access bit
static const uint8_t CC_READ = 0x80;    // Read bit
static const uint8_t CC_TXFIFO = 0x3F;
static const uint8_t CC_RXFIFO = 0x3F;
static const uint8_t CC_RXBYTES = 0x3B;  // Status reg: bytes in RX FIFO
static const uint8_t CC_VERSION = 0x31;  // Status reg: chip version

// TX-мощность, выставляется перед каждой отправкой — как в packetSender()
// рабочего скетча ("выставляем мощность 10dB"). У Alecseyyy этой записи не
// было вовсе; отсутствие может быть причиной того, что счётчик не отвечал.
static const uint8_t CC_PATABLE_VALUE = 0xC4;

// ─── RF-конфигурация 433 МГц / FSK ───────────────────────────────────────────
// Идентична и в My_Mirtek_Demon.ino (rfSettings[]), и в ESPHome-Mirt-830
// (RF_CFG[]) — байт-в-байт. Радиочасть у этих счётчиков не отличается.
static const uint8_t RF_CFG[47] = {0x0D, 0x2E, 0x06, 0x4F, 0xD3, 0x91, 0x3C, 0x00, 0x41, 0x00, 0x16, 0x0F,
                                    0x00, 0x10, 0x8B, 0x54, 0xD9, 0x83, 0x13, 0xD2, 0xAA, 0x31, 0x07, 0x0C,
                                    0x08, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87, 0x6B, 0xF8, 0x56, 0x10, 0xE9,
                                    0x2A, 0x00, 0x1F, 0x41, 0x00, 0x59, 0x59, 0x3F, 0x81, 0x35, 0x09};

// ─── Индексы сенсоров (порядок должен совпадать со списком SENSORS в __init__.py) ──
enum SensorIdx {
  SI_SUM = 0, SI_T1, SI_T2,                  //  0- 2: активная энергия
  SI_KW, SI_KVAR, SI_FREQ, SI_COS,           //  3- 6: суммарные мгновенные
  SI_V1, SI_V2, SI_V3,                       //  7- 9: напряжения
  SI_I1, SI_I2, SI_I3,                       // 10-12: токи
  SI_PA, SI_PB, SI_PC,                       // 13-15: активная P по фазам
  SI_QA, SI_QB, SI_QC,                       // 16-18: реактивная Q по фазам
  SI_SA, SI_SB, SI_SC,                       // 19-21: полная S по фазам
  SI_CA, SI_CB, SI_CC,                       // 22-24: cos φ по фазам
  SI_TEMP,                                   // 25: температура счётчика
  SI_COUNT
};

enum TextIdx { TI_TARIFF = 0, TI_RELAY, TI_SEAL, TI_TYPE, TI_DATE, TI_TIME, TI_STATUS, TI_COUNT };

enum BinIdx { BI_3PH = 0, BI_RELAY, BI_SEAL, BI_CC, BI_COUNT };

// =============================================================================
class MirtekCC1101 : public PollingComponent,
                      public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                             spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  // ── Setters (вызываются из __init__.py → to_code) ──────────────────────────
  void set_gdo0_pin(GPIOPin *p) { gdo0_ = p; }
  void set_meter_address(int a) { addr_ = static_cast<uint16_t>(a); }

  void set_sensor(int i, sensor::Sensor *s) {
    if (i >= 0 && i < SI_COUNT) ss_[i] = s;
  }
  void set_text_sensor(int i, text_sensor::TextSensor *s) {
    if (i >= 0 && i < TI_COUNT) ts_[i] = s;
  }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s) {
    if (i >= 0 && i < BI_COUNT) bs_[i] = s;
  }

  // ── ESPHome lifecycle ───────────────────────────────────────────────────────
  void setup() override {
    ESP_LOGI(TAG, "Инициализация CC1101, адрес счётчика=%u", addr_);
    this->spi_setup();
    if (gdo0_) gdo0_->setup();

    bool ok = cc_init_();
    pub_bin_(BI_CC, ok);
    pub_txt_(TI_STATUS, ok ? "CC1101 OK" : "CC1101 ERR");

    if (!ok) {
      ESP_LOGE(TAG, "CC1101 не обнаружен! Проверьте подключение SPI.");
    } else {
      ESP_LOGI(TAG, "CC1101 готов. Интервал опроса: %u мс", get_update_interval());
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway (МИРТЕК-32-РУ):");
    ESP_LOGCONFIG(TAG, "  Адрес счётчика : %u", addr_);
    ESP_LOGCONFIG(TAG, "  Интервал опроса: %u мс", get_update_interval());
    LOG_PIN("  GDO0 пин: ", gdo0_);
  }

  void update() override { poll_all(); }

  // ── Публичные методы (доступны через HA Services / кнопки) ─────────────────
  // Последовательность запросов и их порядок повторяют главный цикл
  // My_Mirtek_Demon.ino (loop() → tmr_mqtt.tick()): 0x1C, 0x05, 0x2B/00,
  // [0x2B/10 если 3ф], 0x10.
  void poll_all() {
    ESP_LOGI(TAG, "=== Опрос, адрес=%u ===", addr_);

    bool ok1 = do_cmd_(0x1C, -1, -1, 3) && parse_datetime_();

    bool ok2 = do_cmd_(0x05, 0x00, -1, 4) && parse_energy_();

    bool ok3;
    if (three_phase_) {
      ok3 = do_cmd_(0x2B, 0x00, -1, 4) && parse_instant_3ph_();
      bool ok4 = do_cmd_(0x2B, 0x10, -1, 4) && parse_phase_();
      ok3 = ok3 && ok4;
    } else {
      ok3 = do_cmd_(0x2B, 0x00, -1, 4) && parse_instant_1ph_();
    }

    bool ok5 = do_cmd_(0x10, -1, -1, 3) && parse_status_();

    bool all_ok = ok1 && ok2 && ok3 && ok5;
    pub_txt_(TI_STATUS, all_ok ? "OK" : "PARTIAL");
    ESP_LOGI(TAG, "=== Опрос завершён: %s ===", all_ok ? "OK" : "PARTIAL");
  }

  // Управление реле отключения (команда 0x3A, 2-байтовая субкоманда).
  // sub2=0x00 — замкнуть (включить), sub2=0x01 — разомкнуть (выключить) —
  // как в case 7/8 рабочего скетча.
  void relay_on() {
    ESP_LOGI(TAG, "Реле: замкнуть (включить)");
    do_cmd_(0x3A, 0x00, 0x00, 4);
  }
  void relay_off() {
    ESP_LOGI(TAG, "Реле: разомкнуть (выключить)");
    do_cmd_(0x3A, 0x00, 0x01, 4);
  }

  // Сброс состояния электронных пломб (команда 0x04, субкоманда 0x01) —
  // как в case 9 рабочего скетча. У Star-протокола такой команды нет.
  void reset_seals() {
    ESP_LOGI(TAG, "Сброс состояния электронных пломб");
    do_cmd_(0x04, 0x01, -1, 4);
  }

 protected:
  GPIOPin *gdo0_{nullptr};
  uint16_t addr_{1};
  bool three_phase_{true};  // по умолчанию 3ф, как в My_Mirtek_Demon.ino; уточняется командой 0x1C

  sensor::Sensor *ss_[SI_COUNT]{};
  text_sensor::TextSensor *ts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bs_[BI_COUNT]{};

  uint8_t sbuf_[24]{};
  uint8_t rbuf_[64]{};
  size_t rlen_{0};

  // ── SPI helpers — всегда внутри enable() / disable() ────────────────────────
  void cc_strobe_(uint8_t cmd) {
    this->enable();
    this->transfer_byte(cmd);
    this->disable();
  }

  void cc_wreg_(uint8_t reg, uint8_t val) {
    this->enable();
    this->transfer_byte(reg & 0x3F);
    this->transfer_byte(val);
    this->disable();
  }

  void cc_wburst_(uint8_t reg, const uint8_t *data, size_t len) {
    this->enable();
    this->transfer_byte((reg & 0x3F) | CC_BURST);
    for (size_t i = 0; i < len; i++) this->transfer_byte(data[i]);
    this->disable();
  }

  uint8_t cc_rstat_(uint8_t reg) {
    this->enable();
    this->transfer_byte(CC_READ | CC_BURST | (reg & 0x3F));
    uint8_t v = this->transfer_byte(0x00);
    this->disable();
    return v;
  }

  // ── Инициализация CC1101 (программный SRES, без CS toggle) ──────────────────
  bool cc_init_() {
    cc_strobe_(CC_SRES);
    delay(10);

    cc_wburst_(0x00, RF_CFG, sizeof(RF_CFG));

    cc_strobe_(CC_SCAL);
    delay(2);
    cc_strobe_(CC_SFRX);
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SRX);

    uint8_t ver = cc_rstat_(CC_VERSION);
    ESP_LOGD(TAG, "CC1101 version=0x%02X", ver);
    return (ver == 0x14 || ver == 0x04);
  }

  // ── Byte stuffing (протокол Миртек — идентичен для запроса и ответа) ────────
  void stuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    out.push_back(in[0]);
    out.push_back(in[1]);
    out.push_back(in[2]);
    for (size_t i = 3; i < n - 1; i++) {
      if (in[i] == 0x55) {
        out.push_back(0x73);
        out.push_back(0x11);
      } else if (in[i] == 0x73) {
        out.push_back(0x73);
        out.push_back(0x22);
      } else {
        out.push_back(in[i]);
      }
    }
    out.push_back(in[n - 1]);
  }

  void destuff_(const uint8_t *in, size_t n, std::vector<uint8_t> &out) {
    for (size_t i = 0; i < n; i++) {
      if (in[i] == 0x73 && i + 1 < n) {
        if (in[i + 1] == 0x11) {
          out.push_back(0x55);
          i++;
        } else if (in[i + 1] == 0x22) {
          out.push_back(0x73);
          i++;
        } else {
          out.push_back(in[i]);
        }
      } else {
        out.push_back(in[i]);
      }
    }
  }

  // ── Формирование пакета запроса ──────────────────────────────────────────────
  // Формат запроса подтверждённо идентичен у Star и у 32-РУ (совпадает байт-в-
  // байт с RequestPacketShort/Long/Long2b рабочего скетча).
  size_t build_pkt_(uint8_t cmd, int sub1, int sub2) {
    uint8_t *b = sbuf_;
    uint8_t dl = (sub1 < 0) ? 0 : (sub2 < 0) ? 1 : 2;
    b[0] = 0x0F + dl;
    b[1] = 0x73;
    b[2] = 0x55;
    b[3] = 0x20 + dl;
    b[4] = 0x00;
    b[5] = addr_ & 0xFF;
    b[6] = (addr_ >> 8) & 0xFF;
    b[7] = 0xFE;
    b[8] = 0xFF;
    b[9] = cmd;
    b[10] = b[11] = b[12] = b[13] = 0x00;  // PIN (не используется)
    size_t p = 14;
    if (sub1 >= 0) b[p++] = static_cast<uint8_t>(sub1);
    if (sub2 >= 0) b[p++] = static_cast<uint8_t>(sub2);
    b[p] = crc8_mirtek(b + 3, p - 3);
    b[p + 1] = 0x55;
    return p + 2;
  }

  // ── Отправка команды и приём ответа ──────────────────────────────────────────
  bool do_cmd_(uint8_t cmd, int sub1, int sub2, int expected_pkts) {
    size_t raw_len = build_pkt_(cmd, sub1, sub2);
    std::vector<uint8_t> tx;
    stuff_(sbuf_, raw_len, tx);
    tx[0] = static_cast<uint8_t>(tx.size() - 1);

    // Последовательность как в packetSender() рабочего скетча: рекалибровка,
    // очистка TX, IDLE, запись мощности передачи, затем сама отправка.
    // Именно рекалибровки (SCAL) и записи PATABLE не было в базе Alecseyyy.
    cc_strobe_(CC_SCAL);
    delay(1);
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SIDLE);
    cc_wreg_(CC_PATABLE, CC_PATABLE_VALUE);

    this->enable();
    this->transfer_byte(CC_TXFIFO | CC_BURST);
    for (uint8_t byte : tx) this->transfer_byte(byte);
    this->disable();
    cc_strobe_(CC_STX);

    // Ждём завершения передачи по GDO0 (IOCFG0=0x06: assert на sync, deassert
    // по окончании пакета)
    if (gdo0_) {
      uint32_t t0 = millis();
      while (!gdo0_->digital_read() && millis() - t0 < 200) delayMicroseconds(100);
      while (gdo0_->digital_read() && millis() - t0 < 600) delayMicroseconds(100);
    } else {
      delay(100);
    }

    cc_strobe_(CC_SFRX);
    cc_strobe_(CC_SRX);

    // Приём ответных радиопакетов (счётчик может дробить ответ на несколько
    // эфирных посылок — ждём ровно expected_pkts штук, как packetType в
    // рабочем скетче). Таймаут 10000 мс — не мой выбор, это ровно
    // `TimerMs tmr(10000, 0, 0)` из packetReceiver() рабочего скетча.
    std::vector<uint8_t> raw_rx;
    int got = 0;
    uint32_t t0 = millis();
    while (millis() - t0 < 10000 && got < expected_pkts) {
      uint8_t rxb = cc_rstat_(CC_RXBYTES);
      if (rxb > 0 && rxb < 64) {
        got++;
        this->enable();
        this->transfer_byte(CC_RXFIFO | CC_READ | CC_BURST);
        uint8_t lb = this->transfer_byte(0x00);
        if (lb > 0 && lb < 60) {
          for (uint8_t i = 1; i < lb; i++) raw_rx.push_back(this->transfer_byte(0x00));
        }
        this->disable();
        cc_strobe_(CC_SIDLE);
        cc_strobe_(CC_SFRX);
        cc_strobe_(CC_SFTX);
        cc_strobe_(CC_SRX);
      }
      delayMicroseconds(500);
    }

    if (raw_rx.empty()) {
      ESP_LOGW(TAG, "Нет ответа на cmd=0x%02X", cmd);
      return false;
    }

    std::vector<uint8_t> ds;
    destuff_(raw_rx.data(), raw_rx.size(), ds);
    if (ds.size() > sizeof(rbuf_)) {
      ESP_LOGW(TAG, "Переполнение буфера");
      return false;
    }
    memcpy(rbuf_, ds.data(), ds.size());
    rlen_ = ds.size();

    // Заголовок ответа у 32-РУ: [0]=0x73 [1]=0x55 ... [6]=addr_lo [7]=addr_hi
    // [8]=эхо команды — смещения на 2 больше, чем в ответе Star (там адрес на
    // [4]/[5], команда на [6]). Взято из packetParser_1..6 рабочего скетча.
    if (rlen_ < 9 || rbuf_[0] != 0x73 || rbuf_[1] != 0x55) {
      ESP_LOGW(TAG, "Неверный заголовок (cmd=0x%02X)", cmd);
      return false;
    }
    if (rbuf_[6] != (addr_ & 0xFF) || rbuf_[7] != ((addr_ >> 8) & 0xFF)) {
      ESP_LOGW(TAG, "Несоответствие адреса (cmd=0x%02X)", cmd);
      return false;
    }
    if (rbuf_[8] != cmd) {
      ESP_LOGW(TAG, "Несоответствие эха команды (ждали 0x%02X, получили 0x%02X)", cmd, rbuf_[8]);
      return false;
    }
    return true;
  }

  // ── CRC-проверка ответа: считается по rbuf_[2 .. crc_pos-1], сверяется с
  // rbuf_[crc_pos], затем проверяется стоп-байт rbuf_[crc_pos+1]==0x55.
  // Соответствует циклу `for(i=2;i<bytecount-2;i++)` в packetReceiver(). ──────
  bool check_crc_(size_t crc_pos) {
    if (rlen_ <= crc_pos + 1) return false;
    uint8_t calc = crc8_mirtek(rbuf_ + 2, crc_pos - 2);
    if (calc != rbuf_[crc_pos] || rbuf_[crc_pos + 1] != 0x55) {
      ESP_LOGW(TAG, "Ошибка CRC: расчитано 0x%02X, получено 0x%02X", calc, rbuf_[crc_pos]);
      return false;
    }
    return true;
  }

  // ── Разбор целых чисел из rbuf_ (little-endian) ───────────────────────────────
  float u16_(size_t i) {
    return static_cast<float>(static_cast<uint16_t>(rbuf_[i]) | (static_cast<uint16_t>(rbuf_[i + 1]) << 8));
  }
  float u24_(size_t i) {
    return static_cast<float>(static_cast<uint32_t>(rbuf_[i]) | (static_cast<uint32_t>(rbuf_[i + 1]) << 8) |
                               (static_cast<uint32_t>(rbuf_[i + 2]) << 16));
  }
  float u32_(size_t i) {
    return static_cast<float>(static_cast<uint32_t>(rbuf_[i]) | (static_cast<uint32_t>(rbuf_[i + 1]) << 8) |
                               (static_cast<uint32_t>(rbuf_[i + 2]) << 16) |
                               (static_cast<uint32_t>(rbuf_[i + 3]) << 24));
  }
  // Знаковый формат Миртек: старший бит верхнего байта = знак (не дополнение до 2)
  float s16m_(size_t i, float div) {
    bool neg = (rbuf_[i + 1] >= 128);
    float v = static_cast<float>(static_cast<uint16_t>(rbuf_[i]) |
                                  (static_cast<uint16_t>(rbuf_[i + 1] & 0x7F) << 8)) /
              div;
    return neg ? -v : v;
  }
  float s24m_(size_t i, float div) {
    bool neg = (rbuf_[i + 2] >= 128);
    float v = static_cast<float>(static_cast<uint32_t>(rbuf_[i]) | (static_cast<uint32_t>(rbuf_[i + 1]) << 8) |
                                  (static_cast<uint32_t>(rbuf_[i + 2] & 0x7F) << 16)) /
              div;
    return neg ? -v : v;
  }

  void pub_s_(int i, float v) {
    if (ss_[i]) ss_[i]->publish_state(v);
  }
  void pub_txt_(int i, const std::string &v) {
    if (ts_[i]) ts_[i]->publish_state(v);
  }
  void pub_bin_(int i, bool v) {
    if (bs_[i]) bs_[i]->publish_state(v);
  }

  // ── Парсеры ответов — все смещения взяты из packetParser_1..6 в
  // My_Mirtek_Demon.ino, НЕ из ESPHome-Mirt-830 ──────────────────────────────

  // Команда 0x1C — дата/время + тип счётчика (packetParser_1)
  bool parse_datetime_() {
    if (!check_crc_(20)) return false;  // resultbuffer[20]==CRC, [21]==0x55
    uint8_t tp = rbuf_[9];
    switch (tp) {
      case 0xA8:  // 168 — Счетчик 3ф трансформаторный активно-реактивный
        three_phase_ = true;
        pub_txt_(TI_TYPE, "3ф трансформаторный активно-реактивный");
        break;
      case 0x98:  // 152 — Счетчик 1ф 2х элементный активно-реактивный
        three_phase_ = false;
        pub_txt_(TI_TYPE, "1ф 2х элементный активно-реактивный");
        break;
      default: {
        char tb[24];
        snprintf(tb, sizeof(tb), "тип 0x%02X (неизвестен)", tp);
        pub_txt_(TI_TYPE, tb);
        ESP_LOGW(TAG, "Неизвестный тип счётчика 0x%02X — оставляю three_phase=%d", tp, three_phase_);
        break;
      }
    }
    pub_bin_(BI_3PH, three_phase_);

    char tm[10], dt[12];
    snprintf(tm, sizeof(tm), "%02d:%02d:%02d", rbuf_[15], rbuf_[14], rbuf_[13]);
    snprintf(dt, sizeof(dt), "%02d.%02d.%02d", rbuf_[17], rbuf_[18], rbuf_[19]);
    pub_txt_(TI_TIME, tm);
    pub_txt_(TI_DATE, dt);
    ESP_LOGI(TAG, "Дата/время счётчика: %s %s, тип=0x%02X", dt, tm, tp);
    return true;
  }

  // Команда 0x05 sub=0x00 — суммарные показания энергии (packetParser_2)
  // Примечание: T3/T4 и реактивная энергия присутствуют в пакете (CRC/стоп
  // на [43]/[44] как и в 3-фазном 0x2B), но рабочий скетч их не декодирует —
  // точные смещения для 32-РУ не подтверждены, поэтому здесь тоже не читаем.
  bool parse_energy_() {
    if (!check_crc_(43)) return false;
    uint8_t cur_t = (rbuf_[14] >> 2) & 0x03;
    const char *tn[] = {"День", "Ночь", "Полупик", "Специальный"};
    pub_txt_(TI_TARIFF, tn[cur_t]);
    pub_s_(SI_SUM, u32_(19) / 100.f);
    pub_s_(SI_T1, u32_(27) / 100.f);
    pub_s_(SI_T2, u32_(31) / 100.f);
    ESP_LOGI(TAG, "кВт·ч: SUM=%.2f T1=%.2f T2=%.2f, тариф=%s", ss_[SI_SUM] ? ss_[SI_SUM]->state : 0.f,
             ss_[SI_T1] ? ss_[SI_T1]->state : 0.f, ss_[SI_T2] ? ss_[SI_T2]->state : 0.f, tn[cur_t]);
    return true;
  }

  // Команда 0x2B sub=0x00, 3-фазный счётчик — суммарные мгновенные значения
  // (packetParser_3). P — БЕЗ деления (сырое значение уже в ваттах) — это
  // отличается от Star-протокола, там было /1000.
  bool parse_instant_3ph_() {
    if (!check_crc_(43)) return false;
    pub_s_(SI_KW, u24_(18));                 // P, Вт, без деления
    pub_s_(SI_KVAR, s24m_(21, 1000.f));      // Q, квар
    pub_s_(SI_FREQ, u16_(24) / 100.f);       // Гц
    pub_s_(SI_COS, s16m_(26, 1000.f));       // cos φ
    pub_s_(SI_V1, u16_(28) / 100.f);
    pub_s_(SI_V2, u16_(30) / 100.f);
    pub_s_(SI_V3, u16_(32) / 100.f);
    pub_s_(SI_I1, u24_(34) / 1000.f);
    pub_s_(SI_I2, u24_(37) / 1000.f);
    pub_s_(SI_I3, u24_(40) / 1000.f);
    return true;
  }

  // Команда 0x2B sub=0x00, 1-фазный / старый вариант — суммарные мгновенные
  // значения (packetParser_3_1). P здесь 16-битный (не 24-битный).
  // Условие CRC повторяет оригинал: у части счётчиков пакет короче на 1
  // байт, и тогда CRC не проверяется вовсе — только стоп-байт на [43].
  bool parse_instant_1ph_() {
    bool ok = (rlen_ > 43 && rbuf_[43] == 0x55) || check_crc_(41);
    if (!ok) {
      ESP_LOGW(TAG, "Ошибка контроля пакета 0x2B (1ph/старый вариант)");
      return false;
    }
    pub_s_(SI_KW, u16_(18));                 // P, Вт, без деления
    pub_s_(SI_KVAR, s16m_(20, 1000.f));      // Q, квар
    pub_s_(SI_FREQ, u16_(22) / 100.f);       // Гц
    pub_s_(SI_COS, s16m_(24, 1000.f));       // cos φ
    pub_s_(SI_V1, u16_(26) / 100.f);
    pub_s_(SI_V2, u16_(28) / 100.f);
    pub_s_(SI_V3, u16_(30) / 100.f);
    pub_s_(SI_I1, u24_(32) / 1000.f);
    pub_s_(SI_I2, u24_(35) / 1000.f);
    pub_s_(SI_I3, u24_(38) / 1000.f);
    return true;
  }

  // Команда 0x2B sub=0x10 — мгновенные значения по фазам, только 3ф
  // (packetParser_4). Pa/Pb/Pc и Sa/Sb/Sc — БЕЗ деления (уже в Вт/ВА).
  bool parse_phase_() {
    if (!check_crc_(43)) return false;
    pub_s_(SI_CA, s16m_(18, 1000.f));
    pub_s_(SI_CB, s16m_(20, 1000.f));
    pub_s_(SI_CC, s16m_(22, 1000.f));
    pub_s_(SI_PA, u16_(24));
    pub_s_(SI_PB, u16_(26));
    pub_s_(SI_PC, u16_(28));
    pub_s_(SI_QA, s16m_(30, 1000.f));
    pub_s_(SI_QB, s16m_(32, 1000.f));
    pub_s_(SI_QC, s16m_(34, 1000.f));
    pub_s_(SI_SA, u16_(36));
    pub_s_(SI_SB, u16_(38));
    pub_s_(SI_SC, u16_(40));
    // Температура: 1 байт, старший бит — знак (не дополнение до 2)
    float t = (rbuf_[42] >= 128) ? static_cast<float>(rbuf_[42] - 128) * -1.f : static_cast<float>(rbuf_[42]);
    pub_s_(SI_TEMP, t);
    return true;
  }

  // Команда 0x10 — статус реле и пломб (packetParser_6). Не путать с
  // командой 0x01 у Star-протокола — у 32-РУ это другая команда с другим
  // форматом; 0x01 в рабочем скетче использовалась только для экспериментов
  // и ответ никак не разбирался.
  bool parse_status_() {
    if (!check_crc_(32)) return false;
    bool relay_off = ((rbuf_[24] >> 2) & 0x03) == 1;  // 0=Вкл, 1=Выкл
    pub_txt_(TI_RELAY, relay_off ? "Выкл" : "Вкл");
    pub_bin_(BI_RELAY, !relay_off);

    uint8_t seal = rbuf_[28];
    const char *seal_str;
    bool seal_ok = (seal == 0);
    switch (seal) {
      case 0: seal_str = "OK"; break;
      case 1: seal_str = "Вскрыта пломба клеммника"; break;
      case 2: seal_str = "Вскрыта пломба корпуса"; break;
      case 3: seal_str = "Вскрыта пломба клеммника+корпуса"; break;
      default: seal_str = "Неизвестно"; break;
    }
    pub_txt_(TI_SEAL, seal_str);
    pub_bin_(BI_SEAL, seal_ok);
    ESP_LOGI(TAG, "Реле=%s Пломбы=%s", relay_off ? "Выкл" : "Вкл", seal_str);
    return true;
  }
};

}  // namespace mirtek_cc1101
}  // namespace esphome
