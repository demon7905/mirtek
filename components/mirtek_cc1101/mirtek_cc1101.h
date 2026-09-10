#pragma once
// =============================================================================
//  Mirtek CC1101 ESPHome External Component — МИРТЕК-32-РУ
//  Совместимость: ESPHome 2026.8.x, ESP32, Arduino framework
//
//  Источник истины: My_Mirtek_Demon.ino — рабочий скетч именно для МИРТЕК-32-РУ.
//  RX/TX-семантика повторяет SmartRC/ELECHOUSE CC1101.
//
//  ВАЖНО:
//  SmartRC SendData() сначала пишет размер в TXFIFO отдельной записью, затем
//  тем же TXFIFO делает burst самого txBuffer. Поэтому первый байт txBuffer
//  (длина Mirtek-пакета) на приёме является дублирующим байтом и при сборке
//  ответа отбрасывается (for i=1; i<len; i++).
//
//  Внешние настройки:
//    - адрес счётчика — меняется из Home Assistant через template number;
//    - интервал опроса — меняется из Home Assistant через template number;
//    - обе настройки сохраняются в Preferences ESP32 и переживают reboot.
//
//  Архитектура приёма: неблокирующая state machine. RX-окно одной команды
//  остаётся 2 секунды, как в исходной версии Arduino-кода.
// =============================================================================

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
#include "esphome/core/preferences.h"
#include "esphome/components/spi/spi.h"
#include "esphome/components/sensor/sensor.h"
#include "esphome/components/text_sensor/text_sensor.h"
#include "esphome/components/binary_sensor/binary_sensor.h"
#include "esphome/components/number/number.h"

#include <vector>
#include <cstdio>
#include <cstring>

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek32ru";

// ─── CRC-8, полином 0xA9 ──────────────────────────────────────────────────────
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

static void hex_to_(std::string &out, const uint8_t *d, size_t n) {
  char buf[4];
  for (size_t i = 0; i < n; i++) {
    snprintf(buf, sizeof(buf), "%02X ", d[i]);
    out += buf;
  }
}

// ─── CC1101 команды/регистры ─────────────────────────────────────────────────
static const uint8_t CC_SRES = 0x30;
static const uint8_t CC_SCAL = 0x33;
static const uint8_t CC_SRX = 0x34;
static const uint8_t CC_STX = 0x35;
static const uint8_t CC_SIDLE = 0x36;
static const uint8_t CC_SFRX = 0x3A;
static const uint8_t CC_SFTX = 0x3B;
static const uint8_t CC_PATABLE = 0x3E;
static const uint8_t CC_BURST = 0x40;
static const uint8_t CC_READ = 0x80;
static const uint8_t CC_TXFIFO = 0x3F;
static const uint8_t CC_RXFIFO = 0x3F;
static const uint8_t CC_RXBYTES = 0x3B;
static const uint8_t CC_VERSION = 0x31;

static const uint8_t CC_PATABLE_VALUE = 0xC4;

// ─── RF-конфигурация 433 МГц / FSK ───────────────────────────────────────────
static const uint8_t RF_CFG[47] = {
    0x0D, 0x2E, 0x06, 0x4F, 0xD3, 0x91, 0x3C, 0x00,
    0x41, 0x00, 0x16, 0x0F, 0x00, 0x10, 0x8B, 0x54,
    0xD9, 0x83, 0x13, 0xD2, 0xAA, 0x31, 0x07, 0x0C,
    0x08, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87, 0x6B,
    0xF8, 0x56, 0x10, 0xE9, 0x2A, 0x00, 0x1F, 0x41,
    0x00, 0x59, 0x59, 0x3F, 0x81, 0x35, 0x09
};

// ─── Индексы сенсоров ────────────────────────────────────────────────────────
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
  TI_TARIFF = 0, TI_RELAY, TI_SEAL, TI_TYPE, TI_DATE, TI_TIME, TI_STATUS, TI_COUNT
};

enum BinIdx {
  BI_3PH = 0, BI_RELAY, BI_SEAL, BI_CC, BI_COUNT
};

// ─── Шаги общего цикла опроса ─────────────────────────────────────────────────
enum Phase : uint8_t {
  PH_IDLE = 0,
  PH_DATETIME,
  PH_ENERGY,
  PH_INSTANT,
  PH_PHASE_POWER,
  PH_STATUS,
  PH_RELAY_ON,
  PH_RELAY_OFF,
  PH_SEAL_RESET,
};

// ─── Состояния TX/RX state machine ───────────────────────────────────────────
enum PumpState : uint8_t {
  PS_IDLE = 0,
  PS_TX_WAIT_HIGH,
  PS_TX_WAIT_LOW,
  PS_RX_WAIT_HIGH,
  PS_RX_WAIT_LOW,
  PS_DONE,
};

// =============================================================================
class MirtekCC1101 : public PollingComponent,
                      public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST,
                                             spi::CLOCK_POLARITY_LOW,
                                             spi::CLOCK_PHASE_LEADING,
                                             spi::DATA_RATE_4MHZ> {
 public:
  void set_gdo0_pin(GPIOPin *p) { gdo0_ = p; }

  // Адрес счётчика можно менять из Home Assistant.
  // Сохраняется в Preferences ESP32.
  void set_meter_address(int a) {
    if (a < 1)
      a = 1;
    if (a > 65535)
      a = 65535;

    addr_ = static_cast<uint16_t>(a);

    // template number может вызвать set_action во время своего setup(),
    // раньше setup() этого компонента. В этот момент pref_addr_ ещё не
    // инициалирован — сохранять в Preferences нельзя.
    if (!initialized_) {
      ESP_LOGI(TAG, "Адрес %u принят до инициализации; сохранение отложено", addr_);
      return;
    }

    if (pref_addr_.save(&addr_)) {
      ESP_LOGI(TAG, "Адрес счётчика изменён и сохранён: %u", addr_);
    } else {
      ESP_LOGW(TAG, "Адрес счётчика изменён: %u, но сохранить во flash не удалось", addr_);
    }

    if (address_number_ != nullptr) {
      address_number_->publish_state(static_cast<float>(addr_));
    }
  }

  void set_address_number(number::Number *n) {
    address_number_ = n;
  }

  // Интервал опроса можно менять из Home Assistant.
  // Диапазон ограничивается здесь дополнительно для безопасности.
  void set_poll_interval_seconds(float seconds) {
    if (seconds < 10.0f)
      seconds = 10.0f;
    if (seconds > 3600.0f)
      seconds = 3600.0f;

    const uint32_t ms = static_cast<uint32_t>(seconds * 1000.0f);
    poll_interval_ms_ = ms;

    // До setup() Preferences ещё не подготовлены. Значение принимаем в RAM,
    // но запись во flash выполняем только после полной инициализации.
    if (!initialized_) {
      ESP_LOGI(TAG, "Интервал %.0f сек принят до инициализации; сохранение отложено", seconds);
      return;
    }

    this->set_update_interval(ms);
    this->stop_poller();
    this->start_poller();

    if (pref_interval_.save(&poll_interval_ms_)) {
      ESP_LOGI(TAG, "Интервал опроса изменён и сохранён: %.0f сек", seconds);
    } else {
      ESP_LOGW(TAG, "Интервал опроса изменён: %.0f сек, но сохранить во flash не удалось", seconds);
    }

    if (interval_number_ != nullptr) {
      interval_number_->publish_state(seconds);
    }
  }

  void set_sensor(int i, sensor::Sensor *s) {
    if (i >= 0 && i < SI_COUNT)
      ss_[i] = s;
  }

  void set_text_sensor(int i, text_sensor::TextSensor *s) {
    if (i >= 0 && i < TI_COUNT)
      ts_[i] = s;
  }

  void set_binary_sensor(int i, binary_sensor::BinarySensor *s) {
    if (i >= 0 && i < BI_COUNT)
      bs_[i] = s;
  }

  // ── ESPHome lifecycle ───────────────────────────────────────────────────────
  void setup() override {
    initialized_ = false;

    // Уникальные ключи Preferences для адреса и интервала.
    // Адрес: uint16_t, интервал: uint32_t.
    pref_addr_ = global_preferences->make_preference<uint16_t>(0x4D523201);
    pref_interval_ = global_preferences->make_preference<uint32_t>(0x4D523202);

    uint16_t saved_addr = 0;
    if (pref_addr_.load(&saved_addr) && saved_addr >= 1) {
      addr_ = saved_addr;
    }

    uint32_t saved_interval = 0;
    if (pref_interval_.load(&saved_interval) && saved_interval >= 10000 && saved_interval <= 3600000) {
      poll_interval_ms_ = saved_interval;
    }
    this->set_update_interval(poll_interval_ms_);
    // call_setup() уже зарегистрировал poller со старым YAML-интервалом.
    // Перерегистрируем его, чтобы сохранённое значение применилось сразу.
    this->stop_poller();
    this->start_poller();

    // Показываем фактические сохранённые значения в number entities.
    if (address_number_ != nullptr) {
      address_number_->publish_state(static_cast<float>(addr_));
    }
    if (interval_number_ != nullptr) {
      interval_number_->publish_state(static_cast<float>(poll_interval_ms_) / 1000.0f);
    }

    ESP_LOGI(TAG, "Инициализация CC1101, адрес счётчика=%u, интервал=%u мс",
             addr_, (unsigned) poll_interval_ms_);

    this->spi_setup();
    if (gdo0_)
      gdo0_->setup();

    bool ok = cc_init_();
    pub_bin_(BI_CC, ok);
    pub_txt_(TI_STATUS, ok ? "CC1101 OK" : "CC1101 ERR");

    // Только после завершения setup() разрешаем команды реле/пломб и запись
    // пользовательских настроек в Preferences.
    initialized_ = ok;

    if (!ok) {
      ESP_LOGE(TAG, "CC1101 не обнаружен! Проверьте подключение SPI.");
    } else {
      ESP_LOGI(TAG, "CC1101 готов. Интервал опроса: %u мс", (unsigned) get_update_interval());
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway (МИРТЕК-32-РУ):");
    ESP_LOGCONFIG(TAG, "  Адрес счётчика : %u", addr_);
    ESP_LOGCONFIG(TAG, "  Интервал опроса: %u мс", (unsigned) get_update_interval());
    LOG_PIN("  GDO0 пин: ", gdo0_);
  }

  void update() override {
    if (phase_ != PH_IDLE) {
      ESP_LOGW(TAG, "Предыдущий цикл опроса ещё не завершён (фаза=%d) — пропускаю", (int) phase_);
      return;
    }
    ESP_LOGI(TAG, "=== Опрос, адрес=%u ===", addr_);
    poll_ok_accum_ = true;
    start_phase_(PH_DATETIME);
  }

  void loop() override {
    if (phase_ == PH_IDLE) {
      if (oneshot_pending_) {
        oneshot_pending_ = false;
        start_phase_(oneshot_phase_);
      }
      return;
    }

    pump_tick_();
    if (pump_state_ != PS_DONE)
      return;

    bool ok = pump_ok_ && on_pump_done_();
    poll_ok_accum_ = poll_ok_accum_ && ok;
    advance_phase_(ok);
  }

  // ── Публичные методы ────────────────────────────────────────────────────────
  void poll_all() { update(); }
  void relay_on() {
    if (!initialized_) {
      ESP_LOGW(TAG, "Relay ON пропущен: CC1101 ещё не инициализирован");
      return;
    }
    queue_oneshot_(PH_RELAY_ON);
  }

  void relay_off() {
    if (!initialized_) {
      ESP_LOGW(TAG, "Relay OFF пропущен: CC1101 ещё не инициализирован");
      return;
    }
    queue_oneshot_(PH_RELAY_OFF);
  }

  void reset_seals() {
    if (!initialized_) {
      ESP_LOGW(TAG, "Сброс пломб пропущен: CC1101 ещё не инициализирован");
      return;
    }
    queue_oneshot_(PH_SEAL_RESET);
  }

 protected:
  GPIOPin *gdo0_{nullptr};

  uint16_t addr_{35040};
  bool three_phase_{true};

  uint32_t poll_interval_ms_{60000};
  bool initialized_{false};

  number::Number *address_number_{nullptr};
  number::Number *interval_number_{nullptr};

  ESPPreferenceObject pref_addr_;
  ESPPreferenceObject pref_interval_;

  sensor::Sensor *ss_[SI_COUNT]{};
  text_sensor::TextSensor *ts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bs_[BI_COUNT]{};

  uint8_t sbuf_[24]{};
  uint8_t rbuf_[64]{};
  size_t rlen_{0};

  Phase phase_{PH_IDLE};
  bool poll_ok_accum_{true};
  bool oneshot_pending_{false};
  Phase oneshot_phase_{PH_IDLE};

  PumpState pump_state_{PS_IDLE};
  uint8_t pump_cmd_{0};
  int pump_expected_{0};
  int pump_got_{0};
  uint32_t pump_t0_{0};
  uint32_t pump_sub_t0_{0};
  bool pump_ok_{false};
  std::vector<uint8_t> tx_stuffed_;
  std::vector<uint8_t> pump_raw_rx_;

  // ── SPI helpers ─────────────────────────────────────────────────────────────
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
    for (size_t i = 0; i < len; i++)
      this->transfer_byte(data[i]);
    this->disable();
  }

  uint8_t cc_rstat_(uint8_t reg) {
    this->enable();
    this->transfer_byte(CC_READ | CC_BURST | (reg & 0x3F));
    uint8_t v = this->transfer_byte(0x00);
    this->disable();
    return v;
  }

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

  // ── Byte stuffing ──────────────────────────────────────────────────────────
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

  // ── Формирование запроса ────────────────────────────────────────────────────
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
    b[10] = b[11] = b[12] = b[13] = 0x00;
    size_t p = 14;
    if (sub1 >= 0)
      b[p++] = static_cast<uint8_t>(sub1);
    if (sub2 >= 0)
      b[p++] = static_cast<uint8_t>(sub2);
    b[p] = crc8_mirtek(b + 3, p - 3);
    b[p + 1] = 0x55;
    return p + 2;
  }

  // ── Запуск фазы ────────────────────────────────────────────────────────────
  void start_phase_(Phase ph) {
    phase_ = ph;
    switch (ph) {
      case PH_DATETIME:    begin_command_(0x1C, -1, -1, 3); break;
      case PH_ENERGY:      begin_command_(0x05, 0x00, -1, 4); break;
      case PH_INSTANT:     begin_command_(0x2B, 0x00, -1, 4); break;
      case PH_PHASE_POWER: begin_command_(0x2B, 0x10, -1, 4); break;
      case PH_STATUS:      begin_command_(0x10, -1, -1, 3); break;
      case PH_RELAY_ON:    ESP_LOGI(TAG, "Реле: замкнуть (включить)"); begin_command_(0x3A, 0x00, 0x00, 4); break;
      case PH_RELAY_OFF:   ESP_LOGI(TAG, "Реле: разомкнуть (выключить)"); begin_command_(0x3A, 0x00, 0x01, 4); break;
      case PH_SEAL_RESET:  ESP_LOGI(TAG, "Сброс состояния электронных пломб"); begin_command_(0x04, 0x01, -1, 4); break;
      default: phase_ = PH_IDLE; break;
    }
  }

  void advance_phase_(bool /*last_step_ok*/) {
    switch (phase_) {
      case PH_DATETIME:    start_phase_(PH_ENERGY); return;
      case PH_ENERGY:      start_phase_(PH_INSTANT); return;
      case PH_INSTANT:     start_phase_(three_phase_ ? PH_PHASE_POWER : PH_STATUS); return;
      case PH_PHASE_POWER: start_phase_(PH_STATUS); return;
      case PH_STATUS:
        pub_txt_(TI_STATUS, poll_ok_accum_ ? "OK" : "PARTIAL");
        ESP_LOGI(TAG, "=== Опрос завершён: %s ===", poll_ok_accum_ ? "OK" : "PARTIAL");
        phase_ = PH_IDLE;
        return;
      case PH_RELAY_ON:
      case PH_RELAY_OFF:
      case PH_SEAL_RESET:
        phase_ = PH_IDLE;
        return;
      default:
        phase_ = PH_IDLE;
        return;
    }
  }

  void queue_oneshot_(Phase ph) {
    if (phase_ == PH_IDLE) {
      start_phase_(ph);
    } else {
      ESP_LOGI(TAG, "Опрос выполняется — команда будет отправлена сразу после его завершения");
      oneshot_pending_ = true;
      oneshot_phase_ = ph;
    }
  }

  // ── TX одной команды ────────────────────────────────────────────────────────
  void begin_command_(uint8_t cmd, int sub1, int sub2, int expected_pkts) {
    pump_cmd_ = cmd;
    pump_expected_ = expected_pkts;
    pump_got_ = 0;
    pump_ok_ = false;
    pump_raw_rx_.clear();

    size_t raw_len = build_pkt_(cmd, sub1, sub2);
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, sbuf_, raw_len);
      ESP_LOGV(TAG, "TX RAW (cmd=0x%02X): %s", cmd, h.c_str());
    }

    tx_stuffed_.clear();
    stuff_(sbuf_, raw_len, tx_stuffed_);

    // Первый байт txBuffer — длина Mirtek-пакета без внешнего length-префикса.
    // SmartRC SendData() отдельно пишет размер всего txBuffer в TXFIFO, поэтому
    // на эфире получается два одинаковых length-байта подряд.
    tx_stuffed_[0] = static_cast<uint8_t>(tx_stuffed_.size() - 1);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, tx_stuffed_.data(), tx_stuffed_.size());
      ESP_LOGV(TAG, "TX stuffed: %s", h.c_str());
    }

    // Как packetSender(): SCAL → SFTX → SIDLE → PATABLE → TXFIFO → STX.
    cc_strobe_(CC_SCAL);
    delay(2);
    App.feed_wdt();
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SIDLE);
    cc_wreg_(CC_PATABLE, CC_PATABLE_VALUE);

    // Полностью повторяем SmartRC SendData():
    // 1. Отдельная запись внешнего аппаратного length-префикса.
    cc_wreg_(CC_TXFIFO, static_cast<uint8_t>(tx_stuffed_.size()));

    // 2. Burst-загрузка самого txBuffer, начиная с txBuffer[0].
    this->enable();
    this->transfer_byte(CC_TXFIFO | CC_BURST);
    for (uint8_t b : tx_stuffed_) {
      this->transfer_byte(b);
    }
    this->disable();

    // 3. Передача.
    cc_strobe_(CC_STX);

    pump_sub_t0_ = millis();
    pump_state_ = PS_TX_WAIT_HIGH;
  }

  // ── Один тик state machine ──────────────────────────────────────────────────
  void pump_tick_() {
    switch (pump_state_) {
      case PS_TX_WAIT_HIGH: {
        if (!gdo0_ || gdo0_->digital_read() || millis() - pump_sub_t0_ > 500) {
          ESP_LOGV(TAG, "GDO0 HIGH (TX sync отправлен)");
          pump_sub_t0_ = millis();
          pump_state_ = PS_TX_WAIT_LOW;
        }
        break;
      }

      case PS_TX_WAIT_LOW: {
        if (!gdo0_ || !gdo0_->digital_read() || millis() - pump_sub_t0_ > 500) {
          ESP_LOGV(TAG, "GDO0 LOW (TX завершён)");
          cc_strobe_(CC_SFRX);
          cc_strobe_(CC_SRX);
          pump_t0_ = millis();
          pump_sub_t0_ = millis();
          pump_state_ = PS_RX_WAIT_HIGH;
        }
        break;
      }

      case PS_RX_WAIT_HIGH: {
        if (millis() - pump_t0_ > 2000) {
          finish_pump_();
          return;
        }

        if (gdo0_ && gdo0_->digital_read()) {
          ESP_LOGV(TAG, "GDO0 HIGH (RX подпакет %d/%d, синхрослово)",
                   pump_got_ + 1, pump_expected_);
          pump_sub_t0_ = millis();
          pump_state_ = PS_RX_WAIT_LOW;
        } else if (!gdo0_) {
          uint8_t rxb = cc_rstat_(CC_RXBYTES);
          if (rxb > 0 && rxb < 64)
            read_burst_and_rearm_();
        }
        break;
      }

      case PS_RX_WAIT_LOW: {
        // Таймаут 200 мс здесь — ошибка текущего RF-подпакета, а НЕ успешный
        // приём. Это не должно превращаться в read_burst_and_rearm_().
        if (!gdo0_->digital_read()) {
          ESP_LOGV(TAG, "GDO0 LOW (RX подпакет %d/%d принят)",
                   pump_got_ + 1, pump_expected_);
          read_burst_and_rearm_();
        } else if (millis() - pump_sub_t0_ > 200) {
          ESP_LOGW(TAG,
                   "RX timeout: GDO0 не опустился за 200 мс (подпакет %d/%d)",
                   pump_got_ + 1, pump_expected_);
          finish_pump_();
        } else if (millis() - pump_t0_ > 2000) {
          finish_pump_();
        }
        break;
      }

      default:
        break;
    }
  }

  void read_burst_and_rearm_() {
    read_one_burst_();
    pump_got_++;

    // Как packetReceiver(): SIDLE, SFRX, SFTX, SRX.
    cc_strobe_(CC_SIDLE);
    cc_strobe_(CC_SFRX);
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SRX);
    App.feed_wdt();

    if (pump_got_ >= pump_expected_) {
      finish_pump_();
    } else {
      pump_sub_t0_ = millis();
      pump_state_ = PS_RX_WAIT_HIGH;
    }
  }

  // ── Чтение одного RF-подпакета ──────────────────────────────────────────────
  void read_one_burst_() {
    this->enable();
    this->transfer_byte(CC_READ | CC_RXFIFO);
    uint8_t outer_len = this->transfer_byte(0x00);
    this->disable();

    ESP_LOGV(TAG, "RX length (подпакет %d/%d): %u",
             pump_got_ + 1, pump_expected_, outer_len);

    if (outer_len == 0 || outer_len >= 60) {
      ESP_LOGW(TAG, "Подозрительная длина RX-подпакета: %u", outer_len);
      return;
    }

    uint8_t burst[64];
    this->enable();
    this->transfer_byte(CC_READ | CC_BURST | CC_RXFIFO);
    for (uint8_t i = 0; i < outer_len; i++)
      burst[i] = this->transfer_byte(0x00);
    this->disable();

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, burst, outer_len);
      ESP_LOGV(TAG, "RX payload (подпакет %d/%d): %s",
               pump_got_ + 1, pump_expected_, h.c_str());
    }

    // SmartRC ReceiveData() читает ещё 2 байта после payload.
    uint8_t status[2];
    this->enable();
    this->transfer_byte(CC_READ | CC_BURST | CC_RXFIFO);
    status[0] = this->transfer_byte(0x00);
    status[1] = this->transfer_byte(0x00);
    this->disable();

    ESP_LOGV(TAG, "RX status (подпакет %d/%d, не используется): %02X %02X",
             pump_got_ + 1, pump_expected_, status[0], status[1]);

    // rxBuffer[0] — дублирующий length-байт из SmartRC SendData().
    // Поэтому собираем ответ начиная с индекса 1.
    for (uint8_t i = 1; i < outer_len; i++)
      pump_raw_rx_.push_back(burst[i]);
  }

  // ── Завершение приёма команды ───────────────────────────────────────────────
  void finish_pump_() {
    pump_state_ = PS_DONE;

    if (pump_raw_rx_.empty()) {
      ESP_LOGW(TAG, "Нет ответа на cmd=0x%02X (получено %d из %d подпакетов)",
               pump_cmd_, pump_got_, pump_expected_);
      pump_ok_ = false;
      return;
    }

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, pump_raw_rx_.data(), pump_raw_rx_.size());
      ESP_LOGV(TAG, "RX assembled (%d/%d подпакетов): %s",
               pump_got_, pump_expected_, h.c_str());
    }

    std::vector<uint8_t> ds;
    destuff_(pump_raw_rx_.data(), pump_raw_rx_.size(), ds);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE && !ds.empty()) {
      std::string h;
      hex_to_(h, ds.data(), ds.size());
      ESP_LOGV(TAG, "DESTUFF: %s", h.c_str());
    }

    if (ds.size() > sizeof(rbuf_)) {
      ESP_LOGW(TAG, "Переполнение буфера (cmd=0x%02X, %u байт)",
               pump_cmd_, (unsigned) ds.size());
      pump_ok_ = false;
      return;
    }

    memcpy(rbuf_, ds.data(), ds.size());
    rlen_ = ds.size();

    // Заголовок ответа 32-РУ: [0]=73 [1]=55 ... [6]=addr_lo [7]=addr_hi [8]=echo cmd.
    if (rlen_ < 9 || rbuf_[0] != 0x73 || rbuf_[1] != 0x55) {
      ESP_LOGW(TAG, "Неверный заголовок (cmd=0x%02X)", pump_cmd_);
      pump_ok_ = false;
      return;
    }

    if (rbuf_[6] != (addr_ & 0xFF) || rbuf_[7] != ((addr_ >> 8) & 0xFF)) {
      ESP_LOGW(TAG, "Несоответствие адреса (cmd=0x%02X, ждали %u)", pump_cmd_, addr_);
      pump_ok_ = false;
      return;
    }

    if (rbuf_[8] != pump_cmd_) {
      ESP_LOGW(TAG, "Несоответствие эха команды (ждали 0x%02X, получили 0x%02X)",
               pump_cmd_, rbuf_[8]);
      pump_ok_ = false;
      return;
    }

    pump_ok_ = true;
  }

  // ── Диспетчер парсеров ──────────────────────────────────────────────────────
  bool on_pump_done_() {
    switch (phase_) {
      case PH_DATETIME:     return parse_datetime_();
      case PH_ENERGY:       return parse_energy_();
      case PH_INSTANT:      return three_phase_ ? parse_instant_3ph_() : parse_instant_1ph_();
      case PH_PHASE_POWER:  return parse_phase_();
      case PH_STATUS:       return parse_status_();
      case PH_RELAY_ON:
      case PH_RELAY_OFF:
      case PH_SEAL_RESET:   return true;
      default: return false;
    }
  }

  // ── CRC ответа ──────────────────────────────────────────────────────────────
  bool check_crc_(size_t crc_pos) {
    if (rlen_ <= crc_pos + 1)
      return false;

    uint8_t calc = crc8_mirtek(rbuf_ + 2, crc_pos - 2);
    bool ok = (calc == rbuf_[crc_pos]) && (rbuf_[crc_pos + 1] == 0x55);

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      ESP_LOGV(TAG, "CRC calculated=0x%02X received=0x%02X stop=0x%02X → %s",
               calc, rbuf_[crc_pos], rbuf_[crc_pos + 1], ok ? "OK" : "ERROR");
    }

    if (!ok)
      ESP_LOGW(TAG, "Ошибка CRC: расчитано 0x%02X, получено 0x%02X", calc, rbuf_[crc_pos]);

    return ok;
  }

  // ── Числа ───────────────────────────────────────────────────────────────────
  float u16_(size_t i) {
    return static_cast<float>(static_cast<uint16_t>(rbuf_[i]) |
                               (static_cast<uint16_t>(rbuf_[i + 1]) << 8));
  }

  float u24_(size_t i) {
    return static_cast<float>(static_cast<uint32_t>(rbuf_[i]) |
                               (static_cast<uint32_t>(rbuf_[i + 1]) << 8) |
                               (static_cast<uint32_t>(rbuf_[i + 2]) << 16));
  }

  float u32_(size_t i) {
    return static_cast<float>(static_cast<uint32_t>(rbuf_[i]) |
                               (static_cast<uint32_t>(rbuf_[i + 1]) << 8) |
                               (static_cast<uint32_t>(rbuf_[i + 2]) << 16) |
                               (static_cast<uint32_t>(rbuf_[i + 3]) << 24));
  }

  float s16m_(size_t i, float div) {
    bool neg = (rbuf_[i + 1] >= 128);
    float v = static_cast<float>(static_cast<uint16_t>(rbuf_[i]) |
                                 (static_cast<uint16_t>(rbuf_[i + 1] & 0x7F) << 8)) /
              div;
    return neg ? -v : v;
  }

  float s24m_(size_t i, float div) {
    bool neg = (rbuf_[i + 2] >= 128);
    float v = static_cast<float>(static_cast<uint32_t>(rbuf_[i]) |
                                 (static_cast<uint32_t>(rbuf_[i + 1]) << 8) |
                                 (static_cast<uint32_t>(rbuf_[i + 2] & 0x7F) << 16)) /
              div;
    return neg ? -v : v;
  }

  void pub_s_(int i, float v) {
    if (ss_[i])
      ss_[i]->publish_state(v);
  }

  void pub_txt_(int i, const std::string &v) {
    if (ts_[i])
      ts_[i]->publish_state(v);
  }

  void pub_bin_(int i, bool v) {
    if (bs_[i])
      bs_[i]->publish_state(v);
  }

  // ── Парсеры ─────────────────────────────────────────────────────────────────
  bool parse_datetime_() {
    if (!check_crc_(20))
      return false;

    uint8_t tp = rbuf_[9];
    switch (tp) {
      case 0xA8:
        three_phase_ = true;
        pub_txt_(TI_TYPE, "3ф трансформаторный активно-реактивный");
        break;
      case 0x98:
        three_phase_ = false;
        pub_txt_(TI_TYPE, "1ф 2х элементный активно-реактивный");
        break;
      default: {
        char tb[48];
        snprintf(tb, sizeof(tb), "тип 0x%02X (неизвестен)", tp);
        pub_txt_(TI_TYPE, tb);
        ESP_LOGW(TAG, "Неизвестный тип счётчика 0x%02X — оставляю three_phase=%d", tp, three_phase_);
        break;
      }
    }

    pub_bin_(BI_3PH, three_phase_);

    char tm[16], dt[16];
    snprintf(tm, sizeof(tm), "%02d:%02d:%02d", rbuf_[15], rbuf_[14], rbuf_[13]);
    snprintf(dt, sizeof(dt), "%02d.%02d.%02d", rbuf_[17], rbuf_[18], rbuf_[19]);
    pub_txt_(TI_TIME, tm);
    pub_txt_(TI_DATE, dt);
    ESP_LOGI(TAG, "Дата/время счётчика: %s %s, тип=0x%02X", dt, tm, tp);
    return true;
  }

  bool parse_energy_() {
    if (!check_crc_(43))
      return false;

    uint8_t cur_t = (rbuf_[14] >> 2) & 0x03;
    const char *tn[] = {"День", "Ночь", "Полупик", "Специальный"};
    pub_txt_(TI_TARIFF, tn[cur_t]);
    pub_s_(SI_SUM, u32_(19) / 100.f);
    pub_s_(SI_T1, u32_(27) / 100.f);
    pub_s_(SI_T2, u32_(31) / 100.f);
    return true;
  }

  bool parse_instant_3ph_() {
    if (!check_crc_(43))
      return false;

    pub_s_(SI_KW, u24_(18));
    pub_s_(SI_KVAR, s24m_(21, 1000.f));
    pub_s_(SI_FREQ, u16_(24) / 100.f);
    pub_s_(SI_COS, s16m_(26, 1000.f));
    pub_s_(SI_V1, u16_(28) / 100.f);
    pub_s_(SI_V2, u16_(30) / 100.f);
    pub_s_(SI_V3, u16_(32) / 100.f);
    pub_s_(SI_I1, u24_(34) / 1000.f);
    pub_s_(SI_I2, u24_(37) / 1000.f);
    pub_s_(SI_I3, u24_(40) / 1000.f);
    return true;
  }

  bool parse_instant_1ph_() {
    bool ok = (rlen_ > 43 && rbuf_[43] == 0x55) || check_crc_(41);
    if (!ok) {
      ESP_LOGW(TAG, "Ошибка контроля пакета 0x2B (1ph/старый вариант)");
      return false;
    }

    pub_s_(SI_KW, u16_(18));
    pub_s_(SI_KVAR, s16m_(20, 1000.f));
    pub_s_(SI_FREQ, u16_(22) / 100.f);
    pub_s_(SI_COS, s16m_(24, 1000.f));
    pub_s_(SI_V1, u16_(26) / 100.f);
    pub_s_(SI_V2, u16_(28) / 100.f);
    pub_s_(SI_V3, u16_(30) / 100.f);
    pub_s_(SI_I1, u24_(32) / 1000.f);
    pub_s_(SI_I2, u24_(35) / 1000.f);
    pub_s_(SI_I3, u24_(38) / 1000.f);
    return true;
  }

  bool parse_phase_() {
    if (!check_crc_(43))
      return false;

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

    float t = (rbuf_[42] >= 128)
                  ? static_cast<float>(rbuf_[42] - 128) * -1.f
                  : static_cast<float>(rbuf_[42]);
    pub_s_(SI_TEMP, t);
    return true;
  }

  bool parse_status_() {
    if (!check_crc_(32))
      return false;

    bool relay_off = ((rbuf_[24] >> 2) & 0x03) == 1;
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
