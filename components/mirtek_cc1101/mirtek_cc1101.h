#pragma once
// =============================================================================
//  Mirtek CC1101 ESPHome External Component — МИРТЕК-32-РУ
//  Совместимость: ESPHome 2026.8.x, ESP32, Arduino framework
//
//  Источник истины: My_Mirtek_Demon.ino (https://github.com/demon7905/mirtek) —
//  ПРОВЕРЕННЫЙ рабочий скетч именно для МИРТЕК-32-РУ. Это НЕ протокол
//  "Star v1.20" (Mirtek STAR 104/304, github.com/Alecseyyy/ESPHome-Mirt-830):
//  формат ЗАПРОСА совпадает байт-в-байт, формат ОТВЕТА — нет.
//
//  Библиотека-эталон RX/TX-семантики: ELECHOUSE_CC1101_SRC_DRV.h, который в
//  актуальной версии LSatan/SmartRC-CC1101-Driver-Lib — это ссылочный
//  compatibility-заголовок на SmartRC_CC1101.{h,cpp}. Все детали SendData()/
//  ReceiveData()/CheckReceiveFlag() ниже перенесены оттуда, а не придуманы.
//
//  КЛЮЧЕВАЯ НАХОДКА (объясняет `for i=1;i<len` в packetReceiver()):
//  SmartRC_CC1101::SendData(byte*, byte) делает
//      SpiWriteReg(TXFIFO, size);              // внешний аппаратный length-префикс
//      SpiWriteBurstReg(TXFIFO, txBuffer, size);// burst СТАРТУЕТ С txBuffer[0]
//  а в packetSender() (My_Mirtek_Demon.ino) transmitt_byte[0] — это и есть
//  длина пакета, и burst шлётся именно с индекса 0. Значит в эфир уходит
//  ДУБЛИРУЮЩИЙ байт длины: сначала настоящий аппаратный префикс, затем то же
//  значение ещё раз как первый байт полезной нагрузки. Симметрично на приёме,
//  SmartRC_CC1101::ReceiveData() делает
//      size = SpiReadReg(RXFIFO);                    // 1 байт, внешний префикс
//      SpiReadBurstReg(RXFIFO, rxBuffer, size);       // rxBuffer[0..size-1]
//      SpiReadBurstReg(RXFIFO, status, 2);            // ещё 2 байта статуса (не используются)
//  rxBuffer[0] — это НЕ длина, а дублирующий байт из TX. Поэтому
//  `for(i=1;i<len;i++)` в packetReceiver() отбрасывает не полезный байт, а
//  этот дубль. Раньше я читал это ОДНИМ непрерывным burst на `lb` байт и
//  терял настоящий последний байт каждого RF-пакета — реальный баг, найден
//  и исправлен здесь (см. read_one_burst_()).
//
//  АРХИТЕКТУРА: полностью неблокирующий State Machine вместо блокирующего
//  update(). Приём RF-пакетов (до 10с суммарно) размазан по вызовам loop()
//  — каждый тик проверяется GDO0/таймаут и делается минимум работы, loop()
//  почти всегда возвращается за микросекунды. Коротким ожиданиям аппаратной
//  settle-задержки (SCAL ~2мс, детект фронта GDO0 при TX ≤1000мс суммарно —
//  два ожидания по 500мс каждое, подтверждено по SmartRC_CC1101::SendData())
//  оставлено короткое блокирующее ожидание с App.feed_wdt() внутри — это
//  сознательный компромисс, не наводнение stateʼами на 1000мс ожидания,
//  которое не может привести к срабатыванию watchdog (5с) и не заметно
//  для API/WiFi/OTA. Приём RF-подпакетов (до 10с) — вот что ОБЯЗАТЕЛЬНО
//  размазано по тикам, и это сделано.
// =============================================================================

#include "esphome/core/component.h"
#include "esphome/core/hal.h"
#include "esphome/core/log.h"
#include "esphome/core/application.h"
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

static void hex_to_(std::string &out, const uint8_t *d, size_t n) {
  char buf[4];
  for (size_t i = 0; i < n; i++) {
    snprintf(buf, sizeof(buf), "%02X ", d[i]);
    out += buf;
  }
}

// ─── CC1101 команды/регистры ─────────────────────────────────────────────────
static const uint8_t CC_SRES = 0x30;     // Software reset
static const uint8_t CC_SCAL = 0x33;     // Calibrate frequency synthesizer
static const uint8_t CC_SRX = 0x34;      // Enable RX
static const uint8_t CC_STX = 0x35;      // Enable TX
static const uint8_t CC_SIDLE = 0x36;    // Exit RX/TX
static const uint8_t CC_SFRX = 0x3A;     // Flush RX FIFO
static const uint8_t CC_SFTX = 0x3B;     // Flush TX FIFO
static const uint8_t CC_PATABLE = 0x3E;  // Output power table
static const uint8_t CC_BURST = 0x40;    // Burst access bit
static const uint8_t CC_READ = 0x80;     // Read bit
static const uint8_t CC_TXFIFO = 0x3F;
static const uint8_t CC_RXFIFO = 0x3F;
static const uint8_t CC_RXBYTES = 0x3B;  // Status reg: bytes in RX FIFO (диагностика, не триггер)
static const uint8_t CC_VERSION = 0x31;  // Status reg: chip version

// TX-мощность, выставляется перед каждой отправкой — как в packetSender()
// рабочего скетча ("выставляем мощность 10dB"). Значение 0xC4 не
// расшифровывается здесь как конкретное dBm — это просто то же самое
// значение регистра PATABLE, которое пишет рабочий скетч; TI datasheet
// таблицу перевода индекс→dBm для этой комбинации частоты/модуляции не
// подтверждает однозначно, поэтому не домысливаю точное значение мощности.
static const uint8_t CC_PATABLE_VALUE = 0xC4;

// ─── RF-конфигурация 433 МГц / FSK ───────────────────────────────────────────
// Идентична и в My_Mirtek_Demon.ino (rfSettings[]), и в ESPHome-Mirt-830
// (RF_CFG[]) — байт-в-байт, порядок регистров с адреса 0x00 (IOCFG2) по
// 0x2E (TEST0), загружается одним SpiWriteBurstReg(0x00, ..., 0x2F).
static const uint8_t RF_CFG[47] = {0x0D, 0x2E, 0x06, 0x4F, 0xD3, 0x91, 0x3C, 0x00, 0x41, 0x00, 0x16, 0x0F,
                                    0x00, 0x10, 0x8B, 0x54, 0xD9, 0x83, 0x13, 0xD2, 0xAA, 0x31, 0x07, 0x0C,
                                    0x08, 0x16, 0x6C, 0x03, 0x40, 0x91, 0x87, 0x6B, 0xF8, 0x56, 0x10, 0xE9,
                                    0x2A, 0x00, 0x1F, 0x41, 0x00, 0x59, 0x59, 0x3F, 0x81, 0x35, 0x09};

// ─── Индексы сенсоров (порядок должен совпадать со списком SENSORS в __init__.py) ──
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

enum TextIdx { TI_TARIFF = 0, TI_RELAY, TI_SEAL, TI_TYPE, TI_DATE, TI_TIME, TI_STATUS, TI_COUNT };

enum BinIdx { BI_3PH = 0, BI_RELAY, BI_SEAL, BI_CC, BI_COUNT };

// Шаги общего цикла опроса — порядок = порядок вызовов в My_Mirtek_Demon.ino
enum Phase : uint8_t {
  PH_IDLE = 0,
  PH_DATETIME,      // 0x1C
  PH_ENERGY,        // 0x05 / 0x00
  PH_INSTANT,       // 0x2B / 0x00
  PH_PHASE_POWER,   // 0x2B / 0x10 (только если three_phase_)
  PH_STATUS,        // 0x10
  PH_RELAY_ON,       // one-shot: 0x3A 0x00 0x00
  PH_RELAY_OFF,      // one-shot: 0x3A 0x00 0x01
  PH_SEAL_RESET,     // one-shot: 0x04 0x01
};

// Состояния "насоса" одной команды (TX → ждать GDO0 → RX подпакетов → готово)
enum PumpState : uint8_t {
  PS_IDLE = 0,
  PS_TX_WAIT_HIGH,   // ждём фронт GDO0 вверх (синхрослово ушло)
  PS_TX_WAIT_LOW,    // ждём фронт GDO0 вниз (конец TX-пакета)
  PS_RX_WAIT_HIGH,   // ждём фронт GDO0 вверх (пришло синхрослово подпакета)
  PS_RX_WAIT_LOW,    // ждём фронт GDO0 вниз (подпакет принят) — эквивалент CheckReceiveFlag()
  PS_DONE,
};

// =============================================================================
class MirtekCC1101 : public PollingComponent,
                      public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                             spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
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
      ESP_LOGI(TAG, "CC1101 готов. Интервал опроса: %u мс", (unsigned) get_update_interval());
    }
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway (МИРТЕК-32-РУ):");
    ESP_LOGCONFIG(TAG, "  Адрес счётчика : %u", addr_);
    ESP_LOGCONFIG(TAG, "  Интервал опроса: %u мс", (unsigned) get_update_interval());
    LOG_PIN("  GDO0 пин: ", gdo0_);
  }

  // update() ТОЛЬКО взводит флаг — никакой блокирующей работы. Реальная
  // работа делается в loop(), маленькими порциями, каждый тик.
  void update() override {
    if (phase_ != PH_IDLE) {
      ESP_LOGW(TAG, "Предыдущий цикл опроса ещё не завершён (фаза=%d) — пропускаю", (int) phase_);
      return;
    }
    ESP_LOGI(TAG, "=== Опрос, адрес=%u ===", addr_);
    poll_ok_accum_ = true;
    start_phase_(PH_DATETIME);
  }

  // loop() вызывается ESPHome очень часто (обычно каждые несколько мс) —
  // это и есть неблокирующий "тик" state machine.
  void loop() override {
    if (phase_ == PH_IDLE) {
      // Если очередь свободна — забираем отложенную one-shot команду
      // (реле/сброс пломб), если она была запрошена во время предыдущего
      // опроса. Это исключает и гонки по SPI, и повтор последней команды.
      if (oneshot_pending_) {
        oneshot_pending_ = false;
        start_phase_(oneshot_phase_);
      }
      return;
    }
    pump_tick_();
    if (pump_state_ != PS_DONE) return;

    // Пакет данной команды получен (или истёк таймаут) — разобрать и
    // перейти к следующему шагу цикла.
    bool ok = pump_ok_ && on_pump_done_();
    poll_ok_accum_ = poll_ok_accum_ && ok;
    advance_phase_(ok);
  }

  // ── Публичные методы (доступны через HA Services / кнопки) ─────────────────
  void poll_all() { update(); }

  void relay_on() { queue_oneshot_(PH_RELAY_ON); }
  void relay_off() { queue_oneshot_(PH_RELAY_OFF); }
  void reset_seals() { queue_oneshot_(PH_SEAL_RESET); }

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

  Phase phase_{PH_IDLE};
  bool poll_ok_accum_{true};
  bool oneshot_pending_{false};
  Phase oneshot_phase_{PH_IDLE};

  // "насос" одной команды
  PumpState pump_state_{PS_IDLE};
  uint8_t pump_cmd_{0};
  int pump_expected_{0};
  int pump_got_{0};
  uint32_t pump_t0_{0};      // старт общего RX-окна команды (лимит 10000 мс — TimerMs tmr(10000,0,0))
  uint32_t pump_sub_t0_{0};  // старт текущего под-ожидания (фронт GDO0)
  bool pump_ok_{false};      // заголовок (адрес+эхо команды) сошёлся
  std::vector<uint8_t> tx_stuffed_;
  std::vector<uint8_t> pump_raw_rx_;

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

  // ── Формирование пакета запроса — идентично Star и 32-РУ ─────────────────────
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

  // ── Запуск шага цикла (или one-shot команды) ──────────────────────────────────
  void start_phase_(Phase ph) {
    phase_ = ph;
    switch (ph) {
      case PH_DATETIME:     begin_command_(0x1C, -1, -1, 3); break;
      case PH_ENERGY:       begin_command_(0x05, 0x00, -1, 4); break;
      case PH_INSTANT:      begin_command_(0x2B, 0x00, -1, 4); break;
      case PH_PHASE_POWER:  begin_command_(0x2B, 0x10, -1, 4); break;
      case PH_STATUS:       begin_command_(0x10, -1, -1, 3); break;
      case PH_RELAY_ON:     ESP_LOGI(TAG, "Реле: замкнуть (включить)"); begin_command_(0x3A, 0x00, 0x00, 4); break;
      case PH_RELAY_OFF:    ESP_LOGI(TAG, "Реле: разомкнуть (выключить)"); begin_command_(0x3A, 0x00, 0x01, 4); break;
      case PH_SEAL_RESET:   ESP_LOGI(TAG, "Сброс состояния электронных пломб"); begin_command_(0x04, 0x01, -1, 4); break;
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

  // ── Запуск TX одной команды (быстрая синхронная часть — сборка + погрузка
  // в TXFIFO занимает микросекунды, это не то ожидание, что вызывало сбой) ──
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
    tx_stuffed_[0] = static_cast<uint8_t>(tx_stuffed_.size() - 1);
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, tx_stuffed_.data(), tx_stuffed_.size());
      ESP_LOGV(TAG, "TX stuffed: %s", h.c_str());
    }

    // Последовательность как в packetSender(): SCAL → SFTX → SIDLE →
    // PATABLE → загрузка TXFIFO → STX. Это короткая (единицы мс) и быстрая
    // SPI-работа, делается синхронно.
    cc_strobe_(CC_SCAL);
    delay(2);
    App.feed_wdt();
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SIDLE);
    cc_wreg_(CC_PATABLE, CC_PATABLE_VALUE);

    this->enable();
    this->transfer_byte(CC_TXFIFO | CC_BURST);
    for (uint8_t b : tx_stuffed_) this->transfer_byte(b);
    this->disable();
    cc_strobe_(CC_STX);

    pump_sub_t0_ = millis();
    pump_state_ = PS_TX_WAIT_HIGH;
  }

  // ── Один тик state machine — вызывается из loop(), никогда не блокирует
  // дольше пары мс ──────────────────────────────────────────────────────────
  void pump_tick_() {
    switch (pump_state_) {
      case PS_TX_WAIT_HIGH: {
        // Ждём фронт GDO0 вверх (синхрослово ушло) — предел 500мс, взят
        // напрямую из SmartRC_CC1101::SendData(): `while (!digitalRead(GDO0)
        // && (millis()-start<500));`. Раньше здесь стояло 200/600 —
        // непроверенные числа из более раннего поиска; сейчас поправлено
        // на подтверждённые 500/500 из реального исходника.
        if (!gdo0_ || gdo0_->digital_read() || millis() - pump_sub_t0_ > 500) {
          ESP_LOGV(TAG, "GDO0 HIGH (TX sync отправлен)");
          pump_sub_t0_ = millis();
          pump_state_ = PS_TX_WAIT_LOW;
        }
        break;
      }
      case PS_TX_WAIT_LOW: {
        // Ждём фронт GDO0 вниз (конец TX-пакета) — тот же SendData(),
        // второй `while (digitalRead(GDO0) && (millis()-start<500));`.
        if (!gdo0_ || !gdo0_->digital_read() || millis() - pump_sub_t0_ > 500) {
          ESP_LOGV(TAG, "GDO0 LOW (TX завершён)");
          cc_strobe_(CC_SFRX);
          cc_strobe_(CC_SRX);
          pump_t0_ = millis();  // старт общего 10-секундного RX-окна команды
          pump_sub_t0_ = millis();
          pump_state_ = PS_RX_WAIT_HIGH;
        }
        break;
      }
      case PS_RX_WAIT_HIGH: {
        if (millis() - pump_t0_ > 10000) {  // TimerMs tmr(10000,0,0) из packetReceiver()
          finish_pump_();
          return;
        }
        if (gdo0_ && gdo0_->digital_read()) {
          ESP_LOGV(TAG, "GDO0 HIGH (RX подпакет %d/%d, синхрослово)", pump_got_ + 1, pump_expected_);
          pump_sub_t0_ = millis();
          pump_state_ = PS_RX_WAIT_LOW;
        }
        // без GDO0 (не настроен) — деградируем на диагностику по RXBYTES ниже
        else if (!gdo0_) {
          uint8_t rxb = cc_rstat_(CC_RXBYTES);
          if (rxb > 0 && rxb < 64) read_burst_and_rearm_();
        }
        break;
      }
      case PS_RX_WAIT_LOW: {
        // Эквивалент CheckReceiveFlag():
        // ждём фактического перехода GDO0 HIGH -> LOW.
        //
        // Если GDO0 не опустился за 200 мс — это ошибка приёма,
        // а не успешно принятый пакет.
        if (!gdo0_->digital_read()) {
          ESP_LOGV(TAG, "GDO0 LOW (RX подпакет %d/%d принят)",
                   pump_got_ + 1, pump_expected_);
          read_burst_and_rearm_();
        } else if (millis() - pump_sub_t0_ > 200) {
          ESP_LOGW(TAG,
                   "RX timeout: GDO0 не опустился за 200 мс "
                   "(подпакет %d/%d)",
                   pump_got_ + 1, pump_expected_);
          finish_pump_();
        } else if (millis() - pump_t0_ > 10000) {
          finish_pump_();
        }
        break;
      }
      default: break;
    }
  }

  void read_burst_and_rearm_() {
    read_one_burst_();
    pump_got_++;
    // Как в packetReceiver() после каждого подпакета: SIDLE, SFRX, SFTX, SRX
    cc_strobe_(CC_SIDLE);
    cc_strobe_(CC_SFRX);
    cc_strobe_(CC_SFTX);
    cc_strobe_(CC_SRX);
    App.feed_wdt();
    if (pump_got_ >= pump_expected_) {
      finish_pump_();
    } else {
      pump_sub_t0_ = millis();
      pump_state_ = gdo0_ ? PS_RX_WAIT_HIGH : PS_RX_WAIT_HIGH;
    }
  }

  // ── Чтение одного RF-подпакета — 3 РАЗДЕЛЬНЫЕ SPI-транзакции, как в
  // настоящей SmartRC_CC1101::ReceiveData(), а не одна непрерывная посылка:
  //   1) один небайтовый (не burst) регистровый READ RXFIFO → внешняя длина
  //   2) burst READ на `outer_len` байт → это и есть rxBuffer[0..outer_len-1]
  //   3) burst READ ещё 2 байт статуса (не используются, но реальная
  //      библиотека их всегда читает — повторяю для идентичного поведения
  //      шины; в нашем RF_CFG APPEND_STATUS выключен (PKTCTRL1=0x00), так
  //      что эти 2 байта — не настоящий RSSI/LQI, а что бы ни было дальше в
  //      FIFO; они гарантированно отбрасываются и FIFO сразу флашится)
  // Затем, как Arduino: пропускаем rxBuffer[0] (см. большой комментарий в
  // начале файла — это дубль байта длины из TX, а не полезные данные).
  // ──────────────────────────────────────────────────────────────────────────
  void read_one_burst_() {
    this->enable();
    this->transfer_byte(CC_READ | CC_RXFIFO);
    uint8_t outer_len = this->transfer_byte(0x00);
    this->disable();
    ESP_LOGV(TAG, "RX length (подпакет %d/%d): %u", pump_got_ + 1, pump_expected_, outer_len);

    if (outer_len == 0 || outer_len >= 60) {
      ESP_LOGW(TAG, "Подозрительная длина RX-подпакета: %u", outer_len);
      return;
    }

    uint8_t burst[64];
    this->enable();
    this->transfer_byte(CC_READ | CC_BURST | CC_RXFIFO);
    for (uint8_t i = 0; i < outer_len; i++) burst[i] = this->transfer_byte(0x00);
    this->disable();
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, burst, outer_len);
      ESP_LOGV(TAG, "RX payload (подпакет %d/%d): %s", pump_got_ + 1, pump_expected_, h.c_str());
    }

    // 2 "статусных" байта — в нашем RF_CFG APPEND_STATUS выключен
    // (PKTCTRL1=0x00), так что это не настоящие RSSI/LQI, а что бы ни было
    // следом в FIFO; читаются и отбрасываются, как делает сама библиотека
    // (она их тоже нигде не использует, только читает и сразу флашит FIFO).
    uint8_t status[2];
    this->enable();
    this->transfer_byte(CC_READ | CC_BURST | CC_RXFIFO);
    status[0] = this->transfer_byte(0x00);
    status[1] = this->transfer_byte(0x00);
    this->disable();
    ESP_LOGV(TAG, "RX status (подпакет %d/%d, не используется): %02X %02X", pump_got_ + 1, pump_expected_,
              status[0], status[1]);

    for (uint8_t i = 1; i < outer_len; i++) pump_raw_rx_.push_back(burst[i]);
  }

  // ── Завершение приёма команды: destuff, проверка заголовка, публикация
  // диагностики ──────────────────────────────────────────────────────────────
  void finish_pump_() {
    pump_state_ = PS_DONE;

    if (pump_raw_rx_.empty()) {
      ESP_LOGW(TAG, "Нет ответа на cmd=0x%02X (получено 0 из %d подпакетов)", pump_cmd_, pump_expected_);
      pump_ok_ = false;
      return;
    }

    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, pump_raw_rx_.data(), pump_raw_rx_.size());
      ESP_LOGV(TAG, "RX assembled (%d/%d подпакетов): %s", pump_got_, pump_expected_, h.c_str());
    }

    std::vector<uint8_t> ds;
    destuff_(pump_raw_rx_.data(), pump_raw_rx_.size(), ds);
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      std::string h;
      hex_to_(h, ds.data(), ds.size());
      ESP_LOGV(TAG, "DESTUFF: %s", h.c_str());
    }

    if (ds.size() > sizeof(rbuf_)) {
      ESP_LOGW(TAG, "Переполнение буфера (cmd=0x%02X, %u байт)", pump_cmd_, (unsigned) ds.size());
      pump_ok_ = false;
      return;
    }
    memcpy(rbuf_, ds.data(), ds.size());
    rlen_ = ds.size();

    // Заголовок ответа у 32-РУ: [0]=0x73 [1]=0x55 ... [6]=addr_lo [7]=addr_hi
    // [8]=эхо команды — взято из packetParser_1..6 рабочего скетча.
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
      ESP_LOGW(TAG, "Несоответствие эха команды (ждали 0x%02X, получили 0x%02X)", pump_cmd_, rbuf_[8]);
      pump_ok_ = false;
      return;
    }
    pump_ok_ = true;
  }

  // ── Диспетчер парсеров по текущей фазе — вызывается из loop() после
  // pump_state_==PS_DONE и pump_ok_==true ─────────────────────────────────────
  bool on_pump_done_() {
    switch (phase_) {
      case PH_DATETIME:     return parse_datetime_();
      case PH_ENERGY:       return parse_energy_();
      case PH_INSTANT:      return three_phase_ ? parse_instant_3ph_() : parse_instant_1ph_();
      case PH_PHASE_POWER:  return parse_phase_();
      case PH_STATUS:       return parse_status_();
      case PH_RELAY_ON:
      case PH_RELAY_OFF:
      case PH_SEAL_RESET:   return true;  // ответ не разбирается, как в рабочем скетче
      default: return false;
    }
  }

  // ── CRC-проверка ответа: считается по rbuf_[2 .. crc_pos-1], сверяется с
  // rbuf_[crc_pos], затем проверяется стоп-байт rbuf_[crc_pos+1]==0x55. ────────
  bool check_crc_(size_t crc_pos) {
    if (rlen_ <= crc_pos + 1) return false;
    uint8_t calc = crc8_mirtek(rbuf_ + 2, crc_pos - 2);
    bool ok = (calc == rbuf_[crc_pos]) && (rbuf_[crc_pos + 1] == 0x55);
    if (ESPHOME_LOG_LEVEL >= ESPHOME_LOG_LEVEL_VERBOSE) {
      ESP_LOGV(TAG, "CRC calculated=0x%02X received=0x%02X stop=0x%02X → %s", calc, rbuf_[crc_pos],
                rbuf_[crc_pos + 1], ok ? "OK" : "ERROR");
    }
    if (!ok) ESP_LOGW(TAG, "Ошибка CRC: расчитано 0x%02X, получено 0x%02X", calc, rbuf_[crc_pos]);
    return ok;
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

  bool parse_datetime_() {
    if (!check_crc_(20)) return false;
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
    if (!check_crc_(43)) return false;
    uint8_t cur_t = (rbuf_[14] >> 2) & 0x03;
    const char *tn[] = {"День", "Ночь", "Полупик", "Специальный"};
    pub_txt_(TI_TARIFF, tn[cur_t]);
    pub_s_(SI_SUM, u32_(19) / 100.f);
    pub_s_(SI_T1, u32_(27) / 100.f);
    pub_s_(SI_T2, u32_(31) / 100.f);
    return true;
  }

  bool parse_instant_3ph_() {
    if (!check_crc_(43)) return false;
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
    float t = (rbuf_[42] >= 128) ? static_cast<float>(rbuf_[42] - 128) * -1.f : static_cast<float>(rbuf_[42]);
    pub_s_(SI_TEMP, t);
    return true;
  }

  bool parse_status_() {
    if (!check_crc_(32)) return false;
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
