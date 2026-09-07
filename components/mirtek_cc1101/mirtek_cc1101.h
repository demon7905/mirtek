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

namespace esphome {
namespace mirtek_cc1101 {

static const char *const TAG = "mirtek32ru";

static uint8_t crc8_mirtek(const uint8_t *d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    uint8_t b = d[i];
    for (int j = 0; j < 8; j++)
      c = ((b ^ c) & 0x80) ? (uint8_t)((c << 1) ^ 0xA9) : (uint8_t)(c << 1), b <<= 1;
  }
  return c;
}

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

static const uint8_t RF_CFG[47] = {
  0x0D,0x2E,0x06,0x4F,0xD3,0x91,0x3C,0x00,0x41,0x00,0x16,0x0F,
  0x00,0x10,0x8B,0x54,0xD9,0x83,0x13,0xD2,0xAA,0x31,0x07,0x0C,
  0x08,0x16,0x6C,0x03,0x40,0x91,0x87,0x6B,0xF8,0x56,0x10,0xE9,
  0x2A,0x00,0x1F,0x41,0x00,0x59,0x59,0x3F,0x81,0x35,0x09
};

enum SensorIdx {
  SI_SUM=0, SI_T1, SI_T2, SI_KW, SI_KVAR, SI_FREQ, SI_COS,
  SI_V1, SI_V2, SI_V3, SI_I1, SI_I2, SI_I3,
  SI_PA, SI_PB, SI_PC, SI_QA, SI_QB, SI_QC,
  SI_SA, SI_SB, SI_SC, SI_CA, SI_CB, SI_CC, SI_TEMP, SI_COUNT
};
enum TextIdx { TI_TARIFF=0, TI_RELAY, TI_SEAL, TI_TYPE, TI_DATE, TI_TIME, TI_STATUS, TI_COUNT };
enum BinIdx { BI_3PH=0, BI_RELAY, BI_SEAL, BI_CC, BI_COUNT };

class MirtekCC1101 : public PollingComponent,
                     public spi::SPIDevice<spi::BIT_ORDER_MSB_FIRST, spi::CLOCK_POLARITY_LOW,
                                          spi::CLOCK_PHASE_LEADING, spi::DATA_RATE_4MHZ> {
 public:
  void set_gdo0_pin(GPIOPin *p) { gdo0_ = p; }
  void set_meter_address(int a) { addr_ = static_cast<uint16_t>(a); }
  void set_sensor(int i, sensor::Sensor *s) { if (i >= 0 && i < SI_COUNT) ss_[i] = s; }
  void set_text_sensor(int i, text_sensor::TextSensor *s) { if (i >= 0 && i < TI_COUNT) ts_[i] = s; }
  void set_binary_sensor(int i, binary_sensor::BinarySensor *s) { if (i >= 0 && i < BI_COUNT) bs_[i] = s; }

  void setup() override {
    ESP_LOGI(TAG, "Инициализация CC1101, адрес счётчика=%u", addr_);
    this->spi_setup();
    if (gdo0_) gdo0_->setup();
    bool ok = cc_init_();
    pub_bin_(BI_CC, ok);
    pub_txt_(TI_STATUS, ok ? "CC1101 OK" : "CC1101 ERR");
    if (!ok) ESP_LOGE(TAG, "CC1101 не обнаружен! Проверьте подключение SPI.");
    else ESP_LOGI(TAG, "CC1101 готов. Интервал опроса: %u мс", get_update_interval());
  }

  void dump_config() override {
    ESP_LOGCONFIG(TAG, "Mirtek CC1101 Gateway (МИРТЕК-32-РУ):");
    ESP_LOGCONFIG(TAG, "  Адрес счётчика : %u", addr_);
    ESP_LOGCONFIG(TAG, "  Интервал опроса: %u мс", get_update_interval());
    LOG_PIN("  GDO0 пин: ", gdo0_);
  }

  void update() override { poll_all(); }

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

  void relay_on() { ESP_LOGI(TAG, "Реле: замкнуть (включить)"); do_cmd_(0x3A, 0x00, 0x00, 4); }
  void relay_off() { ESP_LOGI(TAG, "Реле: разомкнуть (выключить)"); do_cmd_(0x3A, 0x00, 0x01, 4); }
  void reset_seals() { ESP_LOGI(TAG, "Сброс состояния электронных пломб"); do_cmd_(0x04, 0x01, -1, 4); }

 protected:
  GPIOPin *gdo0_{nullptr};
  uint16_t addr_{1};
  bool three_phase_{true};
  sensor::Sensor *ss_[SI_COUNT]{};
  text_sensor::TextSensor *ts_[TI_COUNT]{};
  binary_sensor::BinarySensor *bs_[BI_COUNT]{};
  uint8_t sbuf_[24]{};
  uint8_t rbuf_[64]{};
  size_t rlen_{0};

  void cc_strobe_(uint8_t cmd) { this->enable(); this->transfer_byte(cmd); this->disable(); }
  void cc_wreg_(uint8_t reg, uint8_t val) { this->enable(); this->transfer_byte(reg & 0x3F); this->transfer_byte(val); this->disable(); }
  void cc_wburst_(uint8_t reg, const uint8_t *data, size_t len) {
    this->enable(); this->transfer_byte((reg & 0x3F) | CC_BURST); for (size_t i=0;i<len;i++) this->transfer_byte(data[i]); this->disable();
  }
  uint8_t cc_rstat_(uint8_t reg) {
    this->enable(); this->transfer_byte(CC_READ | CC_BURST | (reg & 0x3F)); uint8_t v=this->transfer_byte(0x00); this->disable(); return v;
  }

  bool cc_init_() {
    cc_strobe_(CC_SRES); delay(10);
    cc_wburst_(0x00, RF_CFG, sizeof(RF_CFG));
    cc_strobe_(CC_SCAL); delay(2);
    cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
    uint8_t ver=cc_rstat_(CC_VERSION);
    ESP_LOGD(TAG,"CC1101 version=0x%02X",ver);
    return ver==0x14 || ver==0x04;
  }

  void stuff_(const uint8_t *in,size_t n,std::vector<uint8_t>&out) {
    out.push_back(in[0]); out.push_back(in[1]); out.push_back(in[2]);
    for(size_t i=3;i<n-1;i++){ if(in[i]==0x55){out.push_back(0x73);out.push_back(0x11);} else if(in[i]==0x73){out.push_back(0x73);out.push_back(0x22);} else out.push_back(in[i]); }
    out.push_back(in[n-1]);
  }
  void destuff_(const uint8_t *in,size_t n,std::vector<uint8_t>&out) {
    for(size_t i=0;i<n;i++){
      if(in[i]==0x73 && i+1<n){ if(in[i+1]==0x11){out.push_back(0x55);i++;} else if(in[i+1]==0x22){out.push_back(0x73);i++;} else out.push_back(in[i]); }
      else out.push_back(in[i]);
    }
  }

  size_t build_pkt_(uint8_t cmd,int sub1,int sub2){
    uint8_t *b=sbuf_; uint8_t dl=(sub1<0)?0:(sub2<0)?1:2;
    b[0]=0x0F+dl;b[1]=0x73;b[2]=0x55;b[3]=0x20+dl;b[4]=0x00;
    b[5]=addr_&0xFF;b[6]=(addr_>>8)&0xFF;b[7]=0xFE;b[8]=0xFF;b[9]=cmd;
    b[10]=b[11]=b[12]=b[13]=0x00; size_t p=14;
    if(sub1>=0)b[p++]=(uint8_t)sub1; if(sub2>=0)b[p++]=(uint8_t)sub2;
    b[p]=crc8_mirtek(b+3,p-3); b[p+1]=0x55; return p+2;
  }

  bool do_cmd_(uint8_t cmd,int sub1,int sub2,int expected_pkts){
    size_t raw_len=build_pkt_(cmd,sub1,sub2); std::vector<uint8_t> tx; stuff_(sbuf_,raw_len,tx); tx[0]=(uint8_t)(tx.size()-1);
    cc_strobe_(CC_SCAL); delay(1); cc_strobe_(CC_SFTX); cc_strobe_(CC_SIDLE); cc_wreg_(CC_PATABLE,CC_PATABLE_VALUE);
    this->enable(); this->transfer_byte(CC_TXFIFO|CC_BURST); for(uint8_t byte:tx)this->transfer_byte(byte); this->disable();
    cc_strobe_(CC_STX);
    if(gdo0_){
      uint32_t t0=millis();
      while(!gdo0_->digital_read() && millis()-t0<200UL) delay(1);
      while(gdo0_->digital_read() && millis()-t0<600UL) delay(1);
    } else delay(100);

    cc_strobe_(CC_SFRX); cc_strobe_(CC_SRX);
    std::vector<uint8_t> raw_rx; int got=0; uint32_t t0=millis();
    while(millis()-t0<10000UL && got<expected_pkts){
      uint8_t rxb=cc_rstat_(CC_RXBYTES);
      if(rxb>0 && rxb<64){
        got++;
        this->enable(); this->transfer_byte(CC_RXFIFO|CC_READ|CC_BURST); uint8_t lb=this->transfer_byte(0x00);
        if(lb>0 && lb<60) for(uint8_t i=1;i<lb;i++) raw_rx.push_back(this->transfer_byte(0x00));
        this->disable(); cc_strobe_(CC_SIDLE); cc_strobe_(CC_SFRX); cc_strobe_(CC_SFTX); cc_strobe_(CC_SRX);
      }
      // FIX: yield to FreeRTOS/ESPHome so loopTask watchdog is serviced.
      delay(1);
    }
    if(raw_rx.empty()){ESP_LOGW(TAG,"Нет ответа на cmd=0x%02X",cmd);return false;}
    std::vector<uint8_t> ds; destuff_(raw_rx.data(),raw_rx.size(),ds);
    if(ds.size()>sizeof(rbuf_)){ESP_LOGW(TAG,"Переполнение буфера");return false;}
    memcpy(rbuf_,ds.data(),ds.size()); rlen_=ds.size();
    if(rlen_<9 || rbuf_[0]!=0x73 || rbuf_[1]!=0x55){ESP_LOGW(TAG,"Неверный заголовок (cmd=0x%02X)",cmd);return false;}
    if(rbuf_[6]!=(addr_&0xFF) || rbuf_[7]!=((addr_>>8)&0xFF)){ESP_LOGW(TAG,"Несоответствие адреса (cmd=0x%02X)",cmd);return false;}
    if(rbuf_[8]!=cmd){ESP_LOGW(TAG,"Несоответствие эха команды (ждали 0x%02X, получили 0x%02X)",cmd,rbuf_[8]);return false;}
    ESP_LOGD(TAG,"RX cmd=0x%02X: %d пакет(ов), %u байт",cmd,got,(unsigned)rlen_);
    return true;
  }

  bool check_crc_(size_t crc_pos){
    if(rlen_<=crc_pos+1)return false;
    uint8_t calc=crc8_mirtek(rbuf_+2,crc_pos-2);
    if(calc!=rbuf_[crc_pos] || rbuf_[crc_pos+1]!=0x55){ESP_LOGW(TAG,"Ошибка CRC: рассчитано 0x%02X, получено 0x%02X",calc,rbuf_[crc_pos]);return false;}
    return true;
  }
  float u16_(size_t i){return (float)((uint16_t)rbuf_[i]|((uint16_t)rbuf_[i+1]<<8));}
  float u24_(size_t i){return (float)((uint32_t)rbuf_[i]|((uint32_t)rbuf_[i+1]<<8)|((uint32_t)rbuf_[i+2]<<16));}
  float u32_(size_t i){return (float)((uint32_t)rbuf_[i]|((uint32_t)rbuf_[i+1]<<8)|((uint32_t)rbuf_[i+2]<<16)|((uint32_t)rbuf_[i+3]<<24));}
  float s16m_(size_t i,float div){bool neg=rbuf_[i+1]>=128; float v=(float)((uint16_t)rbuf_[i]|((uint16_t)(rbuf_[i+1]&0x7F)<<8))/div; return neg?-v:v;}
  float s24m_(size_t i,float div){bool neg=rbuf_[i+2]>=128; float v=(float)((uint32_t)rbuf_[i]|((uint32_t)rbuf_[i+1]<<8)|((uint32_t)(rbuf_[i+2]&0x7F)<<16))/div; return neg?-v:v;}
  void pub_s_(int i,float v){if(ss_[i])ss_[i]->publish_state(v);}
  void pub_txt_(int i,const std::string&v){if(ts_[i])ts_[i]->publish_state(v);}
  void pub_bin_(int i,bool v){if(bs_[i])bs_[i]->publish_state(v);}

  bool parse_datetime_(){
    if(!check_crc_(20))return false; uint8_t tp=rbuf_[9];
    switch(tp){case 0xA8:three_phase_=true;pub_txt_(TI_TYPE,"3ф трансформаторный активно-реактивный");break;case 0x98:three_phase_=false;pub_txt_(TI_TYPE,"1ф 2х элементный активно-реактивный");break;default:{char tb[24];snprintf(tb,sizeof(tb),"тип 0x%02X (неизвестен)",tp);pub_txt_(TI_TYPE,tb);ESP_LOGW(TAG,"Неизвестный тип счётчика 0x%02X — оставляю three_phase=%d",tp,three_phase_);break;}}
    pub_bin_(BI_3PH,three_phase_); char tm[10],dt[12]; snprintf(tm,sizeof(tm),"%02d:%02d:%02d",rbuf_[15],rbuf_[14],rbuf_[13]); snprintf(dt,sizeof(dt),"%02d.%02d.%02d",rbuf_[17],rbuf_[18],rbuf_[19]); pub_txt_(TI_TIME,tm); pub_txt_(TI_DATE,dt); ESP_LOGI(TAG,"Дата/время счётчика: %s %s, тип=0x%02X",dt,tm,tp); return true;
  }
  bool parse_energy_(){if(!check_crc_(43))return false;uint8_t cur_t=(rbuf_[14]>>2)&0x03;const char*tn[]={"День","Ночь","Полупик","Специальный"};pub_txt_(TI_TARIFF,tn[cur_t]);pub_s_(SI_SUM,u32_(19)/100.f);pub_s_(SI_T1,u32_(27)/100.f);pub_s_(SI_T2,u32_(31)/100.f);ESP_LOGI(TAG,"кВт·ч: SUM=%.2f T1=%.2f T2=%.2f, тариф=%s",ss_[SI_SUM]?ss_[SI_SUM]->state:0.f,ss_[SI_T1]?ss_[SI_T1]->state:0.f,ss_[SI_T2]?ss_[SI_T2]->state:0.f,tn[cur_t]);return true;}
  bool parse_instant_3ph_(){if(!check_crc_(43))return false;pub_s_(SI_KW,u24_(18));pub_s_(SI_KVAR,s24m_(21,1000.f));pub_s_(SI_FREQ,u16_(24)/100.f);pub_s_(SI_COS,s16m_(26,1000.f));pub_s_(SI_V1,u16_(28)/100.f);pub_s_(SI_V2,u16_(30)/100.f);pub_s_(SI_V3,u16_(32)/100.f);pub_s_(SI_I1,u24_(34)/1000.f);pub_s_(SI_I2,u24_(37)/1000.f);pub_s_(SI_I3,u24_(40)/1000.f);return true;}
  bool parse_instant_1ph_(){bool ok=(rlen_>43&&rbuf_[43]==0x55)||check_crc_(41);if(!ok){ESP_LOGW(TAG,"Ошибка контроля пакета 0x2B (1ph/старый вариант)");return false;}pub_s_(SI_KW,u16_(18));pub_s_(SI_KVAR,s16m_(20,1000.f));pub_s_(SI_FREQ,u16_(22)/100.f);pub_s_(SI_COS,s16m_(24,1000.f));pub_s_(SI_V1,u16_(26)/100.f);pub_s_(SI_V2,u16_(28)/100.f);pub_s_(SI_V3,u16_(30)/100.f);pub_s_(SI_I1,u24_(32)/1000.f);pub_s_(SI_I2,u24_(35)/1000.f);pub_s_(SI_I3,u24_(38)/1000.f);return true;}
  bool parse_phase_(){if(!check_crc_(43))return false;pub_s_(SI_CA,s16m_(18,1000.f));pub_s_(SI_CB,s16m_(20,1000.f));pub_s_(SI_CC,s16m_(22,1000.f));pub_s_(SI_PA,u16_(24));pub_s_(SI_PB,u16_(26));pub_s_(SI_PC,u16_(28));pub_s_(SI_QA,s16m_(30,1000.f));pub_s_(SI_QB,s16m_(32,1000.f));pub_s_(SI_QC,s16m_(34,1000.f));pub_s_(SI_SA,u16_(36));pub_s_(SI_SB,u16_(38));pub_s_(SI_SC,u16_(40));float t=(rbuf_[42]>=128)?(float)(rbuf_[42]-128)*-1.f:(float)rbuf_[42];pub_s_(SI_TEMP,t);return true;}
  bool parse_status_(){if(!check_crc_(32))return false;bool relay_off=((rbuf_[24]>>2)&0x03)==1;pub_txt_(TI_RELAY,relay_off?"Выкл":"Вкл");pub_bin_(BI_RELAY,!relay_off);uint8_t seal=rbuf_[28];const char*seal_str;bool seal_ok=(seal==0);switch(seal){case 0:seal_str="OK";break;case 1:seal_str="Вскрыта пломба клеммника";break;case 2:seal_str="Вскрыта пломба корпуса";break;case 3:seal_str="Вскрыта пломба клеммника+корпуса";break;default:seal_str="Неизвестно";break;}pub_txt_(TI_SEAL,seal_str);pub_bin_(BI_SEAL,seal_ok);ESP_LOGI(TAG,"Реле=%s Пломбы=%s",relay_off?"Выкл":"Вкл",seal_str);return true;}
};

}  // namespace mirtek_cc1101
}  // namespace esphome
