//Original firmware by Little_S@tan
//Modifed firmware by SysCat
//Self-mod and idea firmware by DigiBoy, yuriu5985, MrGhost
//Testing by scorch, yuriu5985, MrGhost
//Respect for Dismas
//MQTT topics: "mirtek/xxx/#", "mirtek/xxx/action, xxx - adress"

#include <MQTT.h>
#include <IotWebConf.h>
#include <IotWebConfUsing.h> // This loads aliases for easier class names.
#include <ELECHOUSE_CC1101_SRC_DRV.h>
#include "CRC8.h"
#include <TimerMs.h>
const int tmr_mqtt_time = 1 * 60 * 1000; //раз в 5 минут запрашиваем и отправляем информацию в mqtt

//Настройки для CC1101 с форума (47 бит)
byte rfSettings[] = {
  0x0D,  // IOCFG2              GDO2 Output Pin Configuration
  0x2E,  // IOCFG1              GDO1 Output Pin Configuration
  0x06,  // IOCFG0              GDO0 Output Pin Configuration
  0x4F,  // FIFOTHR             RX FIFO and TX FIFO Thresholds
  0xD3,  // SYNC1               Sync Word, High Byte
  0x91,  // SYNC0               Sync Word, Low Byte
  0x3C,  // PKTLEN              Packet Length
  0x00,  // PKTCTRL1            Packet Automation Control
  0x41,  // PKTCTRL0            Packet Automation Control
  0x00,  // ADDR                Device Address
  0x16,  // CHANNR              Channel Number
  //0x0B,  // CHANNR              Channel Number
  0x0F,  // FSCTRL1             Frequency Synthesizer Control
  0x00,  // FSCTRL0             Frequency Synthesizer Control
  0x10,  // FREQ2               Frequency Control Word, High Byte
  0x8B,  // FREQ1               Frequency Control Word, Middle Byte
  0x54,  // FREQ0               Frequency Control Word, Low Byte
  0xD9,  // MDMCFG4             Modem Configuration
  0x83,  // MDMCFG3             Modem Configuration
  0x13,  // MDMCFG2             Modem Configuration
  0xD2,  // MDMCFG1             Modem Configuration
  0xAA,  // MDMCFG0             Modem Configuration
  0x31,  // DEVIATN             Modem Deviation Setting
  0x07,  // MCSM2               Main Radio Control State Machine Configuration
  0x0C,  // MCSM1               Main Radio Control State Machine Configuration
  0x08,  // MCSM0               Main Radio Control State Machine Configuration
  0x16,  // FOCCFG              Frequency Offset Compensation Configuration
  0x6C,  // BSCFG               Bit Synchronization Configuration
  0x03,  // AGCCTRL2            AGC Control
  0x40,  // AGCCTRL1            AGC Control
  0x91,  // AGCCTRL0            AGC Control
  0x87,  // WOREVT1             High Byte Event0 Timeout
  0x6B,  // WOREVT0             Low Byte Event0 Timeout
  0xF8,  // WORCTRL             Wake On Radio Control
  0x56,  // FREND1              Front End RX Configuration
  0x10,  // FREND0              Front End TX Configuration
  0xE9,  // FSCAL3              Frequency Synthesizer Calibration
  0x2A,  // FSCAL2              Frequency Synthesizer Calibration
  0x00,  // FSCAL1              Frequency Synthesizer Calibration
  0x1F,  // FSCAL0              Frequency Synthesizer Calibration
  0x41,  // RCCTRL1             RC Oscillator Configuration
  0x00,  // RCCTRL0             RC Oscillator Configuration
  0x59,  // FSTEST              Frequency Synthesizer Calibration Control
  0x59,  // PTEST               Production Test
  0x3F,  // AGCTEST             AGC Test
  0x81,  // TEST2               Various Test Settings
  0x35,  // TEST1               Various Test Settings
  //0x0B,  // TEST0               Various Test Settings
  0x09,  // TEST0               Various Test Settings
};

int gdo0 = 22; //for esp32! GDO0 on GPIO pin 2.
//char MeterAdressValue[] = "56655"; //адрес счётчика
CRC8 crc;
//byte transmitt_byte[17] = { 0 }; //массив отправляемого пакета
byte send_tempbuffer[17] = { 0 }; //массив отправляемого пакета
byte buffer[50] = { 0 }; //буффер пакетов, принятых из трансивера (или отправленных в трансивер)
byte resultbuffer[50] = { 0 }; //буфер конечного, сшитого принятого пакета после байтстаффинга
byte tempbuffer[50] = { 0 }; //буфер промежуточного, сшитого принятого пакета
int bytecounttemp = 0; //указатель байтов в промежуточном буфере
int bytecount = 0; //указатель байтов в результирующем буфере
int packetType = 0; //кол-во подпакетов в ответе - зависит от типа запроса (3 для запросов 1-4, 4 для запросов 5,6, осталось от старого кода, пока конкретно не разобрались)
TimerMs tmr(10000, 0, 0); //инициализируем таймер
TimerMs tmr_mqtt(tmr_mqtt_time, 0, 0); //инициализируем таймер, отправляющий значения в MQTT

// -- Initial name of the Thing. Used e.g. as SSID of the own Access Point.
const char thingName[] = "Mirtek_GW";

// -- Initial password to connect to the Thing, when it creates an own Access Point.
const char wifiInitialApPassword[] = "12345678";

#define STRING_LEN 128
#define NUMBER_LEN 6

// -- Configuration specific key. The value should be modified if config structure was changed.
#define CONFIG_VERSION "mirtek_gw_v1"

// -- Status indicator pin.
//      First it will light up (kept LOW), on Wifi connection it will blink,
//      when connected to the Wifi it will turn off (kept HIGH).
#define STATUS_PIN 2 //16

// -- Method declarations.
void handleRoot();
void mqttMessageReceived(String& topic, String& payload);
bool connectMqtt();
bool connectMqttOptions();
// -- Callback methods.
void wifiConnected();
void configSaved();
bool formValidator(iotwebconf::WebRequestWrapper* webRequestWrapper);

DNSServer dnsServer;
WebServer server(80);
WiFiClient net;
MQTTClient mqttClient;

char mqttServerValue[STRING_LEN];
char mqttUserNameValue[STRING_LEN];
char mqttUserPasswordValue[STRING_LEN];
char MeterAdressValue[NUMBER_LEN];
char mqttPortValue[6] = "1883"; // порт MQTT, по умолчанию 1883


IotWebConf iotWebConf(thingName, &dnsServer, &server, wifiInitialApPassword, CONFIG_VERSION);
// -- You can also use namespace formats e.g.: iotwebconf::ParameterGroup
IotWebConfParameterGroup mqttGroup = IotWebConfParameterGroup("mqtt", "MQTT configuration");
IotWebConfTextParameter mqttServerParam = IotWebConfTextParameter("MQTT server", "mqttServer", mqttServerValue, STRING_LEN);
IotWebConfTextParameter mqttUserNameParam = IotWebConfTextParameter("MQTT user", "mqttUser", mqttUserNameValue, STRING_LEN);
IotWebConfPasswordParameter mqttUserPasswordParam = IotWebConfPasswordParameter("MQTT password", "mqttPass", mqttUserPasswordValue, STRING_LEN);
// MQTT порт
IotWebConfNumberParameter mqttPortParam = IotWebConfNumberParameter("MQTT port", "mqttPort", mqttPortValue, 6, "1883", "1..65535", "min='1' max='65535' step='1'");

//Дополнительные параметры
IotWebConfParameterGroup group1 = IotWebConfParameterGroup("group1", "Настройки");
IotWebConfNumberParameter MeterAdress = IotWebConfNumberParameter("Адрес счётчика", "MeterAdress", MeterAdressValue, NUMBER_LEN, "", "1..65000", "min='1' max='65000' step='1'");

bool needMqttConnect = false;
bool needReset = false;
bool horizontal = true; //Горизонтальная или вертикальная таблица
bool three_phase = true; //По-умолчанию считаем что у нас 3-фазный счётчик, если опредим что однофазный то меняем поведение
String type_counter = "Идентификация счётчика не произведена";
int pinState = HIGH;
unsigned long lastReport = 0;
unsigned long lastMqttConnectionAttempt = 0;
byte myCRC = 0;
String packet = "";

// Переменные для показаний счетчика
float sum = 0;
float t1 = 0;
float t2 = 0;
//float t3 = 0;
//float t4 = 0;
float v1 = 0;
float v2 = 0;
float v3 = 0;
float i1 = 0;
float i2 = 0;
float i3 = 0;
float cosin = 0; //Общий косинус фи
float cosinA = 0;
float cosinB = 0;
float cosinC = 0;
float P = 0; //Общая мощность активная
float Pa = 0;
float Pb = 0;
float Pc = 0;
float Q = 0; //Общая мощность реактивная
float Qa = 0;
float Qb = 0;
float Qc = 0;
float Sa = 0;
float Sb = 0;
float Sc = 0;
float T = 0;
float freq = 0;

String last_req = "Нет данных";
String tariff = "Нет данных";
String rele = "Нет данных";
String plomb = "Нет данных";

//Функция формирования короткого пакета 0x20
void RequestPacketShort(byte comm, int pt) {
  send_tempbuffer[0] = 0x0f; //длина пакета 15 байт
  send_tempbuffer[1] = 0x73;
  send_tempbuffer[2] = 0x55; //начало payload
  send_tempbuffer[3] = 0x20; //тип запроса/пакета
  send_tempbuffer[4] = 0x00; //
  send_tempbuffer[5] = (atoi(MeterAdressValue)) & 0xff; //младший байт адреса счётчика
  send_tempbuffer[6] = ((atoi(MeterAdressValue)) >> 8) & 0xff; //старший байт адреса счётчика
  send_tempbuffer[7] = 0xfe; // 0x09
  send_tempbuffer[8] = 0xff; //
  send_tempbuffer[9] = comm; //команда
  send_tempbuffer[10] = 0x00; //PIN
  send_tempbuffer[11] = 0x00; //PIN
  send_tempbuffer[12] = 0x00; //PIN
  send_tempbuffer[13] = 0x00; //PIN
  //вычисляем и добавляем байт crc
  crc.restart();
  crc.setPolynome(0xA9);
  for (int i = 3; i < (send_tempbuffer[0] - 1); i++)
  {
    crc.add(send_tempbuffer[i]);
  }
  send_tempbuffer[14] = crc.getCRC(); //CRC
  send_tempbuffer[15] = 0x55; //конец payload
  packetType = pt; //тип пакета 3 или 4
}

//Функция формирования длинного пакета 0x21
void RequestPacketLong(byte comm, byte c14, int pt) {
  send_tempbuffer[0] = 0x10; //длина пакета 16 байт
  send_tempbuffer[1] = 0x73;
  send_tempbuffer[2] = 0x55; //начало payload
  send_tempbuffer[3] = 0x21; //тип запроса/пакета
  send_tempbuffer[4] = 0x00; //
  send_tempbuffer[5] = (atoi(MeterAdressValue)) & 0xff; //младший байт адреса счётчика
  send_tempbuffer[6] = ((atoi(MeterAdressValue)) >> 8) & 0xff; //старший байт адреса счётчика
  send_tempbuffer[7] = 0xfe; // 0x09
  send_tempbuffer[8] = 0xff; //
  send_tempbuffer[9] = comm; //команда //0x28
  send_tempbuffer[10] = 0x00; //PIN
  send_tempbuffer[11] = 0x00; //PIN
  send_tempbuffer[12] = 0x00; //PIN
  send_tempbuffer[13] = 0x00; //PIN
  send_tempbuffer[14] = c14; //подкомманда //0x00
  //вычисляем и добавляем байт crc
  crc.restart();
  crc.setPolynome(0xA9);
  for (int i = 3; i < (send_tempbuffer[0] - 1); i++)
  {
    crc.add(send_tempbuffer[i]);
  }
  send_tempbuffer[15] = crc.getCRC(); //CRC
  send_tempbuffer[16] = 0x55; //конец payload
  packetType = pt; //типа пакета 3 или 4
}

//Функция формирования длинного 2-х байтового пакета 0x22
void RequestPacketLong2b(byte comm, byte c14, byte c15, int pt) {
  send_tempbuffer[0] = 0x11;
  send_tempbuffer[1] = 0x73;
  send_tempbuffer[2] = 0x55; //начало payload
  send_tempbuffer[3] = 0x22; //тип запроса/пакета
  send_tempbuffer[4] = 0x00; //
  send_tempbuffer[5] = (atoi(MeterAdressValue)) & 0xff; //младший байт адреса счётчика
  send_tempbuffer[6] = ((atoi(MeterAdressValue)) >> 8) & 0xff; //старший байт адреса счётчика
  send_tempbuffer[7] = 0xfe; // 0x09
  send_tempbuffer[8] = 0xff; //
  send_tempbuffer[9] = comm; //команда //0x28
  send_tempbuffer[10] = 0x00; //PIN
  send_tempbuffer[11] = 0x00; //PIN
  send_tempbuffer[12] = 0x00; //PIN
  send_tempbuffer[13] = 0x00; //PIN
  send_tempbuffer[14] = c14;
  send_tempbuffer[15] = c15;
  //вычисляем и добавляем байт crc
  crc.restart();
  crc.setPolynome(0xA9);
  for (int i = 3; i < (send_tempbuffer[0] - 1); i++)
  {
    crc.add(send_tempbuffer[i]);
  }
  send_tempbuffer[16] = crc.getCRC(); //CRC
  send_tempbuffer[17] = 0x55; //конец payload
  packetType = pt; //типа пакета 3 или 4
}

void packetSender()  //функция отправки пакета
{
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Обработка байтстаффингом перед отправкой
  ///////////////////////////////////////////////////////////////////////////////////////////////
  int j = 0;                              // Значение компенсирующего смещения индекса
  byte transmitt_byte[7 + send_tempbuffer[0]] = { 0 }; // Временный буфер для отправки пакета, размерность максимальная +7 при комбо из номера счетчика, пароля и CRC
  transmitt_byte[0] = send_tempbuffer[0]; //Первоначальная длина передаваемого пакета, внутренний параметр для цикла отправки
  transmitt_byte[1] = send_tempbuffer[1]; //Стартовые пакеты, не подвергается байтстаффингу согласно документации
  transmitt_byte[2] = send_tempbuffer[2]; //Стартовые пакеты, не подвергается байтстаффингу согласно документации
  for (int i = 3; i < (send_tempbuffer[0]); i++)
  {
    if (send_tempbuffer[i] == 0x55)
    {
      transmitt_byte[i + j] = 0x73;       // Если байт равен 0x55, добавляем 0x73 во выходной буфер
      transmitt_byte[i + j + 1] = 0x11;   // и 0x11 после него
      j++;                                // Компенсируем индекс выходного буфера при добавлении 1 байта
    }
    else if (send_tempbuffer[i] == 0x73)
    {
      transmitt_byte[i + j] = 0x73;       // Если байт равен 0x73, добавляем 0x73 в выходной буфер
      transmitt_byte[i + j + 1] = 0x22;   // и 0x22 после него
      j++;                                // Компенсируем индекс выходного буфера при добавлении 1 байта
    }
    else
    {
      transmitt_byte[i + j] = send_tempbuffer[i]; // Если байт не требует байтстаффинга, просто копируем
    }
  }
  transmitt_byte[send_tempbuffer[0] + j] = send_tempbuffer[send_tempbuffer[0]]; //добавляем стоповый бит, не подвергается байтстаффингу согласно документации
  transmitt_byte[0] += j; //обновляем длину отправляемого пакета (добавляем количество добавленных байт), внутренний параметр для цикла отправки
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Отправка пакета
  ///////////////////////////////////////////////////////////////////////////////////////////////
  packet = ""; //переменная для вывода передаваемого пакета в MQTT
  ELECHOUSE_cc1101.SpiStrobe(0x33);  //Calibrate frequency synthesizer and turn it off
  delay(1);
  ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x36);  // Exit RX / TX, turn off frequency synthesizer and exit
  ELECHOUSE_cc1101.SpiWriteReg(0x3e, 0xC4); //выставляем мощность 10dB
  ELECHOUSE_cc1101.SendData(transmitt_byte, transmitt_byte[0] + 1); //отправляем пакет
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX
  Serial.println("Packet sent:");
  for (int i = 0; i < transmitt_byte[0] + 1; i++) {
    Serial.print(transmitt_byte[i], HEX);
    Serial.print(" ");
    packet += String(transmitt_byte[i], HEX); //собираем пакет для вывода в MQTT
    packet += " ";
  }
  Serial.println();
  //Вывод отправленного пакета в MQTT
  packet.toUpperCase(); //Приводим все значения к верхнему регистру
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Packet_send", packet);
}

void packetReceiver() //функция приёма пакета (помещает его в resultbuffer[])
{
  for (int i = 0; i < 50; i++) {
    resultbuffer[i] = 0x00;
    tempbuffer[i] = 0x00;
  }

  tmr.start();
  packet = ""; //переменная для вывода полученного пакета в MQTT
  int PackCount = 0; //счётчик принятых из эфира пакетов
  bytecount = 0; //указатель байтов в результирующем буфере
  bytecounttemp = 0; //указатель байтов в промежуточном буфере
  while (!tmr.tick() && PackCount != packetType) {
    if (ELECHOUSE_cc1101.CheckReceiveFlag()) {
      PackCount++;
      /*
        //Rssi Level in dBm
        Serial.print("Rssi: ");
        Serial.println(ELECHOUSE_cc1101.getRssi());

        //Link Quality Indicator
        Serial.print("LQI: ");
        Serial.println(ELECHOUSE_cc1101.getLqi());
      */

      //Get received Data and calculate length
      int len = ELECHOUSE_cc1101.ReceiveData(buffer);

      //подшиваем 1/3 или 1/4 пакета в общий пакет
      for (int i = 1; i < len; i++) {
        tempbuffer[bytecounttemp] = buffer[i];
        bytecounttemp++; //Считаем длину промежуточного буфера
      }
      ELECHOUSE_cc1101.SpiStrobe(0x36);  // Exit RX / TX, turn off frequency synthesizer and exit
      ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
      ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
      //delay(1);
      ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////////
  // Обработка байтстаффингом полученного пакета перед парсингом
  ///////////////////////////////////////////////////////////////////////////////////////////////
  int j = 0; //Значение компенсирующего смещения индекса
  for (int i = 0; i < bytecounttemp; i++)
  {
    resultbuffer [i - j] = tempbuffer[i]; //Записываем текущее значение байта из временного буфера в выходной
    bytecount++; //Считаем длину результирующего буфера
    if (tempbuffer[i] == 0x73 and (i + 1 < bytecounttemp))
    {
      if (tempbuffer[i + 1] == 0x11) //Проверяем что последовательность 0x73 0x11
      {
        resultbuffer[i - j] = 0x55; //Меняем последовательность 0x73 0x11 на 0x55
        i++; //Пропускаем следующий байт
        j++; //Компенсируем индекс результирующего буфера при пропуске байта
      }
      if (tempbuffer[i + 1] == 0x22) //Проверяем что последовательность 0x73 0x22
      {
        i++; //В результирующем буфере уже лежит 0x73 вместо 0x73 0x22, пропускаем следующий байт
        j++; //Компенсируем индекс результирующего буфера при пропуске байта
      }
    }
    /*    //Add by DigiBoy
        if (tempbuffer[i] == 0x55 and tempbuffer[i - 1] != 0x73) {
          break;
          }*/
  }

  Serial.print("Packets received: ");
  Serial.println(PackCount);
  //Печатаем общий пакет
  for (int i = 0; i < bytecount; i++)
  {
    Serial.print(resultbuffer[i], HEX); //исходный пакет
    Serial.print(" ");
    packet += String(resultbuffer[i], HEX); //собираем пакет для вывода в MQTT
    packet += " ";
  }
  Serial.println();
  Serial.print("Packet lengt: ");
  Serial.println(bytecount);
  // Считаем свою CRC ------------------------------
  crc.reset();
  crc.setPolynome(0xA9);
  for (int i = 2; i < (bytecount - 2); i++)
  {
    crc.add(resultbuffer[i]);
    Serial.print(resultbuffer[i], HEX);
    Serial.print(" ");
  }
  Serial.println();
  myCRC = crc.getCRC();
  Serial.print("Calculate myCRC: ");
  Serial.println(myCRC, HEX);
  //-----------------------------------------

  //Вывод полученного пакета в MQTT
  packet.toUpperCase(); //Приводим все значения к верхнему регистру
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Packet_recive", packet);
}

//Парсер пакета дата/время
void packetParser_1() {
  last_req = "";
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x1c) and (resultbuffer[20] == myCRC) and (resultbuffer[21] == 0x55))
  {
    if (resultbuffer[17] < 10) {
      last_req += "0";
    }
    last_req += String(resultbuffer[17], DEC); //дата
    last_req += "-";
    if (resultbuffer[18] < 10) {
      last_req += "0";
    }
    last_req += String(resultbuffer[18], DEC); //месяц
    last_req += "-";
    last_req += String(resultbuffer[19], DEC); //год
    last_req += " ";
    if (resultbuffer[15] < 10) {
      last_req += "0";
    }
    last_req += String(resultbuffer[15], DEC); //час
    last_req += ":";
    if (resultbuffer[14] < 10) {
      last_req += "0";
    }
    last_req += String(resultbuffer[14], DEC); //минуты
    last_req += ":";
    if (resultbuffer[13] < 10) {
      last_req += "0";
    }
    last_req += String(resultbuffer[13], DEC); //секунды
    last_req += " ";
    switch (resultbuffer[16]) {
      case 0:
        last_req += "Вс";
        break;
      case 1:
        last_req += "Пн";
        break;
      case 2:
        last_req += "Вт";
        break;
      case 3:
        last_req += "Ср";
        break;
      case 4:
        last_req += "Чт";
        break;
      case 5:
        last_req += "Пт";
        break;
      case 6:
        last_req += "Сб";
        break;
    }
    switch (resultbuffer[9])
    {
      case 168: //0xA8 Счетчик электроэнергии 3ф трансформаторный активно-реактивный (двунаправленный с параметрами сети)
        three_phase = true;
        type_counter = "Счетчик 3ф трансформаторный активно-реактивный";
        Serial.println("Three phase counter detected!");
        break;
      case 152: //0x98 Счетчик электроэнергии 1ф 2х элементный активно-реактивный (двунаправленный с параметрами сети)
        three_phase = false;
        type_counter = "Счетчик 1ф 2х элементный активно-реактивный";
        Serial.println("One phase counter detected!");
        break;
    }
    Serial.println("Last Request: " + last_req);
  } else {
    Serial.println("PARSING 1 ERROR! Received a damaged package");
  }
}

//Отправка даты/время в MQTT
void packetSender_1_mqtt() {
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/LastRequest", last_req);
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/type_counter", type_counter);
}

//Парсер пакета суммарных показаний счётчика
void packetParser_2() {
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x05) and (resultbuffer[43] == myCRC) and (resultbuffer[44] == 0x55))
  {
    sum = float((resultbuffer[22] << 24) | (resultbuffer[21] << 16) | (resultbuffer[20] << 8) | resultbuffer[19]) / 100;
    Serial.print("SUM:  ");
    Serial.println(sum);

    t1 = float((resultbuffer[30] << 24) | (resultbuffer[29] << 16) | (resultbuffer[28] << 8) | resultbuffer[27]) / 100;
    Serial.print("T1:  ");
    Serial.println(t1);

    t2 = float((resultbuffer[34] << 24) | (resultbuffer[33] << 16) | (resultbuffer[32] << 8) | resultbuffer[31]) / 100;
    Serial.print("T2:  ");
    Serial.println(t2);

    /*t3 = float((resultbuffer[38] << 24) | (resultbuffer[37] << 16) | (resultbuffer[36] << 8) | resultbuffer[35]) / 100;
      Serial.print("T3:  ");
      Serial.println(t3);

      t4 = float((resultbuffer[42] << 24) | (resultbuffer[41] << 16) | (resultbuffer[40] << 8) | resultbuffer[39]) / 100;
      Serial.print("T4:  ");
      Serial.println(t4);*/

    switch (resultbuffer[14] >> 2 & 0b00000011)
    {
      case 0:
        tariff = "День"; break;
      case 1:
        tariff = "Ночь"; break;
      case 2:
        tariff = "Полупик"; break;
      case 3:
        tariff = "Специальный"; break;
    }
  }
  else  {
    Serial.println("PARSING 2 ERROR! Received a damaged package");
  }
}

//Отправка суммарных показаний счётчика в MQTT
void packetSender_2_mqtt() {
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/SUM", String(sum, 2));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/T1", String(t1, 2));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/T2", String(t2, 2));
  //mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/T3", String(t3, 2));
  //mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/T4", String(t4, 2));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Tariff", tariff);
}

//Парсер пакета мгновенных значений напряжений и тока
void packetParser_3() {
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x2b) and (resultbuffer[43] == myCRC) and (resultbuffer[44] == 0x55))
  {
    P = float(resultbuffer[18] | (resultbuffer[19] << 8) | (resultbuffer[20] << 16));
    Serial.print("Power: ");
    Serial.println(P);

    if (resultbuffer[23] >= 128) {
      Q = float(resultbuffer[21] | (resultbuffer[22] << 8) | ((resultbuffer[23] - 128) << 16)) / -1000;
    }
    else {
      Q = float(resultbuffer[21] | (resultbuffer[22] << 8) | (resultbuffer[23] << 16)) / 1000;
    }
    Serial.print("Q: ");
    Serial.println(Q);

    freq = float((resultbuffer[24] | (resultbuffer[25] << 8))) / 100;
    Serial.print("Freq:  ");
    Serial.println(freq);

    //старший бит знаковый. Формат данных X.XXX. Диапазон: от -1.000 до +1.000. 03E8H соответствует 1.000, а 83E8H соответствует -1.000.
    if (resultbuffer[27] >= 128) {
      cosin = float((resultbuffer[26] | ((resultbuffer[27] - 128) << 8))) / -1000;
    }
    else {
      cosin = float((resultbuffer[26] | (resultbuffer[27] << 8))) / 1000;
    }

    Serial.print("Cos:  ");
    Serial.println(cosin);

    v1 = float((resultbuffer[28] | (resultbuffer[29] << 8))) / 100;
    Serial.print("V1:  ");
    Serial.println(v1);

    v2 = float((resultbuffer[30] | (resultbuffer[31] << 8))) / 100;
    Serial.print("V2:  ");
    Serial.println(v2);

    v3 = float((resultbuffer[32] | (resultbuffer[33] << 8))) / 100;
    Serial.print("V3:  ");
    Serial.println(v3);

    i1 = float(resultbuffer[34] | (resultbuffer[35] << 8) | (resultbuffer[36] << 16)) / 1000;
    Serial.print("I1  ");
    Serial.println(i1);

    i2 = float(resultbuffer[37] | (resultbuffer[38] << 8) | (resultbuffer[39] << 16)) / 1000;
    Serial.print("I2  ");
    Serial.println(i2);

    i3 = float(resultbuffer[40] | (resultbuffer[41] << 8) | (resultbuffer[42] << 16)) / 1000;
    Serial.print("I3  ");
    Serial.println(i3);
  } else {
    Serial.println("PARSING 3 ERROR! Received a damaged package");
  }
}

void packetParser_3_1() {
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x2b) and ((resultbuffer[43] == 0x55) or ((resultbuffer[41] == myCRC) and (resultbuffer[42] == 0x55))))
    //(resultbuffer[8] == 0x2b) and (resultbuffer[41] == myCRC) and (resultbuffer[42] == 0x55))
    //с офднофазных старых приходит 43 байта, с новых 44, но при этом CRC считается для 43 байт
  {
    P = float(resultbuffer[18] | (resultbuffer[19] << 8));
    Serial.print("Power: ");
    Serial.println(P);

    if (resultbuffer[21] >= 128) {
      Q = float(resultbuffer[20] | ((resultbuffer[21] - 128) << 8)) / -1000;
    }
    else {
      Q = float(resultbuffer[20] | (resultbuffer[21] << 8)) / 1000;
    }
    Serial.print("Q: ");
    Serial.println(Q);

    freq = float((resultbuffer[22] | (resultbuffer[23] << 8))) / 100;
    Serial.print("Freq:  ");
    Serial.println(freq);

    //старший бит знаковый. Формат данных X.XXX. Диапазон: от -1.000 до +1.000. 03E8H соответствует 1.000, а 83E8H соответствует -1.000.
    if (resultbuffer[25] >= 128) {
      cosin = float((resultbuffer[24] | ((resultbuffer[25] - 128) << 8))) / -1000;
    }
    else {
      cosin = float((resultbuffer[24] | (resultbuffer[25] << 8))) / 1000;
    }
    Serial.print("Cos:  ");
    Serial.println(cosin);

    v1 = float((resultbuffer[26] | (resultbuffer[27] << 8))) / 100;
    Serial.print("V1:  ");
    Serial.println(v1);

    v2 = float((resultbuffer[28] | (resultbuffer[29] << 8))) / 100;
    Serial.print("V2:  ");
    Serial.println(v2);

    v3 = float((resultbuffer[30] | (resultbuffer[31] << 8))) / 100;
    Serial.print("V3:  ");
    Serial.println(v3);

    i1 = float(resultbuffer[32] | (resultbuffer[33] << 8) | (resultbuffer[34] << 16)) / 1000;
    Serial.print("I1  ");
    Serial.println(i1);

    i2 = float(resultbuffer[35] | (resultbuffer[36] << 8) | (resultbuffer[37] << 16)) / 1000;
    Serial.print("I2  ");
    Serial.println(i2);

    i3 = float(resultbuffer[38] | (resultbuffer[39] << 8) | (resultbuffer[40] << 16)) / 1000;
    Serial.print("I3  ");
    Serial.println(i3);
    /*
        // Считаем свою CRC ------------------------------
        //подтверждение теории, что программисты Миртек накосчили
        crc.reset();
          crc.setPolynome(0xA9);
          for (int i = 2; i < (bytecount - 3); i++)
          {
          crc.add(resultbuffer[i]);
          Serial.print(resultbuffer[i], HEX);
          Serial.print(" ");
          }
          Serial.println();
          myCRC = crc.getCRC();
          Serial.print("Calculate in packetParser_3_1 - myCRC: ");
          Serial.println(myCRC, HEX);
        //-----------------------------------------
    */
  } else {
    Serial.println("PARSING 3_1 ERROR! Received a damaged package");
  }
}

/*
  //Парсер пакета мгновенных значений напряжений для 1-ой фазы
  void packetParser_3_voltage() {
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x29) and (resultbuffer[16] == myCRC) and (resultbuffer[17] == 0x55))
  {
    v1 = float((resultbuffer[14] | (resultbuffer[15] << 8))) / 100;
    Serial.print("V1:  ");
    Serial.print(v1);
    Serial.println("V");
    //Serial.print(String(resultbuffer[13], DEC));
    //Serial.println("-phase");
  }
  else {
    Serial.println("PARSING 3 Volgage ERROR! Received a damaged package");
  }
  }


  void packetParser_3_current() {
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x2C) and (resultbuffer[17] == myCRC) and (resultbuffer[18] == 0x55))
  {
    i1 = float(resultbuffer[14] | (resultbuffer[15] << 8) | (resultbuffer[16] << 16)) / 1000;
    Serial.print("I1:  ");
    Serial.print(i1);
    Serial.println("A");
    //Serial.print(String(resultbuffer[13], DEC));
    //Serial.println("-phase");
  }
  else {
    Serial.println("PARSING 3 Volgage ERROR! Received a damaged package");
  }
  }
*/

void packetSender_3_mqtt() {
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/P", String(P));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Q", String(Q));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Freq", String(freq));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Cos", String(cosin));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/V1", String(v1));
  if (three_phase) {
    mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/V2", String(v2));
    mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/V3", String(v3));
  }
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/I1", String(i1));
  if (three_phase) {
    mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/I2", String(i2));
    mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/I3", String(i3));
  }
}

void packetParser_4() { //парсинг мнгновенных значений мощностей
  if ( (resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff)) and
       (resultbuffer[8] == 0x2b) and (resultbuffer[43] == myCRC) and (resultbuffer[44] == 0x55))
  {
    //старший бит знаковый. Формат данных X.XXX. Диапазон: от 1.000 до +1.000. 03E8H соответствует 1.000, а 83E8H соответствует -1.000.
    if (resultbuffer[19] >= 128) {
      cosinA = float((resultbuffer[18] | ((resultbuffer[19] - 128) << 8))) / -1000;
    }
    else {
      cosinA = float((resultbuffer[18] | (resultbuffer[19] << 8))) / 1000;
    }
    Serial.print("CosA: ");
    Serial.println(cosinA);

    if (resultbuffer[21] >= 128) {
      cosinB = float((resultbuffer[20] | ((resultbuffer[21] - 128) << 8))) / -1000;
    }
    else {
      cosinB = float((resultbuffer[20] | (resultbuffer[21] << 8))) / 1000;
    }
    Serial.print("CosB: ");
    Serial.println(cosinB);

    if (resultbuffer[23] >= 128) {
      cosinC = float((resultbuffer[22] | ((resultbuffer[23] - 128) << 8))) / -1000;
    }
    else {
      cosinC = float((resultbuffer[22] | (resultbuffer[23] << 8))) / 1000;
    }
    Serial.print("CosC: ");
    Serial.println(cosinC);

    Pa = float((resultbuffer[24] | (resultbuffer[25] << 8)));
    Serial.print("Pa: ");
    Serial.println(Pa);

    Pb = float((resultbuffer[26] | (resultbuffer[27] << 8)));
    Serial.print("Pb: ");
    Serial.println(Pb);

    Pc = float((resultbuffer[28] | (resultbuffer[29] << 8)));
    Serial.print("Pc: ");
    Serial.println(Pc);

    if (resultbuffer[31] >= 128) {
      Qa = float((resultbuffer[30] | ((resultbuffer[31] - 128) << 8))) / -1000;
    }
    else {
      Qa = float((resultbuffer[30] | (resultbuffer[31] << 8))) / 1000;
    }
    Serial.print("Qa: ");
    Serial.println(Qa);

    if (resultbuffer[33] >= 128) {
      Qb = float((resultbuffer[32] | ((resultbuffer[33] - 128) << 8))) / -1000;
    }
    else {
      Qb = float((resultbuffer[32] | (resultbuffer[33] << 8))) / 1000;
    }
    Serial.print("Qb: ");
    Serial.println(Qb);

    if (resultbuffer[35] >= 128) {
      Qc = float((resultbuffer[34] | ((resultbuffer[35] - 128) << 8))) / -1000;
    }
    else {
      Qc = float((resultbuffer[34] | (resultbuffer[35] << 8))) / 1000;
    }
    Serial.print("Qc: ");
    Serial.println(Qc);

    Sa = float((resultbuffer[36] | (resultbuffer[37] << 8)));
    Serial.print("Sa: ");
    Serial.println(Sa);

    Sb = float((resultbuffer[38] | (resultbuffer[39] << 8)));
    Serial.print("Sb: ");
    Serial.println(Sb);

    Sc = float((resultbuffer[40] | (resultbuffer[41] << 8)));
    Serial.print("Sc: ");
    Serial.println(Sc);

    //1 байт – Температура (старший бит знаковый)
    if (resultbuffer[42] >= 128) {
      T = float((resultbuffer[42] - 128)) / -1;
    }
    else {
      T = float(resultbuffer[42]);
    }
    Serial.print("T: ");
    Serial.println(T);
  }
  else {
    Serial.println("PARSING 4 ERROR! Received a damaged package");
  }
}

void packetSender_4_mqtt() {
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/CosA", String(cosinA));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/CosB", String(cosinB));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/CosC", String(cosinC));

  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Pa", String(Pa));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Pb", String(Pb));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Pc", String(Pc));

  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Qa", String(Qa));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Qb", String(Qb));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Qc", String(Qc));

  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Sa", String(Sa));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Sb", String(Sb));
  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Sc", String(Sc));

  mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/T", String(T));
}

void packetParser_6() {
  if ((resultbuffer[0] == 0x73) and (resultbuffer[1] == 0x55) and (resultbuffer[6] == (atoi(MeterAdressValue) & 0xff)) and (resultbuffer[7] == ((atoi(MeterAdressValue) >> 8) & 0xff))
      and  (resultbuffer[8] == 0x10) and (resultbuffer[32] == myCRC) and (resultbuffer[33] == 0x55))
  {
    /*AdressValue = ((resultbuffer[6] | (resultbuffer[7] << 8)));
      Serial.print("Adress:  ");
      Serial.println(AdressValue);*/

    switch (resultbuffer[28])
    {
      case 0:
        plomb = "OK";
        break;
      case 1:
        plomb = "Вскрыта пломба клемника";
        break;
      case 2:
        plomb = "Вскрыта пломба корпуса";
        break;
      case 3:
        plomb = "Вскрыта пломба клемника+корпуса";
        break;
    }
    Serial.print("Статус пломб: ");
    Serial.println(plomb);

    switch (resultbuffer[24] >> 2 & 0b00000011)
    {
      case 0:
        rele = "Вкл";
        break;
      case 1:
        rele = "Выкл";
        break;
    }
    Serial.print("Статус реле: ");
    Serial.println(rele);
  }
  else
  {
    Serial.println("PARSING 6 ERROR! Received a damaged package");
  }
}
void setup() {
  ELECHOUSE_cc1101.setGDO0(gdo0);
  Serial.begin(115200);
  mqttGroup.addItem(&mqttServerParam);
  mqttGroup.addItem(&mqttUserNameParam);
  mqttGroup.addItem(&mqttUserPasswordParam);
  mqttGroup.addItem(&mqttPortParam);

  group1.addItem(&MeterAdress); //адрес счетчика

  iotWebConf.setStatusPin(STATUS_PIN);
  //iotWebConf.setConfigPin(CONFIG_PIN);
  iotWebConf.addParameterGroup(&mqttGroup);
  iotWebConf.addParameterGroup(&group1);
  iotWebConf.setConfigSavedCallback(&configSaved);
  iotWebConf.setFormValidator(&formValidator);
  iotWebConf.setWifiConnectionCallback(&wifiConnected);

  // -- Initializing the configuration.
  bool validConfig = iotWebConf.init();
  if (!validConfig)
  {
    mqttServerValue[0] = '\0';
    mqttUserNameValue[0] = '\0';
    mqttUserPasswordValue[0] = '\0';
	strcpy(mqttPortValue, "1883");
  }

  // -- Set up required URL handlers on the web server.
  server.on("/", handleRoot);
  server.on("/config", [] { iotWebConf.handleConfig(); });
  server.on("/restart", [] { needReset = true; });
  server.onNotFound([]() {
    iotWebConf.handleNotFound();
  });

  mqttClient.begin(mqttServerValue, atoi(mqttPortValue), net);
  mqttClient.onMessage(mqttMessageReceived);

  Serial.println("Ready.");

  if (ELECHOUSE_cc1101.getCC1101()) {     // Check the CC1101 SPI connection.
    Serial.println("SPI Connection CC1101 OK");
  }
  else {
    Serial.println("SPI Connection CC1101 Error");
  }
  //Инициализируем cc1101
  ELECHOUSE_cc1101.SpiStrobe(0x30);  //reset
  ELECHOUSE_cc1101.SpiWriteBurstReg(0x00, rfSettings, 0x2F);
  ELECHOUSE_cc1101.SpiStrobe(0x33);  //Calibrate frequency synthesizer and turn it off
  delay(1);
  ELECHOUSE_cc1101.SpiStrobe(0x3A);  // Flush the RX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x3B);  // Flush the TX FIFO buffer
  ELECHOUSE_cc1101.SpiStrobe(0x34);  // Enable RX

  //Serial.println("Rx Mode");
  tmr_mqtt.start(); //старт таймера MQTT
}

void handleRoot()
{
  // -- Let IotWebConf test and handle captive portal requests.
  if (iotWebConf.handleCaptivePortal())
  {
    // -- Captive portal request were already served.
    return;
  }
  String s = "<!DOCTYPE html><html lang=\"ru\"><head><meta name=\"viewport\" content=\"width=device-width, initial-scale=1, user-scalable=no\"/><meta name=\"mobile-web-app-capable\" content=\"yes\" />";
  s += "<meta http-equiv=\"Content-Type\" content=\"text/html; charset=utf-8\" />";
  s += "<title>Mirtek-Gateway-"  + (String)MeterAdressValue + "</title>";
  s += "<style>td.v0{background-color:#BBBBBB;text-align:right;} td.v1{background-color:white;text-align:right;}";
  s += "table.v1{background-color: #999999;padding=2px;} body {text-align:center;}</style>";
  s += "<meta http-equiv=\"refresh\" content=\"60\">";

  s += "</head><body>Mirtek Gateway";
  s += "<div style=\"text-align: center\">";
  s += "MQTT сервер: " + (String)mqttServerValue + ":" + (String)mqttPortValue + "<br>";
  s += "Адрес счетчика: " + (String)MeterAdressValue + "<br>";

  s += "Тип счетчика: ";
  if (three_phase) {
    s += "3-фазный<br>";
  } else {
    s += "1-фазный<br>";
  }

  s += "Последний запрос: " + last_req + "<p> </p>";

  s += "Сумма: " + (String)sum + "<br>";
  s += "T1 (день): " + (String)t1 + "<br>";
  s += "T2 (ночь): " + (String)t2 + "<p> </p>";
  //s += "T3 (полупик): " + (String)t3 + "<p> </p>";
  //s += "T4 (спец): " + (String)t4 + "<p> </p>";
  s += "Действующий тариф: " + tariff + "<p> </p>";

  s += "Cos: " + (String)cosin + " %<br>";
  s += "Частота: " + (String)freq + " Гц<br>";
  s += "Активная мощность: " + (String)P + " Вт<br>";
  s += "Реактивная мощность: " + (String)Q + " кВар<p> </p>";

  if (three_phase) {
    s += "Температура счетчика: " + (String)T + "&deg C<p> </p>";
  }

  s += "Состояние пломб: " + plomb + "<br>";
  s += "Состояние реле: " + rele + "<p> </p>";

  if (three_phase) {
    s += "<table class=\"v1\" align=\"center\">";
    if (horizontal) {
      s += "<tr>";
      s += "<th width=\"80px\">Линия</th>";
      s += "<th width=\"80px\">U, В</th>";
      s += "<th width=\"80px\">I, А</th>";
      s += "<th width=\"80px\">P, Вт, рст</th>";
      s += "<th width=\"80px\">P, Вт, счт</th>";
      s += "<th width=\"80px\">Cos, %</th>";
      s += "<th width=\"80px\">Q, кВар</th>";
      s += "<th width=\"80px\">S, ВА</th>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">Фаза 1</td>";
      s += "<td class=\"v1\">" + (String)v1 + "</td>";
      s += "<td class=\"v1\">" + (String)i1 + "</td>";
      s += "<td class=\"v1\">" + (String)(v1 * i1) + "</td>";
      s += "<td class=\"v1\">" + (String)(Pa) + "</td>"; //Пока решили для 1ф счетчика наполнять
      s += "<td class=\"v1\">" + (String)(cosinA) + "</td>"; // таблицу частично общими данными
      s += "<td class=\"v1\">" + (String)(Qa) + "</td>"; // так как фаза одна и общие значения
      s += "<td class=\"v1\">" + (String)(Sa) + "</td>"; // совпадают с фазными
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">Фаза 2</td>";
      s += "<td class=\"v1\">" + (String)v2 + "</td>";
      s += "<td class=\"v1\">" + (String)i2 + "</td>";
      s += "<td class=\"v1\">" + (String)(v2 * i2) + "</td>";
      s += "<td class=\"v1\">" + (String)(Pb) + "</td>";
      s += "<td class=\"v1\">" + (String)(cosinB) + "</td>";
      s += "<td class=\"v1\">" + (String)(Qb) + "</td>";
      s += "<td class=\"v1\">" + (String)(Sb) + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">Фаза 3</td>";
      s += "<td class=\"v1\">" + (String)v3 + "</td>";
      s += "<td class=\"v1\">" + (String)i3 + "</td>";
      s += "<td class=\"v1\">" + (String)(v3 * i3) + "</td>";
      s += "<td class=\"v1\">" + (String)(Pc) + "</td>";
      s += "<td class=\"v1\">" + (String)(cosinC) + "</td>";
      s += "<td class=\"v1\">" + (String)(Qc) + "</td>";
      s += "<td class=\"v1\">" + (String)(Sc) + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">Сумма</td>";
      s += "<td class=\"v1\">----</td>";
      s += "<td class=\"v1\">" + (String)(i1 + i2 + i3) + "</td>";
      s += "<td class=\"v1\">" + (String)(v1 * i1 + v2 * i2 + v3 * i3) + "</td>";
      s += "<td class=\"v1\">" + (String)(Pa + Pb + Pc) + "</td>";
      s += "<td class=\"v1\">----</td>";
      s += "<td class=\"v1\">" + (String)(abs(Qa) + abs(Qb) + abs(Qc)) + "</td>";
      s += "<td class=\"v1\">" + (String)(Sa + Sb + Sc) + "</td>";
      s += "</tr>";
    }

    else {
      s += "<tr>";
      s += "<th width=\"80px\">Параметры</th>";
      s += "<th width=\"80px\">Фаза 1</th>";
      s += "<th width=\"80px\">Фаза 2</th>";
      s += "<th width=\"80px\">Фаза 3</th>";
      s += "<tr/>";

      s += "<tr>";
      s += "<td class=\"v0\">U, В</td>";
      s += "<td class=\"v1\">" + (String)v1 + "</td>";
      s += "<td class=\"v1\">" + (String)v2 + "</td>";
      s += "<td class=\"v1\">" + (String)v3 + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">I, А</td>";
      s += "<td class=\"v1\">" + (String)i1 + "</td>";
      s += "<td class=\"v1\">" + (String)i2 + "</td>";
      s += "<td class=\"v1\">" + (String)i3 + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">P, Вт, рст</td>";
      s += "<td class=\"v1\">" + (String)(v1 * i1) + "</td>";
      s += "<td class=\"v1\">" + (String)(v2 * i2) + "</td>";
      s += "<td class=\"v1\">" + (String)(v3 * i3) + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">P, Вт, счт</td>";
      s += "<td class=\"v1\">" + (String)Pa + "</td>";
      s += "<td class=\"v1\">" + (String)Pb + "</td>";
      s += "<td class=\"v1\">" + (String)Pc + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">Cos, %</td>";
      s += "<td class=\"v1\">" + (String)cosinA + "</td>";
      s += "<td class=\"v1\">" + (String)cosinB + "</td>";
      s += "<td class=\"v1\">" + (String)cosinC + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">Q, кВар</td>";
      s += "<td class=\"v1\">" + (String)Qa + "</td>";
      s += "<td class=\"v1\">" + (String)Qb + "</td>";
      s += "<td class=\"v1\">" + (String)Qc + "</td>";
      s += "</tr>";

      s += "<tr>";
      s += "<td class=\"v0\">S, ВА</td>";
      s += "<td class=\"v1\">" + (String)Sa + "</td>";
      s += "<td class=\"v1\">" + (String)Sb + "</td>";
      s += "<td class=\"v1\">" + (String)Sc + "</td>";
      s += "</tr>";
    }
    s += "</table><p> </p>";
  }
  else
  {
    s += "Напряжение сети: " + (String)v1 + " В<br>";
    s += "Потребляемый ток: " + (String)i1 + " А<br>";
    s += "Расчётная потребляемая мощность: " + (String)(v1 * i1) + " Вт<p> </p>";
  }
  s += "<a href='config'>настройки</a>  ";
  s += "<a href='restart'>перезагрузка</a>";
  s += "</body></html>\n";

  server.send(200, "text/html", s);
}

void wifiConnected()
{
  needMqttConnect = true;
}

void configSaved()
{
  Serial.println("Configuration was updated. Restart.");
  needReset = true;
}

bool formValidator(iotwebconf::WebRequestWrapper * webRequestWrapper)
{
  Serial.println("Validating form.");
  bool valid = true;
  int l = webRequestWrapper->arg(mqttServerParam.getId()).length();
  if (l < 3)
  {
    mqttServerParam.errorMessage = "Please provide at least 3 characters!";
    valid = false;
  }
  return valid;
}

bool connectMqtt() {
  unsigned long now = millis();
  if (1000 > now - lastMqttConnectionAttempt)
  {
    // Do not repeat within 1 sec.
    return false;
  }
  Serial.println("Connecting to MQTT server...");
  if (!connectMqttOptions()) {
    lastMqttConnectionAttempt = now;
    return false;
  }
  Serial.println("Connected!");

  mqttClient.subscribe("mirtek/" + (String)MeterAdressValue + "/action");
  return true;
}

bool connectMqttOptions()
{
  bool result;
  if (mqttUserPasswordValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue, mqttUserPasswordValue);
  }
  else if (mqttUserNameValue[0] != '\0')
  {
    result = mqttClient.connect(iotWebConf.getThingName(), mqttUserNameValue);
  }
  else
  {
    result = mqttClient.connect(iotWebConf.getThingName());
  }
  return result;
}

//Обработка приходящих команд из MQTT
void mqttMessageReceived(String & topic, String & pay)
{
  bool Request_Status = 0;
  Serial.println(pay);
  switch (pay.toInt()) {
    case 1:
      Serial.println("1 reseived from MQTT");
      RequestPacketShort(0x1c, 3); //запрос даты/времени
      packetSender(); //отправляем пакет
      packetReceiver(); //принимаем и склеиваем пакет
      packetParser_1();
      packetSender_1_mqtt();
      break;
    case 2:
      Serial.println("2 reseived from MQTT");
      RequestPacketLong(0x05, 0x00, 4); //запрос суммарных показаний счётчика
      packetSender(); //отправляем пакет
      packetReceiver(); //принимаем и склеиваем пакет
      packetParser_2();
      packetSender_2_mqtt();
      break;
    case 3:
      Serial.println("3 reseived from MQTT");
      if (three_phase) {
        RequestPacketLong(0x2b, 0x00, 4); //запрос мгновенных значений напряжений и тока
        packetSender(); //отправляем пакет
        packetReceiver(); //принимаем и склеиваем пакет
        packetParser_3();
      }
      else {
        RequestPacketLong(0x2b, 0x00, 4); //мгновенные значения напряжений и токов
        packetSender();
        packetReceiver();
        packetParser_3_1();
        /*RequestPacketLong(0x29, 0x01, 4); //запрос мгновенных значений напряжения 1-ой фазы
          packetSender(); //отправляем пакет
          packetReceiver(); //принимаем и склеиваем пакет
          packetParser_3_voltage();
          RequestPacketLong(0x2C, 0x01, 4); //запрос мгновенных значений тока 1-ой фазы
          packetSender(); //отправляем пакет
          packetReceiver(); //принимаем и склеиваем пакет
          packetParser_3_current();*/
      }
      packetSender_3_mqtt();
      break;
    case 4:
      Serial.println("4 reseived from MQTT");
      if (three_phase) {
        RequestPacketLong(0x2b, 0x10, 4); //запрос мгновенных значений мощностей
        packetSender(); //отправляем пакет
        packetReceiver(); //принимаем и склеиваем пакет
        packetParser_4();
        packetSender_4_mqtt();
      }
      break;
  }

  /*  for (int k = 0; k < 3; k++) {  //Пробуем 3 раза сделать запросы и получить ответы
        Request_Status = 1;
        mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Request_Status", "Ok");
        break;
      }
    }
    if (!Request_Status == 1) {
      mqttClient.publish("mirtek/" + (String)MeterAdressValue + "/Request_Status", "Error");
    }*/
}

void loop() {
  // -- doLoop should be called as frequently as possible.
  iotWebConf.doLoop();
  mqttClient.loop();

  if (needMqttConnect)
  {
    if (connectMqtt())
    {
      needMqttConnect = false;
    }
  }
  else if ((iotWebConf.getState() == iotwebconf::OnLine) && (!mqttClient.connected()))
  {
    Serial.println("MQTT reconnect");
    connectMqtt();
  }

  if (needReset)
  {
    Serial.println("Rebooting after 1 second.");
    iotWebConf.delay(1000);
    ESP.restart();
  }

  //Обработка приходящих команд из Serial
  int incomingByte = 0; //командный байт, ожидаемый на входе Serial
  if (Serial.available() > 0) {
    incomingByte = Serial.parseInt();
    Serial.println(incomingByte);
    switch (incomingByte) {
      case 1:
        Serial.println("1 reseived from serial");
        RequestPacketShort(0x1c, 3); //дата/время
        packetSender(); //отправляем пакет
        packetReceiver(); //принимаем и склеиваем пакет
        packetParser_1(); //расскладываем по переменным
        break;
      case 2:
        Serial.println("2 reseived from serial");
        RequestPacketLong(0x05, 0x00, 4); //суммарные значения потребления тип энергии:0x00 - активная прямая
        packetSender();
        packetReceiver();
        packetParser_2();
        break;
      case 3:
        Serial.println("3 reseived from serial");
        if (three_phase) {
          RequestPacketLong(0x2b, 0x00, 4); //мгновенные значения напряжений и токов
          packetSender();
          packetReceiver();
          packetParser_3();
        }
        else {
          RequestPacketLong(0x2b, 0x00, 4); //мгновенные значения напряжений и токов
          packetSender();
          packetReceiver();
          packetParser_3_1();
          /*RequestPacketLong(0x29, 0x01, 4); //запрос мгновенных значений напряжения 1-ой фазы
            packetSender(); //отправляем пакет
            packetReceiver(); //принимаем и склеиваем пакет
            packetParser_3_voltage();
            RequestPacketLong(0x2C, 0x01, 4); //запрос мгновенных значений тока 1-ой фазы
            packetSender(); //отправляем пакет
            packetReceiver(); //принимаем и склеиваем пакет
            packetParser_3_current();*/
        }
        break;
      case 4:
        Serial.println("4 reseived from serial");
        if (three_phase) {
          RequestPacketLong(0x2b, 0x10, 4); //мгновенные значения мощностей
          packetSender();
          packetReceiver();
          packetParser_4();
        }
        break;
      case 5: //эсперементы с командами
        Serial.println("5 reseived from serial (Experiments)");
        RequestPacketShort(0x01, 3);
        packetSender();
        packetReceiver();
        break;
      case 6:
        Serial.println("6 reseived from serial");
        RequestPacketShort(0x10, 3);
        packetSender();
        packetReceiver();
        packetParser_6();
        break;
      case 7:
        Serial.println("7 reseived from serial");
        RequestPacketLong2b(0x3A, 0x00, 0x01, 4); //разомкнуть реле отключения
        packetSender();
        packetReceiver();
        break;
      case 8:
        Serial.println("8 reseived from serial");
        RequestPacketLong2b(0x3A, 0x00, 0x00, 4); //замкнуть реле отключения
        packetSender();
        packetReceiver();
        break;
      case 9:
        Serial.println("9 reseived from serial");
        RequestPacketLong(0x04, 0x01, 4); //сброс состояния электронных пломб
        packetSender();
        packetReceiver();
        break;
    }
  }

  //Запрос показаний по таймеру
  if ((iotWebConf.getState() == iotwebconf::OnLine) && (mqttClient.connected())) {
    if (tmr_mqtt.tick()) //запрос информации по таймеру и отправка в MQTT
    {
      Serial.println("Request MIRTEK by timer");
      //PING от DigiBoy
      /*RequestPacketShort(0x1c, 3);
        packetSender(); //отправляем пакет
        packetReceiver(); //принимаем и склеиваем пакет
      */
      // Запрос даты и времени
      RequestPacketShort(0x1c, 3);
      packetSender(); //отправляем пакет
      packetReceiver(); //принимаем и склеиваем пакет
      packetParser_1(); //раскладывам по переменным
      packetSender_1_mqtt(); //отправляем в MQTT
      //delay(2000);

      // Запрос суммарных показаний счётчика, кВт/ч, тип энергии:0x00 - активная прямая
      RequestPacketLong(0x05, 0x00, 4);
      packetSender();
      packetReceiver();
      packetParser_2();
      packetSender_2_mqtt();
      //delay(2000);

      // Запрос мгновенных показаний напряжения и тока
      if (three_phase) {
        RequestPacketLong(0x2b, 0x00, 4); //мгновенные значения напряжений и токов
        packetSender();
        packetReceiver();
        packetParser_3();
        packetSender_3_mqtt();

        RequestPacketLong(0x2b, 0x10, 4); // Запрос мгновенных показаний мощностей
        packetSender();
        packetReceiver();
        packetParser_4();
        packetSender_4_mqtt();
      }
      else {
        RequestPacketLong(0x2b, 0x00, 4); //мгновенные значения напряжений и токов
        packetSender();
        packetReceiver();
        packetParser_3_1();
        /*RequestPacketLong(0x29, 0x01, 4); //запрос мгновенных значений напряжения 1-ой фазы
          packetSender(); //отправляем пакет
          packetReceiver(); //принимаем и склеиваем пакет
          packetParser_3_voltage();
          RequestPacketLong(0x2C, 0x01, 4); //запрос мгновенных значений тока 1-ой фазы
          packetSender(); //отправляем пакет
          packetReceiver(); //принимаем и склеиваем пакет
          packetParser_3_current();*/
        packetSender_3_mqtt();
      }

      RequestPacketShort(0x10, 3); //запрос статуса
      packetSender();
      packetReceiver();
      packetParser_6();
    }
  }
}
