// by Nikola TV 2022 BETA VERSION

//-----------------------НАСТРОЙКИ--------------------------------------------------
#define mosfet 8             // пин, куда подключен мосфет
#define vibro 2              // пин дополнительного датчика вибрации
#define wakeUpPin 3          // пин датчика 
#define led_pin 5            // пин светодиода
#define discharge_alert true 
#define not_begin_nrf_alert true
#define no_connection_alert true 
#define battery_min 3000     // минимальный уровень заряда батареи
int id = 130; // максимум 3 цифры
//-----------------------НАСТРОЙКИ---------------------------------------------------


//------БИБЛИОТЕКИ------
#include <Wire.h>              // вспомогательная библиотека
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <GyverPower.h>        // библиотека сна
#include <ArduinoJson.h>

//------БИБЛИОТЕКИ------

RF24 radio(9, 10); // "создать" модуль на пинах 9 и 10

byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" }; //возможные номера труб

bool not_begin_nrf = false;  
bool low_battary = false;
bool no_connection = false;
int VCCmin = 6512;
int vibr = 6513;
int voltage, voltage_AAA;
boolean wake_flag = HIGH;
float my_vcc_const = 1.080;    // константа вольтметра
unsigned long lastUpdate = millis();

void setup() {
  Serial.begin(9600);
  voltage = readVcc();         // считать напряжение питания
  pinMode(wakeUpPin, INPUT);
  pinMode(5, OUTPUT);
  pinMode(mosfet, OUTPUT);
  pinMode(vibro, INPUT);

  digitalWrite(mosfet, HIGH); //открыть мосфет
  digitalWrite(5, HIGH);
  power.sleepDelay(1000);

  if (radio.begin()) {
    not_begin_nrf = false;
    Serial.println("connected");
  }
  else {
    not_begin_nrf = true;
    Serial.println("not connected");
  }
  radio.setAutoAck(true);         //режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 50);    //(время между попыткой достучаться, число попыток)
  radio.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32);     //размер пакета, в байтах

  radio.openWritingPipe(address[0]);   //мы - труба 0, открываем канал для передачи данных
  radio.setChannel(0x60);  //выбираем канал (в котором нет шумов!)

  radio.setPALevel(RF24_PA_MAX); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
  //должна быть одинакова на приёмнике и передатчике!
  //при самой низкой скорости имеем самую высокую чувствительность и дальность!!
  // ВНИМАНИЕ!!! enableAckPayload НЕ РАБОТАЕТ НА СКОРОСТИ 250 kbps!
  //  radio.powerUp(); //начать работу
  digitalWrite(mosfet, LOW); //закрыть мосфет
  digitalWrite(5, LOW);
  power.sleepDelay(1000);
  digitalWrite(mosfet, HIGH); //открыть мосфет
  digitalWrite(5, HIGH);
  power.sleepDelay(1000);
  digitalWrite(5, LOW);

}

char counter[33];

void loop() {
  if (wake_flag == true) {
    noInterrupts();
    voltage = readVcc();                            // считать напряжение питания
    voltage_AAA = (analogRead(A1) * (voltage / 1024.00));  // считать напряжение батареек
    if (voltage_AAA < battery_min) {
      low_battary = true;
    }
    else {
      low_battary = false;
    }

    Serial.println(low_battary);
    digitalWrite(mosfet, HIGH); //открыть мосфет
    power.sleepDelay(2000);

    StaticJsonDocument<32> jsonDocument;
    //указать id модуля
    jsonDocument["id"] = id; // максимум 3 цифры
    //указать тип тревоги. 
    //сел аккумулятор - 
    //пожарная - 
    //датчик движения - 

    jsonDocument["b"] = "tre"; // максимум 3 буквы или если убрать кавычки 5 цифр

    serializeJsonPretty(jsonDocument, counter);

    Serial.println(sizeof(counter));

    //radio.begin(); //активировать модуль
    if (radio.begin()) {
      not_begin_nrf = false;
      Serial.println("connected");
    }
    else {
      not_begin_nrf = true;
      Serial.println("not connected");
    }
    radio.setAutoAck(1);         //режим подтверждения приёма, 1 вкл 0 выкл
    radio.setRetries(0, 50);    //(время между попыткой достучаться, число попыток)
    radio.enableAckPayload();    //разрешить отсылку данных в ответ на входящий сигнал
    radio.setPayloadSize(32);     //размер пакета, в байтах

    radio.openReadingPipe(1, address[0]); //хотим слушать трубу 0
    radio.openWritingPipe(address[0]);   //мы - труба 0, открываем канал для передачи данных
    radio.setChannel(0x60);  //выбираем канал (в котором нет шумов!)

    radio.setPALevel(RF24_PA_MAX); //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
    radio.setDataRate(RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

    radio.powerUp(); //начать работу    
    radio.stopListening();  //не слушаем радиоэфир, мы передатчик

    char gotByte[33];
    int kol = 0;
    bool flag_stop = false;
    digitalWrite(5, HIGH);
    power.sleepDelay(1000);

    while (20 - kol > 0) {
      no_connection = true;
      radio.stopListening();  //не слушаем радиоэфир, мы передатчик
      kol++;

      Serial.print("Sending... "); Serial.println(counter);
      // power.sleepDelay(10);   
      unsigned long last_time = millis();         //запоминаем время отправк

      radio.write(&counter, 33);                //отправляем значение counter
      radio.startListening();

      Serial.println(millis());
      while (millis() - last_time < 2000) {
        if (radio.available()) {
          radio.read(gotByte, 33);
          Serial.print("Recieved: "); Serial.println(gotByte);
          StaticJsonDocument<32> jsonDocument;
          deserializeJson(jsonDocument, gotByte);

          if (jsonDocument["id"] == id) {
            flag_stop = true;
            break;
          }
          

        }
      }


      if (flag_stop == true) {
        no_connection = false;
        break;
      }
    }

    //power.sleepDelay(500);

  //}
    digitalWrite(5, LOW);
    power.sleepDelay(30);
    Serial.print("no_connection");
    Serial.println(no_connection);
    power.sleepDelay(100);

    radio.powerDown(); //закончить работу
    digitalWrite(mosfet, LOW); //закрыть мосфет
    wake_flag = false;
    power.sleepDelay(2000);                      // задержка для стабильности
    interrupts();
  }

  attachInterrupt(1, wake_w, CHANGE);
  if (discharge_alert == true && low_battary == true) {
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(150);
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(150);
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(1000);

  }

  if (not_begin_nrf_alert == true && not_begin_nrf == true) {
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(150);
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(150);
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(150);
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(1000);

  }


  if (no_connection_alert == true && no_connection == true) {
    Serial.print("no_connection_alert");


    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(150);
    digitalWrite(5, HIGH);
    power.sleepDelay(150);
    digitalWrite(5, LOW);
    power.sleepDelay(1000);

  }

  if (no_connection != true && low_battary != true && not_begin_nrf != true) {
    Serial.print("SLEEP_FOREVER");
    power.sleep(SLEEP_FOREVER); // вечный сон
  }
  else {
    power.sleep(SLEEP_1024MS); // вечный сон
  }
  power.sleepDelay(100);
}

void wake_w() {
  wake_flag = true;
}

long readVcc() { //функция чтения внутреннего опорного напряжения
#if defined(__AVR_ATmega32U4__) || defined(__AVR_ATmega1280__) || defined(__AVR_ATmega2560__)
  ADMUX = _BV(REFS0) | _BV(MUX4) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#elif defined (__AVR_ATtiny24__) || defined(__AVR_ATtiny44__) || defined(__AVR_ATtiny84__)
  ADMUX = _BV(MUX5) | _BV(MUX0);
#elif defined (__AVR_ATtiny25__) || defined(__AVR_ATtiny45__) || defined(__AVR_ATtiny85__)
  ADMUX = _BV(MUX3) | _BV(MUX2);
#else
  ADMUX = _BV(REFS0) | _BV(MUX3) | _BV(MUX2) | _BV(MUX1);
#endif
  power.sleepDelay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}
