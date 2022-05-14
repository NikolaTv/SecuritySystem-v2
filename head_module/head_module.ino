// by Nikola TV 2022
//-----------------------НАСТРОЙКИ---------------------
#define vibro 6         // пин дополнительного датчика вибрации
#define battery_min 3000     // минимальный уровень заряда батареи для отображения
String phone = "+7xxxxxxxxxx";     // Телефон на который будут отправляться смс о тревоге
String phones = "+7xxxxxxxxxx, +7xxxxxxxxxx, +7xxxxxxxxxx";   // Белый список телефонов от которых будут обрабатываться сообщения
//-----------------------НАСТРОЙКИ---------------------

//------БИБЛИОТЕКИ------
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <SoftwareSerial.h>
#include <ArduinoJson.h>
//------БИБЛИОТЕКИ------

SoftwareSerial SIM800(8, 7); // 8 - RX Arduino (TX SIM800L), 9 - TX Arduino (RX SIM800L)
RF24 radio(9, 10);           // "создать" модуль на пинах 9 и 10

long lastUpdate = millis();
long lastSendSMS = millis();
long updatePeriod = 60000; // Проверять наличие новых сообщений каждую минуту

bool protectionStatus = true;
int supplyVoltage, batteryVoltage;
float my_vcc_const = 1.080;    // константа вольтметра
byte pipeNo;
char gotByte[33];
byte address[][6] = { "1Node", "2Node", "3Node", "4Node", "5Node", "6Node" }; //возможные номера труб
String _response = "";

void setup() {

  Serial.begin(9600);                                         // Скорость обмена данными с компьютером
  SIM800.begin(9600);                                         // Скорость обмена данными с модемом
  Serial.println("Start!");
  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  pinMode(vibro, INPUT);
  
  radio.begin();            //активировать модуль
  radio.setAutoAck(1);      //режим подтверждения приёма, 1 вкл 0 выкл
  radio.setRetries(0, 50);  //(время между попыткой достучаться, число попыток)
  radio.enableAckPayload(); //разрешить отсылку данных в ответ на входящий сигнал
  radio.setPayloadSize(32); //размер пакета, в байтах
  radio.openWritingPipe(address[0]);   //мы - труба 0, открываем канал для передачи данных

  radio.openReadingPipe(1, address[0]); //хотим слушать трубу 0
  radio.setChannel(0x60);               //выбираем канал (в котором нет шумов!)

  radio.setPALevel(RF24_PA_MAX);   //уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
  radio.setDataRate(RF24_250KBPS); //скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS

  radio.powerUp();        //начать работу
  radio.startListening(); //начинаем слушать эфир, мы приёмный модуль

  sendATCommand("AT", true);
  sendATCommand("AT+CMGF=1;&W", true);             // Включаем текстовый режима SMS (Text mode) и сразу сохраняем значение (AT&W)!
  lastUpdate = millis();
}

String sendATCommand(String cmd, bool waiting) {
  String _resp = "";                                              // Переменная для хранения результата
  Serial.println(cmd);                                            // Дублируем команду в монитор порта
  SIM800.println(cmd);                                            // Отправляем команду модулю
  if (waiting) {                                                  // Если необходимо дождаться ответа...
    _resp = waitResponse();                                       // ... ждем, когда будет передан ответ
    // Если Echo Mode выключен (ATE0), то эти 3 строки можно закомментировать
    if (_resp.startsWith(cmd)) {                                  // Убираем из ответа дублирующуюся команду
      _resp = _resp.substring(_resp.indexOf("\r", cmd.length()) + 2);
    }
    Serial.println(_resp);                                        // Дублируем ответ в монитор порта
  }
  return _resp;                                                   // Возвращаем результат. Пусто, если проблема
}

String waitResponse() {                                           // Функция ожидания ответа и возврата полученного результата
  String _resp = "";                                              // Переменная для хранения результата
  long _timeout = millis() + 10000;                               // Переменная для отслеживания таймаута (10 секунд)
  while (!SIM800.available() && millis() < _timeout) {};         // Ждем ответа 10 секунд, если пришел ответ или наступил таймаут, то...
  if (SIM800.available()) {                                       // Если есть, что считывать...
    _resp = SIM800.readString();                                  // ... считываем и запоминаем
  }
  else {                                                          // Если пришел таймаут, то...
    Serial.println("Timeout...");                                 // ... оповещаем об этом и...
  }
  return _resp;                                                   // ... возвращаем результат. Пусто, если проблема
}

bool hasmsg = false;
bool powerFlag = false;
bool akkumul = false;


void loop() {
  if (protectionStatus == 1) {                                     //если охрана включена

    if(digitalRead(vibro) == 1){
        sendSMS(phone, "someone is touching the main module");  //СМС
      }}

  supplyVoltage = readVcc();                               // считать напряжение питания
  batteryVoltage = (analogRead(A2) * (supplyVoltage / 1024)); // считать напряжение аккумулятора

    if (powerFlag == false) {
    if (digitalRead(A3) == LOW) {                  //если внешнее питание отключено

      sendSMS(phone, "external power is turned off. switching to battery");  //СМС
      powerFlag = true;
       delay(1000);
    }
  }

  if (powerFlag == true) {
    if (digitalRead(A3) == HIGH) {                  //если внешнее питание включилось

      sendSMS(phone, "external power is connected");  //СМС
      powerFlag = false;

    }
  }

    if (akkumul == false) {
    if (batteryVoltage < battery_min) {
      sendSMS(phone, "sela batareika");  //СМС
      akkumul = true;
      delay(1000);
    }
  }

  if (batteryVoltage > (battery_min + 500)) {
    akkumul = false;
  }

  while (radio.available(&pipeNo)) {
    radio.read(gotByte, 33);
    Serial.print("Recieved: "); Serial.println(gotByte);
    //for (int i = 0; i < sizeof(gotByte); i++) {
    //radio.writeAckPayload(pipeNo, &gotByte, sizeof(gotByte) );
    radio.stopListening(); //начинаем слушать эфир, мы приёмный модуль
    unsigned long last_time = millis();


    while (millis() - last_time < 2000) {
      radio.write(&gotByte, 33);
    }

    radio.startListening();

    StaticJsonDocument<32> jsonDocument;
    deserializeJson(jsonDocument, gotByte);
    int id = jsonDocument["id"];
    Serial.print("id: ");
    Serial.println(id);

    String b = jsonDocument["b"];
    Serial.print("b: ");
    Serial.println(b);

    if (protectionStatus == true && millis() - lastSendSMS > 2000) {
      sendSMS(phone, "id: " + String(id) + " trevoga");
      lastSendSMS = millis();
    }

  }



  if (lastUpdate + updatePeriod < millis()) {                    // Пора проверить наличие новых сообщений
    _response = sendATCommand("AT+CMGL=\"REC UNREAD\",1", true);// Отправляем запрос чтения непрочитанных сообщений
    if (_response.indexOf("+CMGL: ") > -1) {                    // Если есть хоть одно, получаем его индекс
      int msgIndex = _response.substring(_response.indexOf("+CMGL: ") + 7, _response.indexOf("\"REC UNREAD\"", _response.indexOf("+CMGL: ")) - 1).toInt();
      char i = 0;                                               // Объявляем счетчик попыток
      _response = sendATCommand("AT+CMGR=" + (String)msgIndex + ",1", true);  // Пробуем получить текст SMS по индексу
      _response.trim();                                       // Убираем пробелы в начале/конце
      if (_response.endsWith("OK")) {                         // Если ответ заканчивается на "ОК"
        if (!hasmsg) hasmsg = true;                           // Ставим флаг наличия сообщений для удаления
        sendATCommand("AT+CMGR=" + (String)msgIndex, true);   // Делаем сообщение прочитанным
        sendATCommand("\n", true);                            // Перестраховка - вывод новой строки
        parseSMS(_response);                                  // Отправляем текст сообщения на обработку
      }
      else {                                                  // Если сообщение не заканчивается на OK
        Serial.println("Error answer");                      // Какая-то ошибка
        sendATCommand("\n", true);                            // Отправляем новую строку и повторяем попытку
      }
    }
    else {
              if(_response == ""){
          _response = sendATCommand("AT+CMGDA=\"DEL ALL\"", true); 
          delay(50);  
          sendSMS(phone,"something went wrong, delete all messages");
          delay(1000); 
        }
      lastUpdate = millis();                                    // Обнуляем таймер
      if (hasmsg) {
        sendATCommand("AT+CMGDA=\"DEL READ\"", true);           // Удаляем все прочитанные сообщения
        hasmsg = false;
      }
    }
  }

  if (SIM800.available()) {                         // Если модем, что-то отправил...
    _response = waitResponse();                       // Получаем ответ от модема для анализа
    _response.trim();                                 // Убираем лишние пробелы в начале и конце
    Serial.println(_response);                        // Если нужно выводим в монитор порта
    if (_response.indexOf("+CMTI:") > -1) {           // Пришло сообщение об отправке SMS
      lastUpdate = millis() - updatePeriod;          // Теперь нет необходимости обрабатываеть SMS здесь, достаточно просто
      // сбросить счетчик автопроверки и в следующем цикле все будет обработано
    }
  }
  if (Serial.available()) {                          // Ожидаем команды по Serial...
    SIM800.write(Serial.read());                      // ...и отправляем полученную команду модему
  };
  if (_response.startsWith("RING")) {         // Есть входящий вызов
    //  sendATCommand("ATA", true);               // Отвечаем на вызов
    // или
    sendATCommand("ATH", true);               // Отклоняем вызов
  }
}

void parseSMS(String msg) {                                   // Парсим SMS
  String msgheader = "";
  String msgbody = "";
  String msgphone = "";

  msg = msg.substring(msg.indexOf("+CMGR: "));
  msgheader = msg.substring(0, msg.indexOf("\r"));            // Выдергиваем телефон

  msgbody = msg.substring(msgheader.length() + 2);
  msgbody = msgbody.substring(0, msgbody.lastIndexOf("OK"));  // Выдергиваем текст SMS
  msgbody.trim();

  int firstIndex = msgheader.indexOf("\",\"") + 3;
  int secondIndex = msgheader.indexOf("\",\"", firstIndex);
  msgphone = msgheader.substring(firstIndex, secondIndex);

  Serial.println("Phone: " + msgphone);                       // Выводим номер телефона
  Serial.println("Message: " + msgbody);                      // Выводим текст SMS

  if (msgphone.length() > 6 && phones.indexOf(msgphone) > -1) { // Если телефон в белом списке, то...
    setLedState(msgbody, msgphone);                           // ...выполняем команду
  }
  else {
    Serial.println("Unknown phonenumber");
  }
}

void setLedState(String result, String phone) {
  bool correct = false;                                       // Для оптимизации кода, переменная корректности команды
  String msgToSend = "";
  if (result == "Help" || result == "help" || result == "HELP") {
    sendSMS(phone, "on - enable security, off - disable security");
    correct = true;// Флаг корректности команды
  }

  if (result == "off" || result == "Off" || result == "OFF") {
    protectionStatus = false;
    sendSMS(phone, "disarmed");
    correct = true;// Флаг корректности команды

  }

  if (result == "on" || result == "On" || result == "ON") {
    protectionStatus = true;
    sendSMS(phone, "armed");
    correct = true;// Флаг корректности команды
  }
  

  if (!correct) {
    msgToSend = "Incorrect command: " + result;               // Статус исполнения
  }
  Serial.println(msgToSend);                                  // Выводим результат в терминал
}

void sendSMS(String phone, String message)
{
  sendATCommand("AT+CMGS=\"" + phone + "\"", true);             // Переходим в режим ввода текстового сообщения
  sendATCommand(message + "\r\n" + (String)((char)26), true);   // После текста отправляем перенос строки и Ctrl+Z
}

unsigned char HexSymbolToChar(char c) {
  if      ((c >= 0x30) && (c <= 0x39)) return (c - 0x30);
  else if ((c >= 'A') && (c <= 'F'))   return (c - 'A' + 10);
  else                                 return (0);
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
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA, ADSC)); // measuring
  uint8_t low = ADCL; // must read ADCL first - it then locks ADCH
  uint8_t high = ADCH; // unlocks both
  long result = (high << 8) | low;

  result = my_vcc_const * 1023 * 1000 / result; // расчёт реального VCC
  return result; // возвращает VCC
}
