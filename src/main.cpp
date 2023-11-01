/*
***********************************************************
*
*    Программа управления датчиком присутствия/движения
*          на ESP32 c 25MHz датчиком HLK-LD2410
*               (с) 2023, by Dr@Cosha   
*                     ver 2.0b            
*
***********************************************************
*/

#include <Arduino.h>
#include <WiFi.h>
#include "local.h"

extern "C" {
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
}

#include <AsyncMqttClient.h>
#include <ArduinoJson.h>
#include <ld2410.h>


// ------------------------------------------
// ----------- режимы компиляции  -----------
// ------------------------------------------
#define DEBUG_IN_SERIAL                             // компиляция в отладочном режиме с выводом в порт 
// #define EXTRA_MQTT_REPORT                           // компилируем прошивку в режиме передачи расширенных данных через MQTT
// ------------------------------------------
// ------------------------------------------
// ------------------------------------------

/*  Эта секция вынесена в файл "local.h" в котором содержатся "правильные" значения

#define WIFI_SSID "ssid_of_wifi_iot"                // SSID нашей локальной сети  
#define WIFI_PASSWORD "wifi_pwd"                    // пароль к нашей локальной сети
#define MQTT_USER "mqtt_user"                       // имя пользователя для подключения к MQTT серверу
#define MQTT_PWD "mqtt_pwd"                         // пароль для подключения к MQTT серверу
#define MQTT_HOST IPAddress(192, 168, 1, 1)         // адрес нашего Mosquito MQTT сервера
#define MQTT_PORT 1883                              // порт нашего Mosquito MQTT сервера

#define LWT_TOPIC   "diy/blm32_brm/LWT"             // топик публикации доступности устройства
#define SET_TOPIC   "diy/blm32_brm/set"             // топик публикации команд для устройства
#define STATE_TOPIC "diy/blm32_brm/state"           // топик публикации состояния устройства
*/

// назначаем GPIO контакты для устройств
#define PIN_EXT_OUT1 2                              // назначение выводов выхода на внешний разъем №1
#define PIN_EXT_OUT2 4                              // -..- №2
#define PIN_LED_PRESENS 25                          // выход LED индикации присутствия (2 - PRES)
#define PIN_LED_MOTION 27                           // выход LED индикации движения (15 - MOVE)
#define PIN_LED_IND 26                              // выход LED общей индикации (4 - LINK)
#define PIN_SENSOR_IN 5                             // прямой вход c сенсора - наличие движения/присутсвия

// константа определяющая промежуток времени между событиями публикации статуса в MQTT
#define MQTT_COLD_LAG 60000                         // сколько миллисекунд должно пройти между выводом состояния устройства в MQTT при отсутсвии сигнала сенсора
#define MQTT_HOT_LAG 3000                           // сколько миллисекунд должно пройти между выводом состояния устройства в MQTT при сработке сигнала сенсора
uint32_t  MQTT_STATUS_LAG = MQTT_COLD_LAG;          // изменяемая задержка вывода отчета в MQTT

// константы работы с сенсором
#define SENSOR_READ_DELAY 500                       // читаем значения сенсора каждые 500 ms

// определение JSON тегов для обмена 
#define CN_ALERT              "alert"               // тег состояния устройства  
#define CN_REPORT             "report"              // тег команды принудительной отправки состояния устройства
#define CN_MOVING             "moving"              // тег признака обнаружения движения
#define CN_MOVING_DISTANCE    "moving_distance"     // тег описания растояния до обнаруженного движущегося объекта
#define CN_MOVING_ENERGY      "moving_energy"       // тег описания энергии от обнаруженного движущегося объекта
#define CN_PRESENCE           "presence"            // тег признака обнаружения стационарного объекта
#define CN_PRESENCE_DISTANCE  "presence_distance"   // тег описания растояния до обнаруженного неподвижного объекта
#define CN_PRESENCE_ENERGY    "presence_energy"     // тег описания энергии от обнаруженного неподвижного объекта


// создаем объекты для управления MQTT-клиентом и WiFi соединением
AsyncMqttClient   mqttClient;                       // MQTT клиент
TimerHandle_t     mqttReconnectTimer;               // таймер повторной попытки установки MQTT соединения
TimerHandle_t     wifiReconnectTimer;               // таймер повторной попытки установки WiFi соединения

// создаем объект - сенсор движений
ld2410            radar_sensor;                     // создаем объект - радар-сенсор

// текущие значения сенсора
bool      CommonAlert = false;                      // общая тревога от датчика
bool      HasPresense = false;                      // есть сигнал присутсвия
bool      HasMoving   = false;                      // есть сигнал движения
uint16_t  StationaryDistance = 0,                   // расстояние до неподвижного объекта
          MovingDistance = 0;                       // расстояние до движущегося объекта
uint8_t   StationaryEnergy = 0,                     // энергия от неподвижного объекта
          MovingEnergy = 0;                         // энергия от движущегося объекта

// создаем объект - JSON документ для приема/передачи данных через MQTT
StaticJsonDocument<255> doc;                        // создаем json документ с буфером в 255 байт 

// текущие переменные программы
bool HasChanges = false;                            // есть ли изменения необходимые к отработке/отображению 
unsigned long LastReportToMQTT = 0,                 // время последнего отчета в MQTT
              LastSensorRead = 0;                   // время последнего чтения сенсора
bool Has_MQTT_Command = false;                      // флаг получения MQTT команды 


// -------------------------- вспомогательные процедуры ----------------------------
bool StrCheck(const char *str1, const char *str2){
  int i = 0;
  while(str1[i] != '\0' && str2[i] != '\0'){
    if(str1[i] != str2[i])
      return false;
    i++;
  }
  //Если мы вышли из цикла значит одну из строк мы перебрали до конца
  if(str1[i] == '\0' && str2[i] == '\0')
    return true;
  return false;
}

// ------------------ набор обработчиков событий для MQTT клиента -----------------
void connectToWifi() {
  #ifdef DEBUG_IN_SERIAL                            
    Serial.println("Try to connect Wi-Fi...");
  #endif
  xTimerStart(wifiReconnectTimer, 0);               // запускаем таймер переподключения к WiFi
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);             // запуск соединения с WiFi
}

void connectToMqtt() {
  #ifdef DEBUG_IN_SERIAL                            
    Serial.println("Try to connect MQTT server...");
  #endif
  mqttClient.connect();                             // запуск соединения с MQTT 
}

void WiFiEvent(WiFiEvent_t event) {
  #ifdef DEBUG_IN_SERIAL                              
    Serial.printf("[WiFi-event] event: %d\n", event);
  #endif  
  switch(event) {                                   // обработка событий WiFi соединения             
    case SYSTEM_EVENT_STA_GOT_IP:                   // если получили IP:

      xTimerStop(wifiReconnectTimer, 0);           // останавливаем таймер переподключения к WiFi

      #ifdef DEBUG_IN_SERIAL                                  
        Serial.println("WiFi connected");  
        Serial.println("IP address: ");  
        Serial.println(WiFi.localIP());
      #endif        

      connectToMqtt();                              // соединяемся с MQTT
      break;
    case SYSTEM_EVENT_STA_DISCONNECTED:             // если произошел разрыв соединения

      #ifdef DEBUG_IN_SERIAL                                  
        Serial.println("WiFi lost connection");
      #endif              
                                                    // делаем так, чтобы ESP32 не переподключалась к MQTT во время переподключения к WiFi:      
      xTimerStop(mqttReconnectTimer, 0);            // останавливаем таймер переподключения к MQTT
      xTimerStart(wifiReconnectTimer, 0);           // запускаем таймер переподключения к WiFi      
      digitalWrite(PIN_LED_IND, HIGH);              // зажигаем индикаторный светодиод при обрыве связи с MQTT и WiFi
      break;

    default:                                        // обработка прочих кейсов
      break;  
  }

}

// --- в этом фрагменте добавляем топики, на которые будет подписываться ESP32: SET_TOPIC
void onMqttConnect(bool sessionPresent) {   

  xTimerStop(mqttReconnectTimer, 0);                             // останавливаем таймер переподключения к MQTT

  #ifdef DEBUG_IN_SERIAL                                    
    Serial.println("Connected to MQTT.");  //  "Подключились по MQTT."
    Serial.print("Session present: ");  //  "Текущая сессия: "
    Serial.println(sessionPresent);
  #endif                
                                                                     // далее подписываем ESP32 на набор необходимых для управления топиков:
  uint16_t packetIdSub = mqttClient.subscribe(SET_TOPIC, 0);         // подписываем ESP32 на топик SET_TOPIC

  #ifdef DEBUG_IN_SERIAL                                      
    Serial.print("Subscribing at QoS 0, packetId: ");
    Serial.println(packetIdSub);
    Serial.print("Topic: ");
    Serial.println(SET_TOPIC);
  #endif                  

  mqttClient.publish(LWT_TOPIC, 0, true, "online");                 // публикуем в топик LWT_TOPIC событие о своей жизнеспособности

  #ifdef DEBUG_IN_SERIAL                                      
    Serial.print("Publishing LWT state in [");
    Serial.print(LWT_TOPIC); 
    Serial.println("]. QoS 0. "); 
  #endif                    
  
}


void onMqttDisconnect(AsyncMqttClientDisconnectReason reason) {
  #ifdef DEBUG_IN_SERIAL                                        
    Serial.println("Disconnected from MQTT.");                      // если отключились от MQTT
  #endif                               
  if (WiFi.isConnected()) {
    xTimerStart(mqttReconnectTimer, 0);                             // запускаем таймер переподключения к MQTT
  }
  // зажигаем индикаторный светодиод при обрыве связи  с MQTT
  digitalWrite(PIN_LED_IND, HIGH);                                  // включение через подачу 1
}

void onMqttSubscribe(uint16_t packetId, uint8_t qos) {
  #ifdef DEBUG_IN_SERIAL   
    Serial.println("Subscribe acknowledged.");                        // подписка подтверждена
    Serial.print("  packetId: ");                                     // 
    Serial.println(packetId);                                         // выводим ID пакета
    Serial.print("  qos: ");                                          // 
    Serial.println(qos);                                              // выводим значение QoS
  #endif         
}

void onMqttUnsubscribe(uint16_t packetId) {
  #ifdef DEBUG_IN_SERIAL     
    Serial.println("Unsubscribe acknowledged.");                      // отписка подтверждена
    Serial.print("  packetId: ");                                     //
    Serial.println(packetId);                                         // выводим ID пакета
  #endif                     
}

void onMqttPublish(uint16_t packetId) {
  #ifdef DEBUG_IN_SERIAL     
    Serial.println("Publish acknowledged.");                                              // публикация подтверждена
    Serial.print("  packetId: ");                                                         //
    Serial.println(packetId);                                                             // выводим ID пакета
  #endif                     
}


// в этой функции обрабатываем события получения данных в управляющем топике SET_TOPIC
void onMqttMessage(char* topic, char* payload, AsyncMqttClientMessageProperties properties, size_t len, size_t index, size_t total) {
  String messageTemp;

  #ifdef DEBUG_IN_SERIAL         
    Serial.print("Get message: [");
  #endif                         

  for (int i = 0; i < len; i++) {                                                       // преобразуем полученные в сообщении данные в строку
    #ifdef DEBUG_IN_SERIAL         
      Serial.print((char)payload[i]);
    #endif                         
    messageTemp += (char)payload[i];
  }
  messageTemp[len] = '\0';  

  #ifdef DEBUG_IN_SERIAL         
    Serial.println("]");
  #endif                         

  // проверяем, в каком именно топике получено MQTT сообщение
  if (strcmp(topic, SET_TOPIC) == 0) {
    // разбираем MQTT сообщение и подготавливаем буфер с изменениями для формирования команд
    deserializeJson(doc, messageTemp);                  // десерилизуем сообщение и взводим признак готовности к обработке
    Has_MQTT_Command = true;                            // взводим флаг получения команды по MQTT

  }
 
  #ifdef DEBUG_IN_SERIAL         
    Serial.println("Publish received.");                //  выводим на консоль данные из топика
    Serial.print("  topic: ");                          //  "  топик: "
    Serial.println(topic);                              // название топика 
    Serial.print("  message: ");                        //  "  сообщение: "
    Serial.println(messageTemp);                        //  сообщение 
  #endif                         

}

void get_sensor_command() { // --- процедура получения управляющих команд от сенсора ---

  radar_sensor.read();                                                                  // читаем значения сенсора
  if (radar_sensor.isConnected() and ((millis()-LastSensorRead) > SENSOR_READ_DELAY))
    { // если сенсор на связи и задержка чтения истекла, то переносим значения во внутренние переменные
      if (CommonAlert != digitalRead(PIN_SENSOR_IN)) {                                  
        // если есть изменение по общему входу датчика
        HasChanges = true;                                                              // необходима общая реакция отчет в MQTT - есть изменения в датчике
        CommonAlert = digitalRead(PIN_SENSOR_IN);                                       // запоминаем общее состояние сенсора
      }         
      if (radar_sensor.movingTargetDetected() != HasMoving) {                           
        // если есть изменение по сенсору движения
        HasChanges = true;                                                              // есть изменения значений датчика
        HasMoving = radar_sensor.movingTargetDetected();                                // запоминаем состояние датчика движения         
      }
      if (radar_sensor.presenceDetected() != HasPresense) {                           
        // если есть изменение по сенсору присутствия
        HasChanges = true;                                                              // есть изменения значений датчика
        HasPresense = radar_sensor.presenceDetected();                                  // запоминаем состояние датчика присутсвия
      }
      MovingDistance = radar_sensor.movingTargetDistance();                             // запоминаем расстояние до движущегося объекта
      StationaryDistance = radar_sensor.stationaryTargetDistance();                     // запоминаем расстояние до стационарного объекта
      StationaryEnergy = radar_sensor.stationaryTargetEnergy();                         // запоминаем силу сигнала принятую от стационарного объекта
      MovingEnergy = radar_sensor.movingTargetEnergy();                                 // - .. - от движущегося объекта      
      if (CommonAlert) { MQTT_STATUS_LAG = MQTT_HOT_LAG; }                              // настраиваем лаг вывода значений
        else { MQTT_STATUS_LAG = MQTT_COLD_LAG; } 
    }
  
}

void get_mqtt_command() { // --- процедура получения управляющих команд по каналу MQTT ---
  if (!Has_MQTT_Command) return;                              // если флага о полученной команде от MQTT нет - выходим  
    // обработка тега REPORT
    if (doc.containsKey(CN_REPORT)) { // есть тег report
      LastReportToMQTT = 0;                                   // сбрасываем задержку отправки состояния в MQTT    
    }
  doc.clear();                                                // очищаем документ
  Has_MQTT_Command = false;                                   // сбрасываем флаг об MQTT команде  
}

void get_timer_event() {  // --- обрабатываем временные задержки и генерим события от таймера ---

// по таймеру пока ничего не происходит

}

  
void get_command() {  // --- процедура получения управляющих команд ---
  HasChanges = false;                                         // обнуляем флаг изменений  
  get_timer_event();                                          // получаем событие от таймера  
  get_sensor_command();                                       // процедура получения управляющих команд от сенсора
  get_mqtt_command();                                         // процедура получения управляющих команд по каналу MQTT
}

void applay_changes() { // --- применяем команды/изменения ---
  if (!HasChanges) return;                                    // если нечего применять - выходим без обработки

  digitalWrite(PIN_LED_MOTION,HasMoving);                     // показываем текущее состояние по движению
  digitalWrite(PIN_LED_PRESENS,HasPresense);                  // показываем текущее состояние по наличию объекта

  // дублируем эти сигналы на выходной разъем

  digitalWrite(PIN_EXT_OUT1,HasMoving);                       // на EXT1 текущее состояние по движению
  digitalWrite(PIN_EXT_OUT2,HasPresense);                     // на EXT2 текущее состояние по наличию объекта

}

void report_to_asyncPort() { // --- пишем события и состояние в асинхронный порт ---
  if (!HasChanges) return;                                    // если нет изменений - просто выходим без обработки
  Serial.println();
  Serial.print("Sensor common state: ");
  if (CommonAlert) { Serial.println("Alert"); }               // пишем cостояние сенсора
    else { Serial.println("Wait"); }
  if (HasMoving)  {                                           // если есть движение
      Serial.print("Moving detect: distance=");               // выводим состояние датчика движения      
      Serial.print(MovingDistance);                           // расстояние
      Serial.print(" energy=");      
      Serial.println(MovingEnergy);                           // энергию
  } else { Serial.println("Not moving detection.");}
  if (HasPresense)  {                                         // если есть присутствие
      Serial.print("Presence detect: distance=");             // выводим состояние датчика присутствия
      Serial.print(StationaryDistance);                       // расстояние
      Serial.print(" energy=");      
      Serial.println(StationaryEnergy);                       // энергию
  } else { Serial.println("Not presence detection.");}
}

void report_to_topicMQTT() { // --- пишем события и состояние в MQTT топик ---
  // если нет соединения с MQTT - ничего не генерим, ждем соединения  
  if ((HasChanges or (MQTT_STATUS_LAG < (millis()-LastReportToMQTT)))) { // если есть изменения или наступил таймаут по публикации статуса в MQTT и при этом мы не в циклических изменениях
    if (mqttClient.connected()) {                               // если есть соединение с MQTT - выкладываем статус устройства в MQTT
        // зажигаем индикаторный светодиод при начале процедуры обмена с MQTT и гасим в конце
        // тем самым мигаем светодиодом при обмене с MQTT + гасим светодиод после начального включения и установления связи с MQTT сервером
        digitalWrite(PIN_LED_IND, HIGH);                          // включение через подачу 1
        // создаем JSON сообщение
        doc.clear();   
        if (CommonAlert) { doc[CN_ALERT] = "ON"; }                // пишем текущее состояние устройства
          else { doc[CN_ALERT] = "OFF"; }                
        // выводим состояние отдельных сенсоров 
        if (HasMoving) {                                          // пишем текущее состояние датчика движения
            doc[CN_MOVING] = "ON";                                // при сработке
            doc[CN_MOVING_DISTANCE] = MovingDistance;
            doc[CN_MOVING_ENERGY] = MovingEnergy;
          }                 
          else { 
            doc[CN_MOVING] = "OFF";                               // при выключении
            doc[CN_MOVING_DISTANCE] = 0;
            doc[CN_MOVING_ENERGY] = 0;
          }                
        if (HasPresense) {                                        // пишем текущее состояние датчика присутствия
            doc[CN_PRESENCE] = "ON";                                // при сработке
            doc[CN_PRESENCE_DISTANCE] = StationaryDistance;
            doc[CN_PRESENCE_ENERGY] = StationaryEnergy;
          }                 
          else { 
            doc[CN_PRESENCE] = "OFF";                               // при выключении
            doc[CN_PRESENCE_DISTANCE] = 0;
            doc[CN_PRESENCE_ENERGY] = 0;
          }                
        // серилизуем в строку
        String payload;
        serializeJson(doc, payload);
        // публикуем в топик STATE_TOPIC серилизованный json через буфер buffer
        char buffer[ payload.length()+1 ];
        payload.toCharArray(buffer, sizeof(buffer));   
        mqttClient.publish(STATE_TOPIC, 0, true, buffer );
        // и сбрасываем счетчик лага
        LastReportToMQTT = millis();     
        // гасим индикаторный светодиод 
        digitalWrite(PIN_LED_IND, LOW);                                                  // выключение через подачу 0        
      }     
    }
}

void report_state() { // --- сообщаем об изменении состояния в порт и топик MQTT ---
#ifdef DEBUG_IN_SERIAL                                                                  // если стоит признак отладки через порт
  report_to_asyncPort();                                                                // пишем события и состояние в асинхронный порт
#endif  
  report_to_topicMQTT();                                                                // всегда пишем события и состояние в MQTT топик

}


void setup() {  // --- процедура начальной инициализации устройства ---
  
  #ifdef DEBUG_IN_SERIAL                                                                // условная компиляция при выводе отладки в порт
    // инициализация консольного порта 
    Serial.begin(115200);     
  #endif

  // настраиваем входы и выходы контроллера
  // инициализация входов и выходов  
  pinMode(PIN_LED_IND, OUTPUT);                                                         // инициализируем pin индикаторного светодиода
  pinMode(PIN_LED_MOTION, OUTPUT);                                                      // инициализируем pin индикатора движения
  pinMode(PIN_LED_PRESENS, OUTPUT);                                                     // инициализируем pin индикатора присутствия

  // инициализируем выходные каналы модуля выведенные на разъем
  pinMode(PIN_EXT_OUT1,OUTPUT);                                 
  pinMode(PIN_EXT_OUT2,OUTPUT);
  digitalWrite(PIN_EXT_OUT1,LOW);
  digitalWrite(PIN_EXT_OUT2,LOW);

  // инициализируем прямой вход датчика
  pinMode(PIN_SENSOR_IN, INPUT);                             

  // зажигаем индикаторный светодиод 
  digitalWrite(PIN_LED_IND, HIGH);                                                      // включение через подачу 1
  
  // принудительно гасим светодиоды состояния датчика 
  digitalWrite(PIN_LED_MOTION, LOW);                       
  digitalWrite(PIN_LED_PRESENS, LOW);    

  // настраиваем соединение с датчиком движения/присутствия - LD2410
  Serial2.begin (256000, SERIAL_8N1, 17, 16);                                           // UART for monitoring the radar
  delay(500);
#ifdef DEBUG_IN_SERIAL                                                                  // условная компиляция при выводе отладки в порт       
  radar_sensor.debug(Serial);                           
  Serial.print(F("\nLD2410 radar sensor initialising: "));   
    if(radar_sensor.begin(Serial2)) { Serial.println(F("OK")); }                        // рапортуем в порт об успешности/не успешности инициализации сенсора
    else { Serial.println(F("not connected")); }                  
#elif
  radar_sensor.begin(Serial2);                                                          // просто инициализируем сенсор
#endif

  // настраиваем WiFi клиента
  WiFi.onEvent(WiFiEvent);

  // настраиваем MQTT клиента
  mqttClient.setCredentials(MQTT_USER,MQTT_PWD);
  mqttClient.onConnect(onMqttConnect);
  mqttClient.onDisconnect(onMqttDisconnect);
  mqttClient.onSubscribe(onMqttSubscribe);
  mqttClient.onUnsubscribe(onMqttUnsubscribe);
  mqttClient.onMessage(onMqttMessage);
  mqttClient.onPublish(onMqttPublish);
  mqttClient.setServer(MQTT_HOST, MQTT_PORT);

  // создаем таймеры, которые будут устанавливать и переустанавливать соединение с WiFi и MQ
  mqttReconnectTimer = xTimerCreate("mqttTimer", pdMS_TO_TICKS(8000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToMqtt));
  wifiReconnectTimer = xTimerCreate("wifiTimer", pdMS_TO_TICKS(5000), pdTRUE, (void*)0, reinterpret_cast<TimerCallbackFunction_t>(connectToWifi));  

  // запускаем подключение к WiFi
  connectToWifi();

  // инициализируем данные по последнему отчету в MQTT так, что бы сразу при старте выдать очередной отчет
  LastReportToMQTT = MQTT_STATUS_LAG + millis();

}

void loop() {  // --- основной цикл исполняемого кода устройства

  get_command();                            // получаем текущую команду
  applay_changes();                         // применяем команды/изменения
  report_state();                           // сообщаем об изменении состояния в порт и топик MQTT
 
}