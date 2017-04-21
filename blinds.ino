#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Math.h>
#include <ArduinoOTA.h>
#include "DHT.h"
#include "Timer.h"

Timer t;

/**
static const uint8_t D0   = 16;
static const uint8_t D1   = 5;
static const uint8_t D2   = 4;
static const uint8_t D3   = 0;
static const uint8_t D4   = 2;
static const uint8_t D5   = 14;
static const uint8_t D6   = 12;
static const uint8_t D7   = 13;
static const uint8_t D8   = 15;
static const uint8_t D9   = 3;
static const uint8_t D10  = 1;
*/

// CREDENTIALS SECTION - START
const PROGMEM int OTA_PORT = 8266;
const PROGMEM char* DEFAULT_PW = "****";
const PROGMEM char* MQTT_SERVER_IP = "****";
const PROGMEM char* MQTT_FALLBACK_SERVER_IP = "****";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1883;
const PROGMEM char* MQTT_USER = "****";
const PROGMEM char* MQTT_PASSWORD = "****";
// CREDENTIALS SECTION - END

const PROGMEM uint8_t LED_PIN = D4;

const PROGMEM uint8_t UP_PIN = D0;
const PROGMEM uint8_t DOWN_PIN = D1;

const PROGMEM uint8_t POWER_LINE_PIN = D5;

const int outputPins[] = {LED_PIN, UP_PIN, DOWN_PIN, POWER_LINE_PIN};

String identifier = "1";
const String SENSORNAME = "blinds-" + identifier;

const PROGMEM char* NIGHT_MODE_TOPIC = "home-assistant/nightmode";
const PROGMEM char* PANIC_TOPIC = "home-assistant/panic";
const PROGMEM char* BLINDS_MANUAL_CONTROL_COMMAND_TOPIC = "home-assistant/blinds/manual/command";
const PROGMEM char* BLINDS_MANUAL_CONTROL_STATE_TOPIC = "home-assistant/blinds/manual/status";

String mainTopicsPrefix = "home-assistant/blinds/" + identifier;
String espTopicsPrefix = "home-assistant/esp/blinds/" + identifier;

/**
String espStateTopicStr = espTopicsPrefix + "/status";
String espIpTopicStr = espTopicsPrefix + "/ip";
String blindsStateTopicStr = mainTopicsPrefix + "/status";
String blindsCommandTopicStr = mainTopicsPrefix + "/command";
String blindsAdminCommandTopicStr = mainTopicsPrefix + "/command/admin";
String blindsPositionTopicStr = mainTopicsPrefix + "/position/command";
String blindsPositionStateTopicStr = mainTopicsPrefix + "/position/status";
*/

// buffer used to send/receive data with MQTT
const uint8_t BUFFER_SIZE = 20;
char msgBuffer[BUFFER_SIZE]; 
char posBuffer[BUFFER_SIZE];
char ipBuffer[BUFFER_SIZE];

const PROGMEM char* ESP_STATE_TOPIC = "home-assistant/esp/blinds/1/status";
const PROGMEM char* ESP_IP_TOPIC = "home-assistant/esp/blinds/1/ip";

const PROGMEM char* BLINDS_STATE_TOPIC = "home-assistant/blinds/1/status";
const PROGMEM char* BLINDS_COMMAND_TOPIC = "home-assistant/blinds/1/command";
const PROGMEM char* BLINDS_ADMIN_COMMAND_TOPIC = "home-assistant/blinds/1/command/admin";
const PROGMEM char* BLINDS_POSITION_TOPIC = "home-assistant/blinds/1/position/command";
const PROGMEM char* BLINDS_POSITION_STATE_TOPIC = "home-assistant/blinds/1/position/status";
const PROGMEM char* BLINDS_LOG_TOPIC = "home-assistant/blinds/1/log";

/*
char ESP_STATE_TOPIC[BUFFER_SIZE];
char ESP_IP_TOPIC[BUFFER_SIZE];
char BLINDS_STATE_TOPIC[BUFFER_SIZE];
char BLINDS_COMMAND_TOPIC[BUFFER_SIZE];
char BLINDS_ADMIN_COMMAND_TOPIC[BUFFER_SIZE];
char BLINDS_POSITION_TOPIC[BUFFER_SIZE];
char BLINDS_POSITION_STATE_TOPIC[BUFFER_SIZE];
*/

const PROGMEM char* BLINDS_OPEN = "OPEN";
const PROGMEM char* BLINDS_CLOSE = "CLOSE";
const PROGMEM char* BLINDS_STOP = "STOP";

const String ENABLED = "ENABLED";
const String DISABLED = "DISABLED";

const int BLINDS_DURATION = 5000;//23000; //ms

boolean isInitial = true;
int currentPosition;
boolean nightMode = false;
boolean panicMode = false;
String manualControl = "ENABLED";

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);

#define DHTTYPE DHT22
#define DHTPIN D3 

DHT dht(DHTPIN, DHTTYPE);

int commands = 0;

void setup(void){
  Serial.begin(115200);
  Serial.println("");

  setupPins();
  setupConstants();
  setupWifi();
  setupMqtt();
  setupOTA();
  setupDHT();
  setupTimer();
}

void setupPins() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(UP_PIN, OUTPUT);
  pinMode(DOWN_PIN, OUTPUT);
  pinMode(POWER_LINE_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  Serial.println("Switch off blinds by default");
  digitalWrite(UP_PIN, HIGH);
  digitalWrite(DOWN_PIN, HIGH);
  digitalWrite(POWER_LINE_PIN, HIGH);
}

void setupConstants() {
  /**
  espStateTopicStr.toCharArray(ESP_STATE_TOPIC, espStateTopicStr.length() + 1);
  espIpTopicStr.toCharArray(ESP_IP_TOPIC, espIpTopicStr.length() + 1);

  blindsStateTopicStr.toCharArray(BLINDS_STATE_TOPIC, blindsStateTopicStr.length() + 1);
  blindsCommandTopicStr.toCharArray(BLINDS_COMMAND_TOPIC, blindsCommandTopicStr.length() + 1);
  blindsAdminCommandTopicStr.toCharArray(BLINDS_ADMIN_COMMAND_TOPIC, blindsAdminCommandTopicStr.length() + 1);
  blindsPositionTopicStr.toCharArray(BLINDS_POSITION_TOPIC, blindsPositionTopicStr.length() + 1);
  blindsPositionStateTopicStr.toCharArray(BLINDS_POSITION_STATE_TOPIC, blindsPositionStateTopicStr.length() + 1);
  */
}

void setupWifi() {
  delay(10);
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  if (!wifiManager.autoConnect(SENSORNAME.c_str(), DEFAULT_PW)) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
}

void setupMqtt() {
  pubSubClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  pubSubClient.setCallback(callback);
}

void setupOTA() {
  ArduinoOTA.setPort(OTA_PORT);
  ArduinoOTA.setHostname(SENSORNAME.c_str());
  ArduinoOTA.setPassword(DEFAULT_PW);

  ArduinoOTA.onStart([]() {
    Serial.println("Starting OTA");
  });
  
  ArduinoOTA.onEnd([]() {
    Serial.println("\nEnd");
  });
  
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
  });
  
  ArduinoOTA.onError([](ota_error_t error) {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed");
  });
  
  ArduinoOTA.begin();
}

void setupDHT() {
  dht.begin();
}

void setupTimer() {
  t.every(60000, checkDHT);
}

void checkDHT() {
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();

  String room = (identifier <= 4) ? "wz" : "sz";

  if (!isnan(humidity)) {
    String humidityTopic = "home-assistant/humidity/" + room;
    pubSubClient.publish(humidityTopic, String(humidity));
  }

  if (!isnan(temperature)) {
    String temperatureTopic = "home-assistant/temperature/" + room;
    pubSubClient.publish(temperatureTopic, String(temperature));
  }
}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  Serial.print("Handle topic: '");
  Serial.print(p_topic);
  Serial.print("' with payload: '");
  Serial.print(payload);
  Serial.println("'");

  if (String(PANIC_TOPIC).equals(p_topic)) {
    handlePanicTopic(payload);
  } else if (String(BLINDS_COMMAND_TOPIC).equals(p_topic)) {
    handleCommandTopic(payload);
  } else if (String(BLINDS_ADMIN_COMMAND_TOPIC).equals(p_topic)) {
    handleAdminCommandTopic(payload);
  } else if (String(NIGHT_MODE_TOPIC).equals(p_topic)) {
    handleNightModeTopic(payload);
  } else if (String(BLINDS_POSITION_TOPIC).equals(p_topic)) {
    handlePositionTopic(payload.toInt());
  } else if (String(BLINDS_MANUAL_CONTROL_COMMAND_TOPIC).equals(p_topic)) {
    handleManualControlTopic(payload);
  }
}

void reconnect() {
  while (!pubSubClient.connected()) {
    Serial.print("Attempting MQTT connection... ");
    if (pubSubClient.connect(SENSORNAME.c_str(), MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("connected");

      pubSubClient.subscribe(BLINDS_COMMAND_TOPIC);
      pubSubClient.subscribe(BLINDS_ADMIN_COMMAND_TOPIC);
      pubSubClient.subscribe(BLINDS_POSITION_TOPIC);
      pubSubClient.subscribe(BLINDS_MANUAL_CONTROL_COMMAND_TOPIC);
      pubSubClient.subscribe(NIGHT_MODE_TOPIC);
      pubSubClient.subscribe(PANIC_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.println(pubSubClient.state());
      Serial.println("DEBUG: try again in 5 seconds");
      delay(5000);
    }
  }
}

String ipAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
    String(ipAddress[1]) + String(".") +\
    String(ipAddress[2]) + String(".") +\
    String(ipAddress[3]); 
}

void loop(void) {
  if (!pubSubClient.connected()) {
    reconnect();
  }

  if (isInitial == true) {
    publishStateAndIp();
  }
  
  pubSubClient.loop();

  ArduinoOTA.handle();

  t.update();

  isInitial = false;
}

void publishStateAndIp() {
  String ipString = String(ipAddress2String(WiFi.localIP()));
  ipString.toCharArray(ipBuffer, ipString.length() + 1); 
  
  doPrintln("Publish ESP state and ip");
  doPrintln("IP address: " + ipString);

  pubSubClient.publish(ESP_STATE_TOPIC, "online", false);
  pubSubClient.publish(ESP_IP_TOPIC, ipBuffer, false);

  Serial.println("Done publishing state and ip");
}

void handlePanicTopic(String payload) {
  if (payload.equals("ON")) {
    doPrintln("----------------------------------------");
    doPrintln("-----          PANIC MODE          -----");
    doPrintln("----------------------------------------");

    panicMode = true;
    moveDown(0, true);
  } else if (panicMode != false && payload.equals("OFF")) {
    panicMode = false;
    doPrintln("Set panic mode = false");
  }
}

void handleAdminCommandTopic(String payload) {
  if (payload.equals(String(BLINDS_CLOSE))) {
      moveDown(0, true);
    } else if (payload.equals(String(BLINDS_OPEN))) {
      moveUp(100);
    }
}

void handleCommandTopic(String payload) {
  commands++;
    if (commands == 1) {
      if (payload.equals(String(BLINDS_CLOSE))) {
        currentPosition = 0;
        doPrintln("Set initial blinds position to 0");
      } else if (payload.equals(String(BLINDS_OPEN))) {
        currentPosition = 100;
        doPrintln("Set initial blinds position to 100");
      }
      return;  
    }

    if (panicMode == true) {
      doPrintln("Panic mode prevents action");
      return;
    }

    if (payload.equals(String(BLINDS_CLOSE))) {
      if (currentPosition == 0) {
        doPrintln("Blinds are already down");
        return;
      }

      if (nightMode == true) {
        doPrintln("Night mode prevents action");
        return;
      }

      doPrintln("handleCommandTopic() => moveDown");
      moveDown(0);
    } else if (payload.equals(String(BLINDS_OPEN))) {
      if (currentPosition == 100) {
        doPrintln("Blinds are already up");
        return;
      }

      if (nightMode == true) {
        doPrintln("Night mode prevents action");
        return;
      }

      doPrintln("handleCommandTopic() => moveUp");
      moveUp(100);
    }
}

void handleNightModeTopic(String payload) {
  if (nightMode != true && payload.equals("ON")) {
    nightMode = true;  
  } else if (nightMode != false && payload.equals("ON")) {
    nightMode = false;  
  }
}

void handleManualControlTopic(String payload) {
  if (manualControl != "ENABLED" && payload.equals("ENABLED")) {
    manualControl = "ENABLED";
    unblockManualControl();
  } else if (manualControl != "DISABLED" && payload.equals("DISABLED")) {
    manualControl = "DISABLED";
    blockManualControl();
  }
}

void handlePositionTopic(int newPosition) {
  if (newPosition == currentPosition) {
    doPrintln("Blinds are already in the wanted position");
    return;
  }

  if (panicMode == true) {
    doPrintln("Panic mode prevents action");
    return;
  }
    
  if (nightMode == true) {
    doPrintln("Night mode prevents action");
    return;
  }

  doPrintln("currentPosition: " + String(currentPosition));
  doPrintln("newPosition: " + String(newPosition));

  if (newPosition > currentPosition) {
    doPrintln("handlePositionTopic() => moveUp()");
    moveUp(newPosition);
  } else {
    doPrintln("handlePositionTopic() => moveDown()");
    moveDown(newPosition);
  }
}

void blockManualControl() {
  doPrintln("blockManualControl()");
  digitalWrite(POWER_LINE_PIN, LOW);
  delay(500);
}

void unblockManualControl() {
  doPrintln("unblockManualControl()");
  delay(1000);
  digitalWrite(POWER_LINE_PIN, HIGH);
  delay(100);
}

int getDuration(int currentPosition, int newPosition) {
  // current: 0, new: 50
  // diff: 50
  // ratio = 50/100 = 0.5
  // duration = ratio * BLINDS_DURATION;

  int positionDiff = abs(newPosition - currentPosition);
  //Serial.print("Position diff: ");Serial.println(positionDiff);
  float ratio = positionDiff / 100.0;
  //Serial.print("Ratio: ");Serial.println(String(ratio));
  int duration = round(ratio * BLINDS_DURATION);

  return duration;
}

void moveUp(int newPosition) {
  blockManualControl();

  doPrintln("Move blinds up to position " + String(newPosition));
  digitalWrite(UP_PIN, LOW);
 
  int duration = getDuration(currentPosition, newPosition);
  waitForBlindsToMove(duration);
  
  digitalWrite(UP_PIN, HIGH);

  unblockManualControl();
  
  doPrintln("Done moving blinds up\n");
  updateBlindsPosition(newPosition);
}

void moveDown(int newPosition) {
  moveDown(newPosition, false);
}

void moveDown(int newPosition, boolean forceDuration) {
  blockManualControl();

  doPrintln("Move blinds down to position " + String(newPosition));
  digitalWrite(DOWN_PIN, LOW);
  
  int duration = getDuration(currentPosition, newPosition);
  if (forceDuration) {
    duration = BLINDS_DURATION;
  }
  waitForBlindsToMove(duration);
  
  digitalWrite(DOWN_PIN, HIGH);

  unblockManualControl();
  
  doPrintln("Done moving blinds down\n");
  updateBlindsPosition(newPosition);
}

void waitForBlindsToMove(int duration) {
  doPrintln("Move blinds for " + String(duration) + " ms");

  digitalWrite(LED_PIN, HIGH);
  delay(duration);
  digitalWrite(LED_PIN, LOW);
}

void updateBlindsPosition(int newPosition) {
  doPrintln("Set currentposition variable from " + String(currentPosition) + " to " + String(newPosition));
  currentPosition = newPosition;

  doPrint("Publish blinds state: \"");
  boolean publishedState;
  if (newPosition == 0) {
    pubSubClient.publish(BLINDS_STATE_TOPIC, "closed", true);
    doPrint("closed");
  } else {
    pubSubClient.publish(BLINDS_STATE_TOPIC, "opened", true);
    doPrint("opened");
  }
  
  doPrintln("\" to " + String(BLINDS_STATE_TOPIC));

  String positionString = String(newPosition); 
  positionString.toCharArray(posBuffer, positionString.length() + 1); 
  doPrintln("Publish blinds position: \"" + positionString + "\" to " + String(BLINDS_POSITION_STATE_TOPIC));
  pubSubClient.publish(BLINDS_POSITION_STATE_TOPIC, posBuffer, true);
}

void doPrint(String msg) {
  Serial.print(msg);
  //publishLog(msg);
}

void doPrintln(String msg) {
  Serial.println(msg);
  //publishLog(msg);
}

void publishLog(String msg) {
  String logMsg = "[" + SENSORNAME + "] " + msg;

  char logBuffer[50];
  logMsg.toCharArray(logBuffer, logMsg.length() + 1); 

  pubSubClient.publish(BLINDS_LOG_TOPIC, logBuffer, true);
  delay(100);
}
