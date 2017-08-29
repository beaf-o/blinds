#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Math.h>
#include <ArduinoOTA.h>
#include <ArduinoJson.h>
#include <DHT.h>
#include <Wire.h>
#include <SPI.h>
#include "Timer.h"
#include "Config.h"

//#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#define BMP_SCK 13
#define BMP_MISO 12
#define BMP_MOSI 11
#define BMP_CS 10
#define PIR_PIN    D2
#define DHT_PIN    D7
#define DHTTYPE   DHT22
#define LDR_PIN    A0

int ldr;
float diffLdr = 25;

float diffTemperature = 0.1;
float temperatureDHT;
float temperatureBMP;
float temperatureAvg;

float diffHumidity = 1;
float humidity;

float diffPressure = 20;
float pressure;

float diffAltitude = 1;
float altitude;

int pirValue;
int pirStatus;
String motionStatus;

const PROGMEM uint8_t LED_PIN = D4;
const PROGMEM uint8_t UP_PIN = D0;
const PROGMEM uint8_t DOWN_PIN = D1;
const PROGMEM uint8_t POWER_LINE_PIN = D5;
const int outputPins[] = {LED_PIN, UP_PIN, DOWN_PIN, POWER_LINE_PIN};

String identifier = "10";
const String SENSORNAME = "blinds-" + identifier;
const int sensorNodes[] = {1, 6};
const int sensorNodeCount = 2;

// buffer used to send/receive data with MQTT
const uint8_t BUFFER_SIZE = 222;
char msgBuffer[BUFFER_SIZE];
char ipBuffer[BUFFER_SIZE];

const PROGMEM char* NIGHT_MODE_TOPIC = "home-assistant/nightmode";
const PROGMEM char* PANIC_TOPIC = "home-assistant/panic";
const PROGMEM char* BLINDS_MANUAL_CONTROL_COMMAND_TOPIC = "home-assistant/blinds/manual/command";
const PROGMEM char* BLINDS_MANUAL_CONTROL_STATE_TOPIC = "home-assistant/blinds/manual/status";
const PROGMEM char* BLINDS_RESET_TOPIC = "home-assistant/blinds/reset";

char testTopic[64];
const PROGMEM char* BLINDS_TOPIC_PREFIX = "home-assistant/blinds/";

const PROGMEM char* BLINDS_STATE_TOPIC = "home-assistant/blinds/4/status";
const PROGMEM char* BLINDS_COMMAND_TOPIC = "home-assistant/blinds/4/command";
const PROGMEM char* BLINDS_HEALTH_TOPIC = "home-assistant/blinds/4/health";
const PROGMEM char* BLINDS_ADMIN_COMMAND_TOPIC = "home-assistant/blinds/4/command/admin";
const PROGMEM char* BLINDS_POSITION_TOPIC = "home-assistant/blinds/4/position/command";
const PROGMEM char* BLINDS_POSITION_STATE_TOPIC = "home-assistant/blinds/4/position/status";
const PROGMEM char* BLINDS_LOG_TOPIC = "home-assistant/blinds/4/log";
const PROGMEM char* IP_TOPIC = "home-assistant/blinds/4/ip";
const PROGMEM char* SENSOR_TOPIC = "home-assistant/blinds/4/sensor";

const PROGMEM char* BLINDS_OPEN = "OPEN";
const PROGMEM char* BLINDS_CLOSE = "CLOSE";
const PROGMEM char* BLINDS_STOP = "STOP";

const String ENABLED = "ENABLED";
const String DISABLED = "DISABLED";

const int BLINDS_DURATION = 24000; //ms

int currentPosition;
boolean nightMode = false;
boolean panicMode = false;
boolean initial = true;
String manualControl = "ENABLED";

WiFiClient wifiClient;
PubSubClient pubSubClient(wifiClient);
Timer t;
Adafruit_BMP280 bmp; // I2C
DHT dht(DHT_PIN, DHTTYPE);

int commands = 0;

void setup(void){
  Serial.begin(115200);
  Serial.println("");
  Serial.println("Starting " + String(SENSORNAME));

  setupPins();
  setupConstants();
  setupWifi();
  setupMqtt();
  setupOTA();
  setupSensors();
  setupTimers();
}

boolean isSensorNode() {
  for (int s = 0; s < sensorNodeCount; s++) {
    if (String(identifier) == String(sensorNodes[s])) {
      return true;
    }
  }

  return false;
}

void setupSensors() {
if (isSensorNode()) {
    setupBmp();
    setupDHT();
  }
}

void setupPins() {
  Serial.println("Set up pins");

  pinMode(PIR_PIN, INPUT);
  pinMode(DHT_PIN, INPUT);
  pinMode(LDR_PIN, INPUT);

  pinMode(LED_PIN, OUTPUT);
  pinMode(UP_PIN, OUTPUT);
  pinMode(DOWN_PIN, OUTPUT);
  pinMode(POWER_LINE_PIN, OUTPUT);

  digitalWrite(LED_PIN, LOW);

  Serial.println("Set all blinds to HIGH by default");
  digitalWrite(UP_PIN, HIGH);
  digitalWrite(DOWN_PIN, HIGH);
  digitalWrite(POWER_LINE_PIN, HIGH);
}

void setupConstants() {
  sprintf(testTopic, "%stest", BLINDS_TOPIC_PREFIX);

  Serial.println((String) testTopic);
  delay(60000);
  /**
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
  wifiManager.setTimeout(300);

  if (!wifiManager.autoConnect(SENSORNAME.c_str(), DEFAULT_PW)) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  }
}

void setupMqtt() {
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

void setupBmp() {
  Serial.println("Set up BMP280 sensor");
  Wire.begin(D3,D4);

  boolean hasBmp = false;
  for (uint8_t b = 0; b < 3; b++) {
    if (!bmp.begin()) {
      delay(50);
    } else {
      hasBmp = true;
      return;
    }
  }

  Serial.println("Could not find a valid BMP280 sensor!");
}

void setupDHT() {
  Serial.println("Set up DHT22 sensor");
  dht.begin();
}

void setupTimers() {
  t.every(10000, sendAlive);
  t.every(60000, publishIp); // every minute
  if (isSensorNode()) {
    t.every(100, checkMotion);
    t.every(10000, checkSensors);
    t.every(600000, sendSensorState); //every 10 minutes
  }
}


void sendSensorState() {
  if (temperatureDHT != 0 && temperatureBMP != 0) {
    temperatureAvg = (temperatureDHT + temperatureBMP) / 2;
  } else if (temperatureDHT == 0 && temperatureBMP != 0) {
    temperatureAvg = temperatureBMP;
  } else if (temperatureDHT != 0 && temperatureBMP == 0) {
    temperatureAvg = temperatureDHT;
  } else {
    temperatureAvg = 0.00;
  }

  int brightness = 0;
  if (ldr != 0) {
    brightness = 1023 - ldr;
  }

  float correctedPressure = 0.00;
  if (pressure != 0) {
    correctedPressure = (pressure + 500)/100;
  }

  StaticJsonBuffer<BUFFER_SIZE> jsonBuffer;

  JsonObject& root = jsonBuffer.createObject();
  root["temperature"] = (String)temperatureAvg;
  root["temperature_dht"] = (String)temperatureDHT;
  root["temperature_bmp"] = (String)temperatureBMP;
  root["humidity"] = (String)humidity;
  root["brightness"] = (String)brightness;
  root["pressure"] = (String)correctedPressure;
  root["altitude"] = (String)altitude;
  root["motion"] = (String)motionStatus;

  char buffer[root.measureLength() + 1];
  root.printTo(buffer, sizeof(buffer));

  Serial.println(buffer);
  pubSubClient.publish(SENSOR_TOPIC, buffer, true);
}

void callback(char* p_topic, byte* p_payload, unsigned int p_length) {
  // concat the payload into a string
  String payload;
  for (uint8_t i = 0; i < p_length; i++) {
    payload.concat((char)p_payload[i]);
  }

  Serial.println("");
  Serial.print("Handle topic: '");
  Serial.print(p_topic);
  Serial.print("' with payload: '");
  Serial.print(payload);
  Serial.println("'");

  if (String(PANIC_TOPIC).equals(p_topic)) {
    handlePanicTopic(payload);
  } else if (String(BLINDS_RESET_TOPIC).equals(p_topic)) {
    //handleResetTopic(payload);
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

void loop(void) {
  reconnect();
  pubSubClient.loop();
  t.update();
  ArduinoOTA.handle();
  if (initial == true) {
    publishIp();
    initial = false;
  }
}

void reconnect() {
  while (!pubSubClient.connected()) {
    printState();
    if (connectToPrimary()) {
      subscribeToTopics();
    } else if (connectToSecondary()) {
      subscribeToTopics();
    } else {
      Serial.println("DEBUG: try again in 5 seconds");
      delay(5000);
    }
  }
}

bool connectToPrimary() {
  pubSubClient.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);

  Serial.print("Attempting primary MQTT connection to ");
  Serial.print(String(MQTT_SERVER_IP));
  Serial.print(":");
  Serial.print(String(MQTT_SERVER_PORT));
  Serial.println(" ... ");

  return doConnect();
}

bool connectToSecondary() {
  pubSubClient.setServer(MQTT_FALLBACK_SERVER_IP, MQTT_FALLBACK_SERVER_PORT);

  Serial.print("Attempting secondary MQTT connection to ");
  Serial.print(String(MQTT_FALLBACK_SERVER_IP));
  Serial.print(":");
  Serial.print(String(MQTT_FALLBACK_SERVER_PORT));
  Serial.println(" ... ");

  return doConnect();
}

bool doConnect() {
  bool isConnected = pubSubClient.connect(SENSORNAME.c_str(), MQTT_USER, MQTT_PASSWORD);
  if (isConnected == true) {
    Serial.println("connected");
  } else {
    Serial.println("ERROR: failed, rc=" + pubSubClient.state());
    printState();
  }

  return isConnected;
}

bool printState() {
  switch (pubSubClient.state()) {
    case -4:
      Serial.println("Server didn't respond within the keepalive time");
      break;
    case -3:
      Serial.println("Network connection was broken");
      break;
    case -2:
      Serial.println("Network connection failed");
      break;
    case -1:
      Serial.println("Client is disconnected cleanly");
      break;
    case 0:
      Serial.println("Cient is connected");
      break;
    case 1:
      Serial.println("Server doesn't support the requested version of MQTT");
      break;
    case 2:
      Serial.println("Server rejected the client identifier");
      break;
    case 3:
      Serial.println("Server was unable to accept the connection");
      break;
    case 4:
      Serial.println("username/password were rejected");
      break;
    case 5:
      Serial.println("Client was not authorized to connect");
      break;
  }
}

void subscribeToTopics() {
  Serial.println("Subscribe to " + String(PANIC_TOPIC));
  pubSubClient.subscribe(PANIC_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(NIGHT_MODE_TOPIC));
  pubSubClient.subscribe(NIGHT_MODE_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(BLINDS_COMMAND_TOPIC));
  pubSubClient.subscribe(BLINDS_COMMAND_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(BLINDS_POSITION_TOPIC));
  pubSubClient.subscribe(BLINDS_POSITION_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(BLINDS_ADMIN_COMMAND_TOPIC));
  pubSubClient.subscribe(BLINDS_ADMIN_COMMAND_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(BLINDS_MANUAL_CONTROL_COMMAND_TOPIC));
  pubSubClient.subscribe(BLINDS_MANUAL_CONTROL_COMMAND_TOPIC);
  pubSubClient.loop();

  Serial.println("Subscribe to " + String(BLINDS_RESET_TOPIC));
  pubSubClient.subscribe(BLINDS_RESET_TOPIC);
  pubSubClient.loop();
}

void sendAlive() {
  bool sent = pubSubClient.publish(BLINDS_HEALTH_TOPIC, "alive", true);
  if (sent == true) {
    Serial.println("Successfully sent alive.");
  } else {
    Serial.println("Failed to send alive.");
  }
}

bool checkBoundSensor(float newValue, float prevValue, float maxDiff) {
  return newValue < prevValue - maxDiff || newValue > prevValue + maxDiff;
}

void checkMotion() {
  pirValue = digitalRead(PIR_PIN);

  if (pirValue == LOW && pirStatus != 1) {
    motionStatus = "standby";
    pirStatus = 1;
    sendSensorState();

  } else if (pirValue == HIGH && pirStatus != 2) {
    motionStatus = "action";
    pirStatus = 2;
    sendSensorState();
  }
}

void checkSensors() {
  bool hasChanges = false;

  float temperatureBMPNew = bmp.readTemperature();
  if (checkBoundSensor(temperatureBMPNew, temperatureBMP, diffTemperature)) {
    temperatureBMP = temperatureBMPNew;
    hasChanges = true;
  }

  float temperatureDHTNew = dht.readTemperature(); //to use celsius remove the true text inside the parentheses
  if (checkBoundSensor(temperatureDHTNew, temperatureDHT, diffTemperature)) {
    temperatureDHT = temperatureDHTNew;
    hasChanges = true;
  }

  float pressureNew = bmp.readPressure();
  if (checkBoundSensor(pressureNew, pressure, diffPressure)) {
    pressure = pressureNew;
    hasChanges = true;
  }

  float altitudeNew = bmp.readAltitude(1021);
  if (checkBoundSensor(altitudeNew, altitude, diffAltitude)) {
    altitude = altitudeNew;
    hasChanges = true;
  }

  float humidityNew = dht.readHumidity();
  if (checkBoundSensor(humidityNew, humidity, diffHumidity)) {
    humidity = humidityNew;
    hasChanges = true;
  }

  int ldrNew = analogRead(LDR_PIN);
  if (checkBoundSensor(ldrNew, ldr, diffLdr)) {
    ldr = ldrNew;
    hasChanges = true;
  }

  if (hasChanges == true) {
    sendSensorState();
  }
}

String ipAddress2String(const IPAddress& ipAddress){
  return String(ipAddress[0]) + String(".") +\
    String(ipAddress[1]) + String(".") +\
    String(ipAddress[2]) + String(".") +\
    String(ipAddress[3]);
}

void publishIp() {
  String ipString = String(ipAddress2String(WiFi.localIP()));
  ipString.toCharArray(ipBuffer, ipString.length() + 1);

  bool publishedIp = pubSubClient.publish(IP_TOPIC, ipBuffer, false);
  if (publishedIp == true) {
    doPrintln("Published IP");
  } else {
    doPrintln("Could not publish IP.");
  }
}

void handlePanicTopic(String payload) {
  if (payload.equals("ON")) {
    doPrintln("----------------------------------------");
    doPrintln("-----          PANIC MODE          -----");
    doPrintln("----------------------------------------");

    panicMode = true;
    moveDown(0, true);
    //blockManual ?!
  } else if (panicMode != false && payload.equals("OFF")) {
    panicMode = false;
    doPrintln("Set panic mode = false");
    //unblockManual ?!
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
  if (commands >= 1000000) {
    // reset command counter to prevent overflow
    commands = 10;
  }
  /*
  if (commands == 1) {
    if (payload.equals(String(BLINDS_CLOSE))) {
      currentPosition = 0;
      doPrintln("Set initial blinds position to 0");
    } else if (payload.equals(String(BLINDS_OPEN))) {
      currentPosition = 100;
      doPrintln("Set initial blinds position to 100");
    }
    return;
  }*/

  if (panicMode == true) {
    doPrintln("Panic mode prevents action");
    return;
  }

  if (payload.equals(String(BLINDS_CLOSE))) {
    /**
    if (currentPosition == 0) {
      doPrintln("Blinds are already down");
      return;
    }

    if (nightMode == true) {
      doPrintln("Night mode prevents action");
      return;
    }
    */

    doPrintln("handleCommandTopic() => moveDown");
    moveDown(0);
  } else if (payload.equals(String(BLINDS_OPEN))) {
    /**
     if (currentPosition == 100) {
      doPrintln("Blinds are already up");
      return;
    }
    */

    if (nightMode == true) {
      doPrintln("Night mode prevents action");
      return;
    }

    doPrintln("handleCommandTopic() => moveUp");
    moveUp(100);
  }
}

void handleNightModeTopic(String payload) {
  if (payload.equals("ON")) {
    nightMode = true;
  } else if (payload.equals("OFF")) {
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

  doPrintln("currentPosition: " + String(currentPosition));
  doPrintln("newPosition: " + String(newPosition));

  if (newPosition > currentPosition) {
    if (nightMode == true) {
      doPrintln("Night mode prevents action");
      return;
    }
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
  if (newPosition > 100) {
    newPosition = 100;
  }

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
  if (newPosition < 0) {
    newPosition = 0;
  }

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

  // Send alive messages in between to keep connection alive
  int interval = 2000;
  while (duration > 0) {
    Serial.println("Remaining duration: " + String(duration));
    delay(interval);
    sendAlive();
    duration -= interval;
  }

  digitalWrite(LED_PIN, LOW);
}

void updateBlindsPosition(int newPosition) {
  doPrintln("Set currentposition variable from " + String(currentPosition) + " to " + String(newPosition));
  currentPosition = newPosition;

  reconnect();
  String state;
  if (newPosition == 0) {
    state = "closed";
  } else if (newPosition == 100) {
    state =  "opened";
  } else {
    state =  "intermediate";
  }

  bool publishedState = pubSubClient.publish(BLINDS_STATE_TOPIC, state.c_str(), true);
  pubSubClient.loop();
  if (publishedState == true) {
    doPrintln("Published blinds state: \"" + state + "\" ");
  } else {
    doPrintln("Failed to publish blinds state: \"" + state + "\" ");
  }

  String positionString = String(newPosition);
  bool publishedPosition = pubSubClient.publish(BLINDS_POSITION_STATE_TOPIC, positionString.c_str(), true);
  if (publishedPosition == true) {
    doPrintln("Published blinds position: \"" + positionString + "\" to " + String(BLINDS_POSITION_STATE_TOPIC));
  } else {
    doPrintln("Failed to publish blinds position: \"" + positionString + "\" to " + String(BLINDS_POSITION_STATE_TOPIC));
  }
}

void doPrint(String msg) {
  Serial.print(msg);
  publishLog(msg);
}

void doPrintln(String msg) {
  Serial.println(msg);
  publishLog(msg);
}

void publishLog(String msg) {
  String logMsg = "[" + SENSORNAME + "] " + msg;

  pubSubClient.publish(BLINDS_LOG_TOPIC, logMsg.c_str(), false);
  delay(100);
}
