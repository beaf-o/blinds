#include <ESP8266WiFi.h>
#include <WiFiClient.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <WiFiManager.h>
#include <PubSubClient.h>
#include <Math.h>

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

const PROGMEM uint8_t LED_PIN = D4;

const PROGMEM uint8_t UP_PIN = D0;
const PROGMEM uint8_t DOWN_PIN = D5;

const PROGMEM uint8_t POWER_LINE_PIN = D2;

const int outputPins[] = {LED_PIN, UP_PIN, DOWN_PIN, POWER_LINE_PIN};

String identifier = "1";

// MQTT: ID, server IP, port, username and password
const PROGMEM char* MQTT_CLIENT_ID = "blinds-1";
const PROGMEM char* MQTT_SERVER_IP = "bentleycoupe.no-ip.org";
const PROGMEM uint16_t MQTT_SERVER_PORT = 1885;
const PROGMEM char* MQTT_USER = "home-assistant";
const PROGMEM char* MQTT_PASSWORD = "****";

const PROGMEM char* MQTT_NIGHT_MODE_TOPIC = "home-assistant/nightmode";
const PROGMEM char* MQTT_PANIC_TOPIC = "home-assistant/panic";

const PROGMEM char* MQTT_ESP_STATE_TOPIC = "home-assistant/esp/blinds/1/status";
const PROGMEM char* MQTT_ESP_IP_TOPIC = "home-assistant/esp/blinds/1/ip";

const PROGMEM char* MQTT_BLINDS_STATE_TOPIC = "home-assistant/blinds/1/status";
const PROGMEM char* MQTT_BLINDS_COMMAND_TOPIC = "home-assistant/blinds/1/command";
const PROGMEM char* MQTT_BLINDS_ADMIN_COMMAND_TOPIC = "home-assistant/blinds/1/command/admin";
const PROGMEM char* MQTT_BLINDS_POSITION_TOPIC = "home-assistant/blinds/1/position/command";
const PROGMEM char* MQTT_BLINDS_POSITION_STATE_TOPIC = "home-assistant/blinds/1/position/status";

// buffer used to send/receive data with MQTT
const uint8_t MSG_BUFFER_SIZE = 20;
char m_msg_buffer[MSG_BUFFER_SIZE]; 

const PROGMEM char* BLINDS_OPEN = "OPEN";
const PROGMEM char* BLINDS_CLOSE = "CLOSE";
const PROGMEM char* BLINDS_STOP = "STOP";

const int BLINDS_DURATION = 23; //23s = 23000ms

boolean isInitial = true;
int currentPosition;
boolean nightMode = false;
boolean panicMode = false;

WiFiClient wifiClient;
PubSubClient client(wifiClient);

int commands = 0;

void setup(void){
  Serial.begin(115200);
  Serial.println("");

  setupPins();
  setupBlinds();
  setupWifi();
  setupMqtt();
}

void setupPins() {
  pinMode(LED_PIN, OUTPUT);
  pinMode(UP_PIN, OUTPUT);
  pinMode(DOWN_PIN, OUTPUT);
  pinMode(POWER_LINE_PIN, OUTPUT);
}

void setupBlinds() {
  Serial.println("Switch off blinds by default");
  digitalWrite(UP_PIN, HIGH);
  digitalWrite(DOWN_PIN, HIGH);
  digitalWrite(POWER_LINE_PIN, HIGH);
}

void setupWifi() {
  delay(10);
  WiFiManager wifiManager;
  wifiManager.setTimeout(180);

  String apName = "ESP_AP_" + identifier;
  if (!wifiManager.autoConnect(apName.c_str(), "1234567890")) {
    Serial.println("Failed to connect and hit timeout");
    delay(3000);
    //reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(5000);
  } 
  
  Serial.println("");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());  
}

void setupMqtt() {
  // init the MQTT connection
  client.setServer(MQTT_SERVER_IP, MQTT_SERVER_PORT);
  client.setCallback(callback);
}

// function called when a MQTT message arrived
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

  if (String(MQTT_BLINDS_COMMAND_TOPIC).equals(p_topic)) {
    handleCommandTopic(payload);
  } else if (String(MQTT_BLINDS_ADMIN_COMMAND_TOPIC).equals(p_topic)) {
    if (payload.equals(String(BLINDS_CLOSE))) {
      moveDown(0);
    } else if (payload.equals(String(BLINDS_OPEN))) {
      moveUp(100);
    }
  } else if (String(MQTT_NIGHT_MODE_TOPIC).equals(p_topic)) {
    if (payload.equals("ON")) {
      nightMode = true;  
    } else {
      nightMode = false;  
    }
  } else if (String(MQTT_PANIC_TOPIC).equals(p_topic)) {
    if (payload.equals("ON")) {
      panicMode = true;
      moveDown(0);
      Serial.println("Set panic mode = true");
    } else if (payload.equals("OFF")) {
      panicMode = false;
      Serial.println("Set panic mode = false");
    }
  } else if (String(MQTT_BLINDS_POSITION_TOPIC).equals(p_topic)) {
    handlePositionTopic(payload.toInt());
  }
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("INFO: Attempting MQTT connection...");
    // Attempt to connect
    if (client.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
      Serial.println("INFO: connected");
      client.subscribe(MQTT_BLINDS_COMMAND_TOPIC);
      client.subscribe(MQTT_BLINDS_ADMIN_COMMAND_TOPIC);
      client.subscribe(MQTT_BLINDS_POSITION_TOPIC);
      client.subscribe(MQTT_NIGHT_MODE_TOPIC);
      client.subscribe(MQTT_PANIC_TOPIC);
    } else {
      Serial.print("ERROR: failed, rc=");
      Serial.println(client.state());
      Serial.println("DEBUG: try again in 5 seconds");
      // Wait 5 seconds before retrying
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
  wdt_disable();

  if (!client.connected()) {
    reconnect();
  }

  if (isInitial == true) {
    IPAddress ipAddress = WiFi.localIP();
    String ipString = ipAddress2String(ipAddress);

    Serial.println("Publish ESP state and ip");
    client.publish(MQTT_ESP_STATE_TOPIC, "online", true);
    //client.publish(MQTT_ESP_IP_TOPIC, ipString.c_str(), true);
  }

  client.loop();

  isInitial = false;
}

void handleCommandTopic(String payload) {
  commands++;
    if (commands == 1) {
      if (payload.equals(String(BLINDS_CLOSE))) {
        currentPosition = 0;
        Serial.println("Set initial blinds position to 0");
      } else if (payload.equals(String(BLINDS_OPEN))) {
        currentPosition = 100;
        Serial.println("Set initial blinds position to 100");
      }
      return;  
    }

    if (payload.equals(String(BLINDS_CLOSE))) {
      if (currentPosition == 0) {
        Serial.println("Blinds are already down");
        return;
      }

      if (nightMode == true) {
        Serial.println("Night mode prevents action");
        return;
      }

      Serial.println("handleCommandTopic() => moveDown");
      moveDown(0);
    } else if (payload.equals(String(BLINDS_OPEN))) {
      if (currentPosition == 100) {
        Serial.println("Blinds are already up");
        return;
      }

      if (nightMode == true) {
        Serial.println("Night mode prevents action");
        return;
      }

      if (panicMode == true) {
        Serial.println("Panic mode prevents action");
        return;
      }

      Serial.println("handleCommandTopic() => moveUp");
      moveUp(100);
    }
}

void handleAdminCommandTopic(String payload) {
  if (payload.equals(String(BLINDS_CLOSE))) {
      moveDown(0);
    } else if (payload.equals(String(BLINDS_OPEN))) {
      moveUp(100);
    }

    String positionString = String(currentPosition); 
    positionString.toCharArray(m_msg_buffer, positionString.length() + 1); 
    Serial.println("Publish blinds position: " + positionString + " to " + String(MQTT_BLINDS_POSITION_STATE_TOPIC));
    client.publish(MQTT_BLINDS_POSITION_STATE_TOPIC, m_msg_buffer, true);
}

void handlePositionTopic(int newPosition) {
  if (newPosition == currentPosition) {
    Serial.println("Blinds are already in the wanted position");
    return;
  }

  if (nightMode == true) {
    Serial.println("Night mode prevents action");
    return;
  }

  Serial.println("currentPosition: " + String(currentPosition));
  Serial.println("newPosition: " + String(newPosition));
  double diff = currentPosition - newPosition;
  if (newPosition > currentPosition) { // not working
    Serial.println("handlePositionTopic() => moveUp()");
    moveUp(newPosition);
  } else {
    Serial.println("handlePositionTopic() => moveDown()");
    moveDown(newPosition);
  }
}

void blockManualLine() {
  Serial.println("blockManualLine()");
  delay(100);
  digitalWrite(POWER_LINE_PIN, LOW);
  delay(100);
}

void unblockManualLine() {
  Serial.println("unblockManualLine()");
  delay(100);
  digitalWrite(POWER_LINE_PIN, HIGH);
  delay(100);
}

int getDuration(int currentPosition, int newPosition) {
  // current: 0, new: 50
  // diff: 50
  // ratio = 50/100 = 0.5
  // duration = ratio * BLINDS_DURATION;

  int positionDiff = abs(newPosition - currentPosition);
  double ratio = positionDiff / 100;
  
  return (int) ratio * BLINDS_DURATION;
}

void moveUp(int newPosition) {
  blockManualLine();

  Serial.println("Move blinds up to position " + newPosition);
  digitalWrite(UP_PIN, LOW);

  //int duration = getDuration(currentPosition, newPosition);
  //waitForBlindsToMove(duration);
  waitForBlindsToMove();
  
  digitalWrite(UP_PIN, HIGH);

  unblockManualLine();
  
  Serial.println("Done moving blinds up\n");
  updateBlindsPosition(newPosition);
}

void moveDown(int newPosition) {
  blockManualLine();

  Serial.println("Move blinds down to position " + newPosition);
  //digitalWrite(DOWN_PIN, LOW);
  
  //int duration = getDuration(currentPosition, newPosition);
  //waitForBlindsToMove(duration);
  waitForBlindsToMove();
  
  //digitalWrite(DOWN_PIN, HIGH);

  unblockManualLine();
  
  Serial.println("Done moving blinds down\n");
  updateBlindsPosition(newPosition);
}

void waitForBlindsToMove() {
  waitForBlindsToMove(BLINDS_DURATION);
}
void waitForBlindsToMove(int duration) {
  Serial.println("Move blinds for " + String(duration) + " s");

  for (int i = 1; i < (duration/2) + 1; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(500);
    digitalWrite(LED_PIN, LOW);
    delay(500);
    Serial.print(i);
  }
  Serial.println("");
}

void updateBlindsPosition(int newPosition) {
  Serial.println("Set currentposition from " + String(currentPosition) + " to " + String(newPosition));
  currentPosition = newPosition;

  Serial.print("Publish blinds state: \"");
  if (newPosition == 0) {
    client.publish(MQTT_BLINDS_STATE_TOPIC, "closed", true);
    Serial.print("closed");
  } else {
    client.publish(MQTT_BLINDS_STATE_TOPIC, "opened", true);
    Serial.print("opened");
  }
  Serial.println("\" to " + String(MQTT_BLINDS_STATE_TOPIC));

  String positionString = String(currentPosition); 
  positionString.toCharArray(m_msg_buffer, positionString.length() + 1); 
  Serial.println("Publish blinds position: " + positionString + " to " + String(MQTT_BLINDS_POSITION_STATE_TOPIC));
  client.publish(MQTT_BLINDS_POSITION_TOPIC, m_msg_buffer, true);
}
