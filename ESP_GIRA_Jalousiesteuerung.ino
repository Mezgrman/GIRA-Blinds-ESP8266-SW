/*
   ESP8266 Firmware for GIRA blinds controller
   (C) 2020 Julian Metzler
*/

/*
   UPLOAD SETTINGS

   Board: Generic ESP8266 Module
   Flash Mode: DIO
   Flash Size: 4M
   SPIFFS Size: 1M
   Debug port: Disabled
   Debug Level: None
   Reset Method: ck
   Flash Freq: 40 MHz
   CPU Freq: 80 MHz
   Upload Speed: 115200
*/

#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

#include "settings.h"

#define SERIAL_DEBUG

/*
   CONSTANTS
*/

#define PIN_BUTTON_UP    4
#define PIN_BUTTON_DOWN  5
#define PIN_OUTPUT_UP   13
#define PIN_OUTPUT_DOWN 12

#define WIFI_TIMEOUT 10000

#define CLOSE_DURATION 27000
#define OPEN_DURATION 30000
#define MAX_DURATION 40000

#define OPEN_POS 100
#define CLOSED_POS 0

#define POS_UPDATE_INTERVAL 500
#define MQTT_POS_UPDATE_INTERVAL 2000

/*
   GLOBAL VARIABLES
*/

// Start time of the last WiFi connection attempt
unsigned long wifiTimer = 0;
bool wifiTimedOut = false;

// Button variables
bool lastUpState = false;
bool lastDownState = false;

// MQTT variables
#define MQTT_PAYLOAD_ARR_LEN 256
char mqttPayload[MQTT_PAYLOAD_ARR_LEN] = {0x00};

// State variables
float currentPos = OPEN_POS;
bool movingUp = false;
bool movingDown = false;
unsigned long moveStart = 0;
unsigned long lastPosUpdate = 0;
unsigned long lastMqttPosUpdate = 0;

WiFiClient espClient;
PubSubClient mqttClient(espClient);

/*
   HELPER FUNCTIONS
*/

int str2int(char* str, int len) {
  int i;
  int ret = 0;
  for (i = 0; i < len; ++i)
  {
    ret = ret * 10 + (str[i] - '0');
  }
  return ret;
}

/*
   MQTT FUNCTIONS
*/

void mqttConnect() {
  while (!mqttClient.connected()) {
    Serial.print("Attempting to make MQTT connection...");
    if (mqttClient.connect("LynxLairBlinds", MQTT_USER, MQTT_PASSWORD)) {
      mqttClient.subscribe(MQTT_TOPIC_SET);
      mqttClient.subscribe(MQTT_TOPIC_SET_POS);
      Serial.println("connected");
    } else {
      Serial.print("failed, rc=");
      Serial.print(mqttClient.state());
      Serial.println(" try again in 5 seconds");
      delay(5000);
    }
  }
}

void mqttCallback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, MQTT_TOPIC_SET) == 0) {
    Serial.println("MQTT: Set State");
    if (strncmp((char*)payload, "OPEN", length) == 0) {
      Serial.println("MQTT: Moving up");
      moveUp();
    } else if (strncmp((char*)payload, "CLOSE", length) == 0) {
      Serial.println("MQTT: Moving down");
      moveDown();
    } else if (strncmp((char*)payload, "STOP", length) == 0) {
      Serial.println("MQTT: Stopping");
      stopMoving();
    }
  } else if (strcmp(topic, MQTT_TOPIC_SET_POS) == 0) {
    Serial.println("MQTT: Set Position");
    int pos = str2int((char*)payload, length);
    setPosition(pos);
  }
}

void mqttSendPosition() {
  memset(mqttPayload, 0x00, MQTT_PAYLOAD_ARR_LEN);
  sprintf(mqttPayload, "%d", (int)currentPos);
  mqttClient.publish(MQTT_TOPIC_POS, mqttPayload);
}

/*
   PROGRAM ROUTINES
*/

void handleButtons() {
  bool upState = !digitalRead(PIN_BUTTON_UP);
  bool downState = !digitalRead(PIN_BUTTON_DOWN);

  if (upState != lastUpState) {
    lastUpState = upState;
    if (upState) {
      toggleUp();
      delay(100);
    }
  }

  if (downState != lastDownState) {
    lastDownState = downState;
    if (downState) {
      toggleDown();
      delay(100);
    }
  }
}

void toggleUp() {
  if (movingUp || movingDown) {
    stopMoving();
  } else {
    moveUp();
  }
}

void toggleDown() {
  if (movingUp || movingDown) {
    stopMoving();
  } else {
    moveDown();
  }
}

void moveUp() {
  digitalWrite(PIN_OUTPUT_UP, 1);
  digitalWrite(PIN_OUTPUT_DOWN, 0);
  movingUp = true;
  movingDown = false;
  moveStart = millis();
  lastPosUpdate = moveStart;
  lastMqttPosUpdate = lastPosUpdate;
}

void moveDown() {
  digitalWrite(PIN_OUTPUT_UP, 0);
  digitalWrite(PIN_OUTPUT_DOWN, 1);
  movingUp = false;
  movingDown = true;
  moveStart = millis();
  lastPosUpdate = moveStart;
  lastMqttPosUpdate = lastPosUpdate;
}

void stopMoving() {
  digitalWrite(PIN_OUTPUT_UP, 0);
  digitalWrite(PIN_OUTPUT_DOWN, 0);
  movingUp = false;
  movingDown = false;
  mqttSendPosition();
}

void updatePosition() {
  unsigned long now = millis();
  float timeDiff = now - lastPosUpdate;

  if (timeDiff >= POS_UPDATE_INTERVAL) {
    if (movingUp) {
      currentPos += 100 * timeDiff / OPEN_DURATION;
    } else if (movingDown) {
      currentPos -= 100 * timeDiff / CLOSE_DURATION;
    }
    if (currentPos > OPEN_POS) currentPos = OPEN_POS;
    if (currentPos < CLOSED_POS) currentPos = CLOSED_POS;
    if (now - moveStart >= MAX_DURATION) {
      stopMoving();
      if (movingUp) currentPos = OPEN_POS;
      if (movingDown) currentPos = CLOSED_POS;
    } else {
      if (now - lastMqttPosUpdate >= MQTT_POS_UPDATE_INTERVAL) {
        mqttSendPosition();
        lastMqttPosUpdate = now;
      }
    }
    lastPosUpdate = now;
  }
}

void setPosition(float pos) {
  if (pos > currentPos) {
    moveUp();
  } else if (pos < currentPos) {
    moveDown();
  }

  while (true) {
    yield();
    updatePosition();
    if ((movingUp && (currentPos >= pos)) || (movingDown && (currentPos <= pos))) {
      stopMoving();
      break;
    }
  }
}

/*
   MAIN PROGRAM
*/

void setup() {
  Serial.begin(115200);
#ifdef SERIAL_DEBUG
  Serial.setDebugOutput(1);
#endif

  Serial.println("Setting up pins");
  pinMode(PIN_BUTTON_UP, INPUT_PULLUP);
  pinMode(PIN_BUTTON_DOWN, INPUT_PULLUP);
  pinMode(PIN_OUTPUT_UP, OUTPUT);
  pinMode(PIN_OUTPUT_DOWN, OUTPUT);

  Serial.println("Setting up ArduinoOTA");
  ArduinoOTA.setHostname("Jalousiesteuerung");
  ArduinoOTA.begin();

  Serial.println("Setting up Wi-Fi");
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID.c_str(), STA_PASS.c_str());
  wifiTimer = millis();
  Serial.println("  Trying to connect");
  while (WiFi.status() != WL_CONNECTED) {
    yield();
    if ((millis() - wifiTimer) > WIFI_TIMEOUT) {
      Serial.println("  Timed out!");
      wifiTimedOut = true;
      break;
    }
  }
  Serial.println("  Connected!");

  Serial.println("Setting up MQTT");
  mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
  mqttClient.setCallback(mqttCallback);
}

void loop() {
  ArduinoOTA.handle();

  if (!mqttClient.connected()) {
    ArduinoOTA.handle();
    mqttConnect();
  }
  mqttClient.loop();

  handleButtons();

  if (movingUp || movingDown) updatePosition();
}
