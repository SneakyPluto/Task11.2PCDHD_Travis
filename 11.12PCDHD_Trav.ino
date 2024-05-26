#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
// #include <ESP8266WiFiMulti.h>

#include "Adafruit_seesaw.h"
Adafruit_seesaw ss;

const char* ssid = "Vodafone2.4G-BFC3A";
const char* password = "Leolevine12!";
const char* mqtt_server = "192.168.1.180";

WiFiClient espClient;
PubSubClient client(espClient);
unsigned long lastMsg = 0;
#define MSG_BUFFER_SIZE (50)
char msg[MSG_BUFFER_SIZE];
int value = 0;

const int buttonPin = 2;  // the number of the pushbutton pin
const int relay = 5;      // the number for the relay pin
int threshold = 400;      // Threshold used to automatically turn on  the water pump

// variables will change:
int buttonState = 0;  // variable for reading the pushbutton status

void setup_wifi() {
  delay(100);
  // We start by connecting to a WiFi network
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);

  // WiFi.init(AP_STA_MODE);
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }

  randomSeed(micros());

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
}

void callback(char* topic, byte* payload, unsigned int length) {
}

void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");
    // Create a random client ID
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);
    // Attempt to connect
    if (client.connect(clientId.c_str())) {
      Serial.println("connected");
      // Once connected, publish an announcement...
      client.publish("outTopic", "hello world");
      // ... and resubscribe
      client.subscribe("inTopic");
    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      delay(5000);
    }
  }
}

void setup() {
  Serial.begin(115200);

  // initialize the relay pin as an input:
  pinMode(relay, OUTPUT);

  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);

  while (!Serial)
    ;
  Serial.println(F("Adafruit Moisture Sensor"));

  if (!ss.begin(0x36)) {
    Serial.println("ERROR! seesaw not found");
    while (1) delay(1);
  } else {
    Serial.print("seesaw started! version: ");
    Serial.println(ss.getVersion(), HEX);
  }
  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
}

void loop() {
  float tempC = ss.getTemp();
  uint16_t capread = ss.touchRead(0);

  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  StaticJsonDocument<32> doc;
  char output[55];

  long now = millis();
  if (now - lastMsg > 30000) {
    lastMsg = now;
    doc["t"] = tempC;
    doc["m"] = capread;

    Serial.println("Read");
    serializeJson(doc, output);
    Serial.println(output);
    client.publish("/home/sensors", output);
    Serial.println("Sent");
  }

  if (capread < threshold) {
    digitalWrite(relay, HIGH);
    Serial.println("Current flowing");
  } else if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(relay, HIGH);
    Serial.println("Current flowing");
  } else {
    // No current is flowing, relay is not on
    digitalWrite(relay, LOW);
    
  }
}
