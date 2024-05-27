#include <ArduinoJson.h>
#include <ArduinoJson.hpp>
#include <WiFi.h>
#include <PubSubClient.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>

#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
ArduinoLEDMatrix matrix;  // LED Matrix Object

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

uint8_t frame[8][12] = {
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
  { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 }
};

//Displays a happy face when the MQTT connection is established
const uint32_t happy[] = {
  0x19819,
  0x80000001,
  0x81f8000
};

//Displays a sad face when the MQTT connection has failed
const uint32_t sad[] = {
  0x19819,     // Eyes (same as happy)
  0x80000001,  // Eyes (same as happy)
  0x81F8000    // Sad mouth
};

const uint32_t heart[] = {
  0x3184a444,
  0x44042081,
  0x100a0040
};

void mouth() {
  //Mouth
  frame[5][3] = 1;
  frame[5][9] = 1;
  frame[6][3] = 1;
  frame[6][4] = 1;
  frame[6][5] = 1;
  frame[6][6] = 1;
  frame[6][7] = 1;
  frame[6][8] = 1;
  frame[6][9] = 1;
}

void leftEye() {
  //Left eye
  frame[1][3] = 1;
  frame[1][4] = 1;
  frame[2][3] = 1;
  frame[2][4] = 1;
}

void rightEye() {
  //Right eye
  frame[1][8] = 1;
  frame[1][9] = 1;
  frame[2][8] = 1;
  frame[2][9] = 1;
}

void wink() {
  //Wink with the left eye
  frame[1][3] = 0;
  frame[1][4] = 0;
  frame[2][3] = 1;
  frame[2][4] = 1;
}

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

void setup() {
  Serial.begin(115200);

  matrix.begin();

  //Attempting to draw text
  matrix.begin();
  matrix.stroke(0xFFFFFFFF);
  // add some static text
  // will only show "UNO" (not enough space on the display)
  const char text[] = "UNO r4";
  matrix.textFont(Font_4x6);
  matrix.beginText(0, 1, 0xFFFFFF);
  matrix.println(text);
  matrix.endText();

  matrix.endDraw();

  delay(2000);
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


void reconnect() {
  // Loop until we're reconnected
  while (!client.connected()) {
    Serial.print("Attempting MQTT connection...");

    wink();
    delay(300);
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

      //Displays to the user if topic is connected
      matrix.loadFrame(heart);
      delay(500);

    } else {
      Serial.print("failed, rc=");
      Serial.print(client.state());
      Serial.println(" try again in 5 seconds");
      // Wait 5 seconds before retrying
      // Make it scroll!
      matrix.beginDraw();

      matrix.stroke(0xFFFFFFFF);
      matrix.textScrollSpeed(50);

      // add the text
      const char text[] = "    Error Connection!    ";
      matrix.textFont(Font_5x7);
      matrix.beginText(0, 1, 0xFFFFFF);
      matrix.println(text);
      matrix.endText(SCROLL_RIGHT);
      matrix.endDraw();

      // matrix.loadFrame(heart);
      // matrix.loadFrame(sad);
      delay(4000);
    }
  }
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

  //if soil dry
  if (capread < threshold) {
    digitalWrite(relay, HIGH);
    Serial.println("Current flowing");
  } else if (buttonState == HIGH) {
    // button on:
    digitalWrite(relay, HIGH);
    Serial.println("Current flowing");
  } else {
    // No current is flowing, relay is not on
    digitalWrite(relay, LOW);
  }
}
