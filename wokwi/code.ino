#include "PubSubClient.h"
#include "WiFi.h"
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
#include "HX711.h"

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "broker.hivemq.com"; 
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
LiquidCrystal_I2C lcd(0x27,16,2);
DHTesp dhtSensor;
Servo servo1, servo2;
HX711 scale50, scale5;


// Define pins for LED
#define LED_PIN 2

// Define pins for Servos
#define SERVO1_PIN 26
#define SERVO2_PIN 27

// Define pins for HX711 Load Cells
#define CELL_50_DT_PIN 32
#define CELL_50_SCK_PIN 33
#define CELL_5_DT_PIN 17
#define CELL_5_SCK_PIN 16

// Define pins for DHT22 Sensor
#define DHT22_PIN 15

// Define pins for HC-SR04 Ultrasonic Sensor
#define ULTRASONIC_TRIG_PIN 23
#define ULTRASONIC_ECHO_PIN 19

// Define pins for Buzzer
#define BUZZER_PIN 4

// Define pins for Photoresistor Sensor
#define LDR_PIN 34

// Define pins for LCD1602 Display
#define LCD_SCL_PIN 22
#define LCD_SDA_PIN 21


// Wifi and MQTT setup
void wifiConnect() {
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    lcd.print(".");
  }
  lcd.println(" Connected!");
}

void mqttConnect() {
  while(!mqttClient.connected()) {
    lcd.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if(mqttClient.connect(clientId.c_str())) {
      lcd.println("connected");

      //***Subscribe all topic you need***
     
    }
    else {
      lcd.println("try again in 5 seconds");
      delay(5000);
    }
  }
}


//MQTT Receiver
void callback(char* topic, byte* message, unsigned int length) {
  Serial.println(topic);
  String strMsg;
  for(int i=0; i<length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);

  //***Code here to process the received package***

}

// DHT 22
String getTemp() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  return String(data.temperature, 2);
}

String getHumid() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  return String(data.humidity, 2);
}

// Ultrasonic sensor
long getDistance() {
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(ULTRASONIC_TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(ULTRASONIC_TRIG_PIN, LOW);

  long duration = pulseIn(ULTRASONIC_ECHO_PIN, HIGH);
  long distanceCm = duration * 0.034 / 2;

  return distanceCm;
}


// Load
double getWeight(bool is5) {
  if (is5) return scale5.read() * 1000.0 / 420.0;
  return scale50.read() * 1000.0 / 420.0;
}

// Photoresistor
int getBrightness() {
  return analogRead(LDR_PIN);
}


void setup() {
  Serial.begin(115200);

  // LED
  pinMode(2, OUTPUT);

  // DHT
  dhtSensor.setup(DHT22_PIN, DHTesp::DHT22);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Wifi and MQTT
  // wifiConnect();
  // mqttClient.setServer(mqttServer, port);
  // mqttClient.setCallback(callback);
  // mqttClient.setKeepAlive( 90 );

  lcd.clear();
  lcd.print("Hello");
  // Ultrasonic
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // Servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(80);
  servo2.write(45);

  // Photoresister
  pinMode(LDR_PIN, INPUT);

  // Load cell
  scale5.begin(CELL_5_DT_PIN, CELL_5_SCK_PIN);
  scale50.begin(CELL_50_DT_PIN, CELL_50_SCK_PIN);
  // Link: https://wokwi.com/projects/344192176616374868


  // 

}

void loop() {
  Serial.println(getTemp() + " " + getHumid());
  Serial.println(String(getWeight(true),2) + "g");
  Serial.println(getDistance());
  Serial.println(getBrightness());
  delay(2000);
}
