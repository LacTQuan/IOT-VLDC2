#include "PubSubClient.h"
#include "WiFi.h"
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"

const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "broker.hivemq.com"; 
int port = 1883;

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
LiquidCrystal_I2C lcd(0x27,16,2);
DHTesp dhtSensor;


// Define pins for LED
#define LED_PIN 2

// Define pins for Servos
#define SERVO_ABOVE_PIN 26
#define SERVO2_BELOW_PIN 27

// Define pins for HX711 Load Cells
#define CELL_50_DT_PIN 32
#define CELL_50_SCK_PIN 33
#define CELL_5_DT_PIN 17
#define CELL_5_SCK_PIN 16

// Define pins for DHT22 Sensor
#define DHT22_PIN 15

// Define pins for HC-SR04 Ultrasonic Sensor
#define ULTRASONIC_TRIG_PIN 23
#define ULTRASONIC_ECHO_PIN 22

// Define pins for Buzzer
#define BUZZER_PIN 4

// Define pins for Photoresistor Sensor
#define LDR_PIN 34

// Define pins for LCD1602 Display
#define LCD_SCL_PIN 18
#define LCD_SDA_PIN 19


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


String getTemp() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  return String(data.temperature, 2);
}

String getHumid() {
  TempAndHumidity data = dhtSensor.getTempAndHumidity();
  return String(data.humidity, 2);
}


void setup() {
  // LED
  pinMode(2, OUTPUT);

  // DHT
  pinMode(DHT22_PIN, DHTesp::DHT22);

  // LCD
  lcd.init();
  lcd.backlight();

  // Wifi and MQTT
  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );

}

void loop() {
  delay(1000);
  lcd.clear();
  lcd.print("Hello");
  delay(1000); 
  lcd.print("World!");
}
