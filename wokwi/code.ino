#include "PubSubClient.h"
#include "WiFi.h"
#include <ESP32Servo.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"
#include "HX711.h"
#include <string>
// #include <RTClib.h>


const char* ssid = "Wokwi-GUEST";
const char* password = "";
const char* mqttServer = "broker.hivemq.com";
int port = 1883;
// IFTTT
const char* iftttHost = "maker.ifttt.com";

WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);
LiquidCrystal_I2C lcd(0x27, 16, 2);
DHTesp dhtSensor;
Servo servo1, servo2;
HX711 scale50, scale5;
// Tong khoi luong thuc an ma binh chua duoc la 20kg
double MAX_FOOD = 20000;
// cho an thu cong
bool isFed = false;
//real time  clock
// RTC_DS1307 rtc;
// DateTime now;

// sau mot gio thi bat thong bao len
// DateTime lastFoodNoti;
// DateTime lastEnvNoti;

// init curBrightness
int curBrightness = 120;

// amount for each eating time
// [0]: 8 - 10h
// [1]: 12 - 14h
// [2]: 16 - 18h
// [3]: 20 - 22h
int eatingSchedule[5] = {0, 0, 0, 0, 0};
int schedIndex = 0;

long long lastTime;
int sendFoodAmountFlag = 1;
int realEatingAmount[4] = {0, 0, 0, 0};

// Define pins for LED
#define LED_PIN 2

// Define pins for Servos
#define SERVO1_PIN 14
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
#define BUZZER_PIN 5

// Define pins for Photoresistor Sensor
#define LDR_PIN 34

// Define pins for LCD1602 Display
#define LCD_SCL_PIN 22
#define LCD_SDA_PIN 21

// Note for buzzer
#define NOTE_C4 261.63
#define NOTE_D4 293.66
#define NOTE_E4 329.63
#define NOTE_F4 349.23
#define NOTE_G4 392.00
#define NOTE_A4 440.00
#define NOTE_B4 493.88

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
  while (!mqttClient.connected()) {
    lcd.println("Attemping MQTT connection...");
    String clientId = "ESP32Client-";
    clientId += String(random(0xffff), HEX);
    if (mqttClient.connect(clientId.c_str())) {
      lcd.println("connected");

      //***Subscribe all topic you need***
      mqttClient.subscribe("buzzer/trigger");
      mqttClient.subscribe("home/data");
      mqttClient.subscribe("light/set");
      mqttClient.subscribe("lid/open");
      mqttClient.subscribe("lid/close");
      mqttClient.subscribe("schedule/set");
      mqttClient.subscribe("feed");
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
  for (int i = 0; i < length; i++) {
    strMsg += (char)message[i];
  }
  Serial.println(strMsg);
  String strTopic = String(topic);
  if (strTopic == "buzzer/trigger") {
    playMelody();
  }
  else if (strTopic == "light/set")  {
    setBrightness(strMsg.toInt());
  }
  else if (strTopic == "lid/open") {
    openLid();
  }
  else if (strTopic == "lid/close") {
    closeLid();
  }
  else if (strTopic == "feed") {
    isFed = true;
  }
  else if (strTopic == "schedule/set") {
    eatingSchedule[schedIndex] = strMsg.toInt();
    realEatingAmount[schedIndex] = strMsg.toInt();
    Serial.print(schedIndex);
    Serial.print(": ");
    Serial.println(eatingSchedule[schedIndex]);
    schedIndex++;
    if(schedIndex == 4) schedIndex = 0;
  }
  else if (strTopic == "home/data") {
    sendHomeData();
  }

  //***Code here to process the received package***

}

void sendIFTTTRequest(const char* request, String requestData) {
  WiFiClient client;
  while(!client.connect(iftttHost, 80)) {
    Serial.println("connection fail");
    delay(1000);
  }
  client.print(String("GET ") + request + requestData + " HTTP/1.1\r\n"
              + "Host: " + iftttHost + "\r\n"
              + "Connection: close\r\n\r\n");
  delay(500);

  while(client.available()) {
    String line = client.readStringUntil('\r');
    Serial.print(line);
  }
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
// Load cell for food container
String getFoodAmount() {
  return String(100.0 * getWeight(false) / MAX_FOOD);
}
// Mo va dong nap hop dung do an
void lockUnlockLid(bool lock) {
  if (lock) servo1.write(0);
  else servo1.write(90);
}
// In cac thong so ra man LCD, moi 5s cap nhat mot lan
long start = 0;
void updateLCD() {
  if (!start || millis() - start > 5000) {
    // Serial.println(getWeight(false));
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print(getTemp() + "*C | " + getHumid() + "%");
    lcd.setCursor(0, 1);
    lcd.print("Food: " + getFoodAmount() + "%");
    start = millis();
  }
}
// dieu chinh do sang den
void setBrightness(int value) {
  int brightness_val = map(value, 0, 10, 100, 255);
  Serial.println("Brightness: " + String(brightness_val));
  curBrightness = brightness_val;
}
// dong mo nap dung do an
void openLid() {
  servo1.write(90);
}
void closeLid() {
  servo1.write(0);
}
int checkEatingSchedule(){
  long long cur = millis();
  long long tmp = cur - lastTime;
  // Cử ăn 1
  if(tmp < 15000){
    return 0;
  } 
  if(tmp < 25000){
    if(sendFoodAmountFlag == 1){
      int eatingAmount = eatingSchedule[0] - realEatingAmount[0];
      String payload = String(eatingAmount);
      mqttClient.publish("home/foodConsumed", payload.c_str());

      Serial.print(sendFoodAmountFlag);
      Serial.print("/ consume: ");
      Serial.println(eatingAmount);

      sendFoodAmountFlag++;
    }
    return 4; // Không phải cử ăn => [4] luôn bằng 0
  }
  // Cử ăn 2
  if(tmp < 40000){
    return 1;
  } 
  if(tmp < 50000){
    if(sendFoodAmountFlag == 2){
      int eatingAmount = eatingSchedule[1] - realEatingAmount[1];
      String payload = String(eatingAmount);
      mqttClient.publish("home/foodConsumed", payload.c_str());

      Serial.print(sendFoodAmountFlag);
      Serial.print("/ consume: ");
      Serial.println(eatingAmount);

      sendFoodAmountFlag++;
    }
    return 4; // Không phải cử ăn => [4] luôn bằng 0
  }
  // Cử ăn 3
  if(tmp < 65000){
    return 2;
  } 
  if(tmp < 75000){
    if(sendFoodAmountFlag == 3){
      int eatingAmount = eatingSchedule[2] - realEatingAmount[2];
      String payload = String(eatingAmount);
      mqttClient.publish("home/foodConsumed", payload.c_str());

      Serial.print(sendFoodAmountFlag);
      Serial.print("/ consume: ");
      Serial.println(eatingAmount);

      sendFoodAmountFlag++;
    }
    return 4; // Không phải cử ăn => [4] luôn bằng 0
  }
  // Cử  ăn 4
  if(tmp < 90000){
    return 3;
  } 
  if(tmp < 100000){
    if(sendFoodAmountFlag == 4){
      int eatingAmount = eatingSchedule[3] - realEatingAmount[3];
      String payload = String(eatingAmount);
      mqttClient.publish("home/foodConsumed", payload.c_str());

      Serial.print(sendFoodAmountFlag);
      Serial.print("/ consume: ");
      Serial.println(eatingAmount);

      sendFoodAmountFlag++;
    }
    return 4; // Không phải cử ăn => [4] luôn bằng 0
  }
  // Reset ngày
  lastTime = millis();
  sendFoodAmountFlag = 1;
  Serial.print("======== The Schedule start at ");
  Serial.print(lastTime / 1000);
  Serial.println("s ========");

  requestSchedule();
  return 4;
}

void requestSchedule(){
  String payload = "a";
  mqttClient.publish("schedule/request", payload.c_str());
}

void sendHomeData() {
  String payload = String(int(getWeight(false) * 100 / 20000));
  mqttClient.publish("home/foodProportion", payload.c_str());

  payload = getTemp();
  mqttClient.publish("home/temperature", payload.c_str());

  payload = getHumid();
  mqttClient.publish("home/humidity", payload.c_str());
}

// void alertChecking() {
//   float temp = atof(getTemp().c_str());
//   float humid = atof(getHumid().c_str());
//   float food = float(getWeight(false) * 100 / 20000);

//   /*
//   - Nhiệt độ: 20-38
//   - Độ ẩm: 50-60%
//   - Thức ăn: >=10%
//   */
//   if (food < 10 && (rtc.now() - lastFoodNoti).totalseconds() >= 3600) {
//     const char* request = "/trigger/food_running_out/with/key/lNSH-MlkpFLv_4WLvW5O2Dve1P3aSKc7yvg8H9YJHgW?value1=";
//     sendIFTTTRequest(request, String(food));
//     lastFoodNoti = rtc.now();
//     String payload = "The device is running low on supplies (" + String(food) + "% left). Can you please restock it at your earliest convenience?";
//     mqttClient.publish("alert/food", payload.c_str());
//   }
//   if ((temp < 20 || temp > 38 || humid < 50 || humid > 60) && (rtc.now() - lastEnvNoti).totalseconds() >= 3600) {
//     const char* request = "/trigger/environment/with/key/lNSH-MlkpFLv_4WLvW5O2Dve1P3aSKc7yvg8H9YJHgW?value1=";
//     sendIFTTTRequest(request, String(temp) + "&value2=" + String(humid));
//     lastEnvNoti = rtc.now();
//     String payload = "It seems like the temperature or humidity levels are a bit extreme (" + String(temp) + "°C, " + String(humid) + "%). You might want to check on things and make adjustments if needed.";
//     mqttClient.publish("alert/env", payload.c_str());
//   }
// }

void setup() {
  Serial.begin(115200);
  // LED
  pinMode(LED_PIN, OUTPUT);

  // BUZZER
  pinMode(BUZZER_PIN, OUTPUT);

  // DHT
  dhtSensor.setup(DHT22_PIN, DHTesp::DHT22);

  // LCD
  lcd.init();
  lcd.backlight();
  lcd.setCursor(0, 0);

  // Wifi and MQTT
  wifiConnect();
  mqttClient.setServer(mqttServer, port);
  mqttClient.setCallback(callback);
  mqttClient.setKeepAlive( 90 );

  lcd.clear();
  lcd.print("Hello");
  // Ultrasonic
  pinMode(ULTRASONIC_TRIG_PIN, OUTPUT);
  pinMode(ULTRASONIC_ECHO_PIN, INPUT);

  // Servo
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
  servo1.write(0);
  servo2.write(0);

  // Photoresister
  pinMode(LDR_PIN, INPUT);

  // Load cell
  scale5.begin(CELL_5_DT_PIN, CELL_5_SCK_PIN);
  scale50.begin(CELL_50_DT_PIN, CELL_50_SCK_PIN);
  // Link: https://wokwi.com/projects/344192176616374868
  
  // Real-Time Clock
  // if (! rtc.begin()) {
  //   Serial.println("Couldn't find RTC");
  //   Serial.flush();
  //   abort();
  // }
  // now = rtc.now();
  
  // Bắt đầu thời gian cho ăn
  lastTime = millis();
  Serial.print("======== The Schedule start at ");
  Serial.print(lastTime / 1000);
  Serial.println("s ========");
  // requestNewDay();
}
void loop() {
  // always check if client is disconnected, reconnect
  if (!mqttClient.connected())
    mqttConnect();

  mqttClient.loop(); // giúp giữ kết nối với server và để hàm callback được gọi

  // alertChecking();

  updateLCD();

  int currentEatingTime = checkEatingSchedule();
  // Serial.print("========= ");
  // Serial.print(currentEatingTime);
  // Serial.print(": ");
  // Serial.println(realEatingAmount[currentEatingTime]);

  int distance = getDistance(); // cm
  // If pet is closer the machine or it is clicked in the website
  if (distance <= 5 || isFed) {
    // Setup Light and Buzzer
    if (getBrightness() > 2531) { // 2531 is stairway lighting
      analogWrite(LED_PIN, curBrightness);
    }
    else {
      analogWrite(LED_PIN, 0);
    }
    playMelody();
    
    // Check conditions to open the servo
    if (   getWeight(true) == 0
        && getWeight(false) > 0
        && realEatingAmount[currentEatingTime] > 0) {
      servo2.write(90); // open servo to generate food
      int preWeight5 = getWeight(true);
      int postWeight5;
      // int preWeight50;
      // int postWeight50;
      do {
        postWeight5 = getWeight(true);
        realEatingAmount[currentEatingTime] -= (postWeight5 - preWeight5);
        Serial.println(realEatingAmount[currentEatingTime]);
        preWeight5 = postWeight5;
      } while (realEatingAmount[currentEatingTime] > 0
            && getWeight(true) < 100
            && getWeight(false) > 0);

      postWeight5 = getWeight(true);
      realEatingAmount[currentEatingTime] -= (postWeight5 - preWeight5);

      Serial.print("========= ");
      Serial.print(currentEatingTime);
      Serial.print(": ");
      Serial.println(realEatingAmount[currentEatingTime]);
        
      servo2.write(0);
    }
    isFed = false;
  }
  else {
    analogWrite(LED_PIN, 0);
    servo2.write(0);
  }
}

void playMelody() {
  // Define the melody notes and durations
  // int melody[] = {NOTE_C4, NOTE_D4, NOTE_E4, NOTE_F4, NOTE_G4, NOTE_A4, NOTE_B4};
  int melody[] = {NOTE_C4, NOTE_E4, NOTE_G4, NOTE_B4};

  // Iterate through the melody
  for (int i = 0; i < 4; i++) {
    int noteFrequency = melody[i];

    // Calculate the number of cycles for the PWM signal
    int cycles = 500;

    // Play the note manually with PWM
    for (int j = 0; j < cycles; j++) {
      digitalWrite(BUZZER_PIN, HIGH);
      delayMicroseconds(noteFrequency);
      digitalWrite(BUZZER_PIN, LOW);
      delayMicroseconds(noteFrequency);
    }

    delay(50);  // add a small delay between notes
  }

  digitalWrite(BUZZER_PIN, LOW);  // turn off the buzzer
}
