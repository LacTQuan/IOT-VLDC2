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



void setup() {
  pinMode(2, OUTPUT);
}

void loop() {
  delay(1000);
  digitalWrite(2, HIGH);
  delay(1000); 
  digitalWrite(2, LOW);
}
