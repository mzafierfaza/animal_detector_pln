#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Servo.h>

#define BOARD_ID 2  // board master

#define trig_pin 2  // D4 = 2 
#define echo_pin 0  // D3 = 0
#define pir_pin 13 // D7 = 13
#define biru_pin 5 // D1 = 5
#define ijo_pin 4  // D2 = 4
int servoPin =  16; // D0 = 16

uint8_t broadcastAddress[] = {0x3C, 0x61, 0x05, 0xE9, 0x86, 0x28}; // tessa master -- 3C:61:05:E9:86:28
int last_sensor;
int isDetect;
int pir;
bool jela;
typedef struct struct_message {
  int id;
  int stats;
  int isMonyet;
} struct_message;

struct_message myData;

unsigned long lastTime = 0;
unsigned long timerDelay = 1000;
unsigned long millisServo, previousMillis;

int pos;
int distance1;
bool isSent;
Servo myServo;

void OnDataSent(uint8_t *mac_addr, uint8_t sendStatus) {
  Serial.print("\r\nLast Packet Send Status: ");
  if (sendStatus == 0) {
    //    Serial.println("Delivery success");
    isSent = true;
  }
  else {
    //    Serial.println("Delivery fail");
    isSent = false;
  }
}

void setup() {
  Serial.begin(9600);

  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(pir_pin, INPUT);
  pinMode(ijo_pin, OUTPUT);
  pinMode(biru_pin, OUTPUT);

  myServo.attach(servoPin);
  Serial.println("Testt");
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  if (esp_now_init() != 0) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_register_send_cb(OnDataSent);
  esp_now_add_peer(broadcastAddress, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

}

void loop() {
  //  sonic();
  //  printData();
  for (pos = 45; pos <= 135; pos += 1) {
    sonic();
    //    pir = digitalRead(pir_pin);
    myServo.write(pos);
    sendData();
    printData();
    (isSent) ? digitalWrite(biru_pin, HIGH) : digitalWrite(biru_pin, LOW);
    delay(200);
  }
  for (pos = 135; pos >= 45; pos -= 1) {
    sonic();
    //    pir = digitalRead(pir_pin);
    myServo.write(pos);
    sendData();
    printData();
    (isSent) ? digitalWrite(biru_pin, HIGH) : digitalWrite(biru_pin, LOW);
    delay(200);
  }
}

void printData() {
  Serial.print("PIR = "); Serial.print(pir);
  Serial.print("| Sonic = "); Serial.println(distance1);
}

void sendData() {
  myData.id = BOARD_ID;
  myData.stats = random(1, 100);
  if (distance1 < 300 && jela) {
    digitalWrite(ijo_pin, HIGH);
    myData.isMonyet = 1;
  }
  else {
    digitalWrite(ijo_pin, LOW);
    myData.isMonyet = 0;
  }
  esp_now_send(0, (uint8_t *) &myData, sizeof(myData));
}

void sonic() {
  digitalWrite(trig_pin, LOW);
  delayMicroseconds(2);
  digitalWrite(trig_pin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trig_pin, LOW);
  const unsigned long duration1 = pulseIn(echo_pin, HIGH);
  distance1 = duration1 / 29 / 2;
  if (last_sensor <= 300) jela = true;
  else jela = false;
  last_sensor = distance1;
}
//
//void cekBener() {
//
//  if (currentMillis - previousMillis >= 1000) {
//    previousMillis = currentMillis;
//
//    // if the LED is off turn it on and vice-versa:
//    if (ledState == LOW) {
//      ledState = HIGH;
//    }
//  }
//}
