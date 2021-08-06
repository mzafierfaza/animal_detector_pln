#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Servo.h>

#include <espnow.h>
#include <BlynkSimpleEsp8266.h>
#define CHANNEL 1
#define trig_pin 2    // D4 = 2
#define echo_pin 0    // D3 = 0 
#define pir_pin 13    // D7 = 13
#define relay_pin1 5  // D5 = 5
#define relay_pin2 12 // D6 = 12
#define biru_pin 5    // D1 = 5
#define ijo_pin 4     // D2 = 4
#define servoPin 16   // D0 = 16

int isDetect;
int pir;
bool jela;
bool monyet1, monyet2, monyet3, monyet4, monyet5 = false;
int pos;
int distance1, last_sensor;
bool isSent;
Servo myServo;
bool espnow_cek;
char auth[] = "4BF4xn6ecZMHxFBSXPT4fq_-ImrP3j4g";
char ssid[] = "prokes";
char pass[] = "jagaselalukesehatan";

typedef struct struct_message {
  int id;
  int stats;
  int isMonyet;
} struct_message;

struct_message myData;

struct_message board1;
struct_message board2;
struct_message board3;
struct_message board4;

// Create an array with all the structures
struct_message boardsStruct[4] = {board1, board2, board3, board4};

unsigned long previousMillis, previousMillis2 = 0;
bool board1connected, board2connected, board3connected, board4connected;
bool isDetect1, isDetect2, isDetect3, isDetect4;
bool check = true;
int lastValue1, lastValue2, lastValue3, lastValue4;
int currentValue1, currentValue2, currentValue3, currentValue4;

BlynkTimer timer;
WidgetLED led1(V1);
WidgetLED led2(V2);
WidgetLED led3(V3);
WidgetLED led4(V4);
WidgetLED led5(V9);

// Callback function that will be executed when data is received
void OnDataRecv(uint8_t * mac_addr, uint8_t *incomingData, uint8_t len) {
  char macStr[18];

  //  Serial.print("Packet received from: ");
  //  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x",
  //           mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
  //  Serial.println(macStr);

  memcpy(&myData, incomingData, sizeof(myData));
  //  Serial.printf("Board ID %u: %u bytes\n", myData.id, len);

  // Update the structures with the new incoming data
  boardsStruct[myData.id - 1].stats = myData.stats;
  boardsStruct[myData.id - 1].isMonyet = myData.isMonyet;

}

// config AP SSID
void configDeviceAP() {
  const char *SSID = "Slave";
  bool result = WiFi.softAP(SSID, "Slave_Password", CHANNEL, 0);
  if (!result) {
    Serial.println("AP Config failed.");
  } else {
    Serial.println("AP Config Success. Broadcasting with AP: " + String(SSID));
  }
}


void setup() {
  Serial.begin(9600);
  pinMode(ijo_pin, OUTPUT);
  pinMode(biru_pin, OUTPUT);
  pinMode(trig_pin, OUTPUT);
  pinMode(echo_pin, INPUT);
  pinMode(pir_pin, INPUT);
  pinMode(relay_pin1, OUTPUT);
  pinMode(relay_pin2, OUTPUT);
  digitalWrite(relay_pin1, HIGH);
  digitalWrite(relay_pin2, HIGH);
  digitalWrite(biru_pin, LOW);
  myServo.attach(servoPin);
  Serial.println("Testt");
  digitalWrite(biru_pin, HIGH);
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();

  configDeviceAP();

  (!esp_now_init()) ? espnow_cek = false : espnow_cek = true;

  esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(OnDataRecv);

  Blynk.begin(auth, ssid, pass);
  timer.setInterval(500L, sendDataToBlynk);

}

void loop() {
  for (pos = 45; pos <= 135; pos += 2) {
    sonic();
    pir = digitalRead(pir_pin);
    myServo.write(pos);
    printData();
    Blynk.run();
    timer.run();
    receiveData();
    idupinRelay();
    delay(100);
  }
  for (pos = 135; pos >= 45; pos -= 2) {
    sonic();
    pir = digitalRead(pir_pin);
    myServo.write(pos);
    printData();
    Blynk.run();
    timer.run();
    receiveData();
    idupinRelay();
    delay(100);
  }
}

void idupinRelay() {
  if (monyet1 || monyet2 || monyet3 || monyet4 || monyet5) {
    digitalWrite(relay_pin1, LOW);
    digitalWrite(relay_pin2, LOW);
  }
  else {
    digitalWrite(relay_pin1, HIGH);
    digitalWrite(relay_pin2, HIGH);
  }
}

void printData() {
  Serial.print("PIR = "); Serial.print(pir);
  Serial.print("| Sonic = "); Serial.println(distance1);
}

void receiveData() {

  checkStatus();
  checkConnected();
  currentValue1 = boardsStruct[0].stats;
  currentValue2 = boardsStruct[1].stats;
  currentValue3 = boardsStruct[2].stats;
  currentValue4 = boardsStruct[3].stats;
}
void sendDataToBlynk() {
  (board1connected) ? led1.on() : led1.off();
  (board2connected) ? led2.on() : led2.off();
  (board3connected) ? led3.on() : led3.off();
  (board4connected) ? led4.on() : led4.off();
  Blynk.virtualWrite(V5, boardsStruct[0].isMonyet); //suhu virtual 5
  Blynk.virtualWrite(V6, boardsStruct[1].isMonyet); //suhu virtual 5
  Blynk.virtualWrite(V7, boardsStruct[2].isMonyet); //suhu virtual 5
  Blynk.virtualWrite(V8, boardsStruct[3].isMonyet); //suhu virtual 5

  if (boardsStruct[0].isMonyet && board1connected) {
    monyet1 = true;
    Blynk.notify("Tiang satu terdeteksi Monyet!");
  } else monyet1 = false;

  if (boardsStruct[1].isMonyet && board2connected) {
    monyet2 = true;
    Blynk.notify("Tiang dua terdeteksi Monyet!");
  } else monyet2 = false;

  if (boardsStruct[2].isMonyet && board3connected) {
    monyet3 = true;
    Blynk.notify("Tiang tiga terdeteksi Monyet!");
  } else monyet3 = false;

  if (boardsStruct[3].isMonyet && board4connected) {
    monyet4 = true;
    Blynk.notify("Tiang empat terdeteksi Monyet!");
  } else monyet4 = false;

  if (distance1 < 300 & jela) {
    digitalWrite(ijo_pin, HIGH);
    led5.on();
    monyet5 = true;
    Blynk.notify("Tiang lima terdeteksi Monyet!");
  }
  else {
    led5.off();
    digitalWrite(ijo_pin, LOW);
    monyet5 = false;
  }
}

void checkStatus() {
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= 500 && check) {
    previousMillis = currentMillis;
    lastValue1 = currentValue1;
    lastValue2 = currentValue2;
    lastValue3 = currentValue3;
    lastValue4 = currentValue4;

    Serial.printf("stats_1 value: %d \n", lastValue1);
    Serial.print("1 CONNECT GAK ? ");
    Serial.println(board1connected);
    Serial.print("1 MONYET GAK ? ");
    Serial.println(boardsStruct[0].isMonyet); Serial.println("----------------------");

    Serial.printf("stats_2 value: %d \n", lastValue2);
    Serial.print("2 CONNECT GAK ? ");
    Serial.println(board2connected);
    Serial.print("2 MONYET GAK ? ");
    Serial.println(boardsStruct[1].isMonyet); Serial.println("----------------------");

    Serial.printf("stats_3 value: %d \n", lastValue3);
    Serial.print("3 CONNECT GAK ? ");
    Serial.println(board3connected);
    Serial.print("3 MONYET GAK ? ");
    Serial.println(boardsStruct[2].isMonyet); Serial.println("----------------------");

    Serial.printf("stats_4 value: %d \n", lastValue4);
    Serial.print("4 CONNECT GAK ? ");
    Serial.println(board4connected);
    Serial.print("4 MONYET GAK ? ");
    Serial.println(boardsStruct[3].isMonyet); Serial.println("----------------------");
    check = false;
  }
}

void checkConnected() {
  unsigned long currentMillis2 = millis();
  if (currentMillis2 - previousMillis2 >= 3000) {
    previousMillis2 = currentMillis2;
    (lastValue1 != currentValue1) ? board1connected = true : board1connected = false;
    (lastValue2 != currentValue2) ? board2connected = true : board2connected = false;
    (lastValue3 != currentValue3) ? board3connected = true : board3connected = false;
    (lastValue4 != currentValue4) ? board4connected = true : board4connected = false;
    check = true;
  }
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
