#include "BluetoothSerial.h"
#include <esp_now.h>
#include <WiFi.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 20, 4);

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled! Please run `make menuconfig` to and enable it
#endif

BluetoothSerial SerialBT;


// Structure example to receive data
// Must match the sender structure
typedef struct struct_message {
  float temp;
  int bpm;
  int spo2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

// callback function that will be executed when data is received
void OnDataRecv(const uint8_t * mac, const uint8_t *incomingData, int len) {
  memcpy(&myData, incomingData, sizeof(myData));
  Serial.print("Bytes received: ");
  Serial.println(len);
  Serial.print("Char: ");
  Serial.println(myData.temp);
  Serial.print("Int: ");
  Serial.println(myData.bpm);
  Serial.print("Float: ");
  Serial.println(myData.spo2);
  Serial.println();


  digitalWrite(19, HIGH); //red led
  String s1 = String(myData.temp) + ';' + String(myData.bpm) + ';' + String(myData.spo2);

  SerialBT.println(s1); //transfer data to app via Bluetooth

  delay(200);
  digitalWrite(2, HIGH); //builtin led
}

void setup() {
  Serial.begin(115200);
  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for recv CB to
  // get recv packer info
  esp_now_register_recv_cb(OnDataRecv);

  lcd.init();                      // initialize the lcd
  lcd.backlight();
  lcd.setCursor(3, 0);
  lcd.print("Hello, world!");
  lcd.setCursor(2, 1);
  lcd.print("Ywrobot Arduino!");
  lcd.clear();

  Serial.println(WiFi.macAddress());

  SerialBT.begin("ESP32test"); //Bluetooth device name
  Serial.println("The device started, now you can pair it with bluetooth!");
  pinMode(18, OUTPUT);
  pinMode(19, OUTPUT);
  pinMode(2, OUTPUT);
  digitalWrite(18, LOW); //green led
  digitalWrite(19, LOW); //red led
  digitalWrite(2, LOW); //builtin led
}

uint32_t pmillis = 0;
void loop() {
  if (millis() - pmillis >= 1000) {
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("Temp: ");
    lcd.print(myData.temp);
    lcd.setCursor(0, 1);
    lcd.print("Pulse Rate: ");
    lcd.print(myData.bpm);
    lcd.setCursor(0, 2);
    lcd.print("SPO2: ");
    lcd.print(myData.spo2);
    pmillis = millis();
  }
}
