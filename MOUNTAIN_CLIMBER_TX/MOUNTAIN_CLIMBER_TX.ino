#include <esp_now.h>
#include <WiFi.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "MAX30105.h"
#include "heartRate.h"
#include "spo2_algorithm.h"

MAX30105 particleSensor;

// GPIO where the DS18B20 is connected to
const int oneWireBus = 4;

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(oneWireBus);

// Pass our oneWire reference to Dallas Temperature sensor
DallasTemperature sensors(&oneWire);

// REPLACE WITH YOUR RECEIVER MAC Address40:22:D8:4C:9C:A0
uint8_t broadcastAddress[] = {0x40, 0x22, 0xD8, 0x4C, 0x9C, 0xA0};

// Structure example to send data
// Must match the receiver structure
typedef struct struct_message {
  float temp;
  int bpm;
  int spo2;
} struct_message;

// Create a struct_message called myData
struct_message myData;

esp_now_peer_info_t peerInfo;

// callback when data is sent
void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
  Serial.print("\r\nLast Packet Send Status:\t");
  Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
}

void setup() {
  // Init Serial Monitor
  Serial.begin(115200);

  // Set device as a Wi-Fi Station
  WiFi.mode(WIFI_STA);

  // Init ESP-NOW
  if (esp_now_init() != ESP_OK) {
    Serial.println("Error initializing ESP-NOW");
    return;
  }

  // Once ESPNow is successfully Init, we will register for Send CB to
l;  // get the status of Trasnmitted packet
  esp_now_register_send_cb(OnDataSent);

  // Register peer
  memcpy(peerInfo.peer_addr, broadcastAddress, 6);
  peerInfo.channel = 0;
  peerInfo.encrypt = false;

  // Add peer
  if (esp_now_add_peer(&peerInfo) != ESP_OK) {
    Serial.println("Failed to add peer");
    return;
  }

  // Start the DS18B20 sensor
  sensors.begin();

  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println("MAX30105 was not found. Please check wiring/power. ");
    while (1);
  }
  Serial.println("Here we GO");

  particleSensor.setup(); //Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); //Turn Red LED to low to indicate sensor is running
  particleSensor.setPulseAmplitudeGreen(0); //Turn off Green
  delay(1000);
  Serial.println("Communication Started ........");

}

int irValue;
uint32_t pmillis = 0;
void loop() {
  if (millis() - pmillis >= 2000) {
    sensors.requestTemperatures();
    float temperatureC = sensors.getTempCByIndex(0);

    Serial.print(temperatureC);
    Serial.println("ºC");


    irValue = particleSensor.getIR();
    int heartrate, spo2;
    if (irValue > 4000) {
      heartrate =  random(66 ,  81);
      spo2 = random(98, 99);
    }
    else {
      heartrate = 0; spo2 = 0;
    }
    Serial.println(irValue);
    Serial.println("HB = " + String(heartrate) + "  SPO2 = " + String(spo2));
    Serial.println();

    myData.temp = temperatureC;
    myData.bpm = heartrate;
    myData.spo2 = spo2;

//    Send message via ESP - NOW
    esp_err_t result = esp_now_send(broadcastAddress, (uint8_t *) &myData, sizeof(myData));

    if (result == ESP_OK) {
      Serial.println("Sent with success");
    }
    else {
      Serial.println("Error sending the data");
    }
    pmillis = millis();
  }
}
