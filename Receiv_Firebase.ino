/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://github.com/cosmic-id/
*********/
#include <WiFi.h>
#include<Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <SPI.h>
#include <LoRa.h>

//define the pins used by the transceiver module
#define LORA_AURORA_V2_NSS 15
#define LORA_AURORA_V2_RST 0
#define LORA_AURORA_V2_DIO0 27
#define LORA_AURORA_V2_EN 32

#define LORA_TX_POWER 20
#define LORA_SPREADING_FACTOR 12

#define WIFI_SSID "EZI"
#define WIFI_PASSWORD "12345678"
#define API_KEY "AIzaSyB9Jyg-aZfMYEjJD4nq_v3gcMRv0Lh7EdM"
#define DATABASE_URL "https://aurora-638d6-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

void setup() {
  // Initiate the LoRa Enable pin
  pinMode(LORA_AURORA_V2_EN, OUTPUT);
  // LoRa chip is Active High
  digitalWrite(LORA_AURORA_V2_EN, HIGH);
  
  //initialize Serial Monitor
  Serial.begin(115200);

  //wifi - firebase connecting
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menyambungkan ke wifi..");
  while(WiFi.status()!= WL_CONNECTED){
    Serial.println("."); delay(300);
  }
  Serial.println();
  Serial.print("Connected dengan IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if(Firebase.signUp(&config, &auth, "","")){
    Serial.println("SignUp AMAN");
    signupOK = true;
  } else {
    Serial.printf("%s\n",config.signer.signupError.message.c_str());
  }

  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  //end firebase connecting

  // start lora recieve
  while (!Serial);
  Serial.println("LoRa Receiver");

  //setup LoRa transceiver module
  LoRa.setPins(LORA_AURORA_V2_NSS, LORA_AURORA_V2_RST, LORA_AURORA_V2_DIO0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  while (!LoRa.begin(920E6)) {
    Serial.println(".");
    delay(500);
  }

  LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
  
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
}

void loop() {
  if(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      Serial.print(LoRaData);
      if(Firebase.RTDB.setString(&fbdo,"Sensor/Suhu",LoRaData)){
      Serial.println(); Serial.print(LoRaData);
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")");
      }else{
      Serial.println("Gagal: "+ fbdo.errorReason());
      }
      
    }

    // // print RSSI of packet
    // Serial.print("' with RSSI ");
    // Serial.println(LoRa.packetRssi());

  }
  }
}
