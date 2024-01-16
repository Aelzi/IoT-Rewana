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

#define SEALEVELPRESSURE_HPA (1013.25)

//define the pins used by the transceiver module
#define LORA_AURORA_V2_NSS 15
#define LORA_AURORA_V2_RST 0
#define LORA_AURORA_V2_DIO0 27
#define LORA_AURORA_V2_EN 32

#define LORA_TX_POWER 20
#define LORA_SPREADING_FACTOR 12

#define WIFI_SSID "MBC-Lab 2.4G"
#define WIFI_PASSWORD "gogombc123"
#define API_KEY "AIzaSyB9Jyg-aZfMYEjJD4nq_v3gcMRv0Lh7EdM"
#define DATABASE_URL "https://aurora-638d6-default-rtdb.asia-southeast1.firebasedatabase.app/"

FirebaseData fbdo;
FirebaseAuth auth;
FirebaseConfig config;

unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

String Value1= "";
String Value2= "";
String Value3= "";
String Value4= "";

Adafruit_BME280 bme;

void processLoRaData(String LoRaData);

void setup() {
  // Initiate the LoRa Enable pin
  pinMode(LORA_AURORA_V2_EN, OUTPUT);
  // LoRa chip is Active High
  digitalWrite(LORA_AURORA_V2_EN, HIGH);
  
  //initialize Serial Monitor
  Serial.begin(115200);
  bme.begin(0x76);

  //wifi - firebase connecting
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.print("Menyambungkan ke wifi..");
  while(WiFi.status()!= WL_CONNECTED){
    Serial.println("Menyambungkan ke wifi.."); delay(300);
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
    Serial.println("LoRa begin...");
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
int paketKe = 0;

void loop() {
  if(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.println("_________________________________");
    Serial.println("Received packet = '");
    Serial.print("Paket ke=");
    Serial.println(paketKe);

    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      processLoRaData(LoRaData);
      paketKe++;
    }

    //print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.println("_________________________________");
  }
}
}

void processLoRaData(String LoRaData) {
     // Find the positions of newline characters
  // Temukan posisi awal dan akhir untuk setiap nilai
    int startValue1 = 0;
    int endValue1 = LoRaData.indexOf('\n', startValue1);
    int startValue2 = endValue1 + 1;
    int endValue2 = LoRaData.indexOf('\n', startValue2);
    // int startValue3 = endValue2 + 1;
    // int endValue3 = LoRaData.indexOf('\n', startValue3);
    // int startValue4 = endValue3 + 1;
    // int endValue4 = LoRaData.indexOf('\n', startValue4);

    // Ekstrak nilai menggunakan substring
    String value1 = LoRaData.substring(startValue1, endValue1);
    String value2 = LoRaData.substring(startValue2, endValue2);
    // String value3 = LoRaData.substring(startValue3, endValue3);
    // String value4 = LoRaData.substring(startValue4, endValue4);

    // Konversi nilai ke tipe data yang sesuai jika diperlukan
    float float1 = value1.toFloat();
    float float2 = value2.toFloat();
    // float float3 = value3.toFloat();
    // float float4 = value4.toFloat();

    value1 = String(float1, 2);
    value2 = String(float2, 2);
    // value3 = String(float3, 2);
    // value4 = String(float4, 2);

    // Tampilkan nilai ke Serial Monitor atau gunakan sesuai kebutuhan Anda
    // Serial.println("float value1: " + String(float1, 2));
    // Serial.println("float value2: " + String(float2, 2));
    // Serial.println("float value3: " + String(float3, 2));
    // Serial.println("float value4: " + String(float4, 2));

    // // Tampilkan nilai ke Serial Monitor
    // Serial.println("Temperature: " + temperature );
    // Serial.println("Pressure: " + pressure );
    // Serial.println("Altitude: " + altitude );
    // Serial.println("Humidity: " + humidity );
    
    float value3 = bme.readTemperature();
    float value4 = bme.readPressure() / 100.0F;

    if(Firebase.RTDB.setString(&fbdo,"Sensor/Temperature1",value1)){
      Serial.println(); Serial.println("Temperature1: " + value1 );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"Sensor/Pressure1",value2)){
      Serial.println(); Serial.println("Pressure1: " + value2 );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"Sensor/Temperature2", String (value3, 2))){
      Serial.println(); Serial.println("Temperature2: " + String (value3, 2) );
      Serial.println("Real Temp: "+ String(bme.readTemperature(),3));
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"Sensor/Pressure2",String (value4, 2))){
      Serial.println(); Serial.println("Pressure2: " + String (value4, 2) );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

}
