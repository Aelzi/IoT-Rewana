/*********
  Modified from the examples of the Arduino LoRa library
  More resources: https://github.com/cosmic-id/
*********/
#include <WiFi.h>
#include<Firebase_ESP_Client.h>
#include "addons/TokenHelper.h"
#include "addons/RTDBHelper.h"
#include <LiquidCrystal_I2C.h>

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

// set the LCD number of columns and rows
int lcdColumns = 20;
int lcdRows = 4;

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
int counter = 0;
int TURBI_PIN = 25;
int FLOW_PIN = 26;

int delayTime = 1000;

long currentMillis = 0;
long previousMillis = 0;
int interval = 1000;
boolean ledState = LOW;
float calibrationFactor = 4.5;
volatile byte pulseCount;
byte pulse1Sec = 0;
float flowRate;
unsigned int flowMilliLitres;
unsigned long totalMilliLitres;

// set LCD address, number of columns and rows
// if you don't know your display address, run an I2C scanner sketch
LiquidCrystal_I2C lcd(0x27, lcdColumns, lcdRows); 

void processLoRaData(String LoRaData);

float mappedTurbidityLora = 0;
int flowRateLora = 0;
float totalMilliLitresLora = 0;


// Fungsi yang dipanggil ketika terjadi pulsa dari sensor aliran
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}  


void setup() {
    
  //initialize Serial Monitor
  Serial.begin(9600);
  bme.begin(0x76);

  // Konfigurasi pin sensor flow water sebagai input pull-up
  pinMode(FLOW_PIN, INPUT_PULLUP);
  // Initiate the LoRa Enable pin
  pinMode(LORA_AURORA_V2_EN, OUTPUT);
  // LoRa chip is Active High
  digitalWrite(LORA_AURORA_V2_EN, HIGH);
  Serial.println("LoRa Receiver..");
  lcd.setCursor(0, 0);
  lcd.println("LoRa Receiver..");
  


  //wifi - firebase connecting
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  lcd.setCursor(0, 1);
  Serial.print("Menyambungkan ke wifi..");
  lcd.print("Connecting to wifi..");

  while(WiFi.status()!= WL_CONNECTED){
    Serial.println("Menyambungkan ke wifi..");
    lcd.setCursor(0, 2);
    lcd.print("Menyambungkan ke wifi..");
     delay(300);
  }
  Serial.println();
  lcd.clear();
  Serial.print("Connected dengan IP: ");
  lcd.setCursor(0, 0);
  lcd.print("Connected/IP: ");
  lcd.setCursor(0, 1);
  Serial.println(WiFi.localIP());
  lcd.print(WiFi.localIP());
  Serial.println();

  config.api_key = API_KEY;
  config.database_url = DATABASE_URL;
  if(Firebase.signUp(&config, &auth, "","")){
    Serial.println("SignUp AMAN");
    lcd.setCursor(0, 2);
    lcd.println("SignUp AMAN");

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
  lcd.setCursor(0,3);
  Serial.println("LoRa Receiver...");
  lcd.print("LoRa Receiver...");


  //setup LoRa transceiver module
  LoRa.setPins(LORA_AURORA_V2_NSS, LORA_AURORA_V2_RST, LORA_AURORA_V2_DIO0);
  
  //replace the LoRa.begin(---E-) argument with your location's frequency 
  //433E6 for Asia
  //866E6 for Europe
  //915E6 for North America
  lcd.clear();
  while (!LoRa.begin(920E6)) {
    lcd.setCursor(0,0);
    Serial.println("LoRa begin...");
    lcd.print("LoRa begin...");
    delay(500);
  }

  LoRa.setTxPower(LORA_TX_POWER, PA_OUTPUT_PA_BOOST_PIN);
  LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);

  //inisilisasi water flow sensor
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  
  // Mengaitkan interrupt dengan fungsi pulseCounter saat terjadi falling edge pada pin sensor
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);
  
   // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  lcd.setCursor(0,1);
  lcd.print("LoRa Initializing OK");

}
int paketKe = 0;

void loop() {
  lcd.clear();
  if(Firebase.ready() && signupOK && (millis() - sendDataPrevMillis > 5000 || sendDataPrevMillis == 0)){
    sendDataPrevMillis = millis();
  
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.println("_");
    Serial.println("Received packet = '");
    lcd.setCursor(0,0);
    Serial.print("Paket ke=");
    Serial.println(paketKe);
    lcd.print("ReceivedP:");
    lcd.setCursor(10,0);
    lcd.print(paketKe);
    
    // read packet
    while (LoRa.available()) {
      String LoRaData = LoRa.readString();
      processLoRaData(LoRaData);
      paketKe++;
    }

    //print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.println("_");
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
    int startValue3 = endValue2 + 1;
    int endValue3 = LoRaData.indexOf('\n', startValue3);
    // int startValue4 = endValue3 + 1;
    // int endValue4 = LoRaData.indexOf('\n', startValue4);

    // Ekstrak nilai menggunakan substring
    String value1 = LoRaData.substring(startValue1, endValue1);
    String value2 = LoRaData.substring(startValue2, endValue2);
    String value3 = LoRaData.substring(startValue3, endValue3);
    // String value4 = LoRaData.substring(startValue4, endValue4);

    // Konversi nilai ke tipe data yang sesuai jika diperlukan
    float float1 = value1.toFloat();
    float float2 = value2.toFloat();
    float float3 = value3.toFloat();
    // float float4 = value4.toFloat();

    value1 = String(float1, 2);
    value2 = String(float2, 2);
    value3 = String(float3, 2);
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
    
    float value4 = mappedTurbidityLora;
    unsigned long value5 = flowRateLora;
    float value6 = totalMilliLitresLora;

    lcd.setCursor(0,1);
    lcd.print("Turbidity(NTU):");
    lcd.setCursor(15, 1);
    lcd.print(value4);

    lcd.setCursor(0,2);
    lcd.print("Flow rate:");
    lcd.setCursor(10, 2);
    lcd.print(value5);
    lcd.setCursor(15, 2);
    lcd.print("L/min");

    lcd.setCursor(0, 3);
    lcd.print("Debit:");
    lcd.setCursor(6, 3);
    lcd.print(value6); 
    lcd.setCursor(19, 3);
    lcd.print("L");


    if(Firebase.RTDB.setString(&fbdo,"SensorTransmitter/Turbidity",value1)){
      Serial.println(); Serial.println("Turbidity1: " + value1 );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"SensorTransmitter/FlowRate",value2)){
      Serial.println(); Serial.println("FlowRate: " + value2 );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"SensorTransmitter/Debit(L):",value3)){
      Serial.println(); Serial.println("Debit(L): " + value3 );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"SensorReceiver/Turbidity", String(value4,1))){
      Serial.println(); Serial.println("Turbidity1: " + String(value4,1) );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"SensorReceiver/FlowRate",String(value5))){
      Serial.println(); Serial.println("FlowRate: " + String(value5) );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

    if(Firebase.RTDB.setString(&fbdo,"SensorReceiver/Debit(L):", String (value6, 2))){
      Serial.println(); Serial.println("Debit(L): " + String (value6, 2) );
      Serial.print("Berhasil save ke :"+ fbdo.dataPath());
      Serial.println("(" + fbdo.dataType() + ")"); }
    else{ Serial.println("Gagal: "+ fbdo.errorReason()); }

}



void turbidity(){
  float volt;
  float ntu;
  // Nilai minimum dan maksimum turbidity pada sensor
  float minTurbiditySensor = 680.0;
  float maxTurbiditySensor = 3000.0;

  // Nilai minimum dan maksimum yang diinginkan
  float minTurbidityOutput = 1.0;
  float maxTurbidityOutput = 100.0;

  volt = 0;
    for(int i=0; i<800; i++)
    {
        //volt += ((float)analogRead(TURBI_PIN)/1023)*5;
        volt += ((float)analogRead(TURBI_PIN)/1023);
    }
    volt = volt/800;
    volt = round_to_dp(volt,2);
    //if(volt < 2.5){
    //  ntu = 3000;
    //}else{
      //ntu = -1120.4 * volt * volt + 5742.3 * volt - 4352.9;
       ntu = -1120.4 * volt * volt + 5742.3 * volt - 4353.8;
    //}
  // int turbiValue = analogRead(TURBI_PIN);
  // float voltage = turbiValue * (5.0 / 1024.0);

  // Serial.print(voltage);
  // Serial.print(" V");
  float mappedTurbidity = map(ntu, minTurbiditySensor, maxTurbiditySensor, minTurbidityOutput, maxTurbidityOutput);
  Serial.print("Voltage Turbidity (V)= ");
  Serial.println(volt);
  //LoRa.print("Voltage Turbidity (V)= ");
  //LoRa.print(volt);
 
  mappedTurbidityLora = mappedTurbidity;
  Serial.print("Turbidity(NTU): ");
  Serial.println(mappedTurbidity);
  lcd.setCursor(0, 1);
  lcd.print( "Turbidity(NTU):");
  lcd.setCursor(15, 1);
  lcd.print(mappedTurbidity); 
  //LoRa.print("Turbidity Value (NTU)= ");//output lora receiver
  LoRa.println(mappedTurbidity); //output lora receiver
  //delay(10);
}


void flow(){

 // Menghitung aliran air
  currentMillis = millis();
  if (currentMillis - previousMillis > interval) {   
    pulse1Sec = pulseCount;
    pulseCount = 0;

    // Menghitung flow rate dan total milliliter yang telah mengalir
    flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
    previousMillis = millis();
    flowMilliLitres = (flowRate / 60) * 1000;
    totalMilliLitres += flowMilliLitres;

  // Print the flow rate for this second in litres / minute
    Serial.print("Flow rate: ");
    Serial.print(int(flowRate));
    flowRateLora = flowRate;
    lcd.setCursor(0, 2);
    lcd.print( "Flow rate:");
    lcd.setCursor(10, 2);
    lcd.print(flowRateLora); 
    lcd.setCursor(15, 2);
    lcd.print("L/min");
    
    Serial.print("L/min");
    Serial.print("\t");       // Print tab space

    float totaldebit = totalMilliLitres / 1000;
    totalMilliLitresLora = totaldebit;
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres);
    Serial.print("mL / ");
    Serial.print(float(totalMilliLitres / 1000));
    Serial.println("L");

    lcd.setCursor(0, 3);
    lcd.print( "Debit:");
    lcd.setCursor(6, 3);
    lcd.print(totalMilliLitresLora); 
    lcd.setCursor(19, 3);
    lcd.print("L");
  }

}


float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
