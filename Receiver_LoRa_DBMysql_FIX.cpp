#include <Arduino.h>
#include <WiFi.h>
#include <LiquidCrystal_I2C.h>
#include <WiFiClientSecure.h>
#include <Wire.h>
#include <LoRa.h>
#include <ArduinoJson.h>
//#include <SPI.h>
#include <HTTPClient.h>

//define the pins used by the transceiver module
#define LORA_AURORA_V2_NSS 15
#define LORA_AURORA_V2_RST 0
#define LORA_AURORA_V2_DIO0 27
#define LORA_AURORA_V2_EN 32
#define LORA_TX_POWER 20
#define LORA_SPREADING_FACTOR 12

#define WIFI1_SSID "xxxx"
#define WIFI1_PASSWORD "xxxx"
#define WIFI2_SSID "xxxx"
#define WIFI2_PASSWORD "xxxx"
#define WIFI3_SSID "xxxx"
#define WIFI3_PASSWORD "xxxx"
#define WIFI4_SSID "xxxx"
#define WIFI4_PASSWORD "xxxx."
const char* serverNameTurbi = "https://rewanaweb.com/xxxx/xxxx.php";
const char* serverNameFlow = "https://rewanaweb.com/xxxx/xxxx.php";





// set the LCD number of columns and rows
int lcdColumns = 20;
int lcdRows = 4;

String apiKeyValueTurbi = "xxxx";
String apiKeyValueFlow= "xxxx";
String sensorTurbiHulu = "turbidity_hulu";
String sensorTurbiHilir = "turbidity_hilir";
String sensorLocHulu = "Hulu";
String sensorLocHilir = "Hilir";
String sensorFlowHulu = "flowwater_hulu";
String sensorFlowHilir = "flowwater_hilir";


unsigned long sendDataPrevMillis = 0;
bool signupOK = false;

int counter = 0;
int TURBI_PIN = 33;
int FLOW_PIN = 26;
// int RESET_PIN = 32;
int RESET_PIN = 32;


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
int paketKe = 0;

//value global sensor Turbidity
float voltGlobal;
float mappedTurbidity;
float mappedTurbidityGlobal;

float VoltTurbiHulu;
float TurbiValueHulu;
int FlowRateHulu;
int FlowmiliLiterHulu;

int flowRateHilir;
int totalMiliLiterHilir;
float totalDebitLiterHilir;


int testFlowmiliLiterHulu=0;
int testtotalMiliLiterHilir=0;
int getdebitHulu;
int getdebitHilir;


String LoRaData;
int post=0;
WiFiClientSecure *client = new WiFiClientSecure;

// Fungsi yang dipanggil ketika terjadi pulsa dari sensor aliran
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}  

float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
                                                      


void connectToWifi() {
    const char* ssid[] = {WIFI1_SSID, WIFI2_SSID, WIFI3_SSID, WIFI4_SSID};
    const char* passwords[] = {WIFI1_PASSWORD, WIFI2_PASSWORD, WIFI3_PASSWORD, WIFI3_PASSWORD};
    const int numNetworks = sizeof(ssid) / sizeof(ssid[0]);

    WiFi.disconnect();
    for(int i = 0; i < numNetworks; i++) {
        Serial.print("Connecting to WiFi: ");
        Serial.println(ssid[i]);
        WiFi.begin(ssid[i], passwords[i]);

        unsigned long startTime = millis();
        while(WiFi.status() != WL_CONNECTED && millis() - startTime < 10000) {
            delay(500);
            Serial.print(".");
        }

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected!");
            break;
        }
    }
}

void checkWifiConnection() {
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("Menghubungkan ulang ke WiFi...");
        connectToWifi(); // Fungsi yang Anda miliki untuk menghubungkan ke WiFi
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("Gagal terhubung ke WiFi.");
            return;
        }
    }
}


void lorabegin(){
  // Konfigurasi pin sensor flow water sebagai input pull-up
  pinMode(FLOW_PIN, INPUT_PULLUP);
  // Initiate the LoRa Enable pin
  pinMode(LORA_AURORA_V2_EN, OUTPUT);
  // LoRa chip is Active High
  digitalWrite(LORA_AURORA_V2_EN, HIGH);
  
  Serial.println("LoRa Receiver..");
  lcd.setCursor(0, 0);
  lcd.print("LoRa Receiver..");
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
  // Change sync word (0xF3) to match the receiver
  // The sync word assures you don't get LoRa messages from other LoRa transceivers
  // ranges from 0-0xFF
  LoRa.setSyncWord(0xF3);
  Serial.println("LoRa Initializing OK!");
  lcd.setCursor(0,1);
  lcd.print("LoRa Initializing OK");
  lcd.clear();
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
  mappedTurbidity = map(ntu, minTurbiditySensor, maxTurbiditySensor, minTurbidityOutput, maxTurbidityOutput);
  Serial.print("Voltage Turbidity (V)= ");
  Serial.println(volt);
  voltGlobal = volt;
  //LoRa.print("Voltage Turbidity (V)= ");
  //LoRa.print(volt);

  mappedTurbidityLora = mappedTurbidity;
  Serial.print("Turbidity(NTU): ");
  Serial.println(mappedTurbidity);
  lcd.setCursor(0, 1);
  lcd.print( "Kejernihan:");
  lcd.setCursor(11, 1);
  lcd.print(mappedTurbidity); 
  //mappedTurbidity = mappedTurbidityGlobal;
  //delay(5000);
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
    Serial.println(int(flowRate));
    flowRateHilir = int(flowRate);

    lcd.setCursor(0, 2);
    lcd.print( "Flow rate:");
    lcd.setCursor(10, 2);
    lcd.print(int(flowRate)); 
    lcd.setCursor(15, 2);
    lcd.println("L/min");    // Print tab space

    //float totaldebit = totalMilliLitres / 1000;
    // Print the cumulative total of litres flowed since starting
    Serial.print("Output Liquid Quantity: ");
    Serial.print(totalMilliLitres*2);
    totalMiliLiterHilir= totalMilliLitres*2;
    Serial.println("mL / ");
    totalDebitLiterHilir=totalMilliLitres / 1000;
    Serial.println(totalDebitLiterHilir);
    //Serial.println(float(totalMilliLitres / 1000));
    // Serial.print("Lora Data TOtalDebit:");
    // Serial.println(totaldebit);

    Serial.println("L");

    lcd.setCursor(0, 3);
    lcd.print( "Debit:");
    lcd.setCursor(6, 3);
    lcd.print(totalMilliLitres*2); 
    lcd.setCursor(19, 3);    lcd.print("mL");
  }
  //delay(1000);
}

void setup(){
  //pinMode(RESET_PIN, OUTPUT);

  Serial.begin(9600);
  lcd.init();
  lcd.backlight();
  lcd.clear(); 
  //wifibegin();
  connectToWifi();
  lorabegin();
  
  //inisilisasi water flow sensor
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;
  
  // Mengaitkan interrupt dengan fungsi pulseCounter saat terjadi falling edge pada pin sensor
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);
  Serial.println("Start LoRa Aurora");
  loop();
  //delay(5000);
}

void getLastDebitHulu(){
    if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status

    HTTPClient http;
    http.begin("https://rewanaweb.com/IoT-Test/getLastDebitHulu.php"); //Specify the URL
    int httpCode = http.GET(); //Make the request

    if (httpCode > 0) { //Check for the returning code
        String payload = http.getString();
        Serial.println(httpCode);
        
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, payload); // Parse JSON payload
        getdebitHulu = doc[0]["debit"]; // Access the debit value
        Serial.println("--- Last debit Hulu value -----");
        Serial.println(getdebitHulu); // Print the debit value
        Serial.println("--------------------------");

      }

    else {
      Serial.println("Error on HTTP request");
    }
    http.end(); //Free the resources
  }
}

void getLastDebitHilir(){
    if ((WiFi.status() == WL_CONNECTED)) { //Check the current connection status

    HTTPClient http;
    http.begin("https://rewanaweb.com/xxxx/xxxx.php"); //Specify the URL
    int httpCode = http.GET(); //Make the request

    if (httpCode > 0) { //Check for the returning code
        String payload = http.getString();
        Serial.println(httpCode);
        
        DynamicJsonDocument doc(1024);
        deserializeJson(doc, payload); // Parse JSON payload
        getdebitHilir = doc[0]["debit"]; // Access the debit value
        Serial.println("--- Last debit Hilir value -----");
        Serial.println(getdebitHilir); // Print the debit value
        Serial.println("--------------------------");

      }

    else {
      Serial.println("Error on HTTP request");
    }
    http.end(); //Free the resources
  }
}


void postdataFlow(){
  checkWifiConnection(); // Pastikan WiFi terhubung
  // if(WiFi.status() == WL_CONNECTED){
  //   WiFiClientSecure *client = new WiFiClientSecure;
    client->setInsecure(); 
    HTTPClient https;
    https.begin(*client, serverNameFlow);
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");

    //Kondisi kalo debitnya nol
    if (FlowmiliLiterHulu == 0){
      getLastDebitHulu();
      FlowmiliLiterHulu=getdebitHulu+FlowmiliLiterHulu;
    }
    
    if (totalDebitLiterHilir == 0){
      getLastDebitHilir();
      totalDebitLiterHilir=getdebitHilir+totalDebitLiterHilir;
    }
    

     
    // Prepare  HTTP POST data FLOW WATER HILIR
    String httpRequestDataFlowHilir = "api_key=" + apiKeyValueFlow + "&sensor=" + sensorFlowHilir
                          + "&location=" + sensorLocHilir + "&flowrate=" + String(flowRateHilir) +
                          + "&debit=" + String(totalDebitLiterHilir,2);
    Serial.print("httpRequestDataFlowHilir: ");
    Serial.println(httpRequestDataFlowHilir);

        // Prepare HTTP POST data FLOW WATER HULU
    String httpRequestDataFlowHulu = "api_key=" + apiKeyValueFlow + "&sensor=" + sensorFlowHulu
                          + "&location=" + sensorLocHulu + "&flowrate=" + String(FlowRateHulu) +
                          + "&debit=" + String(FlowmiliLiterHulu,2);
    Serial.print("httpRequestDataFlowHulu: ");
    Serial.println(httpRequestDataFlowHulu);
    
    // Prepare  HTTP POST data FLOW WATER HILIR
    // String httpRequestDataFlowHilir = "api_key=" + apiKeyValueFlow + "&sensor=" + sensorFlowHilir
    //                       + "&location=" + sensorLocHilir + "&flowrate=" + String(flowRateHilir) +
    //                       + "&debit=" + String(testtotalMiliLiterHilir);
    // Serial.print("httpRequestDataFlowHilir: ");
    // Serial.println(httpRequestDataFlowHilir);

  

    // Send HTTP POST request
    int httpResponseCodeFlowHulu = https.POST(httpRequestDataFlowHulu);
    int httpResponseCodeFlowHilir = https.POST(httpRequestDataFlowHilir);

    // If you need an HTTP request with a content type: text/plain
    //https.addHeader("Content-Type", "text/plain");
    //int httpResponseCode = https.POST("Hello, World!");
    
    // If you need an HTTP request with a content type: application/json, use the following:
    //https.addHeader("Content-Type", "application/json");
    //int httpResponseCode = https.POST("{\"value1\":\"19\",\"value2\":\"67\",\"value3\":\"78\"}");
    
    if (httpResponseCodeFlowHulu > 0 && httpResponseCodeFlowHilir > 0) {
      Serial.print("HTTP Response code Hulu: ");
      Serial.println(httpResponseCodeFlowHulu);
      Serial.print("HTTP Response code Hulu: ");
      Serial.println(httpResponseCodeFlowHilir);
    }
    else {
      Serial.print("Error code Hulu: ");
      Serial.println(httpResponseCodeFlowHulu);
      Serial.print("Error code Hilir: ");
      Serial.println(httpResponseCodeFlowHilir);
    }
    https.end(); // Tutup koneksi HTTPS
    client->stop(); // Tutup koneksi client
  // }
  // else {
  //   Serial.println("WiFi Disconnected");
  //   //loop();
  //   //digitalWrite(RESET_PIN, LOW);
  // }

  //Send an HTTP POST request every ... seconds
  //delay(5000);  
}

void postdataTurbi(){
  checkWifiConnection(); // Pastikan WiFi terhubung
  // if(WiFi.status() == WL_CONNECTED){

    client->setInsecure(); 
    HTTPClient https;
    https.begin(*client, serverNameTurbi);
    https.addHeader("Content-Type", "application/x-www-form-urlencoded");
    
    // Prepare HTTP POST data FLOW WATER HULU
    String httpRequestDataTurbiHulu = "api_key=" + apiKeyValueTurbi + "&sensor=" + sensorTurbiHulu
                          + "&location=" + sensorLocHulu + "&volt=" + String(VoltTurbiHulu,2) +
                          + "&turbiValue=" + String(TurbiValueHulu,2);
    Serial.print("httpRequestDataTurbiHulu: ");
    Serial.println(httpRequestDataTurbiHulu);
    
    // Prepare  HTTP POST data FLOW WATER HILIR
    String httpRequestDataTurbiHilir = "api_key=" + apiKeyValueTurbi + "&sensor=" + sensorTurbiHilir
                          + "&location=" + sensorLocHilir + "&volt=" + String(voltGlobal,2) +
                          + "&turbiValue=" + String(mappedTurbidity,2);
    Serial.print("httpRequestDataTurbiHilir: ");
    Serial.println(httpRequestDataTurbiHilir);

    // Send HTTP POST request
    int httpResponseCodeTurbiHulu = https.POST(httpRequestDataTurbiHulu);
    int httpResponseCodeTurbiHilir = https.POST(httpRequestDataTurbiHilir);

    // If you need an HTTP request with a content type: text/plain
    //https.addHeader("Content-Type", "text/plain");
    //int httpResponseCode = https.POST("Hello, World!");
    
    // If you need an HTTP request with a content type: application/json, use the following:
    //https.addHeader("Content-Type", "application/json");
    //int httpResponseCode = https.POST("{\"value1\":\"19\",\"value2\":\"67\",\"value3\":\"78\"}");
    
    if (httpResponseCodeTurbiHulu > 0 && httpResponseCodeTurbiHilir > 0) {
      Serial.print("HTTP Response code Hulu: ");
      Serial.println(httpResponseCodeTurbiHulu);
      Serial.print("HTTP Response code Hilir: ");
      Serial.println(httpResponseCodeTurbiHilir);
    }
    else {
      Serial.print("Error code Hulu: ");
      Serial.println(httpResponseCodeTurbiHulu);
      Serial.print("Error code Hilir: ");
      Serial.println(httpResponseCodeTurbiHilir);
    }
    https.end(); // Tutup koneksi HTTPS
    client->stop(); // Tutup koneksi client
  // }
  // else {
  //   Serial.println("WiFi Disconnected");
  //   //loop();
  //   //digitalWrite(RESET_PIN, LOW);
  // }
  //Send an HTTP POST request every 30 seconds
  //delay(5000);  
}

void getHuluData(){
  String data = LoRaData;
  String values[4]; // Array to hold the split values
  int index = 0; // Index for the array
  int from = 0;
  int to = data.indexOf('\n');

  while (to != -1) {
    values[index++] = data.substring(from, to);
    from = to + 1;
    to = data.indexOf('\n', from);
  }
  values[index] = data.substring(from); // Get the last value (after the last newline)
  VoltTurbiHulu = values[0].toFloat();
  TurbiValueHulu = values[1].toFloat();
  FlowRateHulu = values[2].toInt();
  FlowmiliLiterHulu = values[3].toFloat();

  Serial.println("+++++ Hulu Data +++++");
  Serial.println(VoltTurbiHulu);
  Serial.println(TurbiValueHulu);
  Serial.println(FlowRateHulu);
  Serial.println(FlowmiliLiterHulu);
  Serial.println("+++++++++++++++++++++++");
}

void POST(){
  lcd.setCursor(0,0);
  lcd.print("Post:");
  lcd.setCursor(5,0);
  lcd.print(post);
  postdataTurbi();
  postdataFlow();
  post++;

}

void loop(){
  //digitalWrite(RESET_PIN, HIGH);
  Serial.println("=================================");
  turbidity();
  flow();
  // getLastDebitHulu();
  // getLastDebitHilir();

  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize) {
    // received a packet
    Serial.println("_");
    Serial.println("Received packet = '");
    Serial.print("Paket ke=");
    Serial.println(paketKe);


    // read packet
    while (LoRa.available()) {
      LoRaData = LoRa.readString();
      // Serial.println("+++++");
      // Serial.println(LoRaData);
      // Serial.println("+++++");
      getHuluData();
      paketKe++;
    }

    //print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    Serial.println("_");
  }
  //flow();
  delay(1000);  
  POST();
  //digitalWrite(RESET_PIN, LOW);
  //setup();
} 
