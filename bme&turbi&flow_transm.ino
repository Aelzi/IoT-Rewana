
#include <Wire.h>
#include <SPI.h>
#include <LoRa.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>

#define LORA_AURORA_V2_NSS 15
#define LORA_AURORA_V2_RST 0
#define LORA_AURORA_V2_DIO0 27
#define LORA_AURORA_V2_EN 32
#define LORA_TX_POWER 20
#define LORA_SPREADING_FACTOR 12
#define SEALEVELPRESSURE_HPA (1013.25)

Adafruit_BME280 bme;
int counter = 0;
int TURBI_PIN = 25;
int FLOW_PIN = 26;

unsigned long delayTime = 1000;

// declare variabel untuk atur waktu
  long currentMillis = 0;
  long previousMillis = 0;
  int interval = 1000; //interval baca sensor dalam milidetik

// Faktor kalibrasi untuk mengkonversi pulsa sensor menjadi laju aliran  
  float calibrationFactor = 4.5;

// variabel menghitung pulsa
  volatile byte pulseCount;
  byte pulse1Sec = 0;

// Variabel untuk menyimpan laju aliran dan total cairan
  float flowRate;
  unsigned int flowMilliLitres;
  unsigned long totalMilliLitres;

// Fungsi yang dipanggil ketika terjadi pulsa dari sensor aliran
void IRAM_ATTR pulseCounter()
{
  pulseCount++;
}  

void setup() {
  Serial.begin(115200);
  bme.begin(0x76);

  // Konfigurasi pin sensor flow water sebagai input pull-up
  pinMode(FLOW_PIN, INPUT_PULLUP);
  // Initiate the LoRa Enable pin
  pinMode(LORA_AURORA_V2_EN, OUTPUT);
  // LoRa chip is Active High
  digitalWrite(LORA_AURORA_V2_EN, HIGH);

  while (!Serial);
  Serial.println("LoRa Sender");

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

  //inisilisasi water flow sensor
  pulseCount = 0;
  flowRate = 0.0;
  flowMilliLitres = 0;
  totalMilliLitres = 0;
  previousMillis = 0;

  // Mengaitkan interrupt dengan fungsi pulseCounter saat terjadi falling edge pada pin sensor
  attachInterrupt(digitalPinToInterrupt(FLOW_PIN), pulseCounter, FALLING);


  Serial.println("-- Default Test --");
  delay(delayTime);
  Serial.println();
}


void loop() {
  Serial.print("Sending packet: ");
  Serial.println(counter);

  //Send LoRa packet to receiver
  LoRa.beginPacket();
  //Send value sensor output to receiver
  printValues();
  turbidity();
  flow();
  Serial.println("_________________________________");
  LoRa.print(counter);
  //End packet to receiver
  LoRa.endPacket();

  counter++;

  delay(delayTime);
}

void printValues() {
  Serial.print("Temperature = ");
  Serial.print(bme.readTemperature());
  LoRa.println(bme.readTemperature()); //output lora receiver
  Serial.println(" *C");
  
  // Convert temperature to Fahrenheit
  /*Serial.print("Temperature = ");
  Serial.print(1.8 * bme.readTemperature() + 32);
  Serial.println(" *F");*/
  
  Serial.print("Pressure = ");
  Serial.print(bme.readPressure() / 100.0F);
  LoRa.println(bme.readPressure() / 100.0F); //output lora receiver
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bme.readAltitude(SEALEVELPRESSURE_HPA));
  LoRa.println(bme.readAltitude(SEALEVELPRESSURE_HPA)); //output lora receiver
  Serial.println(" m");

  Serial.print("Humidity = ");
  Serial.print(bme.readHumidity());
  LoRa.println(bme.readHumidity()); //output lora receiver
  Serial.println(" %");

  Serial.println();
}

void turbidity(){
  float volt;
  float ntu;
  // Nilai minimum dan maksimum turbidity pada sensor
  float minTurbiditySensor = 680.0;
  float maxTurbiditySensor = 3000.0;

  // Nilai minimum dan maksimum yang diinginkan
  float minTurbidityOutput = 1.0;
  float maxTurbidityOutput = 10.0;

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
  Serial.print("Voltage Turbidity = ");
  Serial.print(volt);
  Serial.println(" V");
  Serial.print("Turbidity Value= ");
  Serial.print(mappedTurbidity);
  LoRa.println(mappedTurbidity); //output lora receiver
  Serial.println(" NTU");
  Serial.println();
  //delay(10);
}

void flow(){


 // Menghitung aliran air
  currentMillis = millis();
    
  pulse1Sec = pulseCount;
  pulseCount = 0;

  // Menghitung flow rate dan total milliliter yang telah mengalir
  flowRate = ((1000.0 / (millis() - previousMillis)) * pulse1Sec) / calibrationFactor;
  previousMillis = millis();
  flowMilliLitres = (flowRate / 60) * 1000;
  totalMilliLitres += flowMilliLitres;

  // Menampilkan informasi flow rate dan total milliliter pada Serial Monitor
  Serial.print("Flow rate: ");
  Serial.print(int(flowRate));
  LoRa.println(int(flowRate)); //output lora receiver

  Serial.println("L/min");
  Serial.print("Output Liquid Quantity: ");
  Serial.print(totalMilliLitres);
  LoRa.println(totalMilliLitres); //output lora receiver

  Serial.print("mL / ");
  Serial.print(totalMilliLitres / 1000);
  LoRa.println(totalMilliLitres / 1000); //output lora receiver
  Serial.println("L");

}


float round_to_dp( float in_value, int decimal_place )
{
  float multiplier = powf( 10.0f, decimal_place );
  in_value = roundf( in_value * multiplier ) / multiplier;
  return in_value;
}
