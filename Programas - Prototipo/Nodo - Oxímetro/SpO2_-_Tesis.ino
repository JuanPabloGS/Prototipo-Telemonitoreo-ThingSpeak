#include <ESP8266WiFi.h>
#include <Wire.h>
#include "MAX30105.h"
#include "ThingSpeak.h"
#include "heartRate.h"
#include "spo2_algorithm.h"
#define MAX_BRIGHTNESS 255
MAX30105 particleSensor;
const char * myWriteAPIKey = "A8GSVMBOKO4DATL0";
const char *ssid =  "Tesis2019";
const char *pass =  "Tesis2019";
unsigned long myChannelNumber = 665553;
unsigned long tiempo1 = 0;unsigned long tiempo2 = 0;
WiFiClient client;
uint32_t irBuffer[100]; uint32_t redBuffer[100];int32_t bufferLength;
int32_t spo2;int8_t validSPO2; int32_t heartRate; int8_t validHeartRate; 
const byte RATE_SIZE = 4;
byte rates[RATE_SIZE];
byte rateSpot = 0; long lastBeat = 0;
void setup()
{
  Serial.begin(115200);
  Serial.println("Iniciando...");
  tiempo1 = millis();
  WiFi.begin(ssid, pass);
  ThingSpeak.begin(client);
  while (WiFi.status() != WL_CONNECTED) 
     {  delay(500);
      Serial.print(".");     }
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST))
  {while (1);  }
  byte ledBrightness = 60; 
  byte sampleAverage = 4; 
  byte ledMode = 2; 
  byte sampleRate = 100; 
  int pulseWidth = 411; 
  int adcRange = 4096; 
  particleSensor.setup(); 
  particleSensor.setPulseAmplitudeRed(0x0A); 
  particleSensor.setPulseAmplitudeGreen(0); }
void loop()
{  long irValue = particleSensor.getIR();
  bufferLength = 100;
  for (byte i = 0 ; i < bufferLength ; i++)
  {    while (particleSensor.available() == false) 
    particleSensor.check();
    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); 
    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);}
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2);
  while (1)
  {    for (byte i = 25; i < 100; i++)
    {      redBuffer[i - 25] = redBuffer[i];
           irBuffer[i - 25] = irBuffer[i];    }
    for (byte i = 75; i < 100; i++)
    {      while (particleSensor.available() == false)
     particleSensor.check(); 
      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample();
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);
      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);
      Serial.print(F(", SPO2 Valido = "));
      Serial.println(validSPO2, DEC);           }
    tiempo2 = millis();
    if(tiempo2 > (tiempo1+1000)){
    tiempo1 = millis(); 
    if (validSPO2 == true) {
     Serial.println("Enviando valor a thingSpeak");
     ThingSpeak.setField(1, spo2);
     ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);}
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2);    
    }}}


    
