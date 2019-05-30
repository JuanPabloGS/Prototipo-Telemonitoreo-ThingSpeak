#include <ESP8266WiFi.h>
#include <Wire.h>
#include <ThingSpeak.h>               
float BPM;
int Valor_Pulso = 520;
long Pulso_Anterior = 0; 
const char * API_Key = "TAKLOKHDIVKO8UO3";
const char *ssid =  "Tesis2019";   
const char *pass =  "Tesis2019";
unsigned long Canal = 665694;
WiFiClient client;
void setup() {
   Serial.begin(115200);
   WiFi.begin(ssid, pass);
   ThingSpeak.begin(client);
      while (WiFi.status() != WL_CONNECTED) 
     {delay(500);
      Serial.print(".");}
      Serial.println("");
      Serial.println("Conectado al WiFi");
      Wire.begin();}
void loop() {
  int Pulse = analogRead(A0);
  Serial.println(Pulse);
  delay(100);
if (Pulse > Valor_Pulso){
long diferencia = millis() - Pulso_Anterior;
Pulso_Anterior = millis();
BPM = 60 / (diferencia / 1000.0);
}
Serial.println(BPM);
ThingSpeak.setField(1, Pulse);
ThingSpeak.setField(2, BPM);
ThingSpeak.writeFields(Canal, API_Key);
Serial.println("!Datos enviados a ThingSpeak");
}
