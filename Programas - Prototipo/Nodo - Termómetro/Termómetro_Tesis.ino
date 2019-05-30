#include <ESP8266WiFi.h>
#include <Wire.h>
#include <Adafruit_MLX90614.h>
#include "ThingSpeak.h"
const char * myWriteAPIKey = "86Q4S9AHC6XVLCRL";
const char *ssid =  "Tesis2019";
const char *pass =  "Tesis2019";
unsigned long myChannelNumber = 728674;
WiFiClient client;
Adafruit_MLX90614 mlx = Adafruit_MLX90614();
int Temp_Corporal;
int Temp_Ambiente;
int Promedio;
void setup() 
{
       Serial.begin(115200);
       delay(10);
       Serial.println("Connecting to ");
       Serial.println(ssid);
       WiFi.begin(ssid, pass);
       ThingSpeak.begin(client);
      while (WiFi.status() != WL_CONNECTED) 
     {
            delay(500);
            Serial.print(".");
     }
      Serial.println("");
      Serial.println("WiFi connected");
     Wire.begin(D1,D2);
} 
void loop() 
{   
        for (int i=1; i<=10;i++){
        Temp_Corporal = mlx.readObjectTempC();
        Temp_Ambiente = mlx.readAmbientTempC();
        Promedio = Promedio + Temp_Corporal;
        if (Temp_Corporal <= 43 && Temp_Ambiente <= 180){
        ThingSpeak.setField(1, Temp_Corporal);
        ThingSpeak.setField(2, Temp_Ambiente);
        ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
        }
        Serial.print("Temperatura Ambiente = "); Serial.print(Temp_Ambiente);
        Serial.print("*C\tTemperatura Corporal = "); Serial.print(Temp_Corporal); Serial.println("*C");
        delay(1000);
  }
  Promedio = Promedio/10;
  Serial.println("Promedio = "); Serial.print(Promedio);
  ThingSpeak.setField(3, Promedio);
  Promedio = 0;
}
       
//Totalmente funcional con mi thingspeak .. esta op ! y links estan en mlx en el chrome de como se hizo y el codigo.
//es un video, quizas deberia descargarlo. y tiene unos links de github que igual debo bajar

//agregue para que tenga los dos datos enviados a thingspeak. conexiones como en el cuaderno y todo perfecto
//resistencias de 10 . ops

//se jodio asi que se tuvo que cambiar el codigo con la pagina.
//https://openlanuza.com/publicar-datos-de-sensores-en-internet-thingspeak/
