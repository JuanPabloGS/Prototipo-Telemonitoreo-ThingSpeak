#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <ESP8266WiFi.h>    
#include "ThingSpeak.h"
MPU6050 sensor;
const int Direccion = 0x68;
const int Identificacion = 0x75;
const int Administracion = 0x6B;
const int Giroscopio = 0x1B;  
const int Aceleracion = 0x1C;
const int Aceleracion_Salida = 0x3B;
const int pin_SDA = D5;
const int pin_SCL = D6;
int16_t AceX, AceY, AceZ, Tmp, GirX, GirY, GirZ;
long tiempo_prev; float dt; float Angulo_x, Angulo_y; float Angulo_x_prev, Angulo_y_prev;
const char *ssid =  "RedAb";
const char *pass =  "yyar0308";
const char* server = "api.thingspeak.com";
String apiKey = "P2WTSKAFES3KIS1J";
float eje_x; float eje_y; 
bool led_state = false; 
unsigned long myChannelNumber = 665677;
void initI2C() 
{Wire.begin(pin_SDA, pin_SCL);}
void Grabar_Registro(int registro, int valor)      
{Wire.beginTransmission(Direccion);
Wire.write(registro);Wire.write(valor);
Wire.endTransmission(true); }
uint8_t Leer_Registro(uint8_t registro)       
{uint8_t data;Wire.beginTransmission(Direccion);
Wire.write(registro);Wire.endTransmission(false);
Wire.requestFrom(Direccion, 1);       
  data = Wire.read();return data;}
void Identificar_MPU(int Direccion)
{Wire.beginTransmission(Direccion);
int data = Wire.endTransmission(true);
  if(data == 0){Serial.println(Direccion, HEX);}}
void Revisar_MPU(int Direccion)
{Identificar_MPU(Direccion);int data = Leer_Registro(Identificacion);
  if(data == 104) {data = Leer_Registro(Administracion); } }
void initMPU()
{Iniciar_MPU();Establecer_Acelerometro();Establecer_Giroscopio();}
void Iniciar_MPU()
{Grabar_Registro(Administracion, 0);}
void Establecer_Acelerometro()
{Grabar_Registro(Giroscopio, 0);}
void setAccelScale()
{Grabar_Registro(Acelerometro, 0);}
void setup() {
  Serial.begin(115200);    
  pinMode(LED_BUILTIN, OUTPUT); initI2C();
  WiFi.begin(ssid,pass);
  while (WiFi.status() != WL_CONNECTED){ 
    delay(500); Serial.print("."); }  
    initMPU();  checkMPU(MPU_ADDR);}
void loop() {
  Wire.beginTransmission(Direccion);
  Wire.write(Aceleracion_Salida);
  Wire.endTransmission(false);
  Wire.requestFrom(Direccion, 14);
  AceX = Wire.read() << 8;AceX |= Wire.read();
  AceY = Wire.read() << 8;AceY |= Wire.read(); 
  AceZ = Wire.read() << 8; AceZ |= Wire.read();
  Tmp = Wire.read() << 8;Tmp |= Wire.read();
  GirX = Wire.read() << 8;GirX |= Wire.read();
  GirY = Wire.read() << 8;GirY |= Wire.read();
  GirZ = Wire.read() << 8;GirZ |= Wire.read();
  Serial.print("AceX = "); Serial.print(AceX);
  Serial.print(" | AceY = "); Serial.print(AceY);
  Serial.print(" | AceZ = "); Serial.print(AceZ); 
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);
  Serial.print(" | GyX = "); Serial.print(GirX);
  Serial.print(" | GyY = "); Serial.print(GirY);
  Serial.print(" | GyZ = "); Serial.println(GirZ);
  dt = (millis()-tiempo_prev)/1000.0; tiempo_prev=millis();
float Ace_angX=atan(AceY/sqrt(pow(AceX,2) + pow(AceZ,2)))*(180.0/3.14);
float Ace_angY=atan(-AceX/sqrt(pow(AceY,2) + pow(AceZ,2)))*(180.0/3.14);
  Angulo_x = 0.98*(Angulo_x_prev+(GyX/131)*dt) + 0.02*Ace_angX;
  Angulo_y = 0.98*(Angulo_y_prev+(GyX/131)*dt) + 0.02*Ace_angY;
  Angulo_x_prev=Angulo_x;
  Angulo_y_prev=Angulo_y;
  Serial.print("Ángulo en X:  "); 
  Serial.print(Angulo_x); 
  Serial.print("tÁngulo en Y: "); 
  Serial.println(Angulo_y);  
  eje_x = Angulo_x; 
  eje_y = Angulo_y; 
  enviarData(eje_x,eje_y);} 
  void enviarData(float Angulo_x,float Angulo_y)
{WiFiClient client;
if (client.connect(server,80)){  
  String postStr = apiKey;  
  postStr += "&field1=";  postStr += String(eje_x);  postStr += "\r\n\r\n";  
  postStr += "&field2=";  postStr += String(eje_y);  postStr += "\r\n\r\n";   
  client.print(postStr.length());   
  client.print("\n\n");   client.print(postStr);   delay(1000); 
  } client.stop();}
