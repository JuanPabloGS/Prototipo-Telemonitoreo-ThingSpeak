#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"
#include <ESP8266WiFi.h> // biblioteca para usar as funções de Wifi do módulo ESP8266
#include <Wire.h>         // biblioteca de comunicação I2C
#include <ArduinoJson.h>
#include "ThingSpeak.h"

// La dirección del MPU6050 puede ser 0x68 o 0x69, dependiendo 
// del estado de AD0. Si no se especifica, 0x68 estará implicito
MPU6050 sensor;

// Valores RAW (sin procesar) del acelerometro y giroscopio en los ejes x,y,z

const int MPU_ADDR =      0x68; // definição do endereço do sensor MPU6050 (0x68)
const int WHO_AM_I =      0x75; // registro de identificação do dispositivo
const int PWR_MGMT_1 =    0x6B; // registro de configuração do gerenciamento de energia
const int GYRO_CONFIG =   0x1B; // registro de configuração do giroscópio
const int ACCEL_CONFIG =  0x1C; // registro de configuração do acelerômetro
const int ACCEL_XOUT =    0x3B; // registro de leitura do eixo X do acelerômetro

const int sda_pin = D5; 
const int scl_pin = D6;
bool led_state = false;
int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
long tiempo_prev = 0;
float dt;
float ang_x, ang_y;
float ang_x_prev, ang_y_prev;

const char * myWriteAPIKey = "P2WTSKAFES3KIS1J";
const char *ssid =  "RedAb";
const char *pass =  "yyar0308";

const char* server = "api.thingspeak.com";
String apiKey = "P2WTSKAFES3KIS1J";
float eje_x;
float eje_y;


unsigned long myChannelNumber = 665677;
unsigned long tiempo1 = 0;
unsigned long tiempo2 = 0;

void initI2C() 
{
  //Serial.println("---inside initI2C");
  Wire.begin(sda_pin, scl_pin);
}
void writeRegMPU(int reg, int val)      //aceita um registro e um valor como parâmetro
{
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.write(val);                      // escreve o valor no registro
  Wire.endTransmission(true);           // termina a transmissão
}
uint8_t readRegMPU(uint8_t reg)        // aceita um registro como parâmetro
{
  uint8_t data;
  Wire.beginTransmission(MPU_ADDR);     // inicia comunicação com endereço do MPU6050
  Wire.write(reg);                      // envia o registro com o qual se deseja trabalhar
  Wire.endTransmission(false);          // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 1);        // configura para receber 1 byte do registro escolhido acima
  data = Wire.read();                   // lê o byte e guarda em 'data'
  return data;                          //retorna 'data'
}

void findMPU(int mpu_addr)
{
  Wire.beginTransmission(MPU_ADDR);
  int data = Wire.endTransmission(true);
 
  if(data == 0)
  {
    Serial.print("Dispositivo encontrado no endereço: 0x");
    Serial.println(MPU_ADDR, HEX);
  }
  else
  {
    Serial.println("Dispositivo não encontrado!");
  }
}
 
void checkMPU(int mpu_addr)
{
  findMPU(MPU_ADDR);
     
  int data = readRegMPU(WHO_AM_I); // Register 117 – Who Am I - 0x75
   
  if(data == 104) 
  {
    Serial.println("MPU6050 Dispositivo respondeu OK! (104)");
 
    data = readRegMPU(PWR_MGMT_1); // Register 107 – Power Management 1-0x6B
 
    if(data == 64) Serial.println("MPU6050 em modo SLEEP! (64)");
    else Serial.println("MPU6050 em modo ACTIVE!"); 
  }
  else Serial.println("Verifique dispositivo - MPU6050 NÃO disponível!");
}
void initMPU()
{
  setSleepOff();
  setGyroScale();
  setAccelScale();
}
void setSleepOff()
{
  writeRegMPU(PWR_MGMT_1, 0); // escreve 0 no registro de gerenciamento de energia(0x68), colocando o sensor em o modo ACTIVE
}
void setGyroScale()
{
  writeRegMPU(GYRO_CONFIG, 0);
}

void setAccelScale()
{
  writeRegMPU(ACCEL_CONFIG, 0);
}

void setup() {
  Serial.begin(115200);    //Iniciando puerto serial
  pinMode(LED_BUILTIN, OUTPUT);
  
  tiempo1 = millis();
  /*Wire.begin();           //Iniciando I2C  
  sensor.initialize();    //Iniciando el sensor

  if (sensor.testConnection()) Serial.println("Sensor iniciado correctamente");
  else Serial.println("Error al iniciar el sensor");*/

  initI2C();
WiFi.begin(ssid,pass);
while (WiFi.status() != WL_CONNECTED) 
     {
            delay(500);
            Serial.print(".");
     } 

  initMPU();
  checkMPU(MPU_ADDR);
}

void loop() {
  // Leer las aceleraciones y velocidades angulares
  //sensor.getAcceleration(&ax, &ay, &az);
  //sensor.getRotation(&gx, &gy, &gz);
  Wire.beginTransmission(MPU_ADDR);       // inicia comunicação com endereço do MPU6050
  Wire.write(ACCEL_XOUT);                       // envia o registro com o qual se deseja trabalhar, começando com registro 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);            // termina transmissão mas continua com I2C aberto (envia STOP e START)
  Wire.requestFrom(MPU_ADDR, 14);         // configura para receber 14 bytes começando do registro escolhido acima (0x3B)
 
  AcX = Wire.read() << 8;                 // lê primeiro o byte mais significativo
  AcX |= Wire.read();                     // depois lê o bit menos significativo
  AcY = Wire.read() << 8;
  AcY |= Wire.read();
  AcZ = Wire.read() << 8;
  AcZ |= Wire.read();
 
  Tmp = Wire.read() << 8;
  Tmp |= Wire.read();
 
  GyX = Wire.read() << 8;
  GyX |= Wire.read();
  GyY = Wire.read() << 8;
  GyY |= Wire.read();
  GyZ = Wire.read() << 8;
  GyZ |= Wire.read(); 
 
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
 
  //led_state = !led_state;
  //digitalWrite(LED_BUILTIN, led_state);  

  dt = (millis()-tiempo_prev)/1000.0;
  tiempo_prev=millis();
  
  
float accel_ang_x=atan(AcY/sqrt(pow(AcX,2) + pow(AcZ,2)))*(180.0/3.14);
float accel_ang_y=atan(-AcX/sqrt(pow(AcY,2) + pow(AcZ,2)))*(180.0/3.14);
  
  //Calcular angulo de rotación con giroscopio y filtro complemento  
  ang_x = 0.98*(ang_x_prev+(GyX/131)*dt) + 0.02*accel_ang_x;
  ang_y = 0.98*(ang_y_prev+(GyX/131)*dt) + 0.02*accel_ang_y;
  
  
  ang_x_prev=ang_x;
  ang_y_prev=ang_y;

  //Mostrar los angulos separadas por un [tab]

  Serial.print("Rotacion en X:  ");
  Serial.print(ang_x);
  
  Serial.print("tRotacion en Y: ");
  Serial.println(ang_y);  
  eje_x = ang_x;
  eje_y = ang_y;
  if(millis()-tiempo1 >= 1000)
  {enviarData(eje_x,eje_y);
  tiempo1 = millis();
  Serial.print("enviando");
  delay(100);
  
  }
}
  
  void enviarData(float ang_x,float ang_y)
{
WiFiClient client;
if (client.connect(server,80)){

  String postStr = apiKey;
  postStr += "&field1=";
  postStr += String(eje_x);
  postStr += "\r\n\r\n";
  
  postStr += "&field2=";
  postStr += String(eje_y);
  postStr += "\r\n\r\n";
  

   client.print("POST /update HTTP/1.1\n");
   client.print("Host: api.thingspeak.com\n");
   client.print("Connection: close\n");
   client.print("X-THINGSPEAKAPIKEY: " + apiKey + "\n");
   client.print("Content-Type: application/x-www-form-urlencoded\n");
   client.print("Content-Length: ");
   client.print(postStr.length());
   client.print("\n\n");
   client.print(postStr);
   
    
}
client.stop();
}

//solo debo revisar los angulos, donde mide y donde es recto, quizas restarle al valor en el codigo para que sea 0 - 0 , hechado en vertical.
// ESTE ES EL FINAL _ EL ULTIMO_ EL DEFINITIVO PARA LA TESIS
