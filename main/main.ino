// Library
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Library Server
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266WiFiMulti.h>
#include <ArduinoJson.h>

#include <ESP8266HTTPClient.h>
#include <WiFiClient.h>

ESP8266WiFiMulti WiFiMulti;

ESP8266WebServer server(80); // server listens on port 80

// Fingerprint
const uint8_t fingerprint[20] = {0xE5, 0x6D, 0xEE, 0x05, 0x46, 0xC8, 0xD6, 0xF7, 0xF7, 0x2E, 0x48, 0x60, 0x6F, 0x64, 0xE3, 0xCB, 0x65, 0x61, 0x48, 0x3E};

// from ArduinoJson.h
const size_t CAPACITY = JSON_OBJECT_SIZE(1024);
StaticJsonDocument<CAPACITY> doc;   //controllare se si svuota da sola o se serve un clear

// Wifi Connection
const char* ssid = "Honor 9 Lite";  //  address of access point
const char* password = "ciao1234"; //  password of access point

// https server
HTTPClient https;

// Define buzzer
const int buzzer = D4;
int frequency = 2000; //Specified in Hz
int timeOn = 1000;    //specified in milliseconds
int timeOff = 1000;   //specified in millisecods

// Define motor
const int motor = D8; //pin PWM

// Define angle
float max_angle = 0;
float current_angle = 0;
int goal; //obiettivo (angolo da raggiungere). prendere da web
float th;   //soglia (angolo minimo da superare per poter dire di aver iniziato l'esercizio). prendere da web

// Define IMU
MPU6050 mpu;

// MPU6050 Slave Device Address
const uint8_t MPU = 0x68;

// Select SDA and SCL pins for I2C communication
const uint8_t scl = D6;
const uint8_t sda = D7;

// sensitivity scale factor respective to full scale setting provided in datasheet
const uint16_t AccelScaleFactor = 16384;
const uint16_t GyroScaleFactor = 131;

// MPU6050 few configuration register addresses
const uint8_t MPU6050_REGISTER_SMPLRT_DIV = 0x19;
const uint8_t MPU6050_REGISTER_USER_CTRL = 0x6A;
const uint8_t MPU6050_REGISTER_PWR_MGMT_1 = 0x6B;
const uint8_t MPU6050_REGISTER_PWR_MGMT_2 = 0x6C;
const uint8_t MPU6050_REGISTER_CONFIG = 0x1A;
const uint8_t MPU6050_REGISTER_GYRO_CONFIG = 0x1B;
const uint8_t MPU6050_REGISTER_ACCEL_CONFIG = 0x1C;
const uint8_t MPU6050_REGISTER_FIFO_EN = 0x23;
const uint8_t MPU6050_REGISTER_INT_ENABLE = 0x38;
const uint8_t MPU6050_REGISTER_ACCEL_XOUT_H = 0x3B;
const uint8_t MPU6050_REGISTER_SIGNAL_PATH_RESET = 0x68;

// Define data from IMU
int16_t AccelX, AccelY, AccelZ, Temperature, GyroX, GyroY, GyroZ;
double Roll, Pitch;
double Ax, Ay, Az, T, Gx, Gy, Gz;

// Define flag
bool flag1 = true; //variabile booleana per interrompere il ciclo while
bool exercise = true; // per far partire l'esercizio

//Define
float sum = 0;
float mean = 0;
int i = 0;
int counter = 0;

// Define time
int time_milliseconds = 10000;
unsigned long inizio = 0;
unsigned long start_time;
unsigned long wait_time;
int repetition;

// Define function
void I2C_Write(uint8_t deviceAddress, uint8_t regAddress, uint8_t data)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.write(data);
    Wire.endTransmission();
}

void setoffsetIMU()
{
    mpu.setXGyroOffset(253);
    mpu.setYGyroOffset(-16);
    mpu.setZGyroOffset(-68);
    mpu.setXAccelOffset(-3787);
    mpu.setYAccelOffset(981);
    mpu.setZAccelOffset(961);
}

//configure MPU6050
void MPU6050_Init()
{
    delay(150);
    I2C_Write(MPU, MPU6050_REGISTER_SMPLRT_DIV, 0x07);
    I2C_Write(MPU, MPU6050_REGISTER_PWR_MGMT_1, 0x01);
    I2C_Write(MPU, MPU6050_REGISTER_PWR_MGMT_2, 0x00);
    I2C_Write(MPU, MPU6050_REGISTER_CONFIG, 0x00);
    I2C_Write(MPU, MPU6050_REGISTER_GYRO_CONFIG, 0x00);  //set +/-250 degree/second full scale
    I2C_Write(MPU, MPU6050_REGISTER_ACCEL_CONFIG, 0x00); // set +/- 2g full scale
    I2C_Write(MPU, MPU6050_REGISTER_FIFO_EN, 0x00);
    I2C_Write(MPU, MPU6050_REGISTER_INT_ENABLE, 0x01);
    I2C_Write(MPU, MPU6050_REGISTER_SIGNAL_PATH_RESET, 0x00);
    I2C_Write(MPU, MPU6050_REGISTER_USER_CTRL, 0x00);
}

// read all 14 register
void Read_RawValue(uint8_t deviceAddress, uint8_t regAddress)
{
    Wire.beginTransmission(deviceAddress);
    Wire.write(regAddress);
    Wire.endTransmission();
    Wire.requestFrom(deviceAddress, (uint8_t)14);
    AccelX = (((int16_t)Wire.read() << 8) | Wire.read());
    AccelY = (((int16_t)Wire.read() << 8) | Wire.read());
    AccelZ = (((int16_t)Wire.read() << 8) | Wire.read());
    GyroX = (((int16_t)Wire.read() << 8) | Wire.read());
    GyroY = (((int16_t)Wire.read() << 8) | Wire.read());
    GyroZ = (((int16_t)Wire.read() << 8) | Wire.read());
}

void startbuzzer()
{
    digitalWrite(buzzer, HIGH);
    delay(200);
    digitalWrite(buzzer, LOW);
    delay(200);
    tone(buzzer, frequency);
    delay(timeOn);
    noTone(buzzer);
    delay(200);
    delay(timeOff);
}

void errorbuzzer()
{
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
    tone(buzzer, 2000);
    delay(20);
    noTone(buzzer);
    delay(5);
}

void endbuzzer()
{
    digitalWrite(buzzer, HIGH);
    delay(100);
    digitalWrite(buzzer, LOW);
    delay(100);
    tone(buzzer, 2000);
    delay(300);
    noTone(buzzer);
    delay(200);
}

void getAccel()
{
    Ax = (double)AccelX / AccelScaleFactor;
    Ay = (double)AccelY / AccelScaleFactor;
    Az = (double)AccelZ / AccelScaleFactor;
}

void getGyro()
{
    Gx = (double)GyroX / GyroScaleFactor;
    Gy = (double)GyroY / GyroScaleFactor;
    Gz = (double)GyroZ / GyroScaleFactor;
}

void FunctionsPitchRoll(double Ax, double Ay, double Az)
{

    Read_RawValue(MPU, MPU6050_REGISTER_ACCEL_XOUT_H);
    getAccel();

    double x = Ax;
    double y = Ay;
    double z = Az;

    Pitch = atan2(x, sqrt((y * y) + (z * z))); //pitch calculation
    Roll = atan2(y, sqrt((x * x) + (z * z)));  //roll calculation

    //converting radians into degrees
    Pitch = Pitch * (180.0 / 3.14);
    Roll = Roll * (180.0 / 3.14);
}

void CatchAngles()
{
    FunctionsPitchRoll(Ax, Ay, Az); //Calcolo angolo Roll e Pitch
    current_angle = Roll;
    Serial.print("Pitch: ");
    Serial.print(Pitch);
    Serial.print("\t");
    Serial.print("Roll: ");
    Serial.print(Roll);
    Serial.print("\n");
    delay(500);
    if (current_angle > max_angle)
    {
        max_angle = current_angle;
    }
}

// Get function
String getValue(String url) {
  if ((WiFiMulti.run() == WL_CONNECTED)) {
    
    std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);
    client->setFingerprint(fingerprint);
    
    Serial.print("[HTTPS] begin...\n");
    if (https.begin(*client, url)) {  // HTTPS

      Serial.print("[HTTPS] GET...\n");   
      
      // start connection and send HTTP header
      int httpCode = https.GET();

      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        //Serial.printf("[HTTPS] GET... code: %d\n", httpCode);
        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {   
          String payload = https.getString();
          //Serial.println(payload);
          deserializeJson(doc, payload);
          JsonObject obj = doc.as<JsonObject>(); 
          String Value = obj[String("attrValue")];  
          //Serial.print(Value);
          return Value;      
       
           }
      } else {
        Serial.printf("[HTTPS] GET... failed, error: %s\n", https.errorToString(httpCode).c_str());
      }

      https.end();
    } else {
      Serial.printf("[HTTPS] Unable to connect\n");
    }
  }
}

// Put function
void putValue(char* url, char* attrName, char* attrValue) {
  if ((WiFiMulti.run() == WL_CONNECTED)) {

    std::unique_ptr<BearSSL::WiFiClientSecure>client(new BearSSL::WiFiClientSecure);

    client->setFingerprint(fingerprint);

    HTTPClient https;
    
    Serial.print("[HTTPS] begin...\n");
    // https://jigsaw.w3.org/HTTP/connection.html
    if (https.begin(*client, url)) {  // HTTPS

      Serial.print("[HTTPS] PUT...\n");
      https.addHeader("Content-Type", "application/json");

        // Creating JSON object
        // create an object
        JsonObject object = doc.to<JsonObject>();
        object[attrName] = attrValue;
      
      // start connection and send HTTP header
      int httpCode = https.PUT(attrValue);
      
      // httpCode will be negative on error
      if (httpCode > 0) {
        // HTTP header has been send and Server response header has been handled
        Serial.printf("[HTTPS] PUT... code: %d\n", httpCode);

        // file found at server
        if (httpCode == HTTP_CODE_OK || httpCode == HTTP_CODE_MOVED_PERMANENTLY) {
          String payload = https.getString();
          Serial.println(payload);
        }
      } else {
        Serial.printf("[HTTPS] PUT... failed, error: %s\n", https.errorToString(httpCode).c_str());
      }

      https.end();
    } else {
      Serial.printf("[HTTPS] Unable to connect\n");
    }
  }
}

void setup()
{
    Serial.begin(9600);
    Wire.begin(sda, scl);
    setoffsetIMU();
    MPU6050_Init();
    pinMode(buzzer, OUTPUT);
    pinMode(motor, OUTPUT);
    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password); //Connect to the WiFi network
    WiFiMulti.addAP(ssid, password);
    while (WiFi.status() != WL_CONNECTED) { //Wait for connection
      delay(500);
      Serial.print(".");
    }
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP()); //Print the local IP
    server.begin(); //Start the server
    Serial.println("Server listening");
    // Acquisisco l'angolo massimo, cioè il goal. 
    String strGoal = getValue("https://demoapp.humana-vox.com/datahub/default/accumulator.json/3635f4e4e9e332f8a560023dd57490c1?deviceId=urn:ngsi-ld:ProactiveRehabilitationBand&attrName=goal&type=last");
    goal = strGoal.toInt() - 90;
    th = goal*0.3; // La soglia indica che si è iniziato l'esercizio. E' il 30% del goal
    String strRepetition = getValue("https://demoapp.humana-vox.com/datahub/default/accumulator.json/3635f4e4e9e332f8a560023dd57490c1?deviceId=urn:ngsi-ld:ProactiveRehabilitationBand&attrName=goal&type=last");
    repetition = strRepetition.toInt();


    putValue("https://demoapp.humana-vox.com/datahub/default/accumulator.json/3635f4e4e9e332f8a560023dd57490c1", "angleMax", "88.8");
}

void loop()
{
    while (counter <= repetition){
      Serial.println("TI PREGO");     
      server.handleClient();
      FunctionsPitchRoll(Ax, Ay, Az); //Calcolo angolo Roll e Pitch
      current_angle = Roll;
      start_time = millis();
      unsigned long current_time = millis();
  
      // Acquisizione delayAllowed: tempo entro il quale si deve iniziare l'esercizio. Il valore è in secondi.
      String strdelayAllowed = getValue("https://demoapp.humana-vox.com/datahub/default/accumulator.json/3635f4e4e9e332f8a560023dd57490c1?deviceId=urn:ngsi-ld:ProactiveRehabilitationBand&attrName=startTime&type=last");
      int delayAllowed = strdelayAllowed.toInt() * 1000;

      // cicalino fa un bip per iniziare
      startbuzzer();

      //while loop per determinare se l'esercizio è iniziato entro 30s dall'accensione della fascia ----> vero periodo = 5 min
      while (current_time <= start_time + delayAllowed && flag1 == true)
      {
            current_time = millis();
            CatchAngles();
            if (max_angle >= th)
            {
                Serial.print("Threashold raggiunto - l'esercizio è iniziato ");
                Serial.print("\n");
                flag1 = false;
                analogWrite(motor, 0); // spegne la vibrazione
                Serial.print("spengo vibrazione");
                Serial.print("\n");
            }
        }

        if (flag1 == true)//non ha raggiunto la soglia. Parte timer per la vibrazione. uscito da while perché non è iniziato esercizio
        {             
            for (int times = 0; times <= 2; times++)
            { //vibra per tre volte con un ritardo di 60s tra una vibrazione e l'altra a meno che non venga raggiunta la soglia. Bisogna controllare i delay!
                Serial.print("non ha ancora iniziato esercizio");
                Serial.print("\n");
                Serial.print("spengo vibrazione");
                Serial.print("\n");
                analogWrite(motor, 0); // spegne la vibrazione
                delay(20000);          // nella realtà di dieci minuti    
                Serial.print("accendo vibrazione");
                Serial.print("\n");
                analogWrite(motor, 522); // parte vibrazione  //153 dice che è il valore per avere 60% duty cycle
                current_time = millis();
                wait_time = millis();
                while (current_time <= wait_time + delayAllowed && flag1 == true)
                {
                    Serial.print("vibra per un minuto");
                    Serial.print("\n");
                    current_time = millis();
                    CatchAngles();
                    if (max_angle >= th)
                    {
                        Serial.print("Threashold raggiunto - l'esercizio è iniziato ");
                        Serial.print("\n");
                        analogWrite(motor, 0); // spegne la vibrazione perchè l'esercizio è iniziato
                        Serial.print("spengo vibrazione");
                        Serial.print("\n");
                        flag1 = false;
                    }
                }
                if (flag1 == false)
                {
                    break;
                }
            }
            if(flag1 == true)
            {
              // attesa data da dato preso dal server, prima del nuovo trial
              // salvare su server che non è stato fatto l'esercizio
              Serial.print("Salvo sul server che l'esercizio non è stato svolto.");
            }
        }

        if (flag1 == false) //ho raggiunto la soglia e flag1 = false
        { 
            start_time = millis();
            current_angle = Roll;
            // Acquisizione del tempo di mantenimento una volta raggiunta la soglia
            String strMaintenance = getValue("https://demoapp.humana-vox.com/datahub/default/accumulator.json/3635f4e4e9e332f8a560023dd57490c1?deviceId=urn:ngsi-ld:ProactiveRehabilitationBand&attrName=tMin&type=last");
            int maintenance = strMaintenance.toInt() * 60000;
            bool first_goal = true;
            while (current_time <= start_time + maintenance)
            {

                delay(200);
                current_time = millis();
                FunctionsPitchRoll(Ax, Ay, Az); //Calcolo angolo Roll e Pitch
                Serial.print("Pitch: ");
                Serial.print(Pitch);
                Serial.print("\t");
                Serial.print("Roll: ");
                Serial.print(Roll);
                Serial.print("\n");
                current_angle = Roll;
                i = i + 1;
                sum = sum + current_angle;
                if (current_angle > goal && first_goal)
                {
                    Serial.print("reached the goal for the first time\n");
                    startbuzzer();
                    first_goal = false;
                }
                if (current_angle > 1.15 * goal)
                {
                    Serial.print("too high roll\n");
                    errorbuzzer(); //to do: da implementare un suono diverso, magari prolungato
                }
            }
            // to do: sound to understand that exercise is finished
            // to do: salvo il valore sul server
            endbuzzer();
            delay(100);
            endbuzzer();
            delay(100);
            endbuzzer();
            mean = sum / i;
            Serial.print("mean: ");
            Serial.print(mean);
            mean = 0;
            flag1 = true;
            Serial.print("\nEsercizio finito");
            exercise = false;
        }
    counter = counter+1;
    max_angle = 0;
    // Acquisizione periodo: ogni quanto far iniziare l’esercizio. Il valore è in minuti
    String strPeriod = getValue("https://demoapp.humana-vox.com/datahub/default/accumulator.json/3635f4e4e9e332f8a560023dd57490c1?deviceId=urn:ngsi-ld:ProactiveRehabilitationBand&attrName=period&type=last");
    int period = strPeriod.toInt() * 60000;
    delay(period);
    }
}
