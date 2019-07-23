// Library
#include <Wire.h>
#include <SPI.h>
#include <math.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Define buzzer
const int buzzer = 4;
int frequency = 2000; //Specified in Hz
int timeOn = 1000;    //specified in milliseconds
int timeOff = 1000;   //specified in millisecods

// Define motor
const int motor = D3; //pin PWM

// Define angle
float max_angle = 0;
float current_angle = 0;
float goal = 80; //obiettivo (angolo da raggiungere). prendere da web
float th = 30;   //soglia (angolo minimo da superare per poter dire di aver iniziato l'esercizio). prendere da web
//float myarray[]; //array per salvare angoli durante il mantenimento

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

// Define time
int time_milliseconds = 10000;
unsigned long inizio = 0;
unsigned long start_time;
unsigned long current_time = millis();
unsigned long period = 30000;
unsigned long wait_time;

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
    mpu.setXGyroOffset(117);
    mpu.setYGyroOffset(46);
    mpu.setZGyroOffset(-71);
    mpu.setZAccelOffset(-4886);
    mpu.setZAccelOffset(-1691);
    mpu.setZAccelOffset(-203);
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

void setup()
{
    Serial.begin(9600);
    Wire.begin(sda, scl);
    setoffsetIMU();
    MPU6050_Init();
    pinMode(buzzer, OUTPUT);
    pinMode(motor, OUTPUT);
}

void loop()
{
    if (exercise){
        //delay(500);
        FunctionsPitchRoll(Ax, Ay, Az); //Calcolo angolo Roll e Pitch
        current_angle = Roll;
        start_time = millis();

        // cicalino fa un bip per iniziare
        startbuzzer();

        //while loop per determinare se l'esercizio è iniziato entro 30s dall'accensione della fascia ----> vero periodo = 5 min
        while (current_time <= start_time + period && flag1 == true)
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
                period = 30000;        // nella realtà dura 30s
                analogWrite(motor, 153); // parte vibrazione  //153 dice che è il valore per avere 60% duty cycle
                current_time = millis();
                wait_time = millis();
                while (current_time <= wait_time + period && flag1 == true)
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
              Serial.print("Salvo sul server che l'esercizio non è stato svolto.")
            }
        }

        if (flag1 == false) //ho raggiunto la soglia e flag1 = false
        { 
            start_time = millis();
            current_angle = Roll;
            period = 30000;           // quanto deve durare l'esercizio? durata di mantenimento da scaricare dal server
            bool first_goal = true;
            while (current_time <= start_time + period)
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
                    startbuzzer(); //to do: da implementare un suono diverso, magari prolungato
                }
            }
            // to do: sound to understand that exercise is finished
            // to do: salvo il valore sul server
            mean = sum / i;
            Serial.print("mean: ");
            Serial.print(mean);
            Serial.print("\nEsercizio finito");
            exercise = false;
        }
    }
}
