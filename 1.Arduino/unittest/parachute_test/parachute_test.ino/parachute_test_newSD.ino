/***************************************************************************
  This is a library for the BMP280 humidity, temperature & pressure sensor

  Designed specifically to work with the Adafruit BMEP280 Breakout 
  ----> http://www.adafruit.com/products/2651

  These sensors use I2C or SPI to communicate, 2 or 4 pins are required 
  to interface.

  Adafruit invests time and resources providing this open source code,
  please support Adafruit andopen-source hardware by purchasing products
  from Adafruit!

  Written by Limor Fried & Kevin Townsend for Adafruit Industries.  
  BSD license, all text above must be included in any redistribution
 ***************************************************************************/
#include <stdio.h>
#include "SdFat.h"
#include <Wire.h>
#include <SPI.h>
#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

#define periodt 100
#define periodT 200
#define ini_count 20
#define main 25
#define drogue 23

#define drogue_time 20000
#define main_time 40000

// SoftwareSerial Serial2(17, 16); // RX, TX

FaBo9Axis fabo_9axis;
Adafruit_BMP280 bmp; // I2C
File myFile;

float ini_altitude, calc = 0;
uint32_t current_time, pre_time=0;
uint32_t Pre_time=0;
uint32_t full_count = 5;
uint32_t count, on = 0;
uint32_t num = 0;
float max_altitude, pres_altitude = 0;
float A = 0;
String data;
uint32_t init_time;


void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);
  Serial.println("configuring device.");
  Serial2.println("configuring device.");
  
  Wire.begin();
  pinMode(main, OUTPUT);
  digitalWrite(main, LOW);
  pinMode(drogue, OUTPUT);
  digitalWrite(drogue, LOW);
  
  if (fabo_9axis.begin()) {
    Serial2.println("configured FaBo 9Axis I2C Brick");
    Serial.println("configured FaBo 9Axis I2C Brick");
    fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);
  } else {
    Serial2.println("device error");
    Serial.println("device error");
    while(1);
  }

  if (!bmp.begin()) {  
    Serial2.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  } else {
    Serial2.println("initializing bmp 280");
    Serial.println("initializing bmp 280");
    for(int i=0; i<ini_count; i++){
      calc += bmp.readAltitude();
      delay(100);
    }
    calc = calc/ini_count;
    ini_altitude = calc;
    Serial2.print("initial altitude = ");
        Serial.print("initial altitude = ");

    Serial2.println(ini_altitude);
    Serial.println(ini_altitude);
    max_altitude = 0;
  }
  if(!SD.begin(53)){
    Serial2.println("cannot find SD card");
        Serial.println("cannot find SD card");

  }else{
    myFile = SD.open("Ayoyo.txt", FILE_WRITE);
    myFile.print("initial altitude = ");
    //myFile.println(ini_altitude);
    myFile.close();
  }
  init_time = millis();
}


void loop() {

  current_time = millis();
  if(current_time - pre_time >= periodt){
    pre_time = current_time;
    float ax,ay,az;
    float gx,gy,gz;
    String data_string;

    fabo_9axis.readAccelXYZ(&ax,&ay,&az);
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    A = sqrt(ax*ax+ay*ay+az*az);
    pres_altitude = bmp.readAltitude();
      
   
      if((current_time - init_time)<drogue_time){
        max_altitude = pres_altitude;
        count = 0;
      }
      else if((current_time - init_time)>= drogue_time && count == 0){
        digitalWrite(drogue, HIGH);
        count = 1;
      }
      else if((current_time - init_time) >= main_time  && count == 1){
        digitalWrite(main, HIGH);
        count = 2;
      }
      else if((current_time - init_time) >= 50000){
        digitalWrite(drogue,LOW);
        digitalWrite(main,LOW);
        count = 3;
      }

    
    
    data_string = String(current_time) + " " + String(count) + "\n";
    data = data + data_string;
    Serial2.println(data_string);
    Serial.println(data_string);
  }
  
    if (current_time-Pre_time >=periodT){
      Pre_time = current_time;
    myFile = SD.open("Ayoyo.txt", O_CREAT | O_WRITE);
    if(myFile){
    myFile.println(data);
    myFile.close();
    }else{
      Serial2.println("no SD card....... ssibal");
      Serial.println("no SD card....... ssibal");
     }
     data = "";
     Serial.println("data log");
    }
  
}
