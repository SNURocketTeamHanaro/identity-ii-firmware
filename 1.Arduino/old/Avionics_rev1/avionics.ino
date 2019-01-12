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
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

#define periodt 10
#define periodT 100
#define ini_count 20
#define main 25
#define drogue 23

#define MAX_DATA_NUM 15
#define SAVE_CYCLE 15

struct data_packet_t {
  uint32_t current_time; /* 4          */
  float a[3];            /* 4 * 3 = 12 */
  float g[3];            /* 4 * 3 = 12 */
  float m[3];            /* 4 * 3 = 12 */
  float alt_press;       /* 4          */
  float alt_max;         /* 4          */
  uint32_t count;        /* 4          */
  byte on;               /* 1          */
};

typedef union data_t {
  struct data_packet_t data_packet;
  char data_packet_string[sizeof(struct data_packet_t)];
  byte data_packet_bytestring[sizeof(struct data_packet_t)];
};

data_t data_cache;

FaBo9Axis fabo_9axis;
Adafruit_BMP280 bmp; // I2C
File myFile;

float ini_altitude, calc = 0;
uint32_t current_time, pre_time=0;
uint32_t Pre_time=0;
uint32_t full_count = 5;
uint32_t count;
byte on = 0;
uint32_t num = 0;
float max_altitude, pres_altitude = 0;
float A = 0;
byte data[sizeof(struct data_packet_t)][MAX_DATA_NUM];
uint32_t datanum = 0;
uint32_t save_cycle = 0;

void setup() {
  Wire.begin();
  Serial.begin(115200);
  Serial2.begin(115200);
  //Serial.println("[STATUS] configuring device.");
  //Serial2.println("[STATUS] configuring device.");
  pinMode(main, OUTPUT);
  digitalWrite(main, LOW);
  pinMode(drogue, OUTPUT);
  digitalWrite(drogue, LOW);
  
  if (fabo_9axis.begin()) {
    //Serial2.println("[STATUS] configured FaBo 9Axis I2C Brick");
    //Serial.println("[STATUS] configured FaBo 9Axis I2C Brick");
    fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);
  } else {
    //Serial2.println("[STATUS] device error");
    //Serial.println("[STATUS] device error");
    while(1);
  }

  if (!bmp.begin()) {  
    //Serial2.println(F("[STATUS] Could not find a valid BMP280 sensor, check wiring!"));
    //Serial.println(F("[STATUS] Could not find a valid BMP280 sensor, check wiring!"));
    while (1);
  } else {
    //Serial2.println("[STATUS] initializing bmp 280");
    //Serial.println("[STATUS] initializing bmp 280");
    for(int i=0; i<ini_count; i++){
      calc += bmp.readAltitude();
      delay(100);
    }
    calc = calc/ini_count;
    ini_altitude = calc;
    //Serial2.print("[STATUS] initial altitude = ");
    //Serial.print("[STATUS] initial altitude = ");

    Serial2.println(ini_altitude);
    Serial.println(ini_altitude);
    max_altitude = 0;
  }
  if(!SD.begin(53)){
    Serial2.println("[STATUS] cannot find SD card");
    Serial.println("[STATUS] cannot find SD card");
  } else {
    myFile = SD.open("Ayoyo.txt", FILE_WRITE);
    myFile.print("initial altitude = ");
    myFile.println(ini_altitude);
    Serial2.println("Start");
  }
}

void loop() {
  current_time = millis();
  if(current_time - pre_time >= periodt) {
    pre_time = current_time;
    float ax,ay,az;
    float gx,gy,gz;
    float mx,my,mz;
    float temp;
    String data_string;

    fabo_9axis.readAccelXYZ(&ax,&ay,&az);
    fabo_9axis.readGyroXYZ(&gx,&gy,&gz);
    fabo_9axis.readMagnetXYZ(&mx,&my,&mz);
    // fabo_9axis.readTemperature(&temp);
    A = sqrt(ax*ax+ay*ay+az*az);
    pres_altitude = bmp.readAltitude();

    if(A >= 1.1) {
      on = 1;
    }
    
    if(on) {
      if(pres_altitude >=max_altitude){
      max_altitude = pres_altitude;
      count = 0;
      } else {
        if(max_altitude - pres_altitude >= 1.5) {
          digitalWrite(drogue, HIGH);
          count = 1;
        }
        if(max_altitude - pres_altitude >= 50) {
          digitalWrite(main, HIGH);
          count = 2;
        }
      }
    }

    /*
    data_string = String(current_time)+" "+String(ax,4)+" "+String(ay,4)+" "+String(az,4)+" "+
                  String(gx,4)+" "+String(gy,4)+" "+String(gz,4)+" "+
                  String(mx,4)+" "+String(my,4)+" "+String(mz,4)+" "+
                  String(pres_altitude,4)+" "+
                  // String(temp,4)+" "+String(A,4)+" "+
                  String(max_altitude,4)+" "+String(count)+" "+String(on)+" "+"\n";
    data2 = data2 + data_string;
    */

    data_cache.data_packet.current_time = current_time;
    data_cache.data_packet.a[0] = ax;
    data_cache.data_packet.a[1] = ay;
    data_cache.data_packet.a[2] = az;
    data_cache.data_packet.g[0] = gx;
    data_cache.data_packet.g[1] = gy;
    data_cache.data_packet.g[2] = gz;
    data_cache.data_packet.m[0] = mx;
    data_cache.data_packet.m[1] = my;
    data_cache.data_packet.m[2] = mz;
    data_cache.data_packet.alt_press = pres_altitude;
    data_cache.data_packet.alt_max = max_altitude;
    data_cache.data_packet.count = count;
    data_cache.data_packet.on = on;

    memcpy(&data[datanum++], &data_cache.data_packet_bytestring, sizeof(struct data_packet_t));
    
    Serial.write('$'); /* Start character */
    Serial.write(data_cache.data_packet_bytestring, sizeof(data_cache.data_packet_bytestring));
    Serial.write('#'); /* End character */

    Serial2.write('$'); /* Start character */
    Serial2.write(data_cache.data_packet_bytestring, sizeof(data_cache.data_packet_bytestring));
    Serial2.write('#'); /* End character */
  }
    
  if (current_time-Pre_time >= periodT || datanum == MAX_DATA_NUM - 1){
    Pre_time = current_time;
    
    if (myFile) {
      for (uint32_t i = 0; i < datanum; ++i) {
        myFile.write('$'); /* Start character */
        myFile.write(data[i], sizeof(struct data_packet_t));
        myFile.write('#'); /* End character */
        memset(&data[i], '\0', sizeof(struct data_packet_t));
      }
    } else {
      Serial2.println("[STATUS] no SD card....... ssibal");
      Serial.println("[STATUS] no SD card....... ssibal");
    }
    datanum = 0;
    ++save_cycle;
    
    if (save_cycle > SAVE_CYCLE) {
      myFile.close();
      myFile = SD.open("Ayoyo.txt", FILE_WRITE);
      save_cycle = 0;
    }
  }
}
