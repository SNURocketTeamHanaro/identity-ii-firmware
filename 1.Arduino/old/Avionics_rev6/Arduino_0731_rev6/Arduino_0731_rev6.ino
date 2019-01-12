/***************************************************************************
   Filename       : Avionics.ino
   Target machine : Arduino Uno with GY91 sensor module

   Maximum stable thruput : I don't know
   Maximum unstable thruput : I don't know

   Contributor    :
          rev.1 @ 2017. 11. 27.
                HANARO, SNU.
                Beom Park
          rev.2 @ 2017. 12. 01.
                HANARO, SNU.
                Jaerin Lee, Beom Park
          rev.3 @ 2018. 02. 01.
                HANARO, SNU.
                Jaerin Lee
          rev.4 @ 2018. 02. 10.
                HANARO, SNU.
                Jaerin Lee
          rev.5 @ 2018. 06. 30.
                HANARO, SNU.
                Jaerin Lee
          rev.6 @ 2018. 07. 31.
                HANANRO, SNU.
                Mingyu Park
 ***************************************************************************/

#include <stdio.h>
#include <SD.h>
//#include "SdFat.h"
#include <Wire.h>
#include <SPI.h>
//#include <SoftwareSerial.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

#define A_THRESHOLD 1.15
#define DROGUE_PARA_THRESHOLD 2.5
#define MAIN_PARA_THRESHOLD 50

#define INIT_ALT_DUMP_COUNT 10
#define INIT_ALT_COUNT 20
//#define MAIN 25
#define MAIN 6
//#define DROGUE 23
#define DROGUE 7

// #define FILE_NAME "Ayoyo"
#define PERIOD_DATA_RETRIEVAL 5 // ms
#define MAX_DATA_NUM 5
#define SAVE_CYCLE 5

struct data_packet_t {
  uint32_t current_time; /* 4          */
  float a[3];            /* 4 * 3 = 12 */
  float g[3];            /* 4 * 3 = 12 */
  //float m[3];            /* 4 * 3 = 12 */ 
  //float temp;            /* 4          */
  //uint32_t press;        /* 4          */
  float alt_press;       /* 4          */
  float alt_max;         /* 4          */
  uint32_t count;        /* 4          */
  byte on;               /* 1          */
};

typedef union data_t {
  struct data_packet_t data_packet;
  char data_packet_string[sizeof(struct data_packet_t)];
  byte data_packet_bytestring[sizeof(struct data_packet_t)];
} data_t; //Problem of installing C++?? without data_t here, typedef will be ignored @Jaerin

data_t data[MAX_DATA_NUM];
uint32_t datanum = 0;
uint32_t save_cycle = 0;

// SoftwareSerial xBee(17, 16); // RX, TX

FaBo9Axis fabo_9axis;
Adafruit_BMP280 bmp; // I2C
// SdFat SD;
File file_sd;
// char file_name[20];

uint32_t curr_time;
uint32_t prev_time = 0;
// uint32_t full_count = 5;
float ini_altitude;
float last_altitude;
float max_altitude;
uint32_t trigger_altitude = 0;
uint32_t rocket_on = 0;
uint32_t para_drogue = 0;
uint32_t para_main = 0;

void setup() {
  Serial.begin(115200);
  //Serial2.begin(115200);
  delay(3000); // For Leonardo/Micro

  Serial.println("[  WORK] Initiate device configuration.");
  //Serial2.print("[  WORK] Initiate device configuration.\r\n");

  Wire.begin();
  pinMode(10, OUTPUT);//to use ss pin(10) in arduino uno as cs pin
  digitalWrite(10, HIGH);
  pinMode(MAIN, OUTPUT);
  digitalWrite(MAIN, LOW);
  pinMode(DROGUE, OUTPUT);
  digitalWrite(DROGUE, LOW);

  if (fabo_9axis.begin()) {
    fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);

    Serial.println("[STATUS] Configured: FaBo 9Axis I2C Brick.");
    //Serial2.print("[STATUS] Configured: FaBo 9Axis I2C Brick.\r\n");
  } else {
    Serial.println("[ ERROR] Device error: FaBo 9Axis I2C Brick.");
    //Serial2.print("[ ERROR] Device error: FaBo 9Axis I2C Brick.\r\n");

    while (1)
      ; // Die here
  }

  //if (!bmp.begin()) {
  if (!bmp.begin()) {
    Serial.println("[ ERROR] Device error: BMP280 sensor. (Note: Check wiring!)");
    //Serial2.print("[ ERROR] Device error: BMP280 sensor. (Note: Check wiring!)\r\n");

    while (1)
      ; // Die here
  } else {
    Serial.println("[STATUS] Configured: BMP 280.");
    //Serial2.print("[STATUS] Configured: BMP 280.\r\n");

    // Read altitude for calibration
    float tmp;
    Serial.print("[  WORK] Dump altitude measurement on initiation: ");
    //Serial2.print("[  WORK] Dump altitude measurement on initiation: ");
    for (uint32_t i = 0; i < INIT_ALT_DUMP_COUNT; ++i) {
      delay(100);
      tmp = bmp.readAltitude();
      Serial.print(tmp);
      Serial.print(" ");
      //Serial2.print(tmp);
      //Serial2.print(" ");
    }
    Serial.print("\n");
    //Serial2.print("\r\n");

    float calc = 0;
    Serial.print("[  WORK] Evaluate initial altitude: ");
    //Serial2.print("[  WORK] Evaluate initial altitude: ");
    for (uint32_t i = 0; i < INIT_ALT_COUNT; ++i) {
      delay(100);
      calc += (tmp = bmp.readAltitude());
      Serial.print(tmp);
      Serial.print(" ");
      //Serial2.print(tmp);
      //Serial2.print(" ");
    }
    Serial.print("\n");
    //Serial2.print("\r\n");

    calc = calc / INIT_ALT_COUNT;
    max_altitude = last_altitude = ini_altitude = calc;

    Serial.print("[  DATA] Initial altitude: ");
    //Serial2.print("[  DATA] Initial altitude: ");
    Serial.println(ini_altitude);
    //Serial2.print(ini_altitude);
    //Serial2.print("\r\n");
  }

  while (1) {
    if (!SD.begin(4)) {
      Serial.println("[ ERROR] Device error: SD card.");
      //Serial2.print("[ ERROR] Device error: SD card.\r\n");
    } else {
      // Open a new file
      // sprintf(file_name, "UnoShit%d.txt\0", millis());
      file_sd = SD.open("UnoShit.txt", FILE_WRITE); //O_CREAT | O_WRITE);
      Serial.print("[STATUS] File opened: ");
      Serial.println("UnoShit.txt");
      //Serial2.print("[STATUS] File opened: ");
      //Serial2.print(file_name);
      //Serial2.print("\r\n");

      file_sd.print("Initial altitude: ");
      file_sd.println(ini_altitude);

      Serial.println("[STATUS] Start logging ...");
      //Serial2.print("[STATUS] Start logging ...\r\n");
      break;
    }
  }

  // Main loop (Performance issue)
  while (1) {
    curr_time = millis();
    if (curr_time - prev_time >= PERIOD_DATA_RETRIEVAL) {
      prev_time = curr_time;

      // Calculate A first (optimization)
      float ax, ay, az;
      float A = 0;
      fabo_9axis.readAccelXYZ(&ax, &ay, &az);
      A += ax * ax; A += ay * ay; A += az * az;

      // Retrieve rest of the data
      float gx, gy, gz;
      //float mx, my, mz;
      float temp;
      fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
      //fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
      fabo_9axis.readTemperature(&temp);

      //uint32_t press;
      //press = (uint32_t)analogRead(A0);

      // Temporal data in stack (optimization)
      uint32_t count = 0;
      float pres_altitude = 0;
      pres_altitude = bmp.readAltitude();

      A = sqrt(A);
      if (!rocket_on && A >= A_THRESHOLD) {
        rocket_on = 1;
      }

      if (rocket_on) {
        if (pres_altitude >= max_altitude) {
          max_altitude = pres_altitude;
          trigger_altitude = 0;
          count = 0;
        } else if (max_altitude - pres_altitude >= MAIN_PARA_THRESHOLD) {
          digitalWrite(MAIN, HIGH);
          para_main = 1;
          trigger_altitude = 1;
          count = 2;
        } else if (max_altitude - pres_altitude >= DROGUE_PARA_THRESHOLD) {
          if (trigger_altitude && pres_altitude != last_altitude) {
            digitalWrite(DROGUE, HIGH);
            para_drogue = 1;
            trigger_altitude = 1;
            count = 1;
          } else {
            trigger_altitude = 1;
            count = 0;
          }
        } else {
          trigger_altitude = 0;
          count = 0;
        }
      }

      // Package remaining data
      data[datanum].data_packet.a[0] = ax;
      data[datanum].data_packet.a[1] = ay;
      data[datanum].data_packet.a[2] = az;
      data[datanum].data_packet.g[0] = gx;
      data[datanum].data_packet.g[1] = gy;
      data[datanum].data_packet.g[2] = gz;
      // data[datanum].data_packet.m[0] = mx;
      // data[datanum].data_packet.m[1] = my;
      // data[datanum].data_packet.m[2] = mz;
      //data[datanum].data_packet.temp = temp;
      //data[datanum].data_packet.press = press;
      data[datanum].data_packet.current_time = curr_time;
      data[datanum].data_packet.alt_press = pres_altitude;
      data[datanum].data_packet.alt_max = max_altitude;
      data[datanum].data_packet.count = count;
      data[datanum].data_packet.on = rocket_on + para_drogue + para_main;

      /*
        // Serial output of the data
        Serial.write('$'); // Start character
        Serial.write('$'); // Start character
        Serial.write(data[datanum].data_packet_bytestring,
        sizeof(data[datanum].data_packet_bytestring));
        Serial.write('#'); // End character
        Serial.write('#'); // End character
      */

      // Debug code for accelerometer
      /*
      Serial.print(ax);
      Serial.print(" ");
      Serial.print(ay);
      Serial.print(" ");
      Serial.print(az);
      Serial.print("\n");
      */

      /*
      // Send data to ZigBee connection
      // This cause much error due to serial buffer overflow
      //Serial2.write('$'); // Start character
      //Serial2.write('$'); // Start character
      //Serial2.write(data[datanum].data_packet_bytestring,
      sizeof(data[datanum].data_packet_bytestring));
      //Serial2.write('#'); // End character
      //Serial2.write('#'); // End character
      */

      // Send minimal data to Zigbee connction
      // Data to be sent:
      //   - Data extraction time
      //   - Altitude from the barometer (if changed)
      //Serial2.flush(); // Prevent serial buffer to overflow
      //if (last_altitude != pres_altitude) {
        //Serial2.print(curr_time);
        //Serial2.write(' ');
        //Serial2.print(rocket_on);
        //Serial2.print(para_drogue);
        //Serial2.print(para_main);
        //Serial2.write(' ');
        //Serial2.print(pres_altitude);
        //Serial2.write(' ');
        //Serial2.print(max_altitude);
        //Serial2.write(' ');
        //Serial2.print(max_altitude - pres_altitude);
        //Serial2.print("\r\n");

        //last_altitude = pres_altitude;
      //}

      // Next data
      ++datanum;
    }

    if (datanum == MAX_DATA_NUM - 1) {
      // Assume no problem with either the SD card or the connection

      // Write the whole data
      for (uint32_t i = 0; i < datanum; ++i) {
        file_sd.write('$'); /* Start character */
        file_sd.write('$'); /* Start character */
        file_sd.write(data[i].data_packet_bytestring, sizeof(struct data_packet_t));
        file_sd.write('#'); /* End character */
        file_sd.write('#'); /* End character */
        //memset(&data[i], '\0', sizeof(struct data_packet_t));
      }

      // Reset data buffer; go closer to the next saving cycle
      datanum = 0;
      ++save_cycle;

      // Save the file into the SD card
      if (save_cycle >= SAVE_CYCLE) {
        save_cycle = 0;

        file_sd.close();
        //file_sd.flush();
        file_sd = SD.open("UnoShit.txt", FILE_WRITE); //O_CREAT | O_WRITE);

        Serial.print("[STATUS] Data saved at ");
        Serial.print(millis());
        Serial.println("ms.");
      }
    }
  }
}

void loop() {
}
