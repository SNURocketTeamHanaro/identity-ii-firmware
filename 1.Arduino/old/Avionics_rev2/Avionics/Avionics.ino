/***************************************************************************
   Filename       : Avionics.ino
   Target machine : Arduino Mega with GY91 sensor module

   Maximum stable thruput : 1 pkt / 10 ms ; savetime 22 - 35 ms
   Maximum unstable thruput : 1 pkt / 6-7 ms

   Contributor    :
          rev.1 @ 2017. 11. 27.
                HANARO, SNU.
                Beom Park
          rev.2 @ 2017. 12. 01.
                HANARO, SNU.
                Jaerin Lee, Beom Park
 ***************************************************************************/

#include <stdio.h>
#include <SD.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <FaBo9Axis_MPU9250.h>

#define A_THRESHOLD 1.1

#define INI_COUNT 20
#define MAIN 25
#define DROGUE 23

#define FILE_NAME "Ayoyo.txt"
#define PERIOD_DATA_RETRIEVAL 10 // ms
#define MAX_DATA_NUM 10
#define SAVE_CYCLE 5

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

data_t data[MAX_DATA_NUM];
uint32_t datanum = 0;
uint32_t save_cycle = 0;

FaBo9Axis fabo_9axis;
Adafruit_BMP280 bmp; // I2C
File file_sd;


uint32_t curr_time;
uint32_t prev_time = 0;
// uint32_t full_count = 5;
float ini_altitude;
float max_altitude;

void setup() {
  Serial.begin(115200);
  Serial2.begin(115200);

  Serial.println("[STATUS] configuring device.");
  Serial2.println("[STATUS] configuring device.");

  Wire.begin();
  pinMode(MAIN, OUTPUT);
  digitalWrite(MAIN, LOW);
  pinMode(DROGUE, OUTPUT);
  digitalWrite(DROGUE, LOW);

  if (fabo_9axis.begin()) {
    fabo_9axis.configMPU9250(MPU9250_GFS_1000, MPU9250_AFS_16G);

    Serial.println("[STATUS] configured FaBo 9Axis I2C Brick");
    Serial2.println("[STATUS] configured FaBo 9Axis I2C Brick");
  } else {
    Serial.println("[STATUS] device error");
    Serial2.println("[STATUS] device error");

    while (1)
      ; // Die here
  }

  if (!bmp.begin()) {
    Serial.println(F("[STATUS] Could not find a valid BMP280 sensor, check wiring!"));
    Serial2.println(F("[STATUS] Could not find a valid BMP280 sensor, check wiring!"));

    while (1)
      ; // Die here
  } else {
    Serial.println("[STATUS] initializing bmp 280");
    Serial2.println("[STATUS] initializing bmp 280");

    float calc = 0;
    for (int i = 0; i < INI_COUNT; ++i) {
      calc += bmp.readAltitude();
      delay(100);
    }

    calc = calc / INI_COUNT;
    ini_altitude = calc;

    Serial.print("[STATUS] initial altitude = ");
    Serial2.print("[STATUS] initial altitude = ");
    Serial.println(ini_altitude);
    Serial2.println(ini_altitude);

    max_altitude = 0;
  }

  while (1) {
    if (!SD.begin(53)) {
      Serial.println("[STATUS] cannot find SD card");
      Serial2.println("[STATUS] cannot find SD card");
    } else {
      file_sd = SD.open(FILE_NAME, FILE_WRITE);

      // Try to reopen file from SD card
      file_sd.print("initial altitude = ");
      file_sd.println(ini_altitude);

      Serial.println("[STATUS] Start");
      Serial2.println("[STATUS] Start");
      break;
    }
  }
}

void loop() {
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
    float mx, my, mz;
    fabo_9axis.readGyroXYZ(&gx, &gy, &gz);
    fabo_9axis.readMagnetXYZ(&mx, &my, &mz);
    // fabo_9axis.readTemperature(&data[datanum].data_packet.temp); <- not implemented

    // Temporal data in stack (optimization)
    uint32_t count;
    float pres_altitude = 0;
    pres_altitude = bmp.readAltitude();

    A = sqrt(A);
    if ((data[datanum].data_packet.on = (A >= A_THRESHOLD))) {
      if (pres_altitude >= max_altitude) {
        max_altitude = pres_altitude;
        count = 0;
      } else {
        if (max_altitude - pres_altitude >= 1.5) {
          digitalWrite(DROGUE, HIGH);
          count = 1;
        }
        if (max_altitude - pres_altitude >= 50) {
          digitalWrite(MAIN, HIGH);
          count = 2;
        }
      }
    }

    // Package remaining data
    data[datanum].data_packet.a[0] = ax;
    data[datanum].data_packet.a[1] = ay;
    data[datanum].data_packet.a[2] = az;
    data[datanum].data_packet.g[0] = gx;
    data[datanum].data_packet.g[1] = gy;
    data[datanum].data_packet.g[2] = gz;
    data[datanum].data_packet.m[0] = mx;
    data[datanum].data_packet.m[1] = my;
    data[datanum].data_packet.m[2] = mz;
    data[datanum].data_packet.current_time = curr_time;
    data[datanum].data_packet.alt_press = pres_altitude;
    data[datanum].data_packet.alt_max = max_altitude;
    data[datanum].data_packet.count = count;

    /*
      // Serial output of the data
      Serial.write('$'); // Start character
      Serial.write(data[datanum].data_packet_bytestring,
      sizeof(data[datanum].data_packet_bytestring));
      Serial.write('#'); // End character

      Serial2.write('$'); // Start character
      Serial2.write(data[datanum].data_packet_bytestring,
      sizeof(data[datanum].data_packet_bytestring));
      Serial2.write('#'); // End character
    */

    // Next data
    ++datanum;
  }

  if (datanum == MAX_DATA_NUM - 1) {
    // If connection to the SD card is lost, pause data saving
    if (SD.begin(53)) {
      // Nullify the file object
      file_sd = SD.open(FILE_NAME, FILE_WRITE);

      Serial.println("[STATUS] cannot find SD card");
      Serial2.println("[STATUS] cannot find SD card");

      // We found the SD card!
    } else if (!file_sd) {
      Serial.println("[STATUS] SDcard found! Recollecting data ...");
      Serial2.println("[STATUS] SDcard found! Recollecting data ...");

      // Try to reopen file from SD card
      file_sd = SD.open(FILE_NAME, FILE_WRITE);

      file_sd.print("initial altitude = ");
      file_sd.println(ini_altitude);

      // Restart save cycle
      datanum = 0;
      save_cycle = 0;

      // No problem with either the SD card or the connection
    } else {
      // Write the whole data
      for (uint32_t i = 0; i < datanum; ++i) {
        file_sd.write('$'); /* Start character */
        file_sd.write(data[i].data_packet_bytestring, sizeof(struct data_packet_t));
        file_sd.write('#'); /* End character */
        // memset(&data[i], '\0', sizeof(struct data_packet_t));
      }

      // Reset data buffer; go closer to the next saving cycle
      datanum = 0;
      ++save_cycle;

      // Save the file into the SD card
      if (save_cycle >= SAVE_CYCLE) {
        save_cycle = 0;

        file_sd.close();
        file_sd = SD.open(FILE_NAME, FILE_WRITE);
      }
    }
  }
}
