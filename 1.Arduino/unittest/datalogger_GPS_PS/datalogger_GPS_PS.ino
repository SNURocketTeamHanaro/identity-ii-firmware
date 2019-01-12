#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>
 
#define RXPIN 6
#define TXPIN 5
#define GPSBAUD 9600
 
File myFile;
unsigned long gps_time, current_time, previous_time;
String data;


TinyGPS gps;
SoftwareSerial uart_gps(RXPIN, TXPIN); 
void getgps(TinyGPS &gps);

 
void setup()
{
  Serial.begin(115200);
  uart_gps.begin(GPSBAUD);
  
  Serial.println("");
  Serial.println("GPS Shield QuickStart Example Sketch v12");
  Serial.println("       ...waiting for lock...           ");
  Serial.println("");

  Serial.print("Initializing SD card..."); 

  if (!SD.begin(4)) {
    Serial.println("initialization failed!");
    return;
  }
  Serial.println("initialization done.");
  previous_time = millis();
}
 
void loop()
{
  current_time = millis();
  if (current_time - previous_time >= 50) {
    int sensorValue = analogRead(A0);
    
    myFile = SD.open("pressure.txt", FILE_WRITE);
    String p_data;
    if (myFile) {
    p_data = String(current_time) + " " + String(sensorValue);
    Serial.println(p_data);
    myFile.println(p_data);
    myFile.close();
    } else {
    Serial.println("error opening pressure.txt");
    }
    previous_time = current_time;
  }
  while(uart_gps.available())
  {
      int c = uart_gps.read();
      if(gps.encode(c))
      {
        getgps(gps);
      }   
  }
}
 
void getgps(TinyGPS &gps)
{
  float latitude, longitude, Altitude;
 
  gps.f_get_position(&latitude, &longitude);
  gps_time = millis();
  Serial.print("G"); 
  Serial.println(gps_time);
  data = String(gps_time) + " " + String(latitude, 5) + " " + String(longitude, 5);
  Serial.println(data);
  myFile = SD.open("gpstest1.txt", FILE_WRITE);
  
  if (myFile) {
    myFile.println(data);
    myFile.close();
  } else {
    Serial.println("error opening gpstest1.txt");
  }
 
  
}
 
