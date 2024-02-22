// RC IN SPACE
// BME280 -> 0X77
// SD SHIELD PIN -> 4 (!ESP 8266!)
// MPU -> 0X68 (DEFAULT ADDR) for ARDUINO!!!!!!1 CHECK THIS BEFORE WIRING TO ESP!!!

#include <Adafruit_BME280.h>
#include "MPU6050.h"
// #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SD.h>
#include <Wire.h>
#include <Arduino.h>
#include <DallasTemperature.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

#define SEALEVELPRESSURE_HPA (1014)
#define LOW_ALARM 20
#define HIGH_ALARM 25


//SD Card
#define OUTPUT_FILE "UFUK.txt"
const int SD_CHIP_SELECT = 4; //10 veya 8 veya 15 de olabilir
File dataFile;

// !!!!! ONEMLI -> 0X77de olmasi lazim!!!!!!!!!!!!!
Adafruit_BME280 bme; // I2C

// Adafruit_MPU6050 mpu;
MPU6050 mpu;

TinyGPSPlus gps;
SoftwareSerial SoftSerial(5, 6); // GPS RX pin to 5 (green), GPS TX pin to 6 (yellow)

// GPS data variables
double Lat;
double Lng;
int Hour;
int Minute;

/* =================================================== FUNCTIONS =================================================== */

void setup() {

  Serial.begin(9600);
  SoftSerial.begin(9600);
 
  Serial.println("SD Init..."); 
  if(!SD.begin(4)) // SD kart CS bacağı 4 nolu dijital pine bağlı, SD kart takılı değilse
  {
    Serial.println("SD failed");
    while(!SD.begin(4)); // kart takılana kadar beklenir
    Serial.println("SD OK"); // kart takılınca program devam eder
  }
  else
  {
    Serial.println("SD OK"); // kart takılı ise program devam eder
  }

  Serial.println("LoRa hazırlanyor");
  
  if (!LoRa.begin(433E6)) {
    Serial.println(F("Starting Serial failed!"));
    while (1);
  }
  Serial.println("LoRa OK");
  
  LoRa.setTxPower(20);

  Serial.println(F("BME280 & DS18B20 & MPU6050 & GY-63 test"));

  unsigned status = bme.begin();

  Serial.println("BME280 OK");

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    Serial.println("MPU not found");
  }
  Serial.println("MPU OK");


}

void run_sensors() {
  float bmeT, bmeP, bmeA, bmeH, mpuAx, mpuAy, mpuAz, mpuGx, mpuGy, mpuGz;

  { // BME280
    bmeT = bme.readTemperature();
    bmeP = bme.readPressure() / 100.0F;
    bmeA = bme.readAltitude(SEALEVELPRESSURE_HPA);
    bmeH = bme.readHumidity();
  }

  { // Acceleration & gyro
    Vector normAccel = mpu.readNormalizeAccel();
    Vector normGyro = mpu.readNormalizeGyro();
    mpuAx = normAccel.XAxis;
    mpuAy = normAccel.YAxis;
    mpuAz = normAccel.ZAxis;
    // Calculate Pitch, Roll and Yaw
    mpuGx = normGyro.XAxis;
    mpuGy = normGyro.YAxis;
    mpuGz = normGyro.ZAxis;
  }

  //Shrunk data in order to save space.
  Serial.print(bmeT); Serial.print("C "); Serial.print(bmeP); Serial.print("Pa "); Serial.print(bmeA); Serial.print("m ");
  Serial.print(bmeH); Serial.println("% ");
  Serial.print(mpuAx); Serial.print("X "); Serial.print(mpuAy); Serial.print("Y "); Serial.print(mpuAz); Serial.print("Z ");
  Serial.print(mpuGx); Serial.print("Xr "); Serial.print(mpuGy); Serial.print("Yr "); Serial.print(mpuGz); Serial.println("Zr ");
  
  //Write to SD
  Serial.println("Preparing SD Write");

  dataFile = SD.open(OUTPUT_FILE , FILE_WRITE);
  
  if (dataFile) {
    Serial.println("Writing to SD");
    dataFile.println("Entry #1");
    dataFile.print(bmeT); dataFile.print("C "); dataFile.print(bmeP); dataFile.print("Pa "); dataFile.print(bmeA); dataFile.print("m ");
    dataFile.print(bmeH); dataFile.println("% ");
    dataFile.print(mpuAx); dataFile.print("X "); dataFile.print(mpuAy); dataFile.print("Y "); dataFile.print(mpuAz); dataFile.print("Z ");
    dataFile.print(mpuGx); dataFile.print("Xr "); dataFile.print(mpuGy); dataFile.print("Yr "); dataFile.print(mpuGz); dataFile.println("Zr ");
    dataFile.close();
  }
  else {
    Serial.println("error on writing to SD");
  }
  delay(500);
 
}
// Create GPS function to retrieve information from the GPS
void GPS() {
  // Check if the GPS is available and sending data
  if (SoftSerial.available() > 0) {
    // Encode GPS lines using GPS++ Library
    if (gps.encode(SoftSerial.read())) {
      // If GPS location is updated, send the location using Serial and write data to EEPROM.
      if (gps.location.isUpdated()) {
        Lat = gps.location.lat();
        Lng = gps.location.lng();
        Hour = gps.time.hour();
        Minute = gps.time.minute();
      }
    }
    //Send Gps info over Serial

    Serial.println(Lat + Lng + Hour + Minute);

  }

}
void loop() {

  //Serial.beginPacket();
  //Serial.println("Hellow World");
  //Serial.endPacket();
  run_sensors();
  //GPS();
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);

}
