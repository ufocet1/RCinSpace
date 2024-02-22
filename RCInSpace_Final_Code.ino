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
#define OUTPUT_FILE "data.txt"
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

  if (!SD.begin(4)) {
    Serial.println("SD Failed");
  }
  else {
    Serial.println("SD OK");
  }

  if (!LoRa.begin(433E6)) {
    Serial.println(F("Starting LoRa failed!"));
    while (1);
  }
  Serial.println("Lora OK");
  
  LoRa.setTxPower(20);

  LoRa.beginPacket();
  LoRa.println("2 Way LoRa Communication OK");

  //Serial.println(F("BME280 & DS18B20 & MPU6050 & GY-63 test"));

  //sensors.begin();

  unsigned status = bme.begin();
  if (!status) {
    LoRa.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
  }
  LoRa.println("BME280 OK");

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
    LoRa.println("MPU not found");
  }
  LoRa.println("MPU OK");
  LoRa.endPacket();

}

void run_sensors(int write = 0) {
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
  LoRa.beginPacket();
  LoRa.print(bmeT); LoRa.print("C "); LoRa.print(bmeP); LoRa.print("Pa "); LoRa.print(bmeA); LoRa.print("m ");
  LoRa.print(bmeH); LoRa.println("% ");
  LoRa.print(mpuAx); LoRa.print("X "); LoRa.print(mpuAy); LoRa.print("Y "); LoRa.print(mpuAz); LoRa.print("Z ");
  LoRa.print(mpuGx); LoRa.print("Xr "); LoRa.print(mpuGy); LoRa.print("Yr "); LoRa.print(mpuGz); LoRa.println("Zr ");
  LoRa.endPacket();

  //Write to SD
  dataFile = SD.open(OUTPUT_FILE , FILE_WRITE);
  dataFile.print(bmeT); dataFile.print("C "); dataFile.print(bmeP); dataFile.print("Pa "); dataFile.print(bmeA); dataFile.print("m ");
  dataFile.print(bmeH); dataFile.println("% ");
  dataFile.print(mpuAx); dataFile.print("X "); dataFile.print(mpuAy); dataFile.print("Y "); dataFile.print(mpuAz); dataFile.print("Z ");
  dataFile.print(mpuGx); dataFile.print("Xr "); dataFile.print(mpuGy); dataFile.print("Yr "); dataFile.print(mpuGz); dataFile.println("Zr ");
  dataFile.close();

}
// Create GPS function to retrieve information from the GPS
void GPS() {
  // Check if the GPS is available and sending data
  if (SoftSerial.available() > 0) {
    // Encode GPS lines using GPS++ Library
    if (gps.encode(SoftSerial.read())) {
      // If GPS location is updated, send the location using LoRa and write data to EEPROM.
      if (gps.location.isUpdated()) {
        Lat = gps.location.lat();
        Lng = gps.location.lng();
        Hour = gps.time.hour();
        Minute = gps.time.minute();
      }
    }
    //Send Gps info over LoRa
    LoRa.beginPacket();
    LoRa.println(Lat + Lng + Hour + Minute);
    LoRa.endPacket();
  }

}
void loop() {

  //LoRa.beginPacket();
  //LoRa.println("Hellow World");
  //LoRa.endPacket();
  run_sensors();
  //GPS();
  digitalWrite(LED_BUILTIN,HIGH);
  delay(1000);
  digitalWrite(LED_BUILTIN,LOW);

}
