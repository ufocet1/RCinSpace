#include "MPU6050.h"
// RC IN SPACE
// BME280 -> 0X77
// MS5611 -> 0X76
// SD SHIELD PIN -> 4 (!ESP 8266!)
// MPU -> 0X68 (DEFAULT ADDR) for ARDUINO!!!!!!1 CHECK THIS BEFORE WIRING TO ESP!!!
// DS18B20 -> OneWire
#include <Adafruit_BME280.h>
// #include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include "SD.h"
#include <Wire.h>
#include <Arduino.h>
#include <DallasTemperature.h>
#include "MS5611.h"
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>
#define SEALEVELPRESSURE_HPA (1014)
#define ONEWIRE_DS18B20_BUS 2 // d4
#define LOW_ALARM 20
#define HIGH_ALARM 25
#define MS5611_I2C_DEFAULT_ADDRESS 0x77
#define MS5611_I2C_REDEFINED_ADDRESS 0X76
#define OUTPUT_FOLDER "RCInSpace/"
#define OUTPUT_FILE "RCInSpace/output.csv"
// !!!!! ONEMLI -> 0X77de olmasi lazim!!!!!!!!!!!!!
Adafruit_BME280 bme; // I2C
OneWire oneWire(ONEWIRE_DS18B20_BUS);
DallasTemperature sensors(&oneWire);
// Adafruit_MPU6050 mpu;
MPU6050 mpu;
// !!!!!ONEMLI -> EGER CALISMIYOSA 0X77 DENE (BME280 0X77de ama)
MS5611 MS5611(MS5611_I2C_REDEFINED_ADDRESS);
TinyGPSPlus gps;
SoftwareSerial SoftSerial(5, 6); // GPS RX pin to 5 (green), GPS TX pin to 6 (yellow)
// GPS data variables
double Lat;
double Lng;
int Hour;
int Minute;

/* =================================================== FUNCTIONS =================================================== */

File output;

const int SD_CHIP_SELECT = 4; //10 veya 8 veya 15 de olabilir

void setup() {
  Serial.begin(9600);
  SoftSerial.begin(9600);
  
  //Serial.println(F("BME280 & DS18B20 & MPU6050 & GY-63 test"));

  if (!SD.begin(4)) {
    //Serial.println(F("SD Initialization failed..."));
  }

  sensors.begin();

  unsigned status = bme.begin();
  if (!status) {
    //Serial.println(F("Could not find a valid BME280 sensor, check wiring, address, sensor ID!"));
  }

  while (!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G))
  {
  }

  if (MS5611.begin() == true)
  {
    //Serial.println(F("MS5611 Found - using I2C address 0x76."));
  }
  else {
    //Serial.println(F("MS5611 not found. halt."));
  }
  //LoRa Initialization
  while (!Serial);

  if (!LoRa.begin(433E6)) {
    //Serial.println(F("Starting LoRa failed!"));
    while (1);
  }
  LoRa.setTxPower(20);

}

void run_sensors(int write = 0) {
  float dsT, bmeT, bmeP, bmeA, bmeH, mpuAx, mpuAy, mpuAz, mpuGx, mpuGy, mpuGz, msT, msP, msA;

  { // DS18B20
    sensors.requestTemperatures();
    float tempC = sensors.getTempCByIndex(0);

    if (tempC != DEVICE_DISCONNECTED_C)
      dsT = tempC;
    else {
      //Serial.println("Error: Could not read DS18B20 temperature data");
      dsT = -999;
    }
  }

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

  { // MS5611
    float result = MS5611.read();
    if (result != MS5611_READ_OK)
    {
      //Serial.print("Error in read: ");
      //Serial.println(result);
      msT = -999;
      msP = -999;
      msA = -999;
    } else {
      msT = MS5611.getTemperature();
      msP = MS5611.getPressure();
      msA = ((pow((SEALEVELPRESSURE_HPA / msP), 5.257) - 1) * (msT + 273.15)) / 0.0065;
    }
  }
  //Shrunk data in order to save space. Use the decoder google doc to decode incoming info.
  LoRa.beginPacket();
  LoRa.print(bmeT);
  LoRa.print("C ");
  LoRa.print(bmeP);
  LoRa.print("Pa ");
  LoRa.print(bmeA);
  LoRa.print("m ");
  LoRa.print(bmeH);
  LoRa.println("% ");
  LoRa.print(mpuAx);
  LoRa.print("X ");
  LoRa.print(mpuAy);
  LoRa.print("Y ");
  LoRa.print(mpuAz);
  LoRa.print("Z ");
  LoRa.print(mpuGx);
  LoRa.print("Xr ");
  LoRa.print(mpuGy);
  LoRa.print("Yr ");
  LoRa.print(mpuGz);
  LoRa.println("Zr ");
  LoRa.print(msT);
  LoRa.print(" ");
  LoRa.print(msP);
  LoRa.print(" ");
  LoRa.print(msA);
  LoRa.endPacket();
}
// Create GPS function to retrieve information from the GPS
void GPS() {
  // Check if the GPS is available and sending data
  while (SoftSerial.available() > 0) {
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
  GPS();
  delay(1000);
}
