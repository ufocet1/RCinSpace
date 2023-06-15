// Include necessary libraries
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

// Pin Definitions
int RXPin = 5; // Connect GPS module's RX pin to Arduino pin 5 (RX) (GREEN)
int TXPin = 6; // Connect GPS module's TX pin to Arduino pin 6 (TX) (YELLOW)

// Create GPS object and establish GPS serial communication
TinyGPSPlus gps;
SoftwareSerial SoftSerial(RXPin, TXPin);

// Declare variables
int count = 0;
int EEPROM_Address = 0;

// GPS data variables
double Lat;
double Lng;
int Hour;
int Minute;
int Second;
int Satellite;

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
        Second = gps.time.second();
        Satellite = gps.satellites.value();

        // Construct GPS data string
        String gpsData = "Lat:" + String(Lat, 6) +
                         " Lng:" + String(Lng, 6) +
                         " Hour:" + String(Hour) +
                         " Minute:" + String(Minute) +
                         " Second:" + String(Second) +
                         " Satellites:" + String(Satellite);

        // Check if 2 minutes have passed, if so write to EEPROM and reset the timer.
        if (count > 120) {
          int dataSize = gpsData.length() + 1;
          for (int i = 0; i < dataSize; i++) {
            EEPROM.write(EEPROM_Address + i, gpsData[i]);
          }

          Serial.println("EEPROM Updated!"); // Debug line

          LoRa.beginPacket();
          LoRa.println("EEPROM Updated!");
          LoRa.endPacket();

          count = 0;
        }

        // Debug lines
        Serial.println(gpsData);

        // Send packet using LoRa
        LoRa.beginPacket();
        LoRa.println(gpsData);
        LoRa.endPacket();
      }
    }
  }

  // Failsafe method: If the GPS is unable to get a fix, send the last known location using data from EEPROM.
  String lastGPS;
  char dataChar;
  int i = 0;

  while (dataChar = EEPROM.read(EEPROM_Address + i)) {
    lastGPS += dataChar;
    i++;
  }


  Serial.println("Failsafe Active!, Last Known Location:  " + lastGPS); //Debug Lines
  
  // Send last packet to LoRa
  LoRa.beginPacket();
  LoRa.println("Failsafe Active!, Last Known Location:  " + lastGPS);
  LoRa.endPacket();
}


void setup() {
  Serial.begin(9600);
  SoftSerial.begin(9600);

  while (!Serial)
    ;

  Serial.println("This is the Sender (GPS Attached) Module");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1)
      ;
  }

  LoRa.setTxPower(20);

  // Notify the receiver
  LoRa.beginPacket();
  LoRa.print("Wireless Connection Established");
  LoRa.endPacket();
}

void loop() {
  GPS();
  delay(1000);

  // Count 1 second
  count++;
}
