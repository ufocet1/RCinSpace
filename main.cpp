#include <Arduino.h>
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>

// Choose two Arduino pins to use for software serial

int RXPin = 3;
int TXPin = 4;   
int x = 0;
int failsafe= 0;

// Create a TinyGPS++ object

TinyGPSPlus gps;
SoftwareSerial SoftSerial(RXPin, TXPin);

void GPS(){

  LoRa.beginPacket();


  while (SoftSerial.available() > 0) {

    if (gps.encode(SoftSerial.read())) {
 
      if (gps.location.isValid()) {
        LoRa.print("Latitude   = ");
        LoRa.println(gps.location.lat(), 6);
        LoRa.print("Longitude  = ");
        LoRa.println(gps.location.lng(), 6);
      }
        
      if (gps.altitude.isValid()) {
        LoRa.print("Altitude   = ");
        LoRa.print(gps.altitude.meters());
        LoRa.println(" meters");
      }

      if (gps.time.isValid()) {
        LoRa.print("Time (GMT) : ");
        if(gps.time.hour() < 10)     
        LoRa.print(gps.time.hour());
        LoRa.print(":");
        if(gps.time.minute() < 10)  
        LoRa.print(gps.time.minute());
        LoRa.print(":");
        if(gps.time.second() < 10)   
        LoRa.println(gps.time.second());
      }

      if (gps.satellites.isValid()) {
        LoRa.print("Satellites = ");
        LoRa.println(gps.satellites.value());
      }
      
  }
}
LoRa.endPacket();
}

void setup() {
  
  Serial.begin(9600);
  SoftSerial.begin(9600);
  
  while (!Serial);

  Serial.println("This is the Sender (GPS Attached) Module");

  if (!LoRa.begin(433E6)) {
    Serial.println("Starting LoRa failed!");
    while (1);
  }
  LoRa.setTxPower(20);
  
  //Notify the reciever
  LoRa.beginPacket();
  LoRa.print("Wireless Connection Established");
  LoRa.endPacket();

}

void loop() {
  while(x > 5){
    //Heartbeat
    LoRa.beginPacket();
    LoRa.println("C+C"+String(failsafe));
    LoRa.endPacket();
    failsafe++;

    int run = 0;
    while(run<200){
      GPS();
      run++;
    }

    x = 0;
    Serial.println("GPS Complete");
  }
  delay(1000);
  x++;
}