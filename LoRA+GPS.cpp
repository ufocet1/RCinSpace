//Include necessary libraries for LoRa, Arduino, and the GPS module. 
#include <SPI.h>
#include <LoRa.h>
#include <TinyGPS++.h>
#include <SoftwareSerial.h>
#include <EEPROM.h>

//Pin Definitions
int RXPin = 2; // Connect GPS module's RX pin to Arduino pin 2 (RX)
int TXPin = 3; // Connect GPS module's TX pin to Arduino pin 3 (TX)

//Create GPS object, establish GPS serail communication.
TinyGPSPlus gps;
SoftwareSerial SoftSerial(RXPin, TXPin);

//Declare variables
int count = 0;
int EEPROM_Adress = 0;

//Create GPS function which will be used to get info from the GPS
void GPS(){
  
  //Declare GPS variables.
  
  double Lat;
  double Lng;
  int Hour;
  int Minute;
  int Second;
  int Satellite;
  
  
  //Check if the GPS is available and sending DATA
  while(SoftSerial.avaliable() > 0){
    //Encode GPS lines using GPS++ Library
    if(gps.encode(SoftSerial.read())){
     
      //If GPS location is updated send the location using LoRa and write data on to EEPROM.
      //In order to avoid EEPROM memory degredation, EEPROM write cycle is only initiated after 2 minutes have already passed.
      //If after 2 minutes the location is updated the new location is written onto the EEPROM.
      
      if (gps.location.isUpdated()) {
        
        Lat = (gps.location.lat(),6);
        Lng = (gps.location.lng(),6);
        Hour = (gps.time.hour());
        Minute = (gps.time.minute());
        Second = (gps.time.second());
        Satellite = (gps.satellites.value());
        
        //Parse everything into a single variable for EEPROM.
        
        String datGPS = "Lat:" + String(Lat) + " Lng:" + String(Lng) + " Hour:" + String(Hour) + " Minute:" + String(Minute) + " Second:" + String(Second) + " Satellites:" + String(Satellite); 
        
        //Check if 2 minutes has passed, if so write and reset timer.
        
        if(count > 120){
          EEPROM.write(EEPROM_Adress,datGPS);
          count = 0;
        }
        //Send packet to LoRa
        LoRa.beginPacket();
        LoRa.println(datGPS);
        LoRa.endPacket();
    }  
  }
    
  //FAILSAFE METHOD
  //IF the GPS is somehow not able to get a fix properly, send the last known location using data written on the EEPROM. 
  while(SoftSerial.avaliable() == 0){
    //Send Last Packet to LoRa
    LoRa.beginPacket();
    LoRa.println((EEPROM.read(EEPROM_Adress)));
    LoRa.endPacket();
  }  
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

void loop(){
  
  GPS();
  delay(1000);
  
  //Count 120 seconds
  count++;
  
}







