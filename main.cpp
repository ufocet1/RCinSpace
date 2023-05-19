#include <SPI.h> //SPI Library 

#include <LoRa.h> //LoRa Library 

String incoming;

void setup() {

  Serial.begin(9600); //Serial for Debugging 

  if (!LoRa.begin(433E6)) { //Operate on 433MHz

    Serial.println("Starting LoRa failed!");

    while (1);
  }
  else{
    Serial.println("Lora Established");
  }
}
void loop() {

  incoming = "";
  
  int packetSize = LoRa.parsePacket();
  if (packetSize) {     // If packet received

    while (LoRa.available()) {
      
      incoming += char(LoRa.read());

      }
      Serial.println(incoming);
      
    }
}


