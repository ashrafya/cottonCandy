#include <LoRaMesh.h>
 #include <EbyteDeviceDriver.h>
//#include <AdafruitDeviceDriver.h>

#define LORA_RX 10
#define LORA_TX 11

#define LORA_M0 2
#define LORA_M1 3
#define LORA_AUX 4

LoRaMesh* manager;
DeviceDriver* myDriver;
byte myAddr[2] = {0x00, 0xA1};


void onReceiveRequest(byte **data, byte *len) {
  Serial.println("onReciveRequest callback");
  (*data)[0] = 0xD;
  (*data)[1] = 0xE;
  (*data)[2] = 0xA;
  (*data)[3] = 0xD;
  (*data)[4] = 0xB;
  (*data)[5] = 0xE;
  (*data)[6] = 0xE;
  (*data)[7] = 0xF;
  *len = 0x08;
  for(int i = 0; i < *len; i++){
      Serial.print(((*data)[i]),HEX);
  }
}

void setup() {
    Serial.begin(57600);
    while (!Serial) {
      ; // wait for serial port to connect. Needed for native USB port only
    }
     myDriver = new EbyteDeviceDriver(LORA_RX, LORA_TX, LORA_M0, LORA_M1, LORA_AUX, myAddr, 0x09);
//    myDriver = new AdafruitDeviceDriver(myAddr, 0x09);
    myDriver->init();
    
    manager = new LoRaMesh(myAddr,myDriver);
    manager->onReceiveRequest(onReceiveRequest);

    Serial.println("Free memory left: ");
    Serial.println(freeMemory());
}

void loop() {
  
  Serial.println("Loop starts1");
  manager->run();
  // myDriver->send(0x5678, "Hello World", 11);

  //Serial.println("Free memory left: ");
  //Serial.println(freeMemory());
  //delay(3000);

}
