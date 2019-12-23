#include "EbyteDeviceDriver.h"

#define DEBUG 1

EbyteDeviceDriver::EbyteDeviceDriver(uint8_t rx, uint8_t tx, uint8_t m0, uint8_t m1, uint8_t aux_pin, address addr, 
                                    ) : DeviceDriver(){
    this->rx = rx;
    this->tx = tx;
    this->m0 = m0;
    this->m1 = m1;
    this->aux_pin = aux_pin;
    module = new SoftwareSerial(rx, tx);

    myAddr = addr;
    myChannel = channel;
}

EbyteDeviceDriver::~EbyteDeviceDriver(){
    delete module;
}

bool EbyteDeviceDriver::init(){

    pinMode(this->m0, OUTPUT);
    pinMode(this->m1, OUTPUT);
    pinMode(this->aux_pin, INPUT);
    Serial.println("LoRa Module Pins initialized");

    while (digitalRead(this->aux_pin) != HIGH)
    {
        Serial.println("Waiting for LoRa Module to initialize");
        delay(10);
    }

    module->begin(BAUD_RATE);
    Serial.println("LoRa Module initialized");

    enterConfigMode();
    setAddress(myAddr);
    setChannel(myChannel);
    setNetId(0x00);
    setOthers(0x40);

    enterTransMode();
    Serial.println("Enter Transmission Mode");
    return true;
}
/**
 * Here we are using the fixed transmission feature in Ebyte. Thus, for every outgoing message, we
 * need to append a header indicating the destination address and channel.
 * 
 * The reason to use fixed transmission rather than transparent transmission is that fixed transmission
 * enables hardware address filtering in the Ebyte transceiver. Also broadcast can easily be done by
 * setting dest address to FFFF;
 */ 
int EbyteDeviceDriver::send(address destAddr, char* msg, long msgLen){
    char data[3 + msgLen];
    data[0] = (destAddr >> 8) & 0xFF;
    data[1] = destAddr & 0xFF;
    data[2] = (char)myChannel;
    strncpy(data + 3, msg, msgLen);
    return (module->write(data, sizeof(data)));
}


char EbyteDeviceDriver::recv(){
   if(module->available()){
       return (module->read());
   }
   else{
       return -1;
   }
}

void EbyteDeviceDriver::enterConfigMode()
{
    digitalWrite(this->m0, LOW);
    digitalWrite(this->m1, HIGH);
    //Need to wait for the configuration to be in effect
    delay(1000);
    //Make Sure the AUX is now in HIGH state
    while (digitalRead(this->aux_pin) != HIGH)
    {
    }
    Serial.println("Successfully entered CONFIGURATION mode");
    currentMode = Mode::CONFIG;
}

void EbyteDeviceDriver::enterTransMode()
{
    digitalWrite(this->m0, LOW);
    digitalWrite(this->m1, LOW);
    //Need to wait for the configuration to be in effect
    delay(1000);

    //Make Sure the AUX is now in HIGH state
    while (digitalRead(this->aux_pin) != HIGH)
    {
    }
    Serial.println("Successfully entered TRANSMISSION mode");
    currentMode = Mode::TRANSMIT;
}

void EbyteDeviceDriver::enterWorMode()
{
    digitalWrite(this->m0, HIGH);
    digitalWrite(this->m1, LOW);

    //Need to wait for the configuration to be in effect
    delay(1000);
    //Make Sure the AUX is now in HIGH state
    while (digitalRead(this->aux_pin) != HIGH)
    {
    }
    Serial.println("Successfully entered WOR mode");
    currentMode = Mode::WOR;
}

void EbyteDeviceDriver::enterSleepMode()
{
    digitalWrite(this->m0, HIGH);
    digitalWrite(this->m1, HIGH);

    //Need to wait for the configuration to be in effect
    delay(1000);
    //Make Sure the AUX is now in HIGH state
    while (digitalRead(this->aux_pin) != HIGH)
    {
    }
    Serial.println("Successfully entered SLEEP mode");
    currentMode = Mode::SLEEP;
}

uint8_t EbyteDeviceDriver::getCurrentMode()
{
    return this->currentMode;
}

/*-----------LoRa Configuration-----------*/
void EbyteDeviceDriver::setAddress(address addr)
{
    module->write(0xC0);
    module->write((char)0x00);
    module->write(0x02);
    module->write((addr >> 8) & 0xFF);
    module->write(addr & 0xFF);

    //Block and read the reply to clear the buffer
    receiveConfigReply(5);
    
    if(DEBUG){
        Serial.print("Successfully set Address to ");
        Serial.print(addr);
        Serial.print("\n");
    }
}

void EbyteDeviceDriver::setNetId(uint8_t netId)
{
    module->write(0xC0);
    module->write(0x02);
    module->write(0x01);
    module->write((byte)netId);

    //Read the reply to clear the buffer
    receiveConfigReply(4);

    if(DEBUG){
        Serial.print("Successfully set Net Id to ");
        Serial.print(netId);
        Serial.print("\n");
    }
}

void EbyteDeviceDriver::setChannel(uint8_t channel)
{
    module->write(0xC0);
    module->write(0x05);
    module->write(0x01);
    module->write((byte)channel);

    //Read the reply to clear the buffer
    receiveConfigReply(4);

    if(DEBUG){
        //Frequency = 410.125 MHz + channel * 1 MHz
        Serial.print("Successfully set Channel to ");
        Serial.print(channel);
        Serial.print("\n");
    }
}

void EbyteDeviceDriver::setOthers(byte config)
{
    module->write(0xC0);
    module->write(0x06);
    module->write(0x01);
    module->write((byte)config);

    //Read the reply to clear the buffer
    receiveConfigReply(4);
    if(DEBUG){
        Serial.println("Successfully set other configs");
    }
}

void EbyteDeviceDriver::receiveConfigReply(int replyLen)
{
    int bytesRead = 0;
    while (bytesRead < replyLen)
    {
        if (module->available())
        {
        byte b = module->read();
        //Serial.print(b,HEX);
        bytesRead++;
        }
    }
    //Serial.print("\n");
}