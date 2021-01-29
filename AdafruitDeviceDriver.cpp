/*    
    Copyright 2020, Network Research Lab at the University of Toronto.

    This file is part of CottonCandy.

    CottonCandy is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    CottonCandy is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU Lesser General Public License
    along with CottonCandy.  If not, see <https://www.gnu.org/licenses/>.
*/

#include "AdafruitDeviceDriver.h"

#define DEBUG 1

byte msgQueue[MSG_QUEUE_CAPACITY];
uint8_t queueHead; //next index to read;
uint8_t queueTail; //next index to write
uint8_t queueSize;

byte *adafruitAddr;

AdafruitDeviceDriver::AdafruitDeviceDriver(byte *addr,
                                           uint8_t csPin, uint8_t rstPin, uint8_t intPin) : DeviceDriver()
{
    
    LoRa.setPins(csPin, rstPin, intPin);
    
    setAddress(addr);

    this->irqPin = intPin;

    queueHead = 0;
    queueTail = 0;
    queueSize = 0;
}

AdafruitDeviceDriver::~AdafruitDeviceDriver()
{
}

void onReceive(int packetSize)
{
    packetSize -= 2;
    if (queueSize + packetSize > MSG_QUEUE_CAPACITY)
    {
        return;
    }
    byte add0 = LoRa.read();
    byte add1 = LoRa.read();

    if ((adafruitAddr[0] != add0 ||
         adafruitAddr[1] != add1) &&
        !(add0 == 0xFF && add1 == 0xFF))
    {
        // Serial.println("Addr unmatched, packet is dropped");
        return;
    }
    // Serial.print("onReceive to queue, size: ");
    // Serial.println(packetSize);
    // Serial.print("Before, queue size: ");
    // Serial.println(queueSize);
    while (LoRa.available())
    {
        byte result = LoRa.read();
        // Serial.print("0x");
        // Serial.print(result,HEX);
        // Serial.print(" ");
        queueSize++;
        msgQueue[queueTail] = result;
        queueTail = (queueTail + 1) % MSG_QUEUE_CAPACITY;
    }
    //     Serial.println("");
    //     Serial.print("After, queue end: ");
    //     Serial.println(queueSize);
}

bool AdafruitDeviceDriver::init()
{
    return this->init(RF95_FREQ,DEFAULT_SPREADING_FACTOR,DEFAULT_CHANNEL_BW,DEFAULT_CODING_RATE_DENOMINATOR);
}

bool AdafruitDeviceDriver::init(long frequency, uint8_t sf, long bw, uint8_t cr)
{
    // Assign class member variables
    setFrequency(frequency);
    setSpreadingFactor(sf);
    setChannelBandwidth(bw);
    setCodingRateDenominator(cr);

    if (!LoRa.begin(freq))
    {
        Serial.println(F("Starting LoRa failed!"));
        return false;
    }

    // Assign the parameters to the actual LoRa module which writes to the hardware register
    LoRa.setSpreadingFactor(sf);
    LoRa.setSignalBandwidth(channelBW);
    LoRa.setCodingRate4(codingRate);
    // LoRa.setTxPower(23);

    LoRa.onReceive(onReceive);
    LoRa.receive();
    Serial.println(F("LoRa Module initialized"));

    return true;
}

int AdafruitDeviceDriver::send(byte *destAddr, byte *msg, long msgLen)
{
    byte data[2 + msgLen];
    memcpy(data, destAddr, 2);
    // data[2] = (byte)myChannel;

    memcpy(data + 2, msg, msgLen);

    LoRa.beginPacket();
    LoRa.write(data, (size_t)(msgLen + 2));
    int result = LoRa.endPacket(false) == 1 ? 1 : -1;
    LoRa.receive();
    return result;
}

byte AdafruitDeviceDriver::recv()
{
    if (available())
    {
        byte result = msgQueue[queueHead];
        //Serial.print("Received: 0x");
        //Serial.print(result,HEX);
        //Serial.println("");
        queueHead = (queueHead + 1) % MSG_QUEUE_CAPACITY;
        queueSize--;
        return result;
    }
    else
    {
        return -1;
    }
}
int AdafruitDeviceDriver::available()
{
    return queueSize;
}
int AdafruitDeviceDriver::getLastMessageRssi()
{

    int result = LoRa.packetRssi();
    return result;
}

/*-----------LoRa Configuration-----------*/
void AdafruitDeviceDriver::setAddress(byte *addr)
{

    if (sizeof(addr) < 2)
    {
        Serial.println("Error: Node address must be 2-byte long");
    }
    else
    {
        this->addr[0] = addr[0];
        this->addr[1] = addr[1];
        adafruitAddr = this->addr;
    }
}

void AdafruitDeviceDriver::setFrequency(long frequency)
{
    this->freq = frequency;
}

void AdafruitDeviceDriver::setSpreadingFactor(uint8_t sf)
{
    this->sf = sf;
}

void AdafruitDeviceDriver::setChannelBandwidth(long bw)
{
    this->channelBW = bw;
}

void AdafruitDeviceDriver::setCodingRateDenominator(uint8_t cr)
{
    this->codingRate = cr;
}

int AdafruitDeviceDriver::getDeviceType()
{
    return DeviceType::ADAFRUIT_LORA;
}

void AdafruitDeviceDriver::enterSleepMode()
{
    LoRa.sleep();
}

void AdafruitDeviceDriver::enterTransMode()
{
    LoRa.receive();
}

void wakeISR(){
    sleep_disable();
}

void AdafruitDeviceDriver::powerDownMCU(){

    int interruptNumber = translateInterruptPin(irqPin);

    if(interruptNumber == NOT_AN_INTERRUPT){
        Serial.println(F("Refuse to enter sleep: The IRQ pin is not connected to an valid interrupt pin"));
        return;
    }

   /** This feature does not seem to work on Adafruit 32u4 Feather board where the transceiver IRQ is internally
    * connected to pin 7 (INT4).
    * 
    * Reference:
    * 1. https://learn.adafruit.com/adafruit-feather-32u4-radio-with-lora-radio-module/pinouts
    * 2. Page 43 Section 7.3 "Power-down Mode" in the Atmega16u4/32u4 data sheet.
    */
    #if defined (__AVR_ATmega32U4__)
    if(interruptNumber == INTF4){
        Serial.println(F("Refuse to enter sleep: INT4 on 32u4 cannot wake up the MCU from power-down mode"));
        return;
    }
    #endif

    //Make sure the debugging messages are printed correctly before goes to sleep
    Serial.flush();

    byte adc_state = ADCSRA;
    // disable ADC
    ADCSRA = 0;

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();

    // Do not interrupt before we go to sleep, or the
    // ISR will detach interrupts and we won't wake.
    noInterrupts();
    
    EIFR = bit(interruptNumber); // clear flag for transceiver-based interrupt

    // Software brown-out only works in ATmega328p
    #if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168P__)
    // turn off brown-out enable in software
    // BODS must be set to one and BODSE must be set to zero within four clock cycles
    MCUCR = bit(BODS) | bit(BODSE);
    // The BODS bit is automatically cleared after three clock cycles
    MCUCR = bit(BODS);
    #endif

    // We are guaranteed that the sleep_cpu call will be done
    // as the processor executes the next instruction after
    // interrupts are turned on.
    interrupts(); // one cycle
    sleep_cpu();  // one cycle
    //The MCU is turned off after this point

    //Restore the ADC
    ADCSRA = adc_state;
}