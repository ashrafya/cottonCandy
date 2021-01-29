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


#ifndef HEADER_ADAFRUIT_DEVICE_DRIVER
#define HEADER_ADAFRUIT_DEVICE_DRIVER

#include "Arduino.h"
#include "DeviceDriver.h"
#include "SoftwareSerial.h"
#include <SPI.h>
#include <LoRa.h>
#include <avr/sleep.h>
#include "Utilities.h"

/* for feather32u4 */
#define RFM95_CS 8
#define RFM95_RST 4
#define RFM95_INT 7
#define RF95_FREQ 915E6

#define MSG_QUEUE_CAPACITY 255

#define DEFAULT_SPREADING_FACTOR 7
#define DEFAULT_CHANNEL_BW 125E3
#define DEFAULT_CODING_RATE_DENOMINATOR 5

class AdafruitDeviceDriver : public DeviceDriver
{
public:

  /**
   * By default, the software assumes that you are using Adafruit 32u4 LoRa Feather and you do not need to provide the
   * pin mappings explicitly.
   * 
   * However, in case you are using the Adafruit Lora RFM9x Breakout board with a seperate microcontroller, you need to 
   * provide the pin mappings.
   */
  AdafruitDeviceDriver(byte *addr, uint8_t csPin = RFM95_CS, uint8_t rstPin = RFM95_RST, uint8_t intPin = RFM95_INT);

  ~AdafruitDeviceDriver();

  /**
   * Use the default configuration to initilize the LoRa parameters
   * 
   * Frequency = 915 MHz; Spreading factor = 7; Channel bandwith = 125kHz; Coding Rate = 4/5
   * 
   */ 
  bool init();

  bool init(long frequency, uint8_t sf, long bw, uint8_t cr);

  int send(byte *destAddr, byte *msg, long msgLen);

  byte recv();

  int available();

  int getLastMessageRssi();

  int getDeviceType();

  void enterSleepMode();

  void enterTransMode();

  void powerDownMCU();

private:
  byte addr[2];
  long freq;
  uint8_t sf;
  long channelBW;
  uint8_t codingRate;

  uint8_t irqPin;

  /*-----------Module Registers Configuration-----------*/
  void setAddress(byte *addr);
  void setFrequency(long frequency);
  void setSpreadingFactor(uint8_t sf);
  void setChannelBandwidth(long bw);
  void setCodingRateDenominator(uint8_t cr);
};

#endif
