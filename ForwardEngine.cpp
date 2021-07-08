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

#include "ForwardEngine.h"
//#include "LowPower.h"

volatile bool allowReceiving = true;
uint8_t myRTCInterruptPin;
uint8_t myRTCVccPin;

#define SQW_PIN 2


int value = 0; 
int analogPin = A0;
volatile int lastTime;

#include <Streaming.h>      // http://arduiniana.org/libraries/streaming/


time_t compileTime()
{
    const time_t FUDGE(10);    //fudge factor to allow for upload time, etc. (seconds, YMMV)
    const char *compDate = __DATE__, *compTime = __TIME__, *months = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char compMon[4], *m;

    strncpy(compMon, compDate, 3);
    compMon[3] = '\0';
    m = strstr(months, compMon);

    tmElements_t tm;
    tm.Month = ((m - months) / 3 + 1);
    tm.Day = atoi(compDate + 4);
    tm.Year = atoi(compDate + 7) - 1970;
    tm.Hour = atoi(compTime);
    tm.Minute = atoi(compTime + 3);
    tm.Second = atoi(compTime + 6);

    time_t t = makeTime(tm);
    return t + FUDGE;        //add fudge factor to allow for compile time
}

ForwardEngine::ForwardEngine(byte *addr, DeviceDriver *driver)
{
    myAddr[0] = addr[0];
    myAddr[1] = addr[1];

    myDriver = driver;

    //A node is its own parent initially
    memcpy(myParent.parentAddr, myAddr, 2);
    myParent.hopsToGateway = 255;

    numChildren = 0;
    childrenList = nullptr;

    //Here we will set the random seed to analogRead(A0)
    //The node address can also be used. Interesting to find out if it is better
    //unsigned long seed = myAddr[0] << 8 + myAddr[1];

    //Note: To obtain an arbitary seed, make sure Pin A0 is not connected to anything
    randomSeed(analogRead(A0));

    //TODO: user need to set this through API
    sleepMode = SleepMode::NO_SLEEP;
}

ForwardEngine::~ForwardEngine()
{
    //TODO: need to do some clean up here

    ChildNode *iter = childrenList;
    while (iter != nullptr)
    {
        ChildNode *temp = iter;

        iter = temp->next;
        delete temp;
    }
}

void ForwardEngine::setAddr(byte *addr)
{
    memcpy(myAddr, addr, 2);
}

byte *ForwardEngine::getMyAddr()
{
    return this->myAddr;
}

byte *ForwardEngine::getParentAddr()
{
    return this->myParent.parentAddr;
}

void ForwardEngine::setGatewayReqTime(unsigned long gatewayReqTime)
{
    this->gatewayReqTime = gatewayReqTime;
}

unsigned long ForwardEngine::getGatewayReqTime()
{
    return this->gatewayReqTime;
}

void ForwardEngine::onReceiveRequest(void (*callback)(byte **, byte *))
{
    this->onRecvRequest = callback;
}
void ForwardEngine::onReceiveResponse(void (*callback)(byte *, byte, byte *))
{
    this->onRecvResponse = callback;
}

/**
 * The join function is responsible for sending out a beacon to discover neighboring
 * nodes. After sending out the beacon, the node will receive messages for a given
 * period of time. Since the node might receive multiple replies of its beacon, as well
 * as the beacons from other nearby nodes, it waits for a period of time to collect info
 * from the nearby neighbors, and pick the best parent using the replies received.
 *
 * Returns: True if the node has joined a parent
 */
bool ForwardEngine::join()
{
    if (state != INIT)
    {
        //The node has already joined a network
        return true;
    }

    GenericMessage *msg = nullptr;

    ParentInfo bestParentCandidate = myParent;

    //Serial.print("myAddr = 0x");
    //Serial.print(myAddr[0], HEX);
    //Serial.println(myAddr[1], HEX);
    Join beacon(myAddr, BROADCAST_ADDR);

    //Send out the beacon once to discover nearby nodes
    beacon.send(myDriver, BROADCAST_ADDR);

    //Give some time for the transimission and replying
    //sleepForMillis(500);

    //Serial.print("Wait for reply: timeout = ");
    //Serial.println(DISCOVERY_TIMEOUT);

    unsigned long previousTime = getTimeMillis();

    /**
     * In this loop, for a period of DISCOVERY_TIME, the node will wait for the following types
     * of incoming messages:
     *          1. Beacon ACK (sent by a potential parent)
     *          4. For any other types of message, the node will discard them
     *
     * It is possible that the node did not receive any above messages at all. In this case, the
     * loop will timeout after a period of DISCOVERY_TIMEOUT.
     */
    while ((unsigned long)(getTimeMillis() - previousTime) < DISCOVERY_TIMEOUT)
    {

        //Now try to receive the message
        msg = receiveMessage(myDriver, RECEIVE_TIMEOUT);

        if (msg == nullptr)
        {
            //If no message has been received
            continue;
        }

        byte *nodeAddr = msg->srcAddr;
        // Serial.print("Received msg type = ");
        // Serial.println(msg->type);
        switch (msg->type)
        {
        case MESSAGE_JOIN_ACK:
        {
            Serial.print(F("MESSAGE_JOIN_ACK: src=0x"));
            Serial.print(nodeAddr[0], HEX);
            Serial.print(nodeAddr[1], HEX);
            Serial.print(" rssi=");
            Serial.println(msg->rssi, DEC);

            //If it receives an ACK sent by a potential parent, compare with the current parent candidate
            byte newHopsToGateway = ((JoinAck *)msg)->hopsToGateway;

            if (newHopsToGateway != 255)
            {
                //The remote node has a connection to the gateway
                if (bestParentCandidate.hopsToGateway != 255)
                {
                    //Case 1: Both the current parent candidate and new node are connected to the gateway
                    //Choose the candidate with the minimum hops to the gateway while the RSSI is over the threshold
                    //If the hop counts are the same, pick the one with the best signal strength
                    if (msg->rssi >= RSSI_THRESHOLD)
                    {
                        if (newHopsToGateway < bestParentCandidate.hopsToGateway || (newHopsToGateway == bestParentCandidate.hopsToGateway && msg->rssi > bestParentCandidate.Rssi))
                        {
                            memcpy(bestParentCandidate.parentAddr, nodeAddr, 2);
                            bestParentCandidate.hopsToGateway = newHopsToGateway;
                            bestParentCandidate.Rssi = msg->rssi;

                            Serial.println(F("This is a better parent"));
                        }
                    }
                }
                else
                {
                    //Case 2: Only the new node is connected to the gateway
                    //We always favor the candidate with a connection to the gateway
                    memcpy(bestParentCandidate.parentAddr, nodeAddr, 2);
                    bestParentCandidate.hopsToGateway = newHopsToGateway;
                    bestParentCandidate.Rssi = msg->rssi;
                    Serial.println(F("This is the first new parent"));
                }
            }
            else
            {
                Serial.println(F("The node does not have a path to gateway. Discard"));
            }
            //This case is currently ignored
            /*
                else if (bestParentCandidate.hopsToGateway == -1){
                    //Case 3: Both the current and new parent candidates does not have a connection to the gateway
                    //Compare the node address. The smaller node address should be the parent (gateway address is always larger than regular node address)
                    if(nodeAddr < bestParentCandidate.parentAddr){
                        bestParentCandidate.parentAddr = nodeAddr;
                        bestParentCandidate.hopsToGateway = newHopsToGateway;
                        bestParentCandidate.Rssi = newRssi;
                    }
                }
                */

                //Other cases involve: new node -> not connected to gateway, current best parent -> connected to the gateway
                //In this case we will not update the best parent candidate
            break;
        }
        default:
            //Serial.print("MESSAGE: type=");
            //Serial.print(msg->type, HEX);
            //Serial.print(" src=0x");
            //Serial.println(nodeAddr, HEX);
            break;
        }

        delete msg;
    }

    Serial.println("Discovery timeout");

    if (bestParentCandidate.parentAddr[0] != myAddr[0] || bestParentCandidate.parentAddr[1] != myAddr[1])
    {
        //New parent has found
        Serial.print(F("bestParentCandidate.parentAddr = 0x"));
        Serial.print(bestParentCandidate.parentAddr[0], HEX);
        Serial.println(bestParentCandidate.parentAddr[1], HEX);

        myParent = bestParentCandidate;

        hopsToGateway = bestParentCandidate.hopsToGateway + 0b1;

        Serial.print(F(" HopsToGateway = "));
        Serial.println(hopsToGateway);

        Serial.println(F("Send JoinCFM to parent"));
        //Send a confirmation to the parent node
        JoinCFM cfm(myAddr, myParent.parentAddr, numChildren);

        cfm.send(myDriver, myParent.parentAddr);

        //Assign the alive timestamp to the parent
        myParent.lastAliveTime = getTimeMillis();

        myParent.requireChecking = false;

        return true;
    }
    else
    {
        return false;
    }
}

// bool ForwardEngine::run2()
// {

//     // RTC.set(compileTime());  // set the RTC time and date to the compile time

//     // lastTime = RTC.get();  // get latest RTC time   // check if this can work, using the .get() function

//     // getting current time on RTC
//     tmElements_t currTime;
//     lastTime = RTC.read(currTime);

//     // initialize the alarms to known values, clear the alarm flags, clear the alarm interrupt flags
//     Serial.print("initializing RTC");
//     RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
//     RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
//     RTC.alarm(ALARM_1);
//     RTC.alarm(ALARM_2);
//     RTC.alarmInterrupt(ALARM_1, false);
//     RTC.alarmInterrupt(ALARM_2, false);
//     RTC.squareWave(SQWAVE_NONE);
//     pinMode(0, INPUT_PULLUP);
    
//     while(1)    // while loop will run forever 
//     {

//         // reading sensor value and outputting
//         value = analogRead(A0); // read the analog value from sensor
//         Serial.print("Sensor value: ");
//         Serial.println(value);

//         //Set the time for next RTC alarm
//         //creating variable to just hold when the alarm should happen
//         tmElements_t alarmTime;
//         breakTime(lastTime + 5, alarmTime);
        
//         RTC.setAlarm(ALM1_MATCH_DATE, alarmTime.Second, alarmTime.Minute, alarmTime.Hour, alarmTime.Day); // matches all time up to date
        
//         // clear the alarm flag
//         RTC.alarm(ALARM_1);
//         RTC.alarmInterrupt(ALARM_1, true);

//         noInterrupts();

//         // setting the MCU to sleep 
//         myDriver->enterSleepMode();   // putting trans to sleep
//         Serial.println(F("Put MCU to sleep"));
//         Serial.flush();  // send out all outputs to serial

//         //Put the MCU to sleep and set the interrupt handler
//         myDriver->powerDownMCU();  // turn off

//         set_sleep_mode(SLEEP_MODE_PWR_DOWN);
//         sleep_enable();

//         noInterrupts();

   

//         // We are guaranteed that the sleep_cpu call will be done
//         // as the processor executes the next instruction after
//         // interrupts are turned on.
//         interrupts(); // one cycle
//         sleep_cpu();  // one cycle

//         // When MCU wakes up from here, RTC alarm has indicated the start of a new receiving period
//         RTC.alarm(ALARM_1);

//         // Put the Transceiver back on
//         myDriver->enterTransMode();


//         // get time after the MCU has woken up
//         // this variable will be used to 
//         tmElements_t currTime;
//         lastTime = RTC.read(currTime); // get new time on RTC

//     }
// }


bool ForwardEngine::run2()
{
    
    Serial.print("initializing RTC");
    // RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
    // RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
    RTC.alarm(ALARM_1);
    RTC.alarm(ALARM_2);
    RTC.alarmInterrupt(ALARM_1, false);
    RTC.alarmInterrupt(ALARM_2, false);
    RTC.squareWave(SQWAVE_NONE);
    pinMode(0, INPUT_PULLUP);

    while(1)
    {
        // reading sensor values
        Serial.println(F("Joining unsuccessful. Retry joining in 5 seconds"));
        value = analogRead(A0); // read the analog value from sensor
        Serial.print("Sensor value: ");
        Serial.println(value);
        Serial.println();


        // initializing and setting alarm on rtc
        Serial.print("setting alarm on RTC");
        Serial.println();
        tmElements_t tm;
        time_t timeRN = RTC.get();
        Serial.print("1");
        // Serial.println(timeRN);
        breakTime(timeRN + 10, tm);  // +10 sets the alarm for 10 seconds after
        Serial.print("2");
        RTC.setAlarm(ALM1_MATCH_DATE, tm.Second, tm.Minute, tm.Hour, tm.Day); // matches all time up to date
        
        // clear the alarm flag
        Serial.print("3");
        RTC.alarm(ALARM_1);
        Serial.print("4");
        RTC.alarmInterrupt(ALARM_2, false);
        RTC.alarmInterrupt(ALARM_1, true);
        Serial.print("5");
        noInterrupts();       
        Serial.print("6");

        // noInterrupts();
        attachInterrupt(translateInterruptPin(myRTCInterruptPin), rtcISR, FALLING);
        // EIFR = bit(translateInterruptPin(myRTCInterruptPin));
        interrupts();



        Serial.print("MCU going to sleep");
        digitalWrite(myRTCVccPin, LOW);
        // setting the MCU to sleep 
        myDriver->enterSleepMode();   // putting trans to sleep
        Serial.println(F("Put MCU to sleep"));
        Serial.flush();  // send out all outputs to serial

        //Put the MCU to sleep and set the interrupt handler
        myDriver->powerDownMCU();  // turn off

        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
        sleep_enable();

        noInterrupts();

        // We are guaranteed that the sleep_cpu call will be done
        // as the processor executes the next instruction after
        // interrupts are turned on.
        interrupts(); // one cycle
        sleep_cpu();  // one cycle

        // When MCU wakes up from here, RTC alarm has indicated the start of a new receiving period
        RTC.alarm(ALARM_1);

        // Put the Transceiver back on
        myDriver->enterTransMode();

        Serial.print("MCU woken up now");
        continue;

    }

}



bool ForwardEngine::run()
{

    //If the node is a gateway, it does not have to join the network to operate
    //Gateway is distinguished by the highest bit = 1
    if (myAddr[0] & GATEWAY_ADDRESS_MASK)
    {
        state = JOINED;

        //Gateway has the cost of 0
        hopsToGateway = 0;
    }
    else
    {
        state = INIT;

        //Uninitilized gateway cost
        hopsToGateway = 255;

        //If it is a regular node, it needs to join the network to operate
        while (state == INIT)
        {
            if (join())
            {
                state = JOINED;
            }
            else
            {
                Serial.println(F("Joining unsuccessful. Retry joining in 5 seconds"));
                value = analogRead(A0); // read the analog value from sensor
                Serial.print("Sensor value: ");
                Serial.println(value);

                tmElements_t tm;
                time_t timeRN = RTC.get();
                // Serial.println(timeRN);
                breakTime(timeRN, tm);
                Serial.println();

                Serial.println("setting alarm on RTC");
                Serial.println();

                Serial.print("MCU going to sleep");
                Serial.println();



                sleepForMillis(5000);
            }
        }
    }

    Serial.println(F("Joining successful"));

    //bool checkingParent = false;
    //unsigned long checkingStartTime = 0;

    // the request time starts when Gateway is up
    lastReqTime = getTimeMillis();

    GenericMessage *msg = nullptr;

    bool gatewayReqRecv = false;
    //The core network operations are carried out here
    while (state == JOINED)
    {
        //Currently only support nodes with EByte devices
        //TODO: Work for Adafruit devices
        if (!(myAddr[0] & GATEWAY_ADDRESS_MASK))
        {
            //EbyteDeviceDriver *edriver = (EbyteDeviceDriver *)myDriver;

            if (sleepMode == SleepMode::SLEEP_TRANSCEIVER_INTERRUPT || sleepMode == SleepMode::SLEEP_RTC_INTERRUPT)
            {
                /**
                 * This mode turns off the MCU but always leaves the transceiver on in RX mode.
                 * The MCU is woken up as soon as the transceiver receives a packet
                 */


                if(myDriver->getDeviceType() == DeviceType::ADAFRUIT_LORA){
                    //Dison: I added a noInterrupt() here since Adafruit uses interrupt to receive messages. Thus we
                    //do not want the available() to change when we are checking it.
                    noInterrupts();
                }

                /**
                 * Check the flag in case the RTC alarm already indicates that the receiving should be ended
                 * This is important since if the node goes to sleep and there aren't any other incoming packets,
                 * the node CAN GO TO SLEEP WITH transceiver-interrupt forever until the next incoming packet
                 * (which could be a few hours later if unfortunate)
                 */
                if (myDriver->available() < 1)
                {

                    //Make sure there is no RTC interrupt before going to sleep
                    //Otherwise, we will never wake up
                    noInterrupts();
                    if (allowReceiving)
                    {
                       /** 
                        * If you are using the AdafruitDeviceDriver: 
                        * 
                        * The powerDownMCU() in the AdafruitDeviceDriver is written for 
                        * Adafruit RFM9X LoRa breakout.
                        * 
                        * Currently we do not support Adafruit 32u4 LoRa Feather for
                        * transceiver-based interrupt since its DIO0 pin is inaccessible 
                        *(i.e. The DIO0 pin is connected internally to pin 7 which
                        * does not support hardware-interrupt). At this moment, calling
                        * powerDownMCU() on Adafruit 32u4 LoRa Feather will simply return
                        * and will not put the MCU to sleep.
                        * 
                        * However, if you are able to hard-wire the DIO0 pin on transciever
                        * IC of the Adafruit 32u4 board to any of INT3:0, then you might be
                        * able to do the powerDownMCU() correctly.
                        */ 
                        
                        Serial.println(F("Put MCU to sleep"));

                        //Put the MCU to sleep and set the interrupt handler
                        myDriver->powerDownMCU();

                        /**
                         * Now the MCU has woken up, wait a while for the system to fully start up
                         * Note: this is based on experience, without delays, some bytes will be
                         * lost when we read from software serial
                         */
                        sleepForMillis(50);

                        Serial.println(F("MCU wakes up due to an incoming packet or RTC alarm"));
                        /**
                         * There are two causes that might lead to MCU wakes up here:
                         *   1. An incoming packet is detected
                         *   2. RTC alarm has indicated the end of the receiving period
                         *
                    }
                    else
                    {
                        //RTC interrupt has already occurred
                        interrupts();

                        //Here the receiving period has ended

                        RTC.alarm(ALARM_1);

                        /* DS3231 specification says the PPM is less than 2 which is equivalent to a drift of
                        * ~0.17s per day (Acutal measured PPM is much less). Thus we can wake up the
                        * microcontroller 5 seconds before the expected request. 5s is equivalent to the sum of
                        * of maximum drift on DS3231 for almost 30 days.
                        */

                        // In this case, a gatewayReq has not been received during the receiving period.
                        // Thus we use the time from the last gatewayReq to schedule the next wake-up
                        if (!gatewayReqRecv){
                            // Increment the counter
                            consecutiveMissingReqs ++ ;
                            Serial.println(nextGatewayReqTime);
                            Serial.println(gatewayReqTime);


                            nextGatewayReqTime = nextGatewayReqTime + (time_t)(gatewayReqTime / 1e3);

                            Serial.println(F("No gateway request received during this time period"));
                            
                        }else{
                            // Reset the counter
                            consecutiveMissingReqs = 0;
                        }

                        //Set the time for next RTC alarm
                        tmElements_t tm;
                        breakTime(nextGatewayReqTime - 3, tm);

                        RTC.setAlarm(ALM1_MATCH_DATE, tm.Second, tm.Minute, tm.Hour, tm.Day);

                        Serial.print(F("RTC sleep starts until "));
                        Serial.println(nextGatewayReqTime - 3);
                        sleepForMillis(100);

                        digitalWrite(myRTCVccPin, LOW);
                        myDriver->enterSleepMode();
                        Serial.flush();

                        /*
                        // disable the USB prior going to sleep
                        #if defined (__AVR_ATmega32U4__)
                        // disable the USB prior going to sleep
                        USBCON |= _BV(FRZCLK);  //freeze USB clock
                        PLLCSR &= ~_BV(PLLE);   // turn off USB PLL
                        USBCON &= ~_BV(USBE);   // disable USB
                        //LowPower.powerDown(SLEEP_FOREVER,ADC_OFF,BOD_OFF);
                        #endif
                        */

                        // disable ADC
                        byte adc_state = ADCSRA;
                        ADCSRA = 0;

                        set_sleep_mode(SLEEP_MODE_PWR_DOWN);
                        sleep_enable();

                        // Do not interrupt before we go to sleep, or the
                        // ISR will detach interrupts and we won't wake.
                        noInterrupts();

                        EIFR = bit(translateInterruptPin(myRTCInterruptPin)); // clear flag for interrupt 2 on Pin 0

                        #if defined (__AVR_ATmega328P__) || defined (__AVR_ATmega168P__)
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


                        ADCSRA = adc_state;
                        //Now the MCU has woken up, wait a while for the system to fully start up
                        sleepForMillis(50);

                        /*
                        #if defined (__AVR_ATmega32U4__)
                        sleepForMillis(100);
                        USBDevice.attach(); // keep this
                        sleepForMillis(100);
                        Serial.begin(9600);
                        sleepForMillis(100);
                        #endif
                        */
                        // When MCU wakes up from here, RTC alarm has indicated the start of a new receiving period
                        RTC.alarm(ALARM_1);

                        // Put the Transceiver back on
                        myDriver->enterTransMode();

                        // Fault detection
                        if (firstGatewayContact && (float)consecutiveMissingReqs > NEXT_GATEWAY_REQ_TIME_TOLERANCE_FACTOR ){
                            state = INIT;
                            Serial.print(F("No message has been received for "));
                            Serial.print(consecutiveMissingReqs);
                            Serial.println(F(" consecutive receiving periods"));
                            
                            // Reset the fault-detection variables
                            firstGatewayContact = false;
                            consecutiveMissingReqs = 0;
                            allowReceiving = true;

                            //We have disconnected from the parent
                            myParent.parentAddr[0] = myAddr[0];
                            myParent.parentAddr[1] = myAddr[1];
                            myParent.hopsToGateway = 255;
                            return 1;
                        }

                        time_t receivingPeriodEnd = RTC.get() + receivingPeriod;
                        breakTime(receivingPeriodEnd, tm);
                        
                        tmElements_t new_tm;
                        RTC.read(new_tm);
                        // Serial.print(F("Current Day: "));
                        // Serial.print(new_tm.Day);
                        // Serial.print(F(" Current Hour: "));
                        // Serial.print(new_tm.Hour);
                        // Serial.print(F(" Current Minutes: "));
                        // Serial.print(new_tm.Minute);
                        // Serial.print(F(" Current Seconds: "));
                        // Serial.println(new_tm.Second);

                        // Serial.print(F("Matching Day: "));
                        // Serial.print(tm.Day);
                        // Serial.print(F(" Matching Hour: "));
                        // Serial.print(tm.Hour);
                        // Serial.print(F(" Matching Minutes: "));
                        // Serial.print(tm.Minute);
                        // Serial.print(F(" Matching Seconds: "));
                        // Serial.println(tm.Second);

                        //Set up the alarm for the end of the receiving period
                        RTC.setAlarm(ALM1_MATCH_DATE, tm.Second, tm.Minute, tm.Hour, tm.Day);
                        sleepForMillis(100);
                        Serial.println(F("RTC Off"));
                        digitalWrite(myRTCVccPin, LOW);

                        // Just to make sure allowReceiving is set to true
                        if (!allowReceiving)
                        {
                            allowReceiving = true;
                        }

                        Serial.print(F("Wake up from RTC sleep. Receive for "));
                        Serial.print(receivingPeriod);
                        Serial.print(F(" seconds until "));
                        Serial.println(receivingPeriodEnd);

                        //Clear gatewayReq flag
                        gatewayReqRecv = false;

                        /**
                         * We did not make MCU sleep here. Since we are waking up 3 seconds before the
                         * next request, no message will arrive now (i.e. all the following packet parsing
                         * code will be skipped). It will then go to the start of the loop and enters MCU
                         * sleep automatically. 
                         * 
                         * In rare cases where the request comes in right now (may due to a problem in time
                         * synchronization), the MCU receives the packet first before goes to sleep.
                         */ 
                    }
                }
            }
        }
        msg = receiveMessage(myDriver, RECEIVE_TIMEOUT);
        if (msg != nullptr)
        {

            byte *nodeAddr = msg->srcAddr;

            //Based on the received message, do the corresponding actions
            switch (msg->type)
            {
            case MESSAGE_JOIN:
            {
                //If a join message comes from the parent node, it suggests that the parent node has
                //disconnected from the gateway, do not reply back with a JoinACK
                if (msg->srcAddr[0] == myParent.parentAddr[0] && msg->srcAddr[1] == myParent.parentAddr[1])
                {
                    Serial.println(F("Parent node has disconnected from the gateway"));
                }
                else
                {
                    /*
                    //DEBUG code for testing: For gateway only accept node 0xA0 and 0xA1
                    if (myAddr[0] & GATEWAY_ADDRESS_MASK)
                    {
                        if (msg->srcAddr[1] > 0xA1)
                        {
                            break;
                        }
                    }
                    */

                    JoinAck ack(myAddr, nodeAddr, hopsToGateway);

                    // Introduce some random time backoff to prevent collision
                    // From our experiments, we noticed packet losses when multiple nodes send joinACK instantly
                    // upon receiving a join message. This cause some packets to go missing (Even LBT in EBYTE can
                    // not help since the sending happened at almost the same time)

                    long backoff = random(MIN_BACKOFF_TIME, MAX_JOIN_ACK_BACKOFF_TIME);

                    Serial.print(F("Sleep for some time before sending JoinAck: "));
                    Serial.println(backoff);

                    sleepForMillis(backoff);

                    ack.send(myDriver, nodeAddr);

                    Serial.print(F("MESSAGE_JOIN: src=0x"));
                    Serial.print(nodeAddr[0], HEX);
                    Serial.println(nodeAddr[1], HEX);
                }

                break;
            }
            case MESSAGE_JOIN_CFM:
            {
                ChildNode *iter = childrenList;

                while (iter != nullptr)
                {
                    if (iter->nodeAddr[0] == msg->srcAddr[0] && iter->nodeAddr[1] == msg->srcAddr[1])
                    {
                        break;
                    }
                    iter = iter->next;
                }
                // If the child node has already been in the children list (i.e. it reconnects to this
                // parent node), do not add it to the list
                if (iter != nullptr)
                {
                    break;
                }
                //Add the new child to the linked list (Insert at the beginning of the linked list)

                ChildNode *node = new ChildNode();
                node->nodeAddr[0] = msg->srcAddr[0];
                node->nodeAddr[1] = msg->srcAddr[1];

                node->next = childrenList;

                childrenList = node;

                numChildren++;

                Serial.print(F("A new child has joined: 0x"));
                Serial.print(nodeAddr[0], HEX);
                Serial.println(nodeAddr[1], HEX);
                break;
            }
            //Dixin update: we will replace the "Aliveness checking" with the GatewayReq
            /*
            case MESSAGE_REPLY_ALIVE:
            {
                //We do not need to check the message src address here since the parent should
                //only unicast the reply message (Driver does the filtering).
                Serial.println(F("ReplyAlive from parent"));
                //If we have previously issued a checkAlive message
                if (myParent.requireChecking)
                {
                    //The parent node is proven to be alive
                    myParent.requireChecking = false;

                    //Update the last time we confirmed when the parent node was alive
                    myParent.lastAliveTime = getTimeMillis();
                }
                break;
            }
            case MESSAGE_CHECK_ALIVE:
            {
                //Parent replies back to the child node
                Serial.println("I got checked by my child node");
                ReplyAlive reply(myAddr, nodeAddr);
                reply.send(myDriver, nodeAddr);
                break;
            }
            */
            case MESSAGE_GATEWAY_REQ:
            {
                //Dixin Wu update: if we broadcast the gatewayReq, we should only accept REQ from the parent
                if (msg->srcAddr[0] != myParent.parentAddr[0] || msg->srcAddr[1] != myParent.parentAddr[1])
                {
                    //If the message does not come from the parent node
                    Serial.println(F("Req is not received from parent. Ignore."));
                    break;
                }

                //This shouldn't happen, but in case gateway should ignore this message
                if (myAddr[0] & GATEWAY_ADDRESS_MASK)
                {
                    break;
                }
                else
                {
                    time_t receivingPeriodStart;

                    time_t receivingPeriodEnd;

                    if (sleepMode == SleepMode::SLEEP_RTC_INTERRUPT)
                    {
                        receivingPeriodStart = RTC.get();
                    }
                    // we know our parent is alive
                    myParent.requireChecking = false;
                    myParent.lastAliveTime = getTimeMillis();

                    maxBackoffTime = ((GatewayRequest *)msg)->childBackoffTime;
                    Serial.print(F("New maximum backoff time: "));
                    Serial.println(maxBackoffTime);

                    // backoff to avoid collision
                    unsigned long backoff = random(MIN_BACKOFF_TIME, maxBackoffTime);
                    Serial.print(F("Sleep for some time before replying back: "));
                    Serial.println(backoff);

                    sleepForMillis(backoff);

                    // Use callback to get node data
                    byte *nodeData = new byte[MAX_LEN_DATA_NODE_REPLY]; //magic number 64 comes from MP comment
                    byte dataLength;
                    if (onRecvRequest)
                        onRecvRequest(&nodeData, &dataLength);

                    // Dixin update: First send reply to the parent
                    NodeReply nReply(myAddr, myParent.parentAddr, ((GatewayRequest *)msg)->seqNum, dataLength, nodeData);
                    nReply.send(myDriver, myParent.parentAddr);

                    delete[] nodeData;

                    // Dixin update: Get the expected time for the next gateway request
                    gatewayReqTime = ((GatewayRequest *)msg)->nextReqTime;
                    Serial.print(F("Next req will be in "));
                    Serial.println(gatewayReqTime);

                    if (numChildren > 0)
                    {
                        // Dixin update: Other children of the parent will finish transmitting after 3 seconds, so it is better to
                        // wait until all of them finished transmitting before forwarding the messages
                        unsigned long remainingTime = maxBackoffTime - backoff;
                        backoff = random(remainingTime, remainingTime + maxBackoffTime);
                        sleepForMillis(backoff);

                        unsigned long childBackoffTime = numChildren * MAX_BACKOFF_TIME_FOR_ONE_CHILD;

                        if (childBackoffTime > gatewayReqTime)
                        {
                            childBackoffTime = gatewayReqTime;
                        }

                        Serial.print(F("Max backoff time for child nodes: "));
                        Serial.println(childBackoffTime);

                        //Dixin Wu update: We simply broadcast the gatewayReq
                        GatewayRequest gwReq(myAddr, BROADCAST_ADDR, ((GatewayRequest *)msg)->seqNum, gatewayReqTime, childBackoffTime);
                        gwReq.send(myDriver, BROADCAST_ADDR);
                    }

                    if (sleepMode == SleepMode::SLEEP_RTC_INTERRUPT)
                    {
                        //Estimate the time when the next request will arrive
                        nextGatewayReqTime = receivingPeriodStart + (time_t)(gatewayReqTime / 1e3);

                        /*
                        * For less frequent data gathering every 20 minutes or more (e.g. every hours), only receive for 10 minutes
                        * For more frequent data gathering every 20 minutes or less (e.g. every 5 minutes), receive for 50% of the request interval
                        */
                        if (gatewayReqTime <= 1200e3)
                        {
                            //Calculate the end of the receiving period
                            receivingPeriod = (time_t)(gatewayReqTime / 1e3 / 2);
                        }
                        else
                        {
                            receivingPeriod = 600;
                        }

                        receivingPeriodEnd = receivingPeriodStart + receivingPeriod;

                        Serial.print(F("Receiving period: "));
                        Serial.print(receivingPeriodStart);
                        Serial.print(" to ");
                        Serial.println(receivingPeriodEnd);

                        /*
                         * If there is already an alarm set up (usually for fault detection), the following code
                         * will replace it with a new alarm, which is t seconds (e.g. 60s) after the gateway received
                         * request. This way we can guarantee that the node will be receiving for t seconds after the
                         * gateway request regardless of when it actually woke up; It avoids the rare case where the
                         * gateway request arrives very late in the expected receiving period due to delays and the 
                         * node can "prematurely" go to sleep.
                         */ 

                        tmElements_t tm;

                        breakTime(receivingPeriodEnd, tm);

                        //Clear the alarm in case there is one
                        RTC.alarm(ALARM_1);

                        //Set up the alarm
                        RTC.setAlarm(ALM1_MATCH_MINUTES, tm.Second, tm.Minute, 0, 0);

                        // Indicate that we have received a gatewayReq during the receiving period
                        gatewayReqRecv = true;

                        /*
                        * The node needs to set the RTC alarm when it first time contacts with the gateway
                        */
                        if (!firstGatewayContact)
                        {
                            Serial.println(F("First time gateway REQ"));
                            // the first request has been received from the gateway
                            firstGatewayContact = true;

                            //Set receiving flag to be true
                            allowReceiving = true;

                            // Attach the interrupt
                            // The reason we do not attach the interrupt at the beginning is that 
                            // there could be previous alarm in the RTC.
                            noInterrupts();
                            attachInterrupt(translateInterruptPin(myRTCInterruptPin), rtcISR, FALLING);
                            EIFR = bit(translateInterruptPin(myRTCInterruptPin));
                            interrupts();    
                        }

                    }
                }
                break;
            }
            case MESSAGE_NODE_REPLY:
            {
                // Gateway should handle this
                if (myAddr[0] & GATEWAY_ADDRESS_MASK)
                {
                    // Should be what gateway is waiting for
                    if (((NodeReply *)msg)->seqNum != seqNum)
                    {
                        Serial.print(F("Warning: Gateway got wrong seqNum: "));
                        Serial.print(((NodeReply *)msg)->seqNum);
                        Serial.print(F("  It should be: "));
                        Serial.println(seqNum);
                    }

                    // Gateway should use a callback to process the data
                    Serial.print(F("Node Reply Sequence number: "));
                    Serial.println(((NodeReply *)msg)->seqNum);
                    if (onRecvResponse)
                        onRecvResponse(((NodeReply *)msg)->data, ((NodeReply *)msg)->dataLength, ((NodeReply *)msg)->srcAddr);
                }
                // Node should forward this up to its parent
                else
                {
                    //TODO: Dixin update -> should delay a bit here instead of sending immediately
                    NodeReply nReply(msg->srcAddr, myParent.parentAddr, ((NodeReply *)msg)->seqNum, ((NodeReply *)msg)->dataLength, ((NodeReply *)msg)->data);

                    // backoff to avoid collision
                    long backoff = random(MIN_BACKOFF_TIME, maxBackoffTime);
                    Serial.print(F("Sleep for some time before forwarding: "));
                    Serial.println(backoff);
                    sleepForMillis(backoff);

                    nReply.send(myDriver, myParent.parentAddr);
                }
                break;
            }
            }

            delete msg;
        }

        //Serial.print(F("Free Memory: "));
        //Serial.println(freeMemory());

        unsigned long currentTime = getTimeMillis();
        //The gateway does not need to check its parent
        if (myAddr[0] & GATEWAY_ADDRESS_MASK)
        {
            /*
            // prepare to send out request
            if (gatewayReqTime == 0)
            {
                Serial.println("Gateway reqtime is 0 (not set)");
                continue;
            }
            */
            if ((unsigned long)(currentTime - lastReqTime) >= gatewayReqTime)
            {
                // request data from all children
                seqNum += 1;
                lastReqTime = currentTime;

                unsigned long childBackoffTime = numChildren * MAX_BACKOFF_TIME_FOR_ONE_CHILD;

                /** If there are many child nodes and the time interval between requests are much
                 * less than the child backoff time calculated, this can result into asynchronous
                 * requests and replies (e.g. Replies with seq number 5 arrives after request with
                 * seq number 10 has been issued)
                 *
                 * Thus, if the child backoff time is greater than the gateway request time interval,
                 * the backoff time will be at least set to the gateway request time interval. Note
                 * that the asynchronous replies and requests can still occur as the tree has multiple
                 * hierarchies, but hopefully it prevents some extreme cases where the calculated
                 * backoff time is multiple times of the gatewayReqTime.
                 *
                 * In practice, the problem hardly occurs as the gatewayReqTime is usually set to hours
                 * and there are not so many nodes connected to the gateway.
                 */
                if (childBackoffTime > gatewayReqTime)
                {
                    childBackoffTime = gatewayReqTime;
                }

                Serial.print(F("Now Gateway sends out request: SeqNum="));
                Serial.print(seqNum);
                Serial.print(F(", Next Request Time="));
                Serial.print(gatewayReqTime);
                Serial.print(F(", Child Backoff Time="));
                Serial.println(childBackoffTime);

                //Dixin Wu update: what if we simply broadcast the gatewayReq
                GatewayRequest gwReq(myAddr, BROADCAST_ADDR, seqNum, gatewayReqTime, childBackoffTime);
                gwReq.send(myDriver, BROADCAST_ADDR);
            }
        }
        //For regular nodes, check whether a gatewayReq has arrived during the expected time interval
        //Note: if RTC-based sleep is used, we use a different mechanism for fault detection
        else if (sleepMode == SleepMode::NO_SLEEP && (unsigned long)(currentTime - myParent.lastAliveTime) > NEXT_GATEWAY_REQ_TIME_TOLERANCE_FACTOR * gatewayReqTime)
        {
            //This means that the node has not received any gatewayReqs from its parent which it should has received if the connection is still up

            state = INIT;
            Serial.println(F("No message has been received for the time period"));
        }

        //Dixin update: we will replace the "Aliveness checking" with the GatewayReq
        /*
        //The parent is currently being checked (CheckAlive Message has been sent already), but the reply has not been received yet
        if (myParent.requireChecking)
        {
            //If the reply has not been received in 10 seonds
            if (currentTime - checkingStartTime >= CHECK_ALIVE_TIMEOUT)
            {
                state = INIT;
                Serial.println(F("CheckAlive Timeout"));
                //loop will break
            }
        }
        //If the parent is not being checked and has not been checked in the past 30 seconds, we might need to check the parent liveness
        else if ((unsigned long)(currentTime - myParent.lastAliveTime) >= checkAliveInterval)
        {
            Serial.println(F("Time to check parent"));
            myParent.requireChecking = true;
            //Send out the checkAlive message to the parent
            CheckAlive checkMsg(myAddr, myParent.parentAddr, 0);
            checkMsg.send(myDriver, myParent.parentAddr);

            //record the current time
            checkingStartTime = getTimeMillis();

            Serial.print(F("Checking start at "));
            Serial.println(checkingStartTime);
        }*/
    }

    //We have disconnected from the parent
    myParent.parentAddr[0] = myAddr[0];
    myParent.parentAddr[1] = myAddr[1];
    myParent.hopsToGateway = 255;

    return 1;
}

void ForwardEngine::setSleepMode(uint8_t sleepMode, uint8_t rtcInterruptPin, uint8_t rtcVccPin)
{

    if (sleepMode == SleepMode::SLEEP_RTC_INTERRUPT)
    {
        myRTCVccPin = rtcVccPin;
        pinMode(myRTCVccPin, OUTPUT);

        // Turn on the RTC
        digitalWrite(myRTCVccPin, HIGH);

        // Wait for RTC to start up
        sleepForMillis(500);

        tmElements_t tm;
        //If the user intends to use RTC-based interrupt/sleep mode
        //Make sure that a RTC is properly connected to the microcontroller
        if (RTC.read(tm) != 0)
        {
            Serial.println(F("Error: Unable to set RTC-based interrupt. I2C error with the RTC."));
            return;
        }
        // The checking of interrupt pins does not work on Adafruit 32u4
        
        else if (translateInterruptPin(rtcInterruptPin) == NOT_AN_INTERRUPT)
        {
            Serial.println(F("Error: RTC interrupt (SQW) has to be connected to a valid interrupt pin"));
            return;
        }
        

        myRTCInterruptPin = rtcInterruptPin;
        //Initialize the RTC module with Alarm1
        RTC.alarm(ALARM_1);
        RTC.squareWave(SQWAVE_NONE);
        RTC.alarmInterrupt(ALARM_1, true);
        RTC.alarmInterrupt(ALARM_2, false);
        pinMode(myRTCInterruptPin, INPUT_PULLUP);
    }

    this->sleepMode = sleepMode;
    Serial.print(F("SleepMode set to: "));
    Serial.println(sleepMode);
}

void rtcISR()
{
    allowReceiving = !allowReceiving;
    digitalWrite(myRTCVccPin, HIGH);
}