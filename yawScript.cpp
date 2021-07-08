
// #include <yawScript.h>

// #define interruptPin 1


// bool ForwardEngine::run3()
// {
//     Serial.println("Beginning time based reading routine");

//     // Keep pins high until we ground them
//     pinMode(interruptPin, INPUT_PULLUP);
//     RTC.clearAlarm(ALARM_1); // clear alarms
//     RTC.clearAlarm(ALARM_2); // clear alarms
//     Serial.println("setup complete");

//     while(1)
//     {
//         time_t time1 = RTC.get(); // get time that is always read

//         if (digitalRead(interruptPin)==LOW)   // if pin low that means alarm fired and need to set new one
//         {
//             // program new alarm since fired
//             setNextAlarm();

//             // put MCU back to sleep as well

//             // Disable the ADC (Analog to digital converter, pins A0 [14] to A5 [19])
//             static byte prevADCSRA = ADCSRA;
//             ADCSRA = 0;
            
//             set_sleep_mode(SLEEP_MODE_PWR_DOWN);
// 		    sleep_enable();


// 		// Note: Microchip state: BODS and BODSE only available for picoPower devices ATmega48PA/88PA/168PA/328P
// 		//
// 		// BODS must be set to one and BODSE must be set to zero within four clock cycles. This sets
// 		// the MCU Control Register (MCUCR)
// 		MCUCR = bit (BODS) | bit(BODSE);

// 		// The BODS bit is automatically cleared after three clock cycles so we better get on with it
// 		MCUCR = bit(BODS);

// 		// Ensure we can wake up again by first disabling interupts (temporarily) so
// 		// the wakeISR does not run before we are asleep and then prevent interrupts,
// 		// and then defining the ISR (Interrupt Service Routine) to run when poked awake
// 		noInterrupts();
// 		attachInterrupt(digitalPinToInterrupt(interruptPin), disable_sleep_ISR, LOW);

// 		// Send a message just to show we are about to sleep
// 		Serial.println("Good night!");
// 		Serial.flush();

// 		// Allow interrupts now
// 		interrupts();

// 		// And enter sleep mode as set above
// 		sleep_cpu()
// 		;

// 		// --------------------------------------------------------
// 		// µController is now asleep until woken up by an interrupt
// 		// --------------------------------------------------------

// 		// Wakes up at this point when wakePin is brought LOW - interrupt routine is run first
// 		Serial.println("I'm awake!");

// 		// Clear existing alarm so int pin goes high again
//         RTC.clearAlarm(ALARM_1); // clear alarms
//         RTC.clearAlarm(ALARM_2); // clear alarms

// 		// Re-enable ADC if it was previously running
// 		ADCSRA = prevADCSRA;

//         }
//         else
//         {
//             // reading sensor values
//             readSensor();


//             // can use this small block to make sure sensor reading is outputted only once to screen
//             time_t time2 = RTC.get();
//             if (time1 != time2)
//             {
//                 Serial.println();
//             }
//         }
//     }
// }

// void setNextAlarm()
// {
//     // get current time
//     time_t currTIME = RTC.get();

//     // break into tm_elements to set alarm
//     tm_elements SETtm;
//     breaktime(currTIME + 10, SETtm); // adding 10 to make alarm for 10 seconds in future

//     // set the alarm (but not yet active)
//     RTC.setAlarm(ALM1_MATCH_DATE, SETtm.second, SETtm.minute, SETtm.hour, SETtm.Day);

//     // activate alarm
//     RTC.alarmInterrupt(ALARM_1, true);
// }

// void disable_sleep_ISR()
// {
//     // Prevent sleep mode, so we don't enter it again, except deliberately, by code
// 	sleep_disable();

// 	// Detach the interrupt that brought us out of sleep
// 	detachInterrupt(digitalPinToInterrupt(interruptPin));

// 	// Now we continue running the main Loop() just after we went to sleep
// }

// void readSensor()
// {
//     // will be reading sensor values here
//     // currently reading from pin A0
//     value = analogRead(A0); // read the analog value from sensor
//     Serial.print("Sensor value: ");
//     Serial.println(value);
// }











// // bool ForwardEngine::run2()
// // {
    
// //     Serial.print("initializing RTC");
// //     // RTC.setAlarm(ALM1_MATCH_DATE, 0, 0, 0, 1);
// //     // RTC.setAlarm(ALM2_MATCH_DATE, 0, 0, 0, 1);
// //     RTC.alarm(ALARM_1);
// //     RTC.alarm(ALARM_2);
// //     RTC.alarmInterrupt(ALARM_1, false);
// //     RTC.alarmInterrupt(ALARM_2, false);
// //     RTC.squareWave(SQWAVE_NONE);
// //     pinMode(0, INPUT_PULLUP);

// //     while(1)
// //     {
// //         // reading sensor values
// //         Serial.println(F("Joining unsuccessful. Retry joining in 5 seconds"));
// //         value = analogRead(A0); // read the analog value from sensor
// //         Serial.print("Sensor value: ");
// //         Serial.println(value);
// //         Serial.println();


// //         // initializing and setting alarm on rtc
// //         Serial.print("setting alarm on RTC");
// //         Serial.println();
// //         tmElements_t tm;
// //         time_t timeRN = RTC.get();
// //         Serial.print("1");
// //         // Serial.println(timeRN);
// //         breakTime(timeRN + 10, tm);  // +10 sets the alarm for 10 seconds after
// //         Serial.print("2");
// //         RTC.setAlarm(ALM1_MATCH_DATE, tm.Second, tm.Minute, tm.Hour, tm.Day); // matches all time up to date
        
// //         // clear the alarm flag
// //         Serial.print("3");
// //         RTC.alarm(ALARM_1);
// //         Serial.print("4");
// //         RTC.alarmInterrupt(ALARM_2, false);
// //         RTC.alarmInterrupt(ALARM_1, true);
// //         Serial.print("5");
// //         noInterrupts();       
// //         Serial.print("6");

// //         // noInterrupts();
// //         attachInterrupt(translateInterruptPin(myRTCInterruptPin), rtcISR, FALLING);
// //         // EIFR = bit(translateInterruptPin(myRTCInterruptPin));
// //         interrupts();



// //         Serial.print("MCU going to sleep");
// //         digitalWrite(myRTCVccPin, LOW);
// //         // setting the MCU to sleep 
// //         myDriver->enterSleepMode();   // putting trans to sleep
// //         Serial.println(F("Put MCU to sleep"));
// //         Serial.flush();  // send out all outputs to serial

// //         //Put the MCU to sleep and set the interrupt handler
// //         myDriver->powerDownMCU();  // turn off

// //         set_sleep_mode(SLEEP_MODE_PWR_DOWN);
// //         sleep_enable();

// //         noInterrupts();

// //         // We are guaranteed that the sleep_cpu call will be done
// //         // as the processor executes the next instruction after
// //         // interrupts are turned on.
// //         interrupts(); // one cycle
// //         sleep_cpu();  // one cycle

// //         // When MCU wakes up from here, RTC alarm has indicated the start of a new receiving period
// //         RTC.alarm(ALARM_1);

// //         // Put the Transceiver back on
// //         myDriver->enterTransMode();

// //         Serial.print("MCU woken up now");
// //         continue;

// //     }

// // }




