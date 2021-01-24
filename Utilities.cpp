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

#include "Utilities.h"

uint8_t translateInterruptPin(uint8_t digitalPin){

  #if defined (__AVR_ATmega328P__)
  return digitalPinToInterrupt(digitalPin);
  #endif
  
  #if defined (__AVR_ATmega32U4__)
  /* Source: https://www.arduino.cc/reference/en/language/functions/external-interrupts/attachinterrupt/ */
  switch(digitalPin){
    case 0:
      return 2;

    case 1:
      return 3;

    case 2:
      return 1;

    case 3:
      return 0;

    case 7:
      return 4;

    default:
      return NOT_AN_INTERRUPT;
  }
  #endif

  return NOT_AN_INTERRUPT;
}

/**
 * Source: https://learn.adafruit.com/memories-of-an-arduino/measuring-free-memory
 */ 

#ifdef __arm__
// should use uinstd.h to define sbrk but Due causes a conflict
extern "C" char* sbrk(int incr);
#else  // __ARM__
extern char *__brkval;
#endif  // __arm__
 
int freeMemory() {
  char top;
#ifdef __arm__
  return &top - reinterpret_cast<char*>(sbrk(0));
#elif defined(CORE_TEENSY) || (ARDUINO > 103 && ARDUINO != 151)
  return &top - __brkval;
#else  // __arm__
  return __brkval ? &top - __brkval : &top - __malloc_heap_start;
#endif  // __arm__
}