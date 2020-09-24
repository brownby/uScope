/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 24 September 2020
 */

#include "uScope.h"

void setup() {

  uScope_init();
  
}

void loop() {
  
  fngenerator();

}
