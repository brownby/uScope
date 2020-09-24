/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 7 September 2020
 */

#include "uScope.h"

void setup() {

  __disable_irq();

  uart_init();
  delay(100);
  uart_puts("\nInitializing...");
  
  analogWriteResolution(10);

  adc_init();
  dma_init(); 
  usb_init();

  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);

  pinMode(5, OUTPUT);
  digitalWrite(5, LOW);
  
  adc_to_sram_dma();
  start_adc_sram_dma(); 

  uart_puts("\nStarting...");

  __enable_irq();
  
}

void loop() {
  
  fngenerator();

}
