#include "adc.h"

void adc_init() {

  pinPeripheral(ADCPIN, PIO_ANALOG);     // for pin, set function

  ADC->CTRLA.bit.ENABLE = 0x00;          // disable ADC before configuration
  while(ADC->STATUS.bit.SYNCBUSY == 1);  // wait

  ADC->INPUTCTRL.bit.GAIN = 0xF;         // 0xF for DIV2 -> Delay Gain = 1 CLK_ADC
  ADC->REFCTRL.bit.REFSEL = 0x2;         // reference voltage = 0.5 VDDANA (1.65V)
  while(ADC->STATUS.bit.SYNCBUSY == 1); 

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber; // select ADC pin, positive node
  while(ADC->STATUS.bit.SYNCBUSY == 1); 
  
  //ADC->INPUTCTRL.bit.MUXNEG = 0x18;      // negative node, if differential, set to 0x18 = internal GND
  //while(ADC->STATUS.bit.SYNCBUSY == 1);
  
  ADC->AVGCTRL.bit.SAMPLENUM = 0x0;      // 1 sample per conversion, no averaging
  ADC->SAMPCTRL.reg = 0x03;               // add 20 half ADC clk cycle periods to sample time
  while(ADC->STATUS.bit.SYNCBUSY == 1); 

  ADC->CTRLB.bit.PRESCALER = 0x3;        // 0x3 = DIV32, 0x4 = DIV64, 0x5 = DIV128
  ADC->CTRLB.bit.RESSEL = 0x0;           // result resolution, 0x0 = 12 bit, 0x2 = 10 bit, 0x3 = 8 bit
  ADC->CTRLB.bit.FREERUN = 1;            // enable freerun
  ADC->CTRLB.bit.DIFFMODE = 0;           // ADC is single-ended, ignore MUXNEG defined above
  while(ADC->STATUS.bit.SYNCBUSY == 1); 
  
  ADC->CTRLA.bit.ENABLE = 0x01;          // enable ADC after configuration
  while(ADC->STATUS.bit.SYNCBUSY == 1); 
  
}