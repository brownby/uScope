/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 27 July 2020
 * 
 * Checkpoint (v1.0): use EVSYS to trigger alternating SRAM-UART DMA from circular ADC buffers
*/

#include "Arduino.h" // required before wiring_private.h
                     // includes USBDesc.h, USBCore.h, USBAPI.h, and USB_host.h
#include "wiring_private.h" // for pinPeripheral in adc_init()
#include "USB/PluggableUSB.h"
#include "USB/CDC.h"
#include "USB/SAMD21_USBDevice.h"

#define F_CPU 48000000 // CPU clock frequency
#define ADCPIN A1      // selected arbitrarily, consider moving away from DAC / A0
#define NBEATS 64    // number of beats for adc transfer
#define NPTS 1024      // number of points within waveform definition



uint8_t adc_buffer0[NBEATS]; // buffer with length set by NBEATS
uint8_t adc_buffer1[NBEATS];
uint16_t waveout[NPTS];       // buffer for waveform

volatile bool bufnum; // track which buffer to write to, while USB reads
volatile bool dmadone; // track sram to usart transfer

enum type {sine, sawtooth}; // supported waveform types

static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1
//static uint32_t sramtouart0 = 2; // dma channel for sram, buf0 to uart
//static uint32_t sramtouart1 = 3; // dma channel for sram, buf1 to uart

static uint32_t baud = 115200;
uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU;

typedef struct {
  uint16_t BTCTRL;   // block transfer control
  uint16_t BTCNT;    // block transfer count
  uint32_t SRCADDR;  // source address
  uint32_t DSTADDR;  // destination address
  uint32_t DESCADDR; // next descriptor address
} dmacdescriptor;

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16))); // write-back descriptor
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16))); // channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));

//extern uint8_t pluggedEndpoint;
extern USBDevice_SAMD21G18x usbd; // defined in USBCore.cpp


#define CDC_ENDPOINT_IN 3

void adc_init(){

  pinPeripheral(ADCPIN, PIO_ANALOG); // for pin, set function

  ADC->CTRLA.bit.ENABLE = 0x00; // disable ADC before configuration
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait

  ADC->INPUTCTRL.bit.GAIN = 0xF; // for DIV2 or 0x0 for 1X
  ADC->REFCTRL.bit.REFSEL = 0x2; // reference voltage = 0.5 VDDANA
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber; // select ADC pin, positive node
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait
  
  ADC->INPUTCTRL.bit.MUXNEG = 0x18; // negative node, if differential, set to 0x18 = internal GND
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait
  
  ADC->AVGCTRL.bit.SAMPLENUM = 0x0; // 1 sample per conversion, no averaging
  ADC->SAMPCTRL.reg = 0x00; // minimize sample time, given in 1/2 CLK_ADC cycles, p863
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait

  ADC->CTRLB.bit.PRESCALER = 0x4; // 0x3 = DIV32 or 0x2 = DIV16
  ADC->CTRLB.bit.RESSEL = 0x3;    // result resolution, 0x2 = 10 bit, 0x3 = 8 bit
  ADC->CTRLB.bit.FREERUN = 1;     // enable freerun
  ADC->CTRLB.bit.DIFFMODE = 0;    // ADC is single-ended, ignore MUXNEG defined above
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait
  
  ADC->CTRLA.bit.ENABLE = 0x01; // enable ADC after configuration
  while(ADC->STATUS.bit.SYNCBUSY == 1); // wait
  
}

void adc_to_sram_dma() { 
  
  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0); // select adc channel, 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x01; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27; // 0x27 for ADC result ready
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHCTRLB.bit.EVOE = 1; // enable output as event generator
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK; //or bit.TCMPL = 1 for transfer complete
  
  descriptor.DESCADDR = (uint32_t) &descriptor_section[adctobuf1]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer0 + NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL = DMAC_BTCTRL_BEATSIZE(0x0) | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT(0x1) | DMAC_BTCTRL_EVOSEL(0x1); 

  memcpy(&descriptor_section[adctobuf0], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf1); // select adc channel, 1
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x01; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27; // 0x27 for ADC result ready
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHCTRLB.bit.EVOE = 1; // enable output as event generator
  DMAC->CHINTENSET.reg = DMAC_CHINTENSET_MASK; //or bit.TCMPL = 1 for transfer complete
  
  descriptor.DESCADDR = (uint32_t) &descriptor_section[adctobuf0]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer1 + NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL = DMAC_BTCTRL_BEATSIZE(0x0) | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT(0x1) | DMAC_BTCTRL_EVOSEL(0x1); 

  memcpy(&descriptor_section[adctobuf1], &descriptor, sizeof(dmacdescriptor));
}
  
void start_adc_sram_dma() {

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0); // select channel
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf1); // select channel
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void dma_init() {

  PM->AHBMASK.reg |= PM_AHBMASK_DMAC; // enable AHB clock
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC; // enable APBB clock

  NVIC_SetPriority(DMAC_IRQn, 0x00); // top priority
  NVIC_EnableIRQ(DMAC_IRQn); // enable interrupts, will trigger DMAC_Handler

  DMAC->BASEADDR.reg = (uint32_t)descriptor_section; // where to find descriptor
  DMAC->WRBADDR.reg = (uint32_t)wrb; // holds descriptor if interrupted 
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf); // enable DMA, priority level

}

void DMAC_Handler() { // DMAC ISR, so case sensitive nomenclature
  
  __disable_irq(); // disable interrupts

  bufnum =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // grab active channel
  DMAC->CHID.reg = DMAC_CHID_ID(bufnum); // select active channel
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear transfer complete flag

  // tell USB where to find data, tell USB data is ready
  if(bufnum == 0)
  {
    usbd.epBank1SetAddress(CDC_ENDPOINT_IN, &adc_buffer0);
  }
  else
  {
    usbd.epBank1SetAddress(CDC_ENDPOINT_IN, &adc_buffer1);
  }
  usbd.epBank1SetByteCount(CDC_ENDPOINT_IN, NBEATS); // each beat is 8 bits

  usbd.epBank1AckTransferComplete(CDC_ENDPOINT_IN);

  usbd.epBank1SetReady(CDC_ENDPOINT_IN);
  
  //SerialUSB.println(bufnum); // will affect DAC performance, used to verify buffer alternation

  __enable_irq(); // enable interrupts
  
}

void test_dac(type waveform){

  int i;
  float phase = 3.14159*2.0*2./NPTS;

  switch (waveform){
    
    case 0: 
      for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * 500.0f + 512.0f;
      break;
      
    case 1:
      for (i=0;i<NPTS/2;i++) waveout[i] = waveout[NPTS-1-i] = 2*1024*i/NPTS;
      break;

    default:
      Serial.println("please enter an accepted waveform class");
      break;
      
  }
  
  while(1){
    
    for (int i = 0; i < NPTS; i++){ 
      analogWrite(A0,waveout[i]);
    }
    //start_sram_uart_dma(); // brute force transfers, not good practice but POC
  }
}

void test_adc(){

  for (int i = 0; i < NBEATS; i++){
//    Serial.println(adc_buffer0[i]);
  }
  
}

void setup() {
  
  analogWriteResolution(10);

//  Serial.println(pluggedEndpoint);

  adc_init();
  dma_init(); 
  
  adc_to_sram_dma();

  bufnum = 0;
  start_adc_sram_dma(); 
}

void loop() {

  type waveform = sine;
  test_dac(waveform);
  
}
