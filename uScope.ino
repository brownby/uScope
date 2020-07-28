/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 27 July 2020
 * 
 * Checkpoint (v1.0): use EVSYS to trigger alternating SRAM-UART DMA from circular ADC buffers
 * 
 * Revision history
 * ----------------
 * 
 *             v0.9 - transfer ADC buffers to UART (success), setup EVSYS for precision timing
 *             v0.8 - attempt to SW trigger timely, alternating SRAM-UART transfers from DMAC_Handler, consider EVSYS
 *             v0.7 - setup FTDI device output, switch SERCOM 5 to 0, working SRAM (dummy buffer) to UART DMA
 *             v0.6 - add USART support, basic write wrapper / test
 *             v0.5 - clean DAC requirements (CPU only), grab bufnum in DMAC ISR
 *             v0.4 - alternating buffers for circular ADC descriptors, separate dma init from start
 *             v0.3 - wrapper for ADC, evaluate USBSerial for offload
 *             v0.2 - add DAC, with simple wrapper
 *             v0.1 - clean code, ADC to SRAM DMA test
 *             v0.0 - scrub project
 */ 

#include "Arduino.h" // required before wiring_private.h
#include "wiring_private.h" // for pinPeripheral in adc_init()

#define F_CPU 48000000 // CPU clock frequency
#define ADCPIN A1      // selected arbitrarily, consider moving away from DAC / A0
#define NBEATS 64    // number of beats for adc transfer
#define NPTS 1024      // number of points within waveform definition

uint16_t adc_buffer0[NBEATS]; // buffer with length set by NBEATS
uint16_t adc_buffer1[NBEATS]; // buffer with length set by NBEATS
uint16_t waveout[NPTS];       // buffer for waveform

volatile bool bufnum; // track which buffer to write to, while USB reads
volatile bool dmadone; // track sram to usart transfer

enum type {sine, sawtooth}; // supported waveform types

static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1
static uint32_t sramtouart0 = 2; // dma channel for sram, buf0 to uart
static uint32_t sramtouart1 = 3; // dma channel for sram, buf1 to uart

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

void uart_init(){
   
   PORT->Group[0].DIRSET.reg = (1 << 10); // set TX pin direction to output
   PORT->Group[0].PINCFG[10].bit.PMUXEN = 1; 
   PORT->Group[0].PINCFG[10].bit.INEN = 1; 
   PORT->Group[0].PMUX[5].bit.PMUXE = PORT_PMUX_PMUXE_C_Val; 

   PORT->Group[0].DIRCLR.reg = (1 << 11); // set RX pin direction to input
   PORT->Group[0].PINCFG[11].bit.PMUXEN = 1; 
   PORT->Group[0].PINCFG[11].bit.INEN = 1;
   PORT->Group[0].PINCFG[11].reg &= ~PORT_PINCFG_PULLEN; 
   PORT->Group[0].PMUX[5].bit.PMUXO = PORT_PMUX_PMUXO_C_Val;

   PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0; 
   GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_SERCOM0_CORE; 
   GCLK->CLKCTRL.bit.CLKEN = 0x1;
   GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0;

   SERCOM0->USART.CTRLA.bit.ENABLE=0; // disable USART
   while(SERCOM0->USART.SYNCBUSY.bit.ENABLE == 1);
   
   SERCOM0->USART.CTRLA.bit.MODE=0x01;     // 0x1: USART with internal clock
   SERCOM0->USART.CTRLA.bit.CMODE = 0;     // asychronous
   SERCOM0->USART.CTRLA.bit.RXPO = 3;      // pad to be used for RX
   SERCOM0->USART.CTRLA.bit.TXPO = 1;      // pad to be sued for TX
   SERCOM0->USART.CTRLA.bit.DORD = 1;      // transmitted first
   SERCOM0->USART.CTRLA.bit.FORM = 1;      // USART frame
   SERCOM0->USART.CTRLB.bit.CHSIZE = 0;    // 8 bits
   SERCOM0->USART.CTRLB.bit.PMODE = 0;     // parity odd
   SERCOM0->USART.CTRLB.bit.SBMODE = 1;    // 1 stop bit

   SERCOM0->USART.BAUD.reg = (uint16_t)br;
   SERCOM0->USART.INTENSET.bit.TXC = 1;    // TX complete interrupt flag
   
   SERCOM0->USART.CTRLB.bit.RXEN = 1;      // RX enabled or enabled with USART
   SERCOM0->USART.CTRLB.bit.TXEN = 1;      // TX enabled or enabled with USART
   
   SERCOM0->USART.CTRLA.bit.ENABLE = 1;    // enable USART
   
   while(SERCOM0->USART.SYNCBUSY.bit.ENABLE == 1); // wait

}

void uart_putc(char c){
  
  while (SERCOM0->USART.INTFLAG.bit.DRE == 0); // wait for DATA.reg to be empty
  SERCOM0->USART.DATA.reg = c;  

}

void uart_puts(char *s){
 
   while(*s)
     uart_putc(*s++);
   
}

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
  descriptor.DSTADDR = (uint32_t) adc_buffer0 + NBEATS*2; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL = DMAC_BTCTRL_BEATSIZE(0x1) | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT(0x1) | DMAC_BTCTRL_EVOSEL(0x1); 

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
  descriptor.DSTADDR = (uint32_t) adc_buffer1 + NBEATS*2; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL = DMAC_BTCTRL_BEATSIZE(0x1) | DMAC_BTCTRL_DSTINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT(0x1) | DMAC_BTCTRL_EVOSEL(0x1); 

  memcpy(&descriptor_section[adctobuf1], &descriptor, sizeof(dmacdescriptor));
 
}
  
void start_adc_sram_dma() {

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0); // select channel
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable
  
  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf1); // select channel
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable
  
}

void sram_to_uart_dma() { 
  
  DMAC->CHID.reg = DMAC_CHID_ID(sramtouart0); // select adc channel, 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x00; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x02; // for SERCOM0 TX trigger
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHCTRLB.bit.EVIE = 1; // enable input as event user
  DMAC->CHCTRLB.bit.EVACT = 0x5; // transfer trigger
  DMAC->CHINTENSET.reg = ~DMAC_CHINTENSET_MASK; // disable channel interrupts
  
  descriptor.DESCADDR = 0; // one-shot after trigger
  descriptor.SRCADDR = (uint32_t) adc_buffer0 + NBEATS*2;
  descriptor.DSTADDR = (uint32_t) &SERCOM0->USART.DATA.reg; 
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL = DMAC_BTCTRL_BEATSIZE(0x1) | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID | DMAC_BTCTRL_BLOCKACT(0x0); // size 0x1 = half, 0x0 = byte 

  memcpy(&descriptor_section[sramtouart0], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(sramtouart1); // select adc channel, 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x00; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x02; // 0x02 for SERCOM0 TX trigger
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHCTRLB.bit.EVIE = 1; // enable input as event user
  DMAC->CHCTRLB.bit.EVACT =  0x5; // transfer trigger
  DMAC->CHINTENSET.reg = ~DMAC_CHINTENSET_MASK; // disable channel interrupts
  
  descriptor.DESCADDR = 0; // one-shot after trigger  
  descriptor.SRCADDR = (uint32_t) adc_buffer1 + NBEATS*2;
  descriptor.DSTADDR = (uint32_t) &SERCOM0->USART.DATA.reg; 
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL = DMAC_BTCTRL_BEATSIZE(0x1) | DMAC_BTCTRL_SRCINC | DMAC_BTCTRL_VALID |  DMAC_BTCTRL_BLOCKACT(0x0); // size 0x1 = half, 0x0 = byte 

  memcpy(&descriptor_section[sramtouart1], &descriptor, sizeof(dmacdescriptor));

}
  
void start_sram_uart_dma() {

  DMAC->CHID.reg = DMAC_CHID_ID(sramtouart0); // select channel
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

  DMAC->CHID.reg = DMAC_CHID_ID(sramtouart1); // select channel
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable
  
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

void evsys_init(){

  PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;
  PM->APBCMASK.reg |= PM_APBCMASK_TCC0;

  EVSYS->CTRL.reg = EVSYS_CTRL_SWRST; // soft reset
  while (EVSYS->CTRL.reg & EVSYS_CTRL_SWRST); // wait
  
  EVSYS->USER.reg = EVSYS_USER_USER(0x02) | EVSYS_USER_CHANNEL(0x1); // user is sramtouart0 (2), see note about n + 1, p421
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_PATH_RESYNCHRONIZED |           // given for DMA
                       EVSYS_CHANNEL_EDGSEL_BOTH_EDGES |
                       EVSYS_CHANNEL_EVGEN(0x1E) |                   // generator is adctobuf0
                       EVSYS_CHANNEL_CHANNEL(0x1);                   // matches USER.reg above  

  EVSYS->USER.reg = EVSYS_USER_USER(0x03) | EVSYS_USER_CHANNEL(0x2); // user is sramtouart1 (3), see note about n + 1, p421
  EVSYS->CHANNEL.reg = EVSYS_CHANNEL_PATH_RESYNCHRONIZED |           // given for DMA
                       EVSYS_CHANNEL_EDGSEL_BOTH_EDGES |
                       EVSYS_CHANNEL_EVGEN(0x1F) |                   // generator is adctobuf1
                       EVSYS_CHANNEL_CHANNEL(0x2);                   // matches USER.reg above    
}


void DMAC_Handler() { // DMAC ISR, so case sensitive nomenclature
  
  __disable_irq(); // disable interrupts

  bufnum =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // grab active channel
  DMAC->CHID.reg = DMAC_CHID_ID(bufnum); // select active channel
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear transfer complete flag
  
  //SerialUSB.println(bufnum); // will affect DAC performance, used to verify buffer alternation

  __enable_irq(); // enable interrupts
  
}

void test_uart(){
  
  uart_puts("hello there\n");
  delay(1000);
  
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
      SerialUSB.println("please enter an accepted waveform class");
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

  switch (bufnum){    
    case 0: 
      for (int i = 0; i < NBEATS; i++){
        SerialUSB.println(adc_buffer0[i]);
      }
      
    case 1: 
      for (int i = 0; i < NBEATS; i++){
        SerialUSB.println(adc_buffer1[i]);
      }       
  } 
}

void setup() {
  
  SerialUSB.begin(0); // initialize, Serial_ wrapper?
  analogWriteResolution(10);

  uart_init();
  adc_init();
  dma_init(); 
  evsys_init();
  
  adc_to_sram_dma();
  sram_to_uart_dma();

  bufnum = 0;
  start_adc_sram_dma();
//  start_sram_uart_dma(); // need to resolve EVSYS strobes
  
}

void loop() {

  type waveform = sine;
  test_dac(waveform);

//  test_uart();
  
}
