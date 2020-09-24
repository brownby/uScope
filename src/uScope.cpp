#include "uScope.h"

static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1
static uint32_t adctobuf2 = 2;  // dma channel for adc to buf2
static uint32_t adctobuf3 = 3;  // dma channel for adc to buf3

uint16_t adc_buffer0[NBEATS];  // buffer with length set by NBEATS
uint16_t adc_buffer1[NBEATS];  // alternating buffer
uint16_t adc_buffer2[NBEATS];
uint16_t adc_buffer3[NBEATS];

void adc_to_sram_dma() { 
  
  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0);  // select adc channel, 0
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x03; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27; // 0x27 for ADC result ready
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHINTENSET.bit.TCMPL = 1;
  
  descriptor.DESCADDR = 0; //(uint32_t) &descriptor_section[adctobuf1]; 
  descriptor.SRCADDR =  (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR =  (uint32_t) adc_buffer0 + 2*NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1); // beat size is 2 bytes
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0); // disable channel after last block transfer
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0); // disable event outputs
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID; // set descriptor valid

  memcpy(&descriptor_section[adctobuf0], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf1); // select adc channel, 1
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x03; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27; // 0x27 for ADC result ready
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHINTENSET.bit.TCMPL = 1;
  
  descriptor.DESCADDR = 0; //(uint32_t) &descriptor_section[adctobuf2]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer1 + 2*NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1); // beat size is 2 bytes
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0); // disable channel after last block transfer
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0); // disable event outputs
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID; // set descriptor valid

  memcpy(&descriptor_section[adctobuf1], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf2); // select adc channel, 2
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x03; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27; // 0x27 for ADC result ready
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHINTENSET.bit.TCMPL = 1;
  
  descriptor.DESCADDR = 0; //(uint32_t) &descriptor_section[adctobuf3]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer2 + 2*NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1); // beat size is 2 bytes
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0); // disable channel after last block transfer
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0); // disable event outputs
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID; // set descriptor valid

  memcpy(&descriptor_section[adctobuf2], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf3); // select adc channel, 1
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE; // disable channel before configuration
  
  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST; // soft reset
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST); // wait until reset
  
  DMAC->CHCTRLB.bit.LVL = 0x03; // priority for the channel
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27; // 0x27 for ADC result ready
  DMAC->CHCTRLB.bit.TRIGACT = 0x02; // 02 = beat, 03 = transaction, or 00 = block
  DMAC->CHINTENSET.bit.TCMPL = 1;
  
  descriptor.DESCADDR = 0; //(uint32_t) &descriptor_section[adctobuf0]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer3 + 2*NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1); // beat size is 2 bytes
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0); // disable channel after last block transfer
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0); // disable event outputs
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID; // set descriptor valid

  memcpy(&descriptor_section[adctobuf3], &descriptor, sizeof(dmacdescriptor));

}