#ifndef _DMAC_U_H_
#define _DMAC_u_H_

#include "Arduino.h"

typedef struct __attribute__((packed)){
  uint16_t BTCTRL;   // block transfer control
  uint16_t BTCNT;    // block transfer count
  uint32_t SRCADDR;  // source address
  uint32_t DSTADDR;  // destination address
  uint32_t DESCADDR; // next descriptor address
} dmacdescriptor;


volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));        // write-back descriptor
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));  // channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));

void start_adc_sram_dma();

void dma_init();

#endif