#include "uart_u.h"

static uint32_t baud = 115200;
uint64_t br = (uint64_t)65536 * (freq_CPU - 16 * baud) / freq_CPU; // to pass to SERCOM0->USART.BAUD.reg


void uart_init() {

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
   
   SERCOM0->USART.CTRLA.bit.MODE   = 0x01;  // 0x1: USART with internal clock
   SERCOM0->USART.CTRLA.bit.CMODE  = 0;     // asychronous
   SERCOM0->USART.CTRLA.bit.RXPO   = 3;     // pad to be used for RX, pin 3 in Arduino-land
   SERCOM0->USART.CTRLA.bit.TXPO   = 1;     // pad to be sued for TX, pin 2 in Arduino-land
   SERCOM0->USART.CTRLA.bit.DORD   = 1;     // LSB transmitted first
   SERCOM0->USART.CTRLA.bit.FORM   = 1;     // USART frame with parity
   SERCOM0->USART.CTRLB.bit.CHSIZE = 0;     // 8 bits
   SERCOM0->USART.CTRLB.bit.PMODE  = 0;     // parity even
   SERCOM0->USART.CTRLB.bit.SBMODE = 0;     // 1 stop bit

   SERCOM0->USART.BAUD.reg = (uint16_t)br;
   SERCOM0->USART.INTENSET.bit.TXC = 1;     // TX complete interrupt flag
   
   SERCOM0->USART.CTRLB.bit.RXEN = 1;       // RX enabled or enabled with USART
   SERCOM0->USART.CTRLB.bit.TXEN = 1;       // TX enabled or enabled with USART
   
   SERCOM0->USART.CTRLA.bit.ENABLE = 1;     // enable USART
   
   while(SERCOM0->USART.SYNCBUSY.bit.ENABLE == 1); // wait

}

void uart_putc(char c) {

  while (SERCOM0->USART.INTFLAG.bit.DRE == 0); // wait for DATA.reg to be empty
  SERCOM0->USART.DATA.reg = c;  

}

void uart_write(uint16_t data) {
  
  while (SERCOM0->USART.INTFLAG.bit.DRE == 0); // wait for DATA.reg to be empty
  SERCOM0->USART.DATA.reg = (uint32_t)data;  

}

void uart_puts(char *s) {
 
   while(*s)
    uart_putc(*s++);
   
}

void uart_put_hex(uint8_t x) {
  uint8_t highNibble = (x & 0xf0) >> 4;
  uint8_t lowNibble = x & 0xf;

  if(highNibble >= 10)
  {
    uart_putc('a' + (highNibble - 10));
  }
  else
  {
    uart_putc(highNibble + '0');
  }

  if(lowNibble >= 10)
  {
    uart_putc('a' + (lowNibble - 10));
  }
  else
  {
    uart_putc(lowNibble + '0');
  }
  
}
