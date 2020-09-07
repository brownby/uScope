/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 7 September 2020
 */

#include "Arduino.h"          // required before wiring_private.h, also includes USBDesc.h, USBCore.h, USBAPI.h, and USB_host.h
#include "wiring_private.h"   // for pinPeripheral in adc_init()
#include "USB/PluggableUSB.h"
#include "USB/CDC.h"
#include "USB/SAMD21_USBDevice.h"
#include "usb_descriptors.h"
#include "usb_enums.h"

#define freq_CPU 48000000                                           // CPU clock frequency
static uint32_t baud = 115200;                                      // for UART debug of USB
uint64_t br = (uint64_t)65536 * (freq_CPU - 16 * baud) / freq_CPU;  // to pass to SERCOM0->USART.BAUD.reg

#define ADCPIN A1           // selected arbitrarily, consider moving away from DAC / A0
#define NBEATS 1000         // number of beats for adc transfer
#define NPTS 1024           // number of points within waveform definition

#define CONTROL_ENDPOINT  0
#define ISO_ENDPOINT_IN   1
#define ISO_ENDPOINT_OUT  2
#define CDC_ENDPOINT_COMM 3
#define CDC_ENDPOINT_IN   4
#define CDC_ENDPOINT_OUT  5

#define LIMIT(a, b)     (((a) > (b)) ? (b) : (a))
#define USB_CMD(dir, rcpt, type, cmd) \
    ((USB_##cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define USB_AUDIO_CMD(dir, rcpt, type, cmd) \
    ((cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define USB_CDC_CMD(dir, rcpt, type, cmd) \
    ((cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

uint8_t adc_buffer0[NBEATS];  // buffer with length set by NBEATS
uint8_t adc_buffer1[NBEATS];  // alternating buffer
uint8_t adc_buffer2[NBEATS];
uint8_t adc_buffer3[NBEATS];
uint16_t waveout[NPTS];       // buffer for waveform

volatile float amplitude = 250;
volatile bool mute = false;
volatile uint16_t volume = 5;

static uint16_t usb_ctrl_audio;
static uint32_t usb_ctrl_in_buf[16];
static uint8_t usb_ctrl_out_buf[16];
static uint8_t usb_cdc_out_buf[64] = {0}; 
static uint8_t usb_cdc_in_buf[64];

char incoming_string[255] = {0}; // up to 255 character long strings
char command[255] = {0};
uint8_t char_count = 0;
bool cmd_recv = 0;

bool RTS = false;
bool DTR = false;

static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1
static uint32_t adctobuf2 = 2;  // dma channel for adc to buf2
static uint32_t adctobuf3 = 3;  // dma channel for adc to buf3

static uint8_t ascii = 48;      // offset to interpret single digit uart outputs
static int usb_config;

char *usb_strings[100] = {"", "Arduino + Harvard","uScope by Active Learning","ALL-0001","Isochronous Audio","uScope Instrumentation"};

uint8_t usb_string_descriptor_buffer[64] __attribute__ ((aligned (4)));

volatile uint8_t ZLP_c   = 0;
volatile uint8_t bufnum  = 0;  // track which buffer to write to, while USB reads
volatile uint8_t prevBuf = 4;

extern USBDevice_SAMD21G18x usbd; // defined in USBCore.cpp
extern UsbDeviceDescriptor usb_endpoints[];
extern const uint8_t usb_num_endpoints;

enum type {sine, pulse, square, sawtooth}; // supported waveform types

typedef struct __attribute__((packed)){
  uint16_t BTCTRL;   // block transfer control
  uint16_t BTCNT;    // block transfer count
  uint32_t SRCADDR;  // source address
  uint32_t DSTADDR;  // destination address
  uint32_t DESCADDR; // next descriptor address
} dmacdescriptor;

typedef struct __attribute__((packed)) {
  uint8_t   bmRequestType;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} usb_request_t;

typedef struct __attribute__((packed)) {
  usb_request_t request;
  uint16_t      value;
} usb_cdc_notify_serial_state_t;

typedef struct { 
    UsbDeviceDescBank DeviceDescBank[2]; 
} UsbdDescriptor;

static usb_cdc_line_coding_t usb_cdc_line_coding =
{
  .dwDTERate   = 115200,
  .bCharFormat = 0, // 1 stop bit
  .bParityType = 0, // no parity
  .bDataBits   = 8, // 8 data bits
};

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));        // write-back descriptor
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));  // channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));
UsbDeviceDescriptor EP[USB_EPT_NUM] __attribute__ ((aligned (4)));
uint8_t interface_num = 0; // Current interface selected by host
uint8_t alt_setting = 0;

usb_cdc_notify_serial_state_t usb_cdc_notify_message; 
static int usb_cdc_serial_state;
static bool usb_cdc_comm_busy;

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
    uart_putc(highNibble + ascii);
  }

  if(lowNibble >= 10)
  {
    uart_putc('a' + (lowNibble - 10));
  }
  else
  {
    uart_putc(lowNibble + ascii);
  }
  
}

void adc_init() {

  pinPeripheral(ADCPIN, PIO_ANALOG);     // for pin, set function

  ADC->CTRLA.bit.ENABLE = 0x00;          // disable ADC before configuration
  while(ADC->STATUS.bit.SYNCBUSY == 1);  // wait

  ADC->INPUTCTRL.bit.GAIN = 0xF;         // 0xF for DIV2 or 0x0 for 1X
  ADC->REFCTRL.bit.REFSEL = 0x2;         // reference voltage = 0.5 VDDANA
  while(ADC->STATUS.bit.SYNCBUSY == 1); 

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber; // select ADC pin, positive node
  while(ADC->STATUS.bit.SYNCBUSY == 1); 
  
  ADC->INPUTCTRL.bit.MUXNEG = 0x18;      // negative node, if differential, set to 0x18 = internal GND
  while(ADC->STATUS.bit.SYNCBUSY == 1);
  
  ADC->AVGCTRL.bit.SAMPLENUM = 0x0;      // 1 sample per conversion, no averaging
  ADC->SAMPCTRL.reg = 0x00;              // add 8 half ADC clk cycle periods to sample time
  while(ADC->STATUS.bit.SYNCBUSY == 1); 

  ADC->CTRLB.bit.PRESCALER = 0x4;        // 0x5 = DIV128
  ADC->CTRLB.bit.RESSEL = 0x3;           // result resolution, 0x2 = 10 bit, 0x3 = 8 bit
  ADC->CTRLB.bit.FREERUN = 1;            // enable freerun
  ADC->CTRLB.bit.DIFFMODE = 0;           // ADC is single-ended, ignore MUXNEG defined above
  while(ADC->STATUS.bit.SYNCBUSY == 1); 
  
  ADC->CTRLA.bit.ENABLE = 0x01;          // enable ADC after configuration
  while(ADC->STATUS.bit.SYNCBUSY == 1); 
  
}

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
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer0 + NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x0); // beat size is 1 byte
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
  
  descriptor.DESCADDR = 0; //(uint32_t) &descriptor_section[adctobuf0]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer1 + NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x0); // beat size is 1 byte
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
  
  descriptor.DESCADDR = 0; //(uint32_t) &descriptor_section[adctobuf0]; 
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg; 
  descriptor.DSTADDR = (uint32_t) adc_buffer2 + NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x0); // beat size is 1 byte
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
  descriptor.DSTADDR = (uint32_t) adc_buffer3 + NBEATS; // end of target address
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0); // doesn't matter as long as DSTINC=1, SRCINC = 0, and STEPSEL = 1
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL; // apply step size settings to source address
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC; // increment destination address
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC; // do not increment source address
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x0); // beat size is 1 byte
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0); // disable channel after last block transfer
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0); // disable event outputs
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID; // set descriptor valid

  memcpy(&descriptor_section[adctobuf3], &descriptor, sizeof(dmacdescriptor));

}
  
void start_adc_sram_dma() {

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0); // select channel
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

void DMAC_Handler() { 

  __disable_irq(); // disable interrupts

  ZLP_c = 0;

  bufnum =  (uint8_t)(DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk); // grab active channel
  // uart_puts("\n\nd"); uart_write(bufnum + ascii);

  DMAC->CHID.reg = DMAC_CHID_ID(bufnum); // select active channel
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL | DMAC_CHINTFLAG_SUSP | DMAC_CHINTFLAG_TERR;

  if (bufnum == 0)
  {
    // uart_puts("\ncmd1");

    DMAC->CHID.reg = DMAC_CHID_ID(1); // select active channel
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

  } 
  else if (bufnum == 1)
  {
    // uart_puts("\ncmd2");

    DMAC->CHID.reg = DMAC_CHID_ID(2); // select active channel
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

  }
  else if (bufnum == 2)
  {
    // uart_puts("\ncmd3");

    DMAC->CHID.reg = DMAC_CHID_ID(3); // select active channel
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

  }
  else if (bufnum == 3)
  {
    // uart_puts("\ncmd0");

    DMAC->CHID.reg = DMAC_CHID_ID(0); // select active channel
    DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

  }

  __enable_irq();
}

void usb_init() {

  // reset and wait for reset to complete
  // Arduino core enables SOF interrupts by default, causing ISR to be entered a lot

  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while(USB->DEVICE.SYNCBUSY.bit.SWRST);

  memset((uint8_t *)EP, 0, sizeof(EP));
  USB->DEVICE.DESCADD.reg = (uint32_t)EP;

  // pad calibration, copied from SAMD21_USBDevice.h

  uint32_t *pad_transn_p = (uint32_t *) USB_FUSES_TRANSN_ADDR;
  uint32_t *pad_transp_p = (uint32_t *) USB_FUSES_TRANSP_ADDR;
  uint32_t *pad_trim_p   = (uint32_t *) USB_FUSES_TRIM_ADDR;

  uint32_t pad_transn = (*pad_transn_p & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
  uint32_t pad_transp = (*pad_transp_p & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
  uint32_t pad_trim   = (*pad_trim_p   & USB_FUSES_TRIM_Msk  ) >> USB_FUSES_TRIM_Pos;

  if (pad_transn == 0x1F)  // maximum value (31)
    pad_transn = 5;
  if (pad_transp == 0x1F)  // maximum value (31)
    pad_transp = 29;
  if (pad_trim == 0x7)     // maximum value (7)
    pad_trim = 3;

  USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;
  USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;
  USB->DEVICE.PADCAL.bit.TRIM   = pad_trim;
  
  USB->DEVICE.CTRLA.bit.MODE = USB_CTRLA_MODE_DEVICE_Val;
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 1;
  USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
  USB->DEVICE.CTRLB.bit.DETACH = 0;
  
  USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST;  
  USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTENSET.bit.RXSTP = 1;
  
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  usb_cdc_notify_message.request.bmRequestType = USB_IN_TRANSFER | USB_INTERFACE_RECIPIENT | USB_CLASS_REQUEST;
  usb_cdc_notify_message.request.bRequest = NOTIFY_SERIAL_STATE;
  usb_cdc_notify_message.request.wValue = 0;
  usb_cdc_notify_message.request.wIndex = 0;
  usb_cdc_notify_message.request.wLength = sizeof(uint16_t);
  usb_cdc_notify_message.value = 0;

  usb_cdc_serial_state = 0;
  // usb_cdc_comm_busy = false;
  
  NVIC_SetPriority(USB_IRQn, 0x01); // second priority
  NVIC_EnableIRQ(USB_IRQn); // will trigger USB_Handler

}

void usb_cdc_send_state_notify()
{
  if(usb_cdc_comm_busy)
  {
    return;
  }

  if(usb_cdc_serial_state != usb_cdc_notify_message.value)
  {
    usb_cdc_comm_busy = true;
    usb_cdc_notify_message.value = usb_cdc_serial_state;

    EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(usb_cdc_notify_message);

    USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPSTATUSSET.bit.BK1RDY = 1;
  }
}

void USB_Handler(){

  __disable_irq();
  int epint, flags;

  // Reset
  if(USB->DEVICE.INTFLAG.bit.EORST) { // if EORST interrupt

    uart_puts("\n\nReset");
    
    USB->DEVICE.INTFLAG.bit.EORST = 1; // clear interrupt flag
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    for (int i = 0; i < USB_EPT_NUM; i++) // reset all endpoints
    {
      USB->DEVICE.DeviceEndpoint[i].EPCFG.bit.EPTYPE0 = USB_DEVICE_EPCFG_EPTYPE_DISABLED;
      USB->DEVICE.DeviceEndpoint[i].EPCFG.bit.EPTYPE1 = USB_DEVICE_EPCFG_EPTYPE_DISABLED;
    }
    
    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0 = USB_DEVICE_EPCFG_EPTYPE_CONTROL; // Control SETUP/OUT
    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1 = USB_DEVICE_EPCFG_EPTYPE_CONTROL; // Control IN
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

    EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
    EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
    EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
    EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

    EP[CONTROL_ENDPOINT].DeviceDescBank[0].ADDR.reg = (uint32_t)&usb_ctrl_out_buf;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPINTENSET.bit.RXSTP = 1;
    
  }

  // Requests
  if (USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP){

    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP = 1; // acknowledge interrupt
    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSCLR.bit.BK0RDY = 1;
    
    usb_request_t * request = (usb_request_t*) usb_ctrl_out_buf;

    uart_puts("\n\nbRequest: "); uart_put_hex(request->bRequest);
    uart_puts("\nbmRequestType: "); uart_put_hex(request->bmRequestType);

    uint8_t wValue_L = request->wValue & 0xff;
    uint8_t wValue_H = request->wValue >> 8;
    uint8_t wIndex_L = request->wIndex & 0xff;
    uint8_t wIndex_H = request->wIndex >> 8;
    uint8_t type = request->wValue >> 8;
    uint8_t index = request->wValue & 0xff;
    uint16_t leng = request->wLength;

    uart_puts("\nwValueH: "); uart_put_hex(wValue_H);
    uart_puts("\nwValueL: "); uart_put_hex(wValue_L);
    uart_puts("\nwIndexH: "); uart_put_hex(wIndex_H);
    uart_puts("\nwIndexL: "); uart_put_hex(wIndex_L);
    uart_puts("\nwLength: "); uart_put_hex(leng);

    switch ((request->bRequest << 8) | request->bmRequestType){
      case USB_CMD(IN, DEVICE, STANDARD, GET_DESCRIPTOR):{

        uart_puts("\nGetDescriptor");
          
        if (type == USB_DEVICE_DESCRIPTOR){
            
          uart_puts("\nDevice");

          leng = LIMIT(leng, usb_device_descriptor.bLength);

          uint8_t *descAddr_temp = (uint8_t *)&usb_device_descriptor; 

          if (leng <= usb_device_descriptor.bMaxPacketSize0){
          
            memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
          }

          else {
    
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

          }
  
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng;          // how big it is
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;          // clear flag
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;        // start 

          while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait
          
        } 
          
        else if (type == USB_CONFIGURATION_DESCRIPTOR){

          uart_puts("\nConfiguration");

          leng = LIMIT(leng, usb_configuration_hierarchy.configuration.wTotalLength);

          uint8_t *descAddr_temp = (uint8_t *)&usb_configuration_hierarchy; 

          if (leng <= usb_device_descriptor.bMaxPacketSize0){
          
            memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
          }

          else {
    
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

          }
  
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng;          // how big it is
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;          // clear flag
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;        // start 

          while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait

        }
          
        else if (type == USB_STRING_DESCRIPTOR){
            
          uart_puts("\nString: "); uart_write(index + ascii);
            
          if(index == 0){
              
            leng = LIMIT(leng, usb_string_descriptor_zero.bLength);

            uint8_t *descAddr_temp = (uint8_t *)&usb_string_descriptor_zero; 

            if (leng <= usb_device_descriptor.bMaxPacketSize0){
          
              memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
            }

            else {
    
             EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

            }
  
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng;          // how big it is
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;          // clear flag
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;        // start 

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait
              
          }

          else if (index < USB_STR_COUNT){
              
            char *str = usb_strings[index];
            int len = strlen(str);

            memset(usb_string_descriptor_buffer, 0, sizeof(usb_string_descriptor_buffer));

            usb_string_descriptor_buffer[0] = len*2 + 2; // len*2 because USB uses UTF-16
            usb_string_descriptor_buffer[1] = USB_STRING_DESCRIPTOR;

            for (int i = 0; i < len; i++){
              usb_string_descriptor_buffer[2 + i*2] = str[i]; // place in every other byte of buffer to convert from ASCII to UTF-16
              usb_string_descriptor_buffer[3 + i*2] = 0;
            }

            leng = LIMIT(leng, usb_string_descriptor_buffer[0]);

            uint8_t *descAddr_temp = (uint8_t *)&usb_string_descriptor_buffer; 

            if (leng <= usb_device_descriptor.bMaxPacketSize0){
          
              memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
            }

            else {
    
             EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

            }
  
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng;          // how big it is
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;          // clear flag
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;        // start 

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);
           
          } 
           
          else{

            uart_puts("\nStall");
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;

          }

        }
            
        else{

          uart_puts("\nStall");
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;
            
        } 
      } break;

      case USB_CMD(OUT, DEVICE, STANDARD, SET_ADDRESS): {

        uart_puts("\nSetAddress");

        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(request->wValue);
      
      } break;

      case USB_CMD(OUT, DEVICE, STANDARD, SET_CONFIGURATION): {

        uart_puts("\nSetConfiguration");

        usb_config = request->wValue;
        
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        if(usb_config){

          uart_puts("\nConfigured");

          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPCFG.bit.EPTYPE1 = 2; // isochronous IN
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPINTENSET.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
          EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_1023;

          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].ADDR.reg = (uint32_t)&usb_cdc_notify_message;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPCFG.bit.EPTYPE1 = 4; // interrupt IN
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPINTENSET.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPSTATUSSET.bit.BK1RDY = 1;

          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_cdc_in_buf;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPCFG.bit.EPTYPE1 = 3; // bulk IN
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPINTENSET.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
          
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&usb_cdc_out_buf;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 64;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPCFG.bit.EPTYPE0 = 3; // bulk out
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPINTENSET.bit.TRCPT0 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPSTATUSCLR.bit.DTGLOUT = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPSTATUSCLR.bit.BK0RDY = 1; // arm receive

          usb_cdc_serial_state |= USB_CDC_SERIAL_STATE_DSR;
        }
      } break;

      case USB_CMD(IN, DEVICE, STANDARD, GET_CONFIGURATION): {

        uart_puts("\nGetConfiguration");

        uint8_t config = usb_config;
        
        if (sizeof(config) <= usb_device_descriptor.bMaxPacketSize0){

          memcpy(usb_ctrl_in_buf, &config, sizeof(config));
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;

        }
        else {

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)config;          
          
        }
         
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(config);
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        
        while(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

      } break;

      case USB_CMD(OUT, INTERFACE, STANDARD, SET_INTERFACE): { 
        
        uart_puts("\nSetInterface"); // host sending alternate setting for AudioStreaming interface
        
        interface_num = wIndex_L;
        alt_setting = wValue_L;

        uart_puts("\nSending ZLP");

        // send control ZLP
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        // send ISO ZLP - to trigger first TRCPT1
        if(interface_num == 1 && alt_setting == 1) // AudioStreaming interface, stream1 selected, start streaming
        {
          EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
        }
        
      } break;

      case USB_CMD(IN, DEVICE, STANDARD, GET_STATUS):{

        uart_puts("\nGetStatus: Device");
        uart_puts("\nEmpty case");

      }
      
      case USB_CMD(IN, INTERFACE, STANDARD, GET_STATUS):{

        uart_puts("\nGetStatus: Interface");
        uart_puts("\nEmpty case");

      }

      case USB_CMD(IN, ENDPOINT, STANDARD, GET_STATUS): {

        uart_puts("\nGetStatus: Endpoint");

        int ep = request->wIndex & USB_INDEX_MASK; // USB_INDEX_MASK = 0x7f
        int dir = request->wIndex & USB_DIRECTION_MASK; // USB_DIRECTION_MASK = 0x80
        uint16_t status = 0;

        if(dir == USB_IN_ENDPOINT){

          if(USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE1 != USB_DEVICE_EPCFG_EPTYPE_DISABLED){

            status = USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.STALLRQ1;
            
            memcpy(usb_ctrl_in_buf, &status, sizeof(status));
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(status);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;    
            while(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

          }
          else {

            uart_puts("\nStall");
            
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;
          
          }
        }
        
        else {

          if(USB->DEVICE.DeviceEndpoint[ep].EPCFG.bit.EPTYPE0 != USB_DEVICE_EPCFG_EPTYPE_DISABLED){

            status = USB->DEVICE.DeviceEndpoint[ep].EPSTATUS.bit.STALLRQ0;
            
            memcpy(usb_ctrl_in_buf, &status, sizeof(status));
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(status);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
            while(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

          }
          
          else {

            uart_puts("\nStall");

            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;
          
          }
        }
      } break;

      case USB_CMD(OUT, DEVICE, STANDARD, SET_FEATURE): {

        uart_puts("\nSetFeature: Device");
        uart_puts("\nStall");
        
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;

      } break;

      case USB_CMD(OUT, INTERFACE, STANDARD, SET_FEATURE): {

        uart_puts("\nSetFeature: Interface");
        uart_puts("\nSending ZLP");
      
        // send control ZLP
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

      } break;

      case USB_CMD(OUT, ENDPOINT, STANDARD, SET_FEATURE): {

        uart_puts("\nSetFeature: Interface");
        uart_puts("\nEmpty case");
        
        //TODO
      
      } break;

      case USB_CMD(OUT, DEVICE, STANDARD, CLEAR_FEATURE): {
      
        uart_puts("\nClearFeature: Device");
        uart_puts("\nEmpty case");
        
        //TODO
      
      } break;

      case USB_CMD(OUT, INTERFACE, STANDARD, CLEAR_FEATURE): {
      
        uart_puts("\nClearFeature: Interface");
        uart_puts("\nEmpty case");
        
        //TODO
      
      } break;

      case USB_CMD(OUT, ENDPOINT, STANDARD, CLEAR_FEATURE): {

        uart_puts("\nClearFeature: Endpoint");
        uart_puts("\nEmpty case");
        
        //TODO

      } break;

      case USB_CMD(IN, INTERFACE, STANDARD, GET_DESCRIPTOR): {

        uart_puts("\nGetDescriptor: Interface");
        uart_puts("\nEmpty case");
    
      } break;

      case USB_AUDIO_CMD(IN, INTERFACE, CLASS, GET_MIN):{

        uart_puts("\nGetMin");
        
        uint16_t temp_min = 1;
        memcpy(&usb_ctrl_audio, &temp_min, sizeof(temp_min));
        
        uart_puts(": "); uart_write(usb_ctrl_audio+ascii);

        EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&usb_ctrl_audio;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(temp_min);
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
      
      } break;

      case USB_AUDIO_CMD(IN, INTERFACE, CLASS, GET_MAX):{
      
        uart_puts("\nGetMax");

        uint16_t temp_max = 9;
        memcpy(&usb_ctrl_audio, &temp_max, sizeof(temp_max));

        uart_puts(": "); uart_write(usb_ctrl_audio+ascii);

        EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&usb_ctrl_audio;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(temp_max);
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
      
      } break;

      case USB_AUDIO_CMD(IN, INTERFACE, CLASS, GET_RES):{

        uart_puts("\nGetRes");

        uint16_t temp_res = 1;
        memcpy(&usb_ctrl_audio, &temp_res, sizeof(temp_res));

        uart_puts(": ");uart_write(usb_ctrl_audio+ascii);

        EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&usb_ctrl_audio;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_8;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(temp_res);

        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);
      
      } break;

      case USB_AUDIO_CMD(IN, INTERFACE, CLASS, GET_CUR):{

        uart_puts("\nGetCurrent");

        uart_puts("\nControlSelector: "); uart_write(wValue_H + ascii);
        uart_puts("\nChannelNumber: "); uart_write(wValue_L + ascii);

        if (wValue_H == 1){

          bool temp_mute = mute;
          memcpy(&usb_ctrl_audio, &temp_mute, sizeof(temp_mute));

          uart_puts("\nCurrentMute: ");uart_write(usb_ctrl_audio+ascii);

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&usb_ctrl_audio;
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(temp_mute);
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
          while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        }

        else if (wValue_H == 2){

          uint16_t temp_volume = volume;
          memcpy(&usb_ctrl_audio, &temp_volume, sizeof(temp_volume));

          uart_puts("\nCurrentVolume: ");uart_write(usb_ctrl_audio+ascii);

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&usb_ctrl_audio;
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(temp_volume);
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
          while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        }

        else{uart_puts("\nCHECK ELSE");}

      } break;

      case USB_AUDIO_CMD(OUT, INTERFACE, CLASS, SET_CUR):{

        uart_puts("\nSetCurrent");

        uart_puts("\nControlSelector: "); uart_write(wValue_H + ascii);
        uart_puts("\nChannelNumber: "); uart_write(wValue_L + ascii);

        if (wValue_H == 1){
        
          mute = !mute;

          uart_puts("\n\n"); uart_write(mute+ascii);

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
          while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        }
        
        else if (wValue_H == 2){

          volume = volume; 

          uart_puts("\n\n"); uart_write(volume+ascii);

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
          while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        }

        else{ uart_puts("CHECKELSE"); }

      } break;

      case USB_CDC_CMD(OUT, INTERFACE, CLASS, SET_LINE_CODING): {
        uart_puts("\nSetLineCoding");

        usb_cdc_line_coding_t *line_coding = (usb_cdc_line_coding_t *)(usb_ctrl_out_buf + sizeof(usb_request_t));

        uart_puts("\nbCharFormat: "); uart_put_hex(line_coding->bCharFormat);
        uart_puts("\nbDataBits: "); uart_put_hex(line_coding->bDataBits);
        uart_puts("\nbParity: "); uart_put_hex(line_coding->bParityType);

        if(leng == sizeof(usb_cdc_line_coding_t))
        {
          usb_cdc_line_coding = *line_coding;
        }

        // send control ZLP
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

      } break;

      case USB_CDC_CMD(IN, INTERFACE, CLASS, GET_LINE_CODING): {
        uart_puts("\nGetLineCoding");

        uint8_t *codingAddr_temp = (uint8_t *)&usb_cdc_line_coding; 

        if (leng <= usb_device_descriptor.bMaxPacketSize0){
        
          memcpy(usb_ctrl_in_buf, codingAddr_temp, leng);
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
        
        }

        else {
  
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)codingAddr_temp;

        }

        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng;          // how big it is
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
    
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;          // clear flag
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;        // start 

        while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait

      } break;

      case USB_CDC_CMD(OUT, INTERFACE, CLASS, SET_CONTROL_LINE_STATE): {
        uart_puts("\nSetControlLineState");

        uart_puts("\nRTS: "); uart_put_hex((wValue_L & 0x02) >> 1);
        uart_puts("\nDTR: "); uart_put_hex(wValue_L & 0x01);

        RTS = (wValue_L & 0x02) >> 1;
        DTR = wValue_L & 0x01;

        // send control ZLP
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

      } break;
      
      default: {

          uart_puts("\nDefaultStall");
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;
          
      } break;
    }
  }  
  
  // Endpoint interrupts
  epint = USB->DEVICE.EPINTSMRY.reg;

  for (int i = 0; epint && i < USB_EPT_NUM; i++){
    
    if (0 == (epint & (1 << i)))
    {
      continue;
    }

    epint &= ~(1 << i);
    
    flags = USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg;
    if (flags & USB_DEVICE_EPINTFLAG_TRCPT0){

      if(i == CDC_ENDPOINT_OUT) {
      
        int incoming_length = EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT & 0xff;
        for (int i=0; i<incoming_length; i++){ incoming_string[i] = usb_cdc_out_buf[i]; }

        uart_puts("\nCDC OUT");
        uart_puts("\nBYTECOUNT: "); uart_put_hex((uint8_t)(EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT & 0xff));
        uart_puts("\nincoming_string: "); uart_puts(incoming_string);

        cmd_recv = true;
        memcpy(command, incoming_string, sizeof(command));
        memset(incoming_string, 0, sizeof(incoming_string)); // reset incoming_string array

        USB->DEVICE.DeviceEndpoint[i].EPSTATUSSET.bit.BK0RDY = 1;
        EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

        USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT0 = 1;
        USB->DEVICE.DeviceEndpoint[i].EPSTATUSCLR.bit.BK0RDY = 1;

        continue;
      }

      USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT0 = 1;
      USB->DEVICE.DeviceEndpoint[i].EPSTATUSSET.bit.BK0RDY = 1;
    
    }
    
    if (flags & USB_DEVICE_EPINTFLAG_TRCPT1){

      if(i == ISO_ENDPOINT_IN)
      {
        if(interface_num != 0)
        {            
          if(bufnum != prevBuf || prevBuf == 4)
          {
            
            prevBuf = bufnum;


            USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
          
            if(bufnum == 0)
            {
              EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer3;
              // uart_puts("\n3--");
            }
            else if(bufnum == 1)
            {
              EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer0;
              // uart_puts("\n0--");
            }
            else if(bufnum == 2)
            {
              EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer1;
              // uart_puts("\n1--");
            }
            else if(bufnum == 3)
            {
              EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer2;
              // uart_puts("\n2--");
            }

            EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = NBEATS;    // size of ADC buffer in SRAM
            EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

            USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;   // start transfer when host sends IN token

          }
        
          else if (bufnum == prevBuf)
          {

            // uart_puts("\nZLP");
            
            USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;

            EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
            EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
            
            USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;

            ZLP_c++;
            
            if (ZLP_c > 10){

              ZLP_c = 0;

              // uart_puts("\nCONTINUE");
              bufnum += 2;
              if (bufnum == 4)
              {
                bufnum = 0;
              }
              
              DMAC->CHID.reg = DMAC_CHID_ID(bufnum); // select active channel
              DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE; // enable

            }

          }
        continue;
        }
      }

      if(i == CDC_ENDPOINT_IN)
      {
        // char * test_message = "Test sending\n";

        // EP[CDC_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)test_message;
        // EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(*test_message);
        // EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
        // USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      }

      if(i == CDC_ENDPOINT_COMM)
      {
        uart_puts("\nCDC COMM");

        usb_cdc_comm_busy = false;

        int one_shot = USB_CDC_SERIAL_STATE_BREAK | USB_CDC_SERIAL_STATE_RING |
          USB_CDC_SERIAL_STATE_FRAMING | USB_CDC_SERIAL_STATE_PARITY |
          USB_CDC_SERIAL_STATE_OVERRUN;

        usb_cdc_notify_message.value &= ~one_shot;
        usb_cdc_serial_state &= ~one_shot;

        usb_cdc_send_state_notify();
      }

      USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT1 = 1;
      USB->DEVICE.DeviceEndpoint[i].EPSTATUSCLR.bit.BK1RDY = 1;
  
    }
  }
  __enable_irq();
}

void usb_pipe_status(){

  __disable_irq();

  uart_puts("\n");

  uart_puts("\nTRFAIL0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRFAIL0+ascii);
  uart_puts("\nTRFAIL1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRFAIL1+ascii);
  uart_puts("\nTRCPT0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT0+ascii);
  uart_puts("\nTRCPT1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1+ascii);

  uart_puts("\n");

  __enable_irq();

}

void fngenerator(){

  int i;
  float phase = 3.14159*3.0*2./NPTS;

  for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * 250.0f + 251.0f; // default
  type waveform = sine;
  
  while(true) {

    if(cmd_recv) {

      cmd_recv = false;

      switch (command[0]){

        case '0':  
          waveform = sine;
          break;
      
        case '1':  
          waveform = pulse;
          break;

        case '2':  
          waveform = square;
          break;

        case '3': 
          waveform = sawtooth;
          break;

        case 'x':
          mute = false; 
          digitalWrite(LED_BUILTIN,HIGH);
          break;

        case 'o':
          mute = true; 
          digitalWrite(LED_BUILTIN,LOW);
          break;

        case 'a':
          //amplitude = map(command[1],0,9,0,250);
          uart_puts("\namplitude: ");uart_put_hex(command[1]);
          uart_puts("\namplitude: ");uart_write(command[1]+ascii);
          //uart_puts("\namplitude_map: ");uart_put_hex(amplitude);
          break;

        default:
          break;
      }

      switch(waveform){
        case 0:  // sine wave
          for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * 250.0f + 251.0f;
          break;
      
        case 1:  // pulse wave
          for (i=0;i<NPTS/20;i++) waveout[i] = 500.0f;
          for (i=NPTS/20;i<NPTS;i++) waveout[i] = 0.0f;
          break;

        case 2:  // square wave
          for (i=0;i<NPTS/2;i++) waveout[i] = 500.0f;
          for (i=NPTS/2;i<NPTS;i++) waveout[i] = 0.0f;
          break;

        case 3:  // sawtooth wave
          for (i=0;i<NPTS/2;i++) waveout[i] = waveout[NPTS-1-i] = 2*510*i/NPTS;
          break;

      }
    }

    if(!mute) {
      for (int i = 0; i < NPTS; i++) { 
        analogWrite(A0,waveout[i]);
      }
    }
  }
}

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
  
  adc_to_sram_dma();
  start_adc_sram_dma(); 

  uart_puts("\nStarting...");

  __enable_irq();
  
}

void loop() {

  fngenerator();
  
}
