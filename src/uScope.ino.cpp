# 1 "C:\\Users\\beb539\\AppData\\Local\\Temp\\tmpesqss1k9"
#include <Arduino.h>
# 1 "C:/Users/beb539/Documents/Courses/F20/GenEd1080/uScope/src/uScope.ino"
# 10 "C:/Users/beb539/Documents/Courses/F20/GenEd1080/uScope/src/uScope.ino"
#include "Arduino.h"
#include "wiring_private.h"
#include "USB/PluggableUSB.h"
#include "USB/CDC.h"
#include "USB/SAMD21_USBDevice.h"
#include "usb_descriptors.h"
#include "usb_enums.h"

#define freq_CPU 48000000
static uint32_t baud = 115200;
uint64_t br = (uint64_t)65536 * (freq_CPU - 16 * baud) / freq_CPU;

#define ADCPIN A6
#define NBEATS 45
#define NPTS 1000

#define CONTROL_ENDPOINT 0
#define ISO_ENDPOINT_IN 1
#define ISO_ENDPOINT_OUT 2
#define CDC_ENDPOINT_COMM 3
#define CDC_ENDPOINT_IN 4
#define CDC_ENDPOINT_OUT 5

#define LIMIT(a,b) (((a) > (b)) ? (b) : (a))
#define USB_CMD(dir,rcpt,type,cmd) \
    ((USB_##cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define USB_AUDIO_CMD(dir,rcpt,type,cmd) \
    ((cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define USB_CDC_CMD(dir,rcpt,type,cmd) \
    ((cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

uint16_t adc_buffer0[NBEATS];
uint16_t adc_buffer1[NBEATS];
uint16_t adc_buffer2[NBEATS];
uint16_t adc_buffer3[NBEATS];
uint16_t waveout[NPTS];

float amplitude = 510.0;
float frequency = 5.0;
float offset = 510.0;

volatile bool mute = false;
volatile uint16_t volume = 5;

static uint16_t usb_ctrl_audio;
static uint32_t usb_ctrl_in_buf[16];
static uint8_t usb_ctrl_out_buf[16];
static uint8_t usb_cdc_out_buf[64] = {0};
static uint8_t usb_cdc_in_buf[64];

char incoming_string[255] = {0};
String control_str = "";
char command[6] = {0};
uint8_t char_count = 0;
bool cmd_recv = 0;

bool RTS = false;
bool DTR = false;

static uint32_t adctobuf0 = 0;
static uint32_t adctobuf1 = 1;
static uint32_t adctobuf2 = 2;
static uint32_t adctobuf3 = 3;

static uint8_t ascii = 48;
static int usb_config;

char *usb_strings[100] = {"", "Arduino + Harvard","uScope by Active Learning","ALL-0001","Isochronous Audio","uScope Instrumentation"};

uint8_t usb_string_descriptor_buffer[64] __attribute__ ((aligned (4)));

volatile uint8_t bufnum = 0;

enum type {sine, pulse, square, sawtooth};

typedef struct __attribute__((packed)){
  uint16_t BTCTRL;
  uint16_t BTCNT;
  uint32_t SRCADDR;
  uint32_t DSTADDR;
  uint32_t DESCADDR;
} dmacdescriptor;

typedef struct __attribute__((packed)) {
  uint8_t bmRequestType;
  uint8_t bRequest;
  uint16_t wValue;
  uint16_t wIndex;
  uint16_t wLength;
} usb_request_t;

typedef struct {
    UsbDeviceDescBank DeviceDescBank[2];
} UsbdDescriptor;

typedef struct {
  uint8_t interface_num;
  uint8_t alternate_setting;
} usb_interface_status_t;

static usb_cdc_line_coding_t usb_cdc_line_coding =
{
  .dwDTERate = 115200,
  .bCharFormat = 0,
  .bParityType = 0,
  .bDataBits = 8,
};

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));
dmacdescriptor descriptor __attribute__ ((aligned (16)));
UsbDeviceDescriptor EP[USB_EPT_NUM] __attribute__ ((aligned (4)));
uint8_t interface_num = 0;
uint8_t alt_setting = 0;

usb_interface_status_t audio_stream_interface;
void uart_init();
void uart_putc(char c);
void uart_write(uint16_t data);
void uart_puts(char *s);
void uart_put_hex(uint8_t x);
void adc_init();
void adc_to_sram_dma();
void start_adc_sram_dma();
void dma_init();
void DMAC_Handler();
void usb_init();
void USB_Handler();
void fngenerator();
void setup();
void loop();
#line 129 "C:/Users/beb539/Documents/Courses/F20/GenEd1080/uScope/src/uScope.ino"
void uart_init() {

   PORT->Group[0].DIRSET.reg = (1 << 10);
   PORT->Group[0].PINCFG[10].bit.PMUXEN = 1;
   PORT->Group[0].PINCFG[10].bit.INEN = 1;
   PORT->Group[0].PMUX[5].bit.PMUXE = PORT_PMUX_PMUXE_C_Val;

   PORT->Group[0].DIRCLR.reg = (1 << 11);
   PORT->Group[0].PINCFG[11].bit.PMUXEN = 1;
   PORT->Group[0].PINCFG[11].bit.INEN = 1;
   PORT->Group[0].PINCFG[11].reg &= ~PORT_PINCFG_PULLEN;
   PORT->Group[0].PMUX[5].bit.PMUXO = PORT_PMUX_PMUXO_C_Val;

   PM->APBCMASK.reg |= PM_APBCMASK_SERCOM0;
   GCLK->CLKCTRL.bit.ID = GCLK_CLKCTRL_ID_SERCOM0_CORE;
   GCLK->CLKCTRL.bit.CLKEN = 0x1;
   GCLK->CLKCTRL.bit.GEN = GCLK_CLKCTRL_GEN_GCLK0;

   SERCOM0->USART.CTRLA.bit.ENABLE=0;
   while(SERCOM0->USART.SYNCBUSY.bit.ENABLE == 1);

   SERCOM0->USART.CTRLA.bit.MODE = 0x01;
   SERCOM0->USART.CTRLA.bit.CMODE = 0;
   SERCOM0->USART.CTRLA.bit.RXPO = 3;
   SERCOM0->USART.CTRLA.bit.TXPO = 1;
   SERCOM0->USART.CTRLA.bit.DORD = 1;
   SERCOM0->USART.CTRLA.bit.FORM = 1;
   SERCOM0->USART.CTRLB.bit.CHSIZE = 0;
   SERCOM0->USART.CTRLB.bit.PMODE = 0;
   SERCOM0->USART.CTRLB.bit.SBMODE = 0;

   SERCOM0->USART.BAUD.reg = (uint16_t)br;
   SERCOM0->USART.INTENSET.bit.TXC = 1;

   SERCOM0->USART.CTRLB.bit.RXEN = 1;
   SERCOM0->USART.CTRLB.bit.TXEN = 1;

   SERCOM0->USART.CTRLA.bit.ENABLE = 1;

   while(SERCOM0->USART.SYNCBUSY.bit.ENABLE == 1);

}

void uart_putc(char c) {

  while (SERCOM0->USART.INTFLAG.bit.DRE == 0);
  SERCOM0->USART.DATA.reg = c;

}

void uart_write(uint16_t data) {

  while (SERCOM0->USART.INTFLAG.bit.DRE == 0);
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

  pinPeripheral(ADCPIN, PIO_ANALOG);

  ADC->CTRLA.bit.ENABLE = 0x00;
  while(ADC->STATUS.bit.SYNCBUSY == 1);

  ADC->INPUTCTRL.bit.GAIN = 0xF;
  ADC->REFCTRL.bit.REFSEL = 0x2;
  while(ADC->STATUS.bit.SYNCBUSY == 1);

  ADC->INPUTCTRL.bit.MUXPOS = g_APinDescription[ADCPIN].ulADCChannelNumber;
  while(ADC->STATUS.bit.SYNCBUSY == 1);




  ADC->AVGCTRL.bit.SAMPLENUM = 0x0;
  ADC->SAMPCTRL.reg = 0x14;
  while(ADC->STATUS.bit.SYNCBUSY == 1);

  ADC->CTRLB.bit.PRESCALER = 0x4;
  ADC->CTRLB.bit.RESSEL = 0x0;
  ADC->CTRLB.bit.FREERUN = 1;
  ADC->CTRLB.bit.DIFFMODE = 0;
  while(ADC->STATUS.bit.SYNCBUSY == 1);

  ADC->CTRLA.bit.ENABLE = 0x01;
  while(ADC->STATUS.bit.SYNCBUSY == 1);

}

void adc_to_sram_dma() {

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST);

  DMAC->CHCTRLB.bit.LVL = 0x03;
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27;
  DMAC->CHCTRLB.bit.TRIGACT = 0x02;
  DMAC->CHINTENSET.bit.TCMPL = 1;

  descriptor.DESCADDR = 0;
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg;
  descriptor.DSTADDR = (uint32_t) adc_buffer0 + 2*NBEATS;
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL;
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC;
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC;
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1);
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID;

  memcpy(&descriptor_section[adctobuf0], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf1);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST);

  DMAC->CHCTRLB.bit.LVL = 0x03;
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27;
  DMAC->CHCTRLB.bit.TRIGACT = 0x02;
  DMAC->CHINTENSET.bit.TCMPL = 1;

  descriptor.DESCADDR = 0;
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg;
  descriptor.DSTADDR = (uint32_t) adc_buffer1 + 2*NBEATS;
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL;
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC;
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC;
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1);
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID;

  memcpy(&descriptor_section[adctobuf1], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf2);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST);

  DMAC->CHCTRLB.bit.LVL = 0x03;
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27;
  DMAC->CHCTRLB.bit.TRIGACT = 0x02;
  DMAC->CHINTENSET.bit.TCMPL = 1;

  descriptor.DESCADDR = 0;
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg;
  descriptor.DSTADDR = (uint32_t) adc_buffer2 + 2*NBEATS;
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL;
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC;
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC;
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1);
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID;

  memcpy(&descriptor_section[adctobuf2], &descriptor, sizeof(dmacdescriptor));

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf3);
  DMAC->CHCTRLA.reg &= ~DMAC_CHCTRLA_ENABLE;

  DMAC->CHCTRLA.reg = DMAC_CHCTRLA_SWRST;
  while(DMAC->CHCTRLA.reg & DMAC_CHCTRLA_SWRST);

  DMAC->CHCTRLB.bit.LVL = 0x03;
  DMAC->CHCTRLB.bit.TRIGSRC = 0x27;
  DMAC->CHCTRLB.bit.TRIGACT = 0x02;
  DMAC->CHINTENSET.bit.TCMPL = 1;

  descriptor.DESCADDR = 0;
  descriptor.SRCADDR = (uint32_t) &ADC->RESULT.reg;
  descriptor.DSTADDR = (uint32_t) adc_buffer3 + 2*NBEATS;
  descriptor.BTCNT = NBEATS;
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSIZE(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_STEPSEL;
  descriptor.BTCTRL |= DMAC_BTCTRL_DSTINC;
  descriptor.BTCTRL &= ~DMAC_BTCTRL_SRCINC;
  descriptor.BTCTRL |= DMAC_BTCTRL_BEATSIZE(0x1);
  descriptor.BTCTRL |= DMAC_BTCTRL_BLOCKACT(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_EVOSEL(0x0);
  descriptor.BTCTRL |= DMAC_BTCTRL_VALID;

  memcpy(&descriptor_section[adctobuf3], &descriptor, sizeof(dmacdescriptor));

}

void start_adc_sram_dma() {

  DMAC->CHID.reg = DMAC_CHID_ID(adctobuf0);
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;
}

void dma_init() {

  PM->AHBMASK.reg |= PM_AHBMASK_DMAC;
  PM->APBBMASK.reg |= PM_APBBMASK_DMAC;

  NVIC_SetPriority(DMAC_IRQn, 0x00);
  NVIC_EnableIRQ(DMAC_IRQn);

  DMAC->BASEADDR.reg = (uint32_t)descriptor_section;
  DMAC->WRBADDR.reg = (uint32_t)wrb;
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf);

}

void DMAC_Handler() {

  __disable_irq();
  digitalWrite(5, !digitalRead(5));

  bufnum = (uint8_t)(DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk);
  DMAC->CHID.reg = DMAC_CHID_ID(bufnum);
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL;


  DMAC->CHID.reg = (bufnum + 1 == 4) ? (DMAC_CHID_ID(0)) : (DMAC_CHID_ID(bufnum + 1));
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
  EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

  switch((audio_stream_interface.alternate_setting << 4) | bufnum)
  {
    case 0x10:
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer3;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
    case 0x11:
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer0;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
    case 0x12:
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer1;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
    case 0x13:
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer2;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
  }

  __enable_irq();
}

void usb_init() {




  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while(USB->DEVICE.SYNCBUSY.bit.SWRST);

  memset((uint8_t *)EP, 0, sizeof(EP));
  USB->DEVICE.DESCADD.reg = (uint32_t)EP;



  uint32_t *pad_transn_p = (uint32_t *) USB_FUSES_TRANSN_ADDR;
  uint32_t *pad_transp_p = (uint32_t *) USB_FUSES_TRANSP_ADDR;
  uint32_t *pad_trim_p = (uint32_t *) USB_FUSES_TRIM_ADDR;

  uint32_t pad_transn = (*pad_transn_p & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
  uint32_t pad_transp = (*pad_transp_p & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
  uint32_t pad_trim = (*pad_trim_p & USB_FUSES_TRIM_Msk ) >> USB_FUSES_TRIM_Pos;

  if (pad_transn == 0x1F)
    pad_transn = 5;
  if (pad_transp == 0x1F)
    pad_transp = 29;
  if (pad_trim == 0x7)
    pad_trim = 3;

  USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;
  USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;
  USB->DEVICE.PADCAL.bit.TRIM = pad_trim;

  USB->DEVICE.CTRLA.bit.MODE = USB_CTRLA_MODE_DEVICE_Val;
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 1;
  USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
  USB->DEVICE.CTRLB.bit.DETACH = 0;

  USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST;


  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

  audio_stream_interface.interface_num = 1;

  NVIC_SetPriority(USB_IRQn, 0x01);
  NVIC_EnableIRQ(USB_IRQn);

}

void USB_Handler(){

  int epint, flags;


  if(USB->DEVICE.INTFLAG.bit.EORST) {

    uart_puts("\n\nReset");

    USB->DEVICE.INTFLAG.bit.EORST = 1;
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    for (int i = 0; i < USB_EPT_NUM; i++)
    {
      USB->DEVICE.DeviceEndpoint[i].EPCFG.bit.EPTYPE0 = USB_DEVICE_EPCFG_EPTYPE_DISABLED;
      USB->DEVICE.DeviceEndpoint[i].EPCFG.bit.EPTYPE1 = USB_DEVICE_EPCFG_EPTYPE_DISABLED;
    }

    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0 = USB_DEVICE_EPCFG_EPTYPE_CONTROL;
    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1 = USB_DEVICE_EPCFG_EPTYPE_CONTROL;
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


  if (USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP){

    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP = 1;
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

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = leng;
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

          while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

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

          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = leng;
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

          while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

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

            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = leng;
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

          }

          else if (index < USB_STR_COUNT){

            char *str = usb_strings[index];
            int len = strlen(str);

            memset(usb_string_descriptor_buffer, 0, sizeof(usb_string_descriptor_buffer));

            usb_string_descriptor_buffer[0] = len*2 + 2;
            usb_string_descriptor_buffer[1] = USB_STRING_DESCRIPTOR;

            for (int i = 0; i < len; i++){
              usb_string_descriptor_buffer[2 + i*2] = str[i];
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

            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = leng;
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

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

          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPCFG.bit.EPTYPE1 = 2;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
          EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_1023;
          EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = NBEATS*2;

          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPCFG.bit.EPTYPE1 = 4;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPSTATUSSET.bit.BK1RDY = 1;

          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_cdc_in_buf;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPCFG.bit.EPTYPE1 = 3;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPINTENSET.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;

          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].ADDR.reg = (uint32_t)&usb_cdc_out_buf;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 64;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPCFG.bit.EPTYPE0 = 3;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPINTENSET.bit.TRCPT0 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPSTATUSCLR.bit.DTGLOUT = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPSTATUSCLR.bit.BK0RDY = 1;

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

        uart_puts("\nSetInterface");

        interface_num = wIndex_L;
        alt_setting = wValue_L;

        uart_puts("\nSending ZLP");


        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

        if(interface_num == 1)
        {
          if(alt_setting == 0)
          {
            audio_stream_interface.alternate_setting = 0;
          }
          else if(alt_setting == 1)
          {
            audio_stream_interface.alternate_setting = 1;
          }
        }

      } break;

      case USB_CMD(IN, ENDPOINT, STANDARD, GET_STATUS): {

        uart_puts("\nGetStatus: Endpoint");

        int ep = request->wIndex & USB_INDEX_MASK;
        int dir = request->wIndex & USB_DIRECTION_MASK;
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


        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while (0 == USB->DEVICE.DeviceEndpoint[0].EPINTFLAG.bit.TRCPT1);

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

        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = leng;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

        while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

      } break;

      case USB_CDC_CMD(OUT, INTERFACE, CLASS, SET_CONTROL_LINE_STATE): {
        uart_puts("\nSetControlLineState");

        uart_puts("\nRTS: "); uart_put_hex((wValue_L & 0x02) >> 1);
        uart_puts("\nDTR: "); uart_put_hex(wValue_L & 0x01);

        RTS = (wValue_L & 0x02) >> 1;
        DTR = wValue_L & 0x01;


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


  epint = USB->DEVICE.EPINTSMRY.reg;

  for (int i = 0; epint && i < USB_EPT_NUM; i++){

    if (0 == (epint & (1 << i))){ continue; }
    epint &= ~(1 << i);

    flags = USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg;

    if (flags & USB_DEVICE_EPINTFLAG_TRCPT1){

      USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT1 = 1;
      USB->DEVICE.DeviceEndpoint[i].EPSTATUSCLR.bit.BK1RDY = 1;

    }

    if (flags & USB_DEVICE_EPINTFLAG_TRCPT0){

      if(i == CDC_ENDPOINT_OUT){

        int incoming_length = EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT & 0xff;
        for (int i=0; i<incoming_length; i++) { incoming_string[i] = usb_cdc_out_buf[i]; }

        uart_puts("\nCDC OUT");
        uart_puts("\nBYTECOUNT: "); uart_put_hex((uint8_t)(EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT & 0xff));
        uart_puts("\nincoming_string: "); uart_puts(incoming_string);

        cmd_recv = true;
        memcpy(command, incoming_string, sizeof(command));
        memset(incoming_string, 0, sizeof(incoming_string));

        USB->DEVICE.DeviceEndpoint[i].EPSTATUSSET.bit.BK0RDY = 1;
        EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

        USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT0 = 1;
        USB->DEVICE.DeviceEndpoint[i].EPSTATUSCLR.bit.BK0RDY = 1;

        continue;
      }

      USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT0 = 1;
      USB->DEVICE.DeviceEndpoint[i].EPSTATUSSET.bit.BK0RDY = 1;

    }
  }
}

void fngenerator(){

  int i; type waveform = sine;
  float phase = (2.0*3.14159*frequency)/(NPTS);
  for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * amplitude + amplitude + 2.0f;

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

        case 'i':
          mute = true;
          digitalWrite(LED_BUILTIN,LOW);
          break;

        case 'a':

          control_str = "";
          for (int i = 0; i < 5; i++){ control_str += command[i+1]; }

          amplitude = map(control_str.toInt(),100,3300,15,510);



          break;

        case 'o':

          control_str = "";
          for (int i = 0; i < 5; i++){ control_str += command[i+1]; }

          offset = map(control_str.toInt(),300,3000,15,900);



          break;

        case 'f':

          control_str = "";
          for (int i = 0; i < 5; i++){ control_str += command[i+1]; }

          frequency = map(control_str.toInt(),200,20000,1,100);



          break;

        default:
          break;
      }

      phase = (2.0*3.14159*frequency)/(NPTS);

      switch(waveform){

        case 0:
          for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * amplitude + offset;
          break;

        case 1:
          for (i=0;i<NPTS/20;i++) waveout[i] = 2.0 * amplitude;
          for (i=NPTS/20;i<NPTS;i++) waveout[i] = 0.0f;
          break;

        case 2:
          for (i=0;i<NPTS/2;i++) waveout[i] = 2.0 * amplitude;
          for (i=NPTS/2;i<NPTS;i++) waveout[i] = 0.0f;
          break;

        case 3:
          for (i=0;i<NPTS/2;i++) waveout[i] = waveout[NPTS-1-i] = 4.0*amplitude*i/NPTS;
          break;

      }
    }

    if(!mute) {
      for (int i = 0; i < NPTS; i++) {
          analogWrite(A0,waveout[i]);
          delayMicroseconds(1);
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