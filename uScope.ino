/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 11 August 2020
 */

#include "Arduino.h"          // required before wiring_private.h, also includes USBDesc.h, USBCore.h, USBAPI.h, and USB_host.h
#include "wiring_private.h"   // for pinPeripheral in adc_init()
#include "USB/PluggableUSB.h"
#include "USB/CDC.h"
#include "USB/SAMD21_USBDevice.h"

#define freq_CPU 48000000 // CPU clock frequency
static uint32_t baud = 115200; // for UART debug of USB
uint64_t br = (uint64_t)65536 * (freq_CPU - 16 * baud) / freq_CPU; // to pass to SERCOM0->USART.BAUD.reg

#define ADCPIN A1           // selected arbitrarily, consider moving away from DAC / A0
#define NBEATS 64           // number of beats for adc transfer
#define NPTS 1024           // number of points within waveform definition
#define CONTROL_ENDPOINT 0
#define CDC_ENDPOINT_OUT 2
#define CDC_ENDPOINT_IN  3

#define LIMIT(a, b)     (((a) > (b)) ? (b) : (a))
#define USB_CMD(dir, rcpt, type, cmd) \
    ((USB_##cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

uint8_t adc_buffer0[NBEATS];              // buffer with length set by NBEATS
uint8_t adc_buffer1[NBEATS];              // alternating buffer
uint16_t waveout[NPTS];                   // buffer for waveform

static uint32_t usb_ctrl_in_buf[16];
static uint32_t usb_ctrl_out_buf[16];

static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1

static uint8_t ascii = 48;      // offset to interpret single digit uart outputs
static int usb_config;

char *usb_strings[] = {"Arduino + Harvard Active Learning","MKR Zero uScope","ALL-0001","Main Configuration","Main Interface"};

volatile bool bufnum;  // track which buffer to write to, while USB reads

extern USBDevice_SAMD21G18x usbd; // defined in USBCore.cpp
extern UsbDeviceDescriptor usb_endpoints[];
extern const uint8_t usb_num_endpoints;

enum type {sine, sawtooth}; // supported waveform types

enum{
  USB_OUT_TRANSFER         = 0,
  USB_IN_TRANSFER          = 1,
};

enum{
  USB_STANDARD_REQUEST     = 0,
  USB_CLASS_REQUEST        = 1,
  USB_VENDOR_REQUEST       = 2,
};

enum{
  USB_IN_ENDPOINT          = 0x80,
  USB_OUT_ENDPOINT         = 0x00,
  USB_INDEX_MASK           = 0x7f,
  USB_DIRECTION_MASK       = 0x80,
};

enum
{
  USB_CONTROL_ENDPOINT     = 0 << 0,
  USB_ISOCHRONOUS_ENDPOINT = 1 << 0,
  USB_BULK_ENDPOINT        = 2 << 0,
  USB_INTERRUPT_ENDPOINT   = 3 << 0,

  USB_NO_SYNCHRONIZATION   = 0 << 2,
  USB_ASYNCHRONOUS         = 1 << 2,
  USB_ADAPTIVE             = 2 << 2,
  USB_SYNCHRONOUS          = 3 << 2,
};

enum{
  USB_DEVICE_EPCFG_EPTYPE_DISABLED    = 0,
  USB_DEVICE_EPCFG_EPTYPE_CONTROL     = 1,
  USB_DEVICE_EPCFG_EPTYPE_ISOCHRONOUS = 2,
  USB_DEVICE_EPCFG_EPTYPE_BULK        = 3,
  USB_DEVICE_EPCFG_EPTYPE_INTERRUPT   = 4,
  USB_DEVICE_EPCFG_EPTYPE_DUAL_BANK   = 5,
};

enum{
  USB_DEVICE_RECIPIENT     = 0,
  USB_INTERFACE_RECIPIENT  = 1,
  USB_ENDPOINT_RECIPIENT   = 2,
  USB_OTHER_RECIPIENT      = 3,
};

enum{
  USB_DEVICE_PCKSIZE_SIZE_8    = 0,
  USB_DEVICE_PCKSIZE_SIZE_16   = 1,
  USB_DEVICE_PCKSIZE_SIZE_32   = 2,
  USB_DEVICE_PCKSIZE_SIZE_64   = 3,
  USB_DEVICE_PCKSIZE_SIZE_128  = 4,
  USB_DEVICE_PCKSIZE_SIZE_256  = 5,
  USB_DEVICE_PCKSIZE_SIZE_512  = 6,
  USB_DEVICE_PCKSIZE_SIZE_1023 = 7,
};

enum{
  USB_GET_STATUS        = 0,
  USB_CLEAR_FEATURE     = 1,
  USB_SET_FEATURE       = 3,
  USB_SET_ADDRESS       = 5,
  USB_GET_DESCRIPTOR    = 6,
  USB_SET_DESCRIPTOR    = 7,
  USB_GET_CONFIGURATION = 8,
  USB_SET_CONFIGURATION = 9,
  USB_GET_INTERFACE     = 10,
  USB_SET_INTERFACE     = 11,
  USB_SYNCH_FRAME       = 12,
};

enum{
  USB_STR_ZERO,
  USB_STR_MANUFACTURER,
  USB_STR_PRODUCT,
  USB_STR_SERIAL_NUMBER,
  USB_STR_CONFIGURATION,
  USB_STR_INTERFACE,
  USB_STR_COUNT,
};

enum{
  USB_DEVICE_DESCRIPTOR                    = 1,
  USB_CONFIGURATION_DESCRIPTOR             = 2,
  USB_STRING_DESCRIPTOR                    = 3,
  USB_INTERFACE_DESCRIPTOR                 = 4,
  USB_ENDPOINT_DESCRIPTOR                  = 5,
};

typedef struct {
  uint16_t BTCTRL;   // block transfer control
  uint16_t BTCNT;    // block transfer count
  uint32_t SRCADDR;  // source address
  uint32_t DSTADDR;  // destination address
  uint32_t DESCADDR; // next descriptor address
} dmacdescriptor;

typedef struct {
  uint8_t   bmRequestType;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} usb_request_t;

typedef struct {
  uint8_t  bLength            = 18; // bytes
  uint8_t  bDescriptorType    = 1; // for device
  uint16_t bcdUSB             = 0x0200; // version of USB spec, here 2.0
  uint8_t  bDeviceClass       = 0x00; // 0x00 = none / defined at interface level later, 0x0a = CDC data
  uint8_t  bDeviceSubClass    = 0x00; // within audio, ...
  uint8_t  bDeviceProtocol    = 0x00; // 0x00 = none
  uint8_t  bMaxPacketSize0    = 64;
  uint16_t idVendor           = 0x2341; // Arduino 0x2341
  uint16_t idProduct          = 0x804f; // MKZero 0x804f
  uint16_t bcdDevice          = 0x0100; // release number of the device
  uint8_t  iManufacturer      = USB_STR_MANUFACTURER;
  uint8_t  iProduct           = USB_STR_PRODUCT;
  uint8_t  iSerialNumber      = USB_STR_SERIAL_NUMBER;
  uint8_t  bNumConfigurations = 1;
} deviceDescriptor;

typedef struct {
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t   bNumInterfaces;
  uint8_t   bConfigurationValue;
  uint8_t   iConfiguration;
  uint8_t   bmAttributes;
  uint8_t   bMaxPower;
} usb_configuration_descriptor_t;

typedef struct {
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bInterfaceNumber;
  uint8_t   bAlternateSetting;
  uint8_t   bNumEndpoints;
  uint8_t   bInterfaceClass;
  uint8_t   bInterfaceSubClass;
  uint8_t   bInterfaceProtocol;
  uint8_t   iInterface;
} usb_interface_descriptor_t;

typedef struct {
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bEndpointAddress;
  uint8_t   bmAttributes;
  uint16_t  wMaxPacketSize;
  uint8_t   bInterval;
} usb_endpoint_descriptor_t;

typedef struct {
  usb_configuration_descriptor_t  configuration;
  usb_interface_descriptor_t      interface;
  usb_endpoint_descriptor_t       ep_in;
  usb_endpoint_descriptor_t       ep_out;
} usb_configuration_hierarchy_t;

alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy = {
  
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 1,
    .bConfigurationValue = 1,
    .iConfiguration      = USB_STR_CONFIGURATION,
    .bmAttributes        = 0x80,
    .bMaxPower           = 200, // 400 mA
  },

  .interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0,
    .bAlternateSetting   = 0,
    .bNumEndpoints       = 2,
    .bInterfaceClass     = 0x03,
    .bInterfaceSubClass  = 0x00,
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .ep_in =
  {
    .bLength             = sizeof(usb_endpoint_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = USB_IN_ENDPOINT | 1,
    .bmAttributes        = USB_BULK_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 1,
  },

  .ep_out =
  {
    .bLength             = sizeof(usb_endpoint_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = USB_OUT_ENDPOINT | 2,
    .bmAttributes        = USB_BULK_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 1,
  },
};

typedef struct  {
  uint8_t bLength         = 4;
  uint8_t bDescriptorType = USB_STRING_DESCRIPTOR; 
  uint16_t wLANGID        = 0x0409;   // US English
} stringDescriptor;

typedef struct { 
    UsbDeviceDescBank DeviceDescBank[2]; 
} UsbdDescriptor;

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16))); // write-back descriptor
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16))); // channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));
deviceDescriptor deviceDescriptor_usb __attribute__ ((aligned (4)));
stringDescriptor string0Descriptor_usb __attribute__ ((aligned (4)));
UsbDeviceDescriptor EP[USB_EPT_NUM] __attribute__ ((aligned (4)));
uint8_t usb_string_descriptor_buffer[64] __attribute__ ((aligned (4)));

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
   SERCOM0->USART.CTRLA.bit.RXPO = 3;      // pad to be used for RX, pin 3 in Arduino-land
   SERCOM0->USART.CTRLA.bit.TXPO = 1;      // pad to be sued for TX, pin 2 in Arduino-land
   SERCOM0->USART.CTRLA.bit.DORD = 1;      // LSB transmitted first
   SERCOM0->USART.CTRLA.bit.FORM = 1;      // USART frame with parity
   SERCOM0->USART.CTRLB.bit.CHSIZE = 0;    // 8 bits
   SERCOM0->USART.CTRLB.bit.PMODE = 0;     // parity even
   SERCOM0->USART.CTRLB.bit.SBMODE = 0;    // 1 stop bit

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

void uart_write(uint16_t data){
  
  while (SERCOM0->USART.INTFLAG.bit.DRE == 0); // wait for DATA.reg to be empty
  SERCOM0->USART.DATA.reg = (uint32_t)data;  

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

  ADC->CTRLB.bit.PRESCALER = 0x3; // 0x3 = DIV32 or 0x2 = DIV16
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

  NVIC_SetPriority(DMAC_IRQn, 0x01); // second priority
  NVIC_EnableIRQ(DMAC_IRQn); // enable interrupts, will trigger DMAC_Handler

  DMAC->BASEADDR.reg = (uint32_t)descriptor_section; // where to find descriptor
  DMAC->WRBADDR.reg = (uint32_t)wrb; // holds descriptor if interrupted 
  DMAC->CTRL.reg = DMAC_CTRL_DMAENABLE | DMAC_CTRL_LVLEN(0xf); // enable DMA, priority level

}

void DMAC_Handler() { // primary, non-USB DMAC ISR

  __disable_irq(); // disable interrupts
  
  bufnum =  DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk; // grab active channel
  DMAC->CHID.reg = DMAC_CHID_ID(bufnum); // select active channel
  DMAC->CHINTFLAG.reg = DMAC_CHINTENCLR_TCMPL; // clear transfer complete flag

  if(bufnum == 0){
    usbd.epBank1SetAddress(CDC_ENDPOINT_IN, &adc_buffer0);
  }
  else{
    usbd.epBank1SetAddress(CDC_ENDPOINT_IN, &adc_buffer1);
  }
 
  usbd.epBank1SetByteCount(CDC_ENDPOINT_IN, NBEATS); // each beat is 8 bits
  usbd.epBank1AckTransferComplete(CDC_ENDPOINT_IN);
  usbd.epBank1SetReady(CDC_ENDPOINT_IN);

  __enable_irq(); // enable interrupts

}

void usb_init() 
{

  memset((uint8_t *)EP, 0, sizeof(EP));
  USB->DEVICE.DESCADD.reg = (uint32_t)EP;
  
  USB->DEVICE.CTRLA.bit.MODE = USB_CTRLA_MODE_DEVICE_Val;
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 1;
  USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
  USB->DEVICE.CTRLB.bit.DETACH = 0;
  
  USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST;  
  USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTENSET.bit.RXSTP = 1;
  
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;
  
  NVIC_SetPriority(USB_IRQn, 0x00); // top priority
  NVIC_EnableIRQ(USB_IRQn); // will trigger USB_Handler

}

void USB_Handler(){

  int epint, flags;

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

    EP[CONTROL_ENDPOINT].DeviceDescBank[0].ADDR.reg = (uint32_t)usb_ctrl_out_buf;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPINTENSET.bit.RXSTP = 1;
    
  }

  if (USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP){

    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP = 1;  // acknowledge interrupt
    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSCLR.bit.BK0RDY = 1;
    
    usb_request_t * request = (usb_request_t*) usb_ctrl_out_buf;

    uart_puts("\n\nbRequest: ");uart_write(request->bRequest+ascii); // key for 0,1,3,5,6,7,8,9,10,11,12

    uint8_t wValue_L = request->wValue & 0xff;
    uint8_t wValue_H = request->wValue >> 8;
    uint8_t type = request->wValue >> 8;
    uint8_t index = request->wValue & 0xff;
    uint16_t leng = request->wLength;

    switch ((request->bRequest << 8) | request->bmRequestType){
      case USB_CMD(IN, DEVICE, STANDARD, GET_DESCRIPTOR):{

        uart_puts("\nGetDescriptor");
          
        if (type == USB_DEVICE_DESCRIPTOR){
            
          uart_puts("\nDevice");

          leng = LIMIT(leng, deviceDescriptor_usb.bLength);

          uint8_t *descAddr_temp = (uint8_t *)&deviceDescriptor_usb; 

          if (leng <= deviceDescriptor_usb.bMaxPacketSize0){
          
            memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
          }

          else {
    
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

          }
  
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng; // how big it is
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1; // clear flag
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1; // start 

          while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait
          
        } 
          
        else if (type == USB_CONFIGURATION_DESCRIPTOR){

          uart_puts("\nConfiguration");

          leng = LIMIT(leng, usb_configuration_hierarchy.configuration.wTotalLength);

          uint8_t *descAddr_temp = (uint8_t *)&usb_configuration_hierarchy; 

          if (leng <= deviceDescriptor_usb.bMaxPacketSize0){
          
            memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
          }

          else {
    
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

          }
  
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng; // how big it is
          EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1; // clear flag
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1; // start 

          while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait
          
          usb_pipe_status();

        }
          
        else if (type == USB_STRING_DESCRIPTOR){
            
          uart_puts("\nString0");
            
          if(index == 0){
              
            leng = LIMIT(leng, string0Descriptor_usb.bLength);

            uint8_t *descAddr_temp = (uint8_t *)&string0Descriptor_usb; 

            if (leng <= deviceDescriptor_usb.bMaxPacketSize0){
          
              memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
            }

            else {
    
             EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

            }
  
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng; // how big it is
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1; // clear flag
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1; // start 

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait
              
          }

          else if (0 < index < USB_STR_COUNT){

            uart_puts("\nStringN");
              
            char *str = usb_strings[index];
            int len = strlen(str);

            memset(usb_string_descriptor_buffer, 0, sizeof(usb_string_descriptor_buffer));

            usb_string_descriptor_buffer[0] = len*2 + 2;
            usb_string_descriptor_buffer[1] = USB_STRING_DESCRIPTOR;

            for (int i = 0; i < len; i++){
              usb_string_descriptor_buffer[2 + i*2] = str[i];
            }

            leng = LIMIT(leng, string0Descriptor_usb.bLength);

            uint8_t *descAddr_temp = (uint8_t *)&usb_string_descriptor_buffer; 

            if (leng <= deviceDescriptor_usb.bMaxPacketSize0){
          
              memcpy(usb_ctrl_in_buf, descAddr_temp, leng);
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
          
            }

            else {
    
             EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)descAddr_temp;

            }
  
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = leng; // how big it is
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1; // clear flag
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1; // start 

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1); // wait  
           
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

        uart_puts("\nDADD_L from request: "); uart_write(wValue_L);
        uart_puts("\nDADD_H from request: "); uart_write(wValue_H);

        USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN | USB_DEVICE_DADD_DADD(request->wValue);

        uart_puts("\nDADD in register: "); uart_write(USB->DEVICE.DADD.bit.DADD);
      
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

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPCFG.bit.EPTYPE1 = 3; // Bulk IN
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPINTENSET.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
          EP[CDC_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPCFG.bit.EPTYPE0 = 3; // Bulk OUT
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPINTENSET.bit.TRCPT0 = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPSTATUSCLR.bit.DTGLOUT = 1;
          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_OUT].EPSTATUSSET.bit.BK0RDY = 1;
          EP[CDC_ENDPOINT_OUT].DeviceDescBank[0].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
        
        }
      } break;

      case USB_CMD(IN, DEVICE, STANDARD, GET_CONFIGURATION): {

        uart_puts("\nGetConfiguration");

        uint8_t config = usb_config;

        memcpy(usb_ctrl_in_buf, &config, sizeof(config));
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = sizeof(config);
        EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;
        while(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);

      } break;

      // dummy cases from ataradov's code
      // USB_CMD(IN, DEVICE, STANDARD, GET_STATUS)
      // USB_CMD(IN, INTERFACE, STANDARD, GET_STATUS)

      case USB_CMD(IN, ENDPOINT, STANDARD, GET_STATUS): {

        uart_puts("\nGetStatus");

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

        uart_puts("\nStall");
        
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;

      } break;

      case USB_CMD(OUT, INTERFACE, STANDARD, SET_FEATURE): {

        uart_puts("\nStall");
      
        USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;

      } break;

      case USB_CMD(OUT, ENDPOINT, STANDARD, SET_FEATURE): {
        
        int ep = request->wIndex & USB_INDEX_MASK; // USB_INDEX_MASK = 0x7f
        int dir = request->wIndex & USB_DIRECTION_MASK; // USB_DIRECTION_MASK = 0x80

        // TODO
      
      } break;

      // TOD0:

      // USB_CMD(OUT, DEVICE, STANDARD, CLEAR_FEATURE) (stall in ataradov)
      // USB_CMD(OUT, INTERFACE, STANDARD, CLEAR_FEATURE) (stall in ataradov)
      // USB_CMD(OUT, ENDPOINT, STANDARD, CLEAR_FEATURE)
      // USB_CMD(IN, INTERFACE, STANDARD, GET_DESCRIPTOR) (see line 253 of usb.c in ataradov's)
      
      default: {

          uart_puts("\nDefaultStall");
          USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.STALLRQ1 = 1;
          
      } break;
    }
  }  
  
  epint = USB->DEVICE.EPINTSMRY.reg;

  for (int i = 0; epint && i < USB_EPT_NUM; i++){
    
    if (0 == (epint & (1 << i)))
      continue;
    
    epint &= ~(1 << i);
    
    flags = USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.reg;

    if (flags & USB_DEVICE_EPINTFLAG_TRCPT0){
      
      USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT0 = 1;
      USB->DEVICE.DeviceEndpoint[i].EPSTATUSSET.bit.BK0RDY = 1;
    
    }
    
    if (flags & USB_DEVICE_EPINTFLAG_TRCPT1){
          
      USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT1 = 1;
      USB->DEVICE.DeviceEndpoint[i].EPSTATUSCLR.bit.BK1RDY = 1;
  
    }
  }
}

void usb_status(){

  __disable_irq();

  uart_puts("\n\nStatus at this point (see above line)\n");

  uart_puts("\nFSMSTATE: "); uart_write(USB->DEVICE.FSMSTATUS.bit.FSMSTATE+ascii);
  uart_puts("\nSPEEDCONF: "); uart_write(USB->DEVICE.CTRLB.bit.SPDCONF+ascii);

  uart_puts("\nEORST: "); uart_write(USB->DEVICE.INTFLAG.bit.EORST+ascii);
  uart_puts("\nSOF: "); uart_write(USB->DEVICE.INTFLAG.bit.SOF+ascii);

  uart_puts("\nEPCFG.EPTYPE0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPCFG.bit.EPTYPE0+ascii);
  uart_puts("\nEPCFG.EPTYPE1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPCFG.bit.EPTYPE1+ascii);

  uart_puts("\nCURBK: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.CURBK+ascii);
  uart_puts("\nBK0RDY: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.BK0RDY+ascii);
  uart_puts("\nBK1RDY: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.BK1RDY+ascii);
  uart_puts("\nSTALLRQ0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.STALLRQ0+ascii);
  uart_puts("\nSTALLRQ1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.STALLRQ1+ascii); 
  uart_puts("\nDTGLIN: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.DTGLIN+ascii);
  uart_puts("\nDTGLOUT: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUS.bit.DTGLOUT+ascii);
  
  uart_puts("\nRXSTP: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP+ascii);
  uart_puts("\nSTALL0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.STALL0+ascii);
  uart_puts("\nSTALL1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.STALL1+ascii);
  uart_puts("\nTRFAIL0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRFAIL0+ascii);
  uart_puts("\nTRFAIL1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRFAIL1+ascii);
  uart_puts("\nTRCPT0: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT0+ascii);
  uart_puts("\nTRCPT1: "); uart_write(USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1+ascii);

  uart_puts("\n");

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
      break;
      
  }
  
  while(1){
    
    for (int i = 0; i < NPTS; i++){ 
      analogWrite(A0,waveout[i]);
    }
  }
}

void setup() {

  __disable_irq();

  uart_init();
  delay(1000);
  uart_puts("\nInitializing...");
  
  analogWriteResolution(10);

  adc_init();
  dma_init(); 
  usb_init();
  
  adc_to_sram_dma();

  bufnum = 0;
  start_adc_sram_dma(); 

  uart_puts("\nStarting...");

  __enable_irq();
  
}

void loop() {

  type waveform = sine;
  test_dac(waveform);
  
}
