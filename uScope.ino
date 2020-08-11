/*
 * Harvard University, Active Learning Labs
 * Simultaneous oscilloscope / waveform generator
 * Intended for use with MKR Zero (SAMD21)
 * 
 * J. Evan Smith, Ben Y. Brown
 * Last revised: 10 August 2020
 * 
 * Checkpoint (v1.3): add USB descriptors, add USB_Handler, clean up USB utility
 * 
 */

#include "Arduino.h"          // required before wiring_private.h, also includes USBDesc.h, USBCore.h, USBAPI.h, and USB_host.h
#include "wiring_private.h"   // for pinPeripheral in adc_init()
#include "USB/PluggableUSB.h"
#include "USB/CDC.h"
#include "USB/SAMD21_USBDevice.h"

#define F_CPU 48000000 // CPU clock frequency
static uint32_t baud = 115200; // for UART debug of USB
uint64_t br = (uint64_t)65536 * (F_CPU - 16 * baud) / F_CPU; // to pass to SERCOM0->USART.BAUD.reg

#define ADCPIN A1           // selected arbitrarily, consider moving away from DAC / A0
#define NBEATS 2048         // number of beats for adc transfer
#define NPTS 1024           // number of points within waveform definition
#define TX_TIMEOUT_MS 70    // timeout for USB transfer in ms
#define CONTROL_ENDPOINT 0
#define CDC_ENDPOINT_OUT 2
#define CDC_ENDPOINT_IN  3
#define USB_CMD(dir, rcpt, type, cmd) \
    ((USB_##cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

uint8_t adc_buffer0[NBEATS];              // buffer with length set by NBEATS
uint8_t adc_buffer1[NBEATS];              // alternating buffer
uint8_t empty_buffer[NBEATS] = {0};       // for testing
uint8_t usb_string_descriptor_buffer[64];
uint16_t waveout[NPTS];                   // buffer for waveform

static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1
static uint8_t ascii = 48;      // offset to interpret single digit uart outputs

static char prevTransmitTimedOut[7] ={0,0,0,0,0,0,0};
char *usb_strings[] = {"Arduino + Harvard Active Learning","MKR Zero uScope","ALL-0001","Main Configuration","Main Interface"};

volatile bool bufnum;  // track which buffer to write to, while USB reads
volatile bool dmadone; // track sram to usart transfer

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
  USB_DEVICE_RECIPIENT     = 0,
  USB_INTERFACE_RECIPIENT  = 1,
  USB_ENDPOINT_RECIPIENT   = 2,
  USB_OTHER_RECIPIENT      = 3,
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
  USB_DEVICE_EPCFG_EPTYPE_DISABLED    = 0,
  USB_DEVICE_EPCFG_EPTYPE_CONTROL     = 1,
  USB_DEVICE_EPCFG_EPTYPE_ISOCHRONOUS = 2,
  USB_DEVICE_EPCFG_EPTYPE_BULK        = 3,
  USB_DEVICE_EPCFG_EPTYPE_INTERRUPT   = 4,
  USB_DEVICE_EPCFG_EPTYPE_DUAL_BANK   = 5,
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
  USB_DEVICE_DESCRIPTOR                    = 1,
  USB_CONFIGURATION_DESCRIPTOR             = 2,
  USB_STRING_DESCRIPTOR                    = 3,
  USB_INTERFACE_DESCRIPTOR                 = 4,
  USB_ENDPOINT_DESCRIPTOR                  = 5,
  USB_DEVICE_QUALIFIER_DESCRIPTOR          = 6,
  USB_OTHER_SPEED_CONFIGURATION_DESCRIPTOR = 7,
  USB_INTERFACE_POWER_DESCRIPTOR           = 8,
  USB_OTG_DESCRIPTOR                       = 9,
  USB_DEBUG_DESCRIPTOR                     = 10,
  USB_INTERFACE_ASSOCIATION_DESCRIPTOR     = 11,
  USB_BINARY_OBJECT_STORE_DESCRIPTOR       = 15,
  USB_DEVICE_CAPABILITY_DESCRIPTOR         = 16,
};

typedef union{
  UsbDeviceDescBank    bank[2];
  struct{
    UsbDeviceDescBank  out;
    UsbDeviceDescBank  in;
  };
} udc_mem_t;

static udc_mem_t udc_mem[USB_EPT_NUM];
static uint32_t usb_ctrl_in_buf[16];
static uint32_t usb_ctrl_out_buf[16];

typedef struct {
  uint16_t BTCTRL;   // block transfer control
  uint16_t BTCNT;    // block transfer count
  uint32_t SRCADDR;  // source address
  uint32_t DSTADDR;  // destination address
  uint32_t DESCADDR; // next descriptor address
} dmacdescriptor;

typedef struct PACK{
  uint8_t   bmRequestType;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} usb_request_t;

typedef struct configuration{
    typedef struct interface{
      typedef struct endpointOut{
        uint8_t bDescriptorType     = 5; // for endpoint
        uint8_t bEndpointAddress    = 0x00 | 2; // 0x00 = outEP
        uint8_t bmAttributes        = 3 << 0; // 0:1 = transfer type, 2:3 iso_sync_type, 4:5 = iso_usage_type, 6:7 = reserved
        uint8_t wMaxPacketSize      = 64;
        uint8_t bInterval           = 1;  // interval for polling endpoint for data transfers
      }; endpointOut epOutDescriptor;
      typedef struct endpointIn{
        uint8_t bDescriptorType     = 5; // for endpoint
        uint8_t bEndpointAddress    = 0x80 | 1; // 0x80 = inEP
        uint8_t bmAttributes        = 3 << 0; // 0:1 = transfer type, 2:3 iso_sync_type, 4:5 = iso_usage_type, 6:7 = reserved
        uint8_t wMaxPacketSize      = 64;
        uint8_t bInterval           = 1;  // interval for polling endpoint for data transfers
      }; endpointIn epInDescriptor;
      uint8_t bDescriptorType     = 4; // for interface
      uint8_t bInterfaceNumber    = 0; 
      uint8_t bAlternateSetting   = 0; // see audio10.pdf --> 0 = zero-bandwidth alt, 1 = operational setting
      uint8_t bNumEndpoints       = 2;
      uint8_t bInterfaceClass     = 0x01; // 0x01 = audio class, others...
      uint8_t bInterfaceSubClass  = 0x00; // 0x00 = undefined, 0x01 = control, 0x02 = streaming, 0x03 = MIDI streaming
      uint8_t bInterfaceProtocol  = 0x00; // none, not applicable to audio
      uint8_t iInterface          = 0x00; // none, index of a string to describe interface, often unused
    }; interface interDescriptor;
    uint8_t bDescriptorType     = 2; // for configuration
    uint8_t bNumInterfaces      = 1;
    uint8_t bConfigurationValue = 1; // ID for configuration
    uint8_t iConfiguration      = 0x00; // none, index of a string to describe configuration, often unused
    uint8_t bmAttributes        = 0x80; // bit 5 = remote wakeup, bit 6 = self-powered
    uint8_t bMaxPower           = 200; // 400 mA  
} confDescriptor;

typedef struct device{
  uint8_t bDescriptorType = 1; // for device
  uint16_t bcdUSB             = 0x0200; // version of USB spec, here 2.0
  uint8_t bDeviceClass       = 0x00; // 0x00 = none / defined at interface level later, 0x0a = CDC data
  uint8_t bDeviceSubClass    = 0x00; // within audio, ...
  uint8_t bDeviceProtocol    = 0x00; // 0x00 = none
  uint8_t bMaxPacketSize0    = 64;
  uint16_t idVendor           = 0x2341; // Arduino
  uint16_t idProduct          = 0x804f; // MKZero
  uint16_t bcdDevice          = 0x0100; // release number of the device
  char iManufacturer[50]  = "Arduion + Harvard Active Learning";
  char iProduct[50]       = "MKR Zero uScope";
  char iSerialNumber[50]  = "ALL-0001";
  uint8_t bNumConfigurations = 1;
} deviceDescriptor;

typedef struct stringDesc {
  uint8_t bDescriptorType = 3; // for string descriptor
  uint16_t wLANGID = 0x0409;   // US English
} stringDescriptor;

typedef struct { 
    UsbDeviceDescBank DeviceDescBank[2]; 
} UsbdDescriptor;

volatile dmacdescriptor wrb[12] __attribute__ ((aligned (16))); // write-back descriptor
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16))); // channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));

__attribute__((__aligned__(4))) deviceDescriptor descriptor_usb;
__attribute__((__aligned__(4))) confDescriptor confDescriptor_usb;
__attribute__((__aligned__(4))) stringDescriptor string0Descriptor_usb;
__attribute__((__aligned__(4)))  UsbDeviceDescriptor EP[USB_EPT_NUM];

//__attribute__((__aligned__(4))) uint8_t usbRecv[64];

extern USBDevice_SAMD21G18x usbd; // defined in USBCore.cpp
extern UsbDeviceDescriptor usb_endpoints[];
extern const uint8_t usb_num_endpoints;

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
   SERCOM0->USART.CTRLB.bit.SBMODE = 1;    // 2 stop bits

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
  
  //SerialUSB.println(bufnum); // will affect DAC performance, used to verify buffer alternation

  __enable_irq(); // enable interrupts

}

void usb_init() 
{
  
  USB->DEVICE.CTRLA.bit.MODE = USB_CTRLA_MODE_DEVICE_Val;
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 1;
  USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
  USB->DEVICE.CTRLB.bit.DETACH = 0;
  
  USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST;
  USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTENSET.bit.RXSTP = 1;
  
  //USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTENSET.reg =  1 << 3; // enable interrupts from resets initiated by host (EORST bit)
  //USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTENSET.bit.RXSTP = 1; // enable interrupts from device setup requests

  NVIC_SetPriority(USB_IRQn, 0x00); // top priority
  NVIC_EnableIRQ(USB_IRQn); // will trigger USB_Handler
  
}

void USB_Handler(){

  int epint, flags;
  
  __disable_irq();

  if(USB->DEVICE.INTFLAG.reg & (1 << 3)) { // if EORST interrupt
  
    uart_puts("\nReset");
    USB->DEVICE.INTFLAG.reg = (1 << 3); // clear interrupt flag
    USB->DEVICE.DADD.reg = USB_DEVICE_DADD_ADDEN;

    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0 = USB_DEVICE_EPCFG_EPTYPE_DISABLED;
    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1 = USB_DEVICE_EPCFG_EPTYPE_DISABLED;
    
    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE0 = USB_DEVICE_EPCFG_EPTYPE_CONTROL; // Control SETUP/OUT
    USB->DEVICE.DeviceEndpoint[0].EPCFG.bit.EPTYPE1 = USB_DEVICE_EPCFG_EPTYPE_CONTROL; // Control IN
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK1RDY = 1;

    EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)usb_ctrl_in_buf;
    EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.SIZE = 64;
    EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
    EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

    EP[CONTROL_ENDPOINT].DeviceDescBank[0].ADDR.reg = (uint32_t)usb_ctrl_out_buf;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.SIZE = 64;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.MULTI_PACKET_SIZE = 8;
    EP[CONTROL_ENDPOINT].DeviceDescBank[0].PCKSIZE.bit.BYTE_COUNT = 0;

    // Here is where configuration for EP2 and 3 would go, possibly as isochronous? they're disabled by default on reset since EPCFG is cleared

    USB->DEVICE.DeviceEndpoint[0].EPSTATUSCLR.bit.BK0RDY = 1;
    USB->DEVICE.DeviceEndpoint[0].EPINTENSET.bit.RXSTP = 1;
    
  }

  if (USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP){
    
    uart_puts("\nSetup");
    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.RXSTP = 1;  // acknowledge interrupt
    USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSCLR.bit.BK0RDY = 1;
    
    usb_request_t *request = (usb_request_t*) usb_ctrl_out_buf;
//    usb_request_t request;
//    memcpy(&request, &usbRecv, sizeof(usb_request_t));

    uart_puts("\nRequestIn");

    uint8_t type = request->wValue >> 8;
    uint8_t index = request->wValue & 0xff;
    
    uart_puts("\nType: ");uart_write(type+ascii);
    uart_puts("\nIndex: ");uart_write(index+ascii);

    switch ((request->bRequest << 8) | request->bmRequestType){
        case USB_CMD(IN, DEVICE, STANDARD, GET_DESCRIPTOR):

          uart_puts("\nInDeviceStandardGetDescriptor");
          
          uint8_t type = request->wValue >> 8;
          uint8_t index = request->wValue & 0xff;
          uint16_t leng = request->wLength;
      
          if (type == USB_DEVICE_DESCRIPTOR){
            
            uart_puts("\ndevice");
            
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&descriptor_usb; // tell where to find
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = sizeof(deviceDescriptor);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);
          
          } 
          
          else if (type == USB_CONFIGURATION_DESCRIPTOR){
            
            uart_puts("\nconfig");
            
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&confDescriptor_usb; // tell where to find
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = sizeof(confDescriptor);
            EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
            USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

            while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);
            
          }
          
          else if (type == USB_STRING_DESCRIPTOR){
            
            uart_puts("\nstring0");
            
            if(index == 0){
              
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&string0Descriptor_usb; // tell where to find
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = sizeof(stringDescriptor);
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

              for (int i = 0; i < len; i++)
                usb_string_descriptor_buffer[2 + i*2] = str[i];

              uart_puts("\nstringN");
            
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].ADDR.reg = (uint32_t)&confDescriptor_usb; // tell where to find
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT  = sizeof(confDescriptor);
              EP[CONTROL_ENDPOINT].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
      
              USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1 = 1;
              USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPSTATUSSET.bit.BK1RDY = 1;

              while (0 == USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTFLAG.bit.TRCPT1);  
           
           } 
           
           else{

              uart_puts("\nstall0");
              USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;

            }

          }
            
          else{

            uart_puts("\nstall1");
            USB->DEVICE.DeviceEndpoint[0].EPSTATUSSET.bit.STALLRQ1 = 1;
            
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
    
          //udc_recv_callback(i);
        }
    
        if (flags & USB_DEVICE_EPINTFLAG_TRCPT1){
          
          USB->DEVICE.DeviceEndpoint[i].EPINTFLAG.bit.TRCPT1 = 1;
          USB->DEVICE.DeviceEndpoint[i].EPSTATUSCLR.bit.BK1RDY = 1;
    
          //udc_send_callback(i);
        }
      }
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
  
  analogWriteResolution(10);

  adc_init();
  dma_init(); 
  uart_init();
  usb_init();
  
  adc_to_sram_dma();

  bufnum = 0;
  start_adc_sram_dma(); 

}

void loop() {

  type waveform = sine;
  test_dac(waveform);
  
}