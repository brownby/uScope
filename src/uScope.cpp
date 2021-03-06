#include "uScope.h"

// DMAC variables
static uint32_t adctobuf0 = 0;  // dma channel for adc to buf0
static uint32_t adctobuf1 = 1;  // dma channel for adc to buf1
static uint32_t adctobuf2 = 2;  // dma channel for adc to buf2
static uint32_t adctobuf3 = 3;  // dma channel for adc to buf3
uint16_t adc_buffer0[NBEATS];  // buffer with length set by NBEATS
uint16_t adc_buffer1[NBEATS];  // alternating buffer
uint16_t adc_buffer2[NBEATS];
uint16_t adc_buffer3[NBEATS];
dmacdescriptor descriptor_section[12] __attribute__ ((aligned (16)));  // channel descriptors
dmacdescriptor descriptor __attribute__ ((aligned (16)));

// Function generator variables
float amplitude = 509.0;
float frequency = 5.0;
float freq_scalar = 22.0;
float freq_scalar_old = freq_scalar;
float offset = 510.0;
char incoming_string[255] = {0}; // up to 255 character long strings
String control_str = "";  // range up to 99,999 
char command[6] = {0};
uint8_t char_count = 0;
bool cmd_recv = 0;
uint16_t waveout[NPTS];       // buffer for waveform

// USB variables
static uint16_t usb_ctrl_audio;
static uint32_t usb_ctrl_in_buf[16];
static uint8_t usb_ctrl_out_buf[16];
static uint8_t usb_cdc_out_buf[64] = {0}; 
static uint8_t usb_cdc_in_buf[64];
static int usb_config;
char *usb_strings[100] = {"", "Arduino + Harvard","uScope by Active Learning","-uScope-","Isochronous Audio","uScope Instrumentation"};
uint8_t usb_string_descriptor_buffer[64] __attribute__ ((aligned (4)));
static usb_cdc_line_coding_t usb_cdc_line_coding =
{
  .dwDTERate   = 115200,
  .bCharFormat = 0, // 1 stop bit
  .bParityType = 0, // no parity
  .bDataBits   = 8, // 8 data bits
};
uint8_t interface_num = 0;
uint8_t alt_setting = 0;
usb_interface_status_t audio_stream_interface;
UsbDeviceDescriptor EP[USB_EPT_NUM] __attribute__ ((aligned(4)));

volatile uint8_t bufnum  = 0;  // track which buffer to write to, while USB reads

static char ascii = '0';

// Possibly obsolete?
volatile bool mute = false;
volatile uint16_t volume = 5;
bool RTS = false;
bool DTR = false;

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

void DMAC_Handler() { 

  __disable_irq(); // disable interrupts
  //digitalWrite(5, !digitalRead(5));

  bufnum =  (uint8_t)(DMAC->INTPEND.reg & DMAC_INTPEND_ID_Msk); // grab active channel
  DMAC->CHID.reg = DMAC_CHID_ID(bufnum); // select active channel
  DMAC->CHINTFLAG.reg = DMAC_CHINTFLAG_TCMPL; // | DMAC_CHINTFLAG_SUSP | DMAC_CHINTFLAG_TERR;
  
  DMAC->CHID.reg = (bufnum + 1 == 4) ? (DMAC_CHID_ID(0)) : (DMAC_CHID_ID(bufnum + 1));
  DMAC->CHCTRLA.reg |= DMAC_CHCTRLA_ENABLE;

  USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
  EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;

  switch((audio_stream_interface.alternate_setting << 4) | bufnum)
  {
    case 0x10: // bufnum == 0
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer3;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
    case 0x11: // bufnum == 1
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer0;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
    case 0x12: // bufnum == 2
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer1;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
    case 0x13: // bufnum == 3
      EP[ISO_ENDPOINT_IN].DeviceDescBank[1].ADDR.reg = (uint32_t)&adc_buffer2;
      USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSSET.bit.BK1RDY = 1;
      break;
  }

  __enable_irq();
}

void USB_Handler(){

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
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.DTGLIN = 1;
          USB->DEVICE.DeviceEndpoint[ISO_ENDPOINT_IN].EPSTATUSCLR.bit.BK1RDY = 1;
          EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_1023;
          EP[ISO_ENDPOINT_IN].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = NBEATS*2;    // size of ADC buffer in SRAM

          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT = 0;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE = 0;
          EP[CDC_ENDPOINT_COMM].DeviceDescBank[1].PCKSIZE.bit.SIZE = USB_DEVICE_PCKSIZE_SIZE_64;

          USB->DEVICE.DeviceEndpoint[CDC_ENDPOINT_COMM].EPCFG.bit.EPTYPE1 = 4; // interrupt IN
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

        if(interface_num == 1) // AudioStreaming interface
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
  }
}

void fngenerator(){

  int i; type waveform = sine;
  float phase = (2.0*3.14159*frequency)/(NPTS);
  for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * amplitude + offset;

  while(true) {

    if(cmd_recv) {

      cmd_recv = false;

      switch (command[0]){

        case '0':  
          waveform = sine;
          freq_scalar = 1.0;
          break;
      
        case '1':  
          waveform = pulse;
          freq_scalar = freq_scalar_old;
          break;

        case '2':  
          waveform = square;
          freq_scalar = freq_scalar_old;
          break;

        case '3': 
          waveform = sawtooth;
          freq_scalar = freq_scalar_old;
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

          amplitude = map(control_str.toInt(),100,3300,15,509);

          // uart_puts("\nAmplitude: "); uart_put_hex(control_str.toInt());
          // uart_puts("\nAmplitude_map: "); uart_put_hex(amplitude);
          break;

        case 'o':
          
          control_str = "";
          for (int i = 0; i < 5; i++){ control_str += command[i+1]; }

          offset = map(control_str.toInt(),50,3250,16,1008);

          break;

        case 'f':
          
          control_str = "";
          for (int i = 0; i < 5; i++){ control_str += command[i+1]; }

          frequency = map(control_str.toInt(),200,20000,1,61);
          freq_scalar = map(control_str.toInt(),200,20000,3,511);
          freq_scalar_old = freq_scalar;
          
          // uart_puts("\nFrequency: "); uart_put_hex(control_str.toInt());            
          // uart_puts("\nFrequency_map: "); uart_put_hex(frequency);
          // uart_puts("\nScalar_map: "); uart_put_hex(freq_scalar);
          break;

        default:
          break;
      }

      phase = (2.0*3.14159*frequency)/(NPTS);

      switch(waveform){

        case 0:  // sine wave
          for (i=0;i<NPTS;i++) waveout[i]= sinf(i*phase) * amplitude + offset;
          break;
      
        case 1:  // pulse wave
          for (i=0;i<1;i++) waveout[i] = offset + amplitude;
          for (i=1;i<NPTS/freq_scalar;i++) waveout[i] = offset - amplitude;
          break;

        case 2:  // square wave
          for (i=0;i<(NPTS/freq_scalar)/2;i++) { waveout[i] = offset + amplitude; }
          for (i=(NPTS/freq_scalar)/2;i<NPTS/freq_scalar;i++) { waveout[i] = offset - amplitude; }
          break;

        case 3:  // sawtooth wave
          for (i=0;i<NPTS;i++) waveout[i-1] = 2.0*amplitude*((NPTS/freq_scalar)-i)/(NPTS/freq_scalar) + offset - amplitude;
          break;

      }
    }

    if(!mute && waveform == sine) {
      for (int i = 0; i < NPTS; i++) { 
          analogWrite(A0,waveout[i]);
      }
    }
    else if(!mute) {
      for (int i = 0; i < NPTS/freq_scalar; i++) { 
          analogWrite(A0,waveout[i]);
      }
    }
  }
}

void uScope_init() {
    
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

  audio_stream_interface.interface_num = 1;

  __enable_irq();
}