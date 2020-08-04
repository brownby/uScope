#include "USB/SAMD21_USBDevice.h"

extern USBDevice_SAMD21G18x usbd;
UsbDevice* usb;
UsbDeviceDescriptor* EP;

uint8_t ep = 3; // endpoint to explore
  
void setup() {

  usb = usbd.getUsbPtr();
  EP = usbd.getEpPtr();

  for(int i = 0; i < ep; i ++) {EP++; }

}

void loop() {
  Serial.print("EPCFG.EPTYPE1: ");
  Serial.println(usb->DeviceEndpoint[ep].EPCFG.bit.EPTYPE1);

  Serial.print("PCKSIZE.BYTE_COUNT: ");
  Serial.println(EP->DeviceDescBank[1].PCKSIZE.bit.BYTE_COUNT);

  Serial.print("PCKSIZE.MULTI_PACKET_SIZE: ");
  Serial.println(EP->DeviceDescBank[1].PCKSIZE.bit.MULTI_PACKET_SIZE);

  Serial.print("PCKSIZE.SIZE: ");
  Serial.println(EP->DeviceDescBank[1].PCKSIZE.bit.SIZE);

  Serial.println();

}
