#ifndef _USB_U_H_
#define _USB_U_H_

#include "Arduino.h"

#define LIMIT(a, b)     (((a) > (b)) ? (b) : (a))
#define USB_CMD(dir, rcpt, type, cmd) \
    ((USB_##cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define USB_AUDIO_CMD(dir, rcpt, type, cmd) \
    ((cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

#define USB_CDC_CMD(dir, rcpt, type, cmd) \
    ((cmd << 8) | (USB_##dir##_TRANSFER << 7) | (USB_##type##_REQUEST << 5) | (USB_##rcpt##_RECIPIENT << 0))

typedef struct {
  uint8_t interface_num;
  uint8_t alternate_setting;
} usb_interface_status_t;

typedef struct __attribute__((packed)) {
  uint8_t   bmRequestType;
  uint8_t   bRequest;
  uint16_t  wValue;
  uint16_t  wIndex;
  uint16_t  wLength;
} usb_request_t;

typedef struct { 
    UsbDeviceDescBank DeviceDescBank[2]; 
} UsbdDescriptor;

UsbDeviceDescriptor EP[USB_EPT_NUM] __attribute__ ((aligned (4)));

void usb_init();

#endif