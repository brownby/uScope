#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "Arduino.h"
#include "usb_enums.h"

enum{
  USB_STR_ZERO,
  USB_STR_MANUFACTURER,
  USB_STR_PRODUCT,
  USB_STR_SERIAL_NUMBER,
  USB_STR_CONFIGURATION,
  USB_STR_INTERFACE,
  USB_STR_COUNT,
};

typedef struct __attribute__((packed)) {
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

typedef struct __attribute__((packed)) {
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint16_t  wTotalLength;
  uint8_t   bNumInterfaces;
  uint8_t   bConfigurationValue;
  uint8_t   iConfiguration;
  uint8_t   bmAttributes;
  uint8_t   bMaxPower;
} usb_configuration_descriptor_t;

typedef struct __attribute__((packed)) {
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

typedef struct __attribute__((packed)) {
  uint8_t   bLength;
  uint8_t   bDescriptorType;
  uint8_t   bEndpointAddress;
  uint8_t   bmAttributes;
  uint16_t  wMaxPacketSize;
  uint8_t   bInterval;
} usb_endpoint_descriptor_t;

typedef struct __attribute__((packed)) {
  
} usb_audio_control_descriptor_t;

typedef struct __attribute__((packed)) {
  
} usb_audio_stream_descriptor_t;

typedef struct __attribute__((packed)) {
  
} usb_audio_alternate_descriptor_t;

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint16_t bcdHID;
  uint8_t bCountryCode;
  uint8_t bNumDescriptors;
  uint8_t bDescriptorType1;
  uint16_t wDescriptorLength;
} usb_hid_descriptor_t;

typedef struct __attribute__((packed)) {
  usb_configuration_descriptor_t  configuration;
  usb_interface_descriptor_t      interface;
  usb_hid_descriptor_t            hid;
  usb_endpoint_descriptor_t       ep_in;
  usb_endpoint_descriptor_t       ep_out;
} usb_configuration_hierarchy_t;


typedef struct  {
  uint8_t bLength         = 4;
  uint8_t bDescriptorType = USB_STRING_DESCRIPTOR; 
  uint16_t wLANGID        = 0x0409;   // US English
} stringDescriptor;

extern alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy;
extern alignas(4) uint8_t usb_hid_report_descriptor[33];

#endif
