#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "Arduino.h"
#include "usb_enums.h"

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

// Class-Specific AudioControl Interface Descriptor
// pg. 37 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint16_t bcdADC;
  uint16_t wTotalLength;
  uint8_t bInCollection;
  uint8_t baInterfaceNr;
} usb_audio_header_descriptor_t;

// Input Terminal Descriptor
// pg. 39 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorySubtype;
  uint8_t bTerminalID;
  uint16_t wTerminalType;
  uint8_t bAssocTerminal;
  uint8_t bNrChannels;
  uint16_t wChannelConfig;
  uint8_t iChannelNames;
  uint8_t iTerminal;
} usb_audio_input_descriptor_t;

// Output Terminal Descriptor
// pg. 40 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorySubtype;
  uint8_t bTerminalID;
  uint16_t wTerminalType;
  uint8_t bAssocTerminal;
  uint8_t bSourceID;
  uint8_t iTerminal;
} usb_audio_output_descriptor_t;

// Feature Unit Descriptor
// Other units we can add if needed: mixer, selector, processing
// pg. 41 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorySubtype;
  uint8_t bUnitID;
  uint8_t bSourceID;
  uint8_t bControlSize;
  uint16_t bmControls0; // Need one of these bitmaps per channel, may need to add more
  uint8_t iFeature;
} usb_audio_feature_descriptor_t;

// Class-Specific Audio Stream Interface Descriptor
// pg. 59 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorySubtype;
  uint8_t bTerminalLink;
  uint8_t bDelay;
  uint16_t wFormatTag;
} usb_audio_stream_descriptor_t;

// Type 1 Format Descriptor
// pg. 10 of USB Audio Data Formats 1.0 spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorySubtype;
  uint8_t bFormatType;
  uint8_t bNrChannels;
  uint8_t bSubframeSize;
  uint8_t bBitResolution;
  uint8_t bSamFreqType; // 0 for continous, any other number for number of discrete sampling frequencies,
  uint8_t bSamFreq0_byte0;
  uint8_t bSamFreq0_byte1;
  uint8_t bSamFreq0_byte2;
  // if we want more sample frequencies, need to add to the list here with more options, each specified with 3 bytes
} usb_audio_format_descriptor_t;

// Standard AudioStream Isochronous Audio Data Endpoint Descriptor
// pg. 61 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bEndpointAddress;
  uint8_t bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t bInterval;
  uint8_t bRefresh;
  uint8_t bSynchAddress;
} usb_audio_iso_ep_std_descriptor_t;

// Class-Specific AudioStream Isochronous Audio Data Endpoint Descriptor
// pg. 62 of spec
typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorySubtype;
  uint8_t bmAttributes;
  uint8_t bLockDelayUnits;
  uint16_t wLockDelay;
} usb_audio_iso_ep_descriptor_t;

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
