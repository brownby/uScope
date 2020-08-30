#ifndef _USB_DESCRIPTORS_H_
#define _USB_DESCRIPTORS_H_

#include "Arduino.h"
#include "usb_enums.h"

typedef struct __attribute__((packed)) {
  uint8_t  bLength;  
  uint8_t  bDescriptorType; 
  uint16_t bcdUSB; 
  uint8_t  bDeviceClass;
  uint8_t  bDeviceSubClass;
  uint8_t  bDeviceProtocol;
  uint8_t  bMaxPacketSize0;
  uint16_t idVendor;
  uint16_t idProduct;
  uint16_t bcdDevice;
  uint8_t  iManufacturer;
  uint8_t  iProduct;
  uint8_t  iSerialNumber;
  uint8_t  bNumConfigurations;
} usb_device_descriptor_t;

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
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bInterfaceNumber;
  uint8_t bAlternateSetting;
  uint8_t bNumEndpoints;
  uint8_t bInterfaceClass;
  uint8_t bInterfaceSubClass;
  uint8_t bInterfaceProtocol;
  uint8_t iInterface;
} usb_interface_descriptor_t; // pg. 36 of spec

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubType;
  uint16_t bcdADC;
  uint16_t wTotalLength;
  uint8_t bInCollection;
  uint8_t baInterfaceNr;
} usb_class_AC_interface_descriptor_t; // pg. 37 of spec

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bTerminalID;
  uint16_t wTerminalType;
  uint8_t bAssocTerminal;
  uint8_t bNrChannels;
  uint16_t wChannelConfig;
  uint8_t iChannelNames;
  uint8_t iTerminal;
} usb_audio_input_terminal_descriptor_t; // pg. 39 of spec

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype; 
  uint8_t bUnitID;
  uint8_t bSourceID;
  uint8_t bControlSize;
  uint16_t bmaControls;
  uint8_t iFeature;
} usb_audio_feature_unit_descriptor_t;

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bTerminalID;
  uint16_t wTerminalType;
  uint8_t bAssocTerminal;
  uint8_t bSourceID;
  uint8_t iTerminal;
} usb_audio_output_terminal_descriptor_t; // pg. 40 of spec

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bTerminalLink;
  uint8_t bDelay;
  uint16_t wFormatTag;
} usb_audio_stream_class_descriptor_t; 

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType;
  uint8_t bDescriptorSubtype;
  uint8_t bFormatType;
  uint8_t bNrChannels;
  uint8_t bSubframeSize;
  uint8_t bBitResolution;
  uint8_t bSamFreqType; // 0 for continous, any other number for discrete sampling frequencies
  uint8_t bSamFreq0_byte0; 
  uint8_t bSamFreq0_byte1; 
  uint8_t bSamFreq0_byte2;      
} usb_audio_format_descriptor_t; // pg. 10 of USB Audio Data Formats 1.0 spec

typedef struct __attribute__((packed)) {
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bEndpointAddress;
  uint8_t  bmAttributes;
  uint16_t wMaxPacketSize;
  uint8_t  bInterval;
  uint8_t  bRefresh;
  uint8_t  bSynchAddress;
} usb_ep_descriptor_t; // pg. 61 of spec

typedef struct __attribute__((packed)) {
  uint8_t  bLength;
  uint8_t  bDescriptorType;
  uint8_t  bDescriptorSubtype;
  uint8_t  bmAttributes;
  uint8_t  bLockDelayUnits;
  uint16_t wLockDelay;
} usb_audio_iso_ep_descriptor_t; // pg. 62 of spec

typedef struct __attribute__((packed)) {
  uint8_t bLength;
  uint8_t bDescriptorType; 
  uint16_t wLANGID;  
} usb_string_descriptor_zero_t;

typedef struct __attribute__((packed)) {
  usb_configuration_descriptor_t             configuration;
  usb_interface_descriptor_t                 standard_AC_interface;
  usb_class_AC_interface_descriptor_t        class_AC_interface;
  usb_audio_input_terminal_descriptor_t      input_terminal;
  usb_audio_feature_unit_descriptor_t        feature_unit;
  usb_audio_output_terminal_descriptor_t     output_terminal;
  usb_interface_descriptor_t                 stream0_interface;
  usb_interface_descriptor_t                 stream1_interface;
  usb_audio_stream_class_descriptor_t        stream1_class_detail;
  usb_audio_format_descriptor_t              format_type;
  usb_ep_descriptor_t                        iso_ep;
  usb_audio_iso_ep_descriptor_t              iso_ep_class_detail;
} usb_configuration_hierarchy_t;

extern usb_configuration_hierarchy_t usb_configuration_hierarchy;
extern usb_device_descriptor_t usb_device_descriptor;
extern usb_string_descriptor_zero_t usb_string_descriptor_zero;

#endif