#include "usb_descriptors.h"

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

  .standard_AC_interface =
  {
    .bLength             = sizeof(usb_standard_AC_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x00,
    .bAlternateSetting   = 0x00,
    .bNumEndpoints       = 0x00,
    .bInterfaceClass     = 0x01,
    .bInterfaceSubClass  = 0x01,
    .bInterfaceProtocol  = 0x00,
    .iInterface          = 0x00,
  },

  .class_AC_interface =
  {
    .bLength             = sizeof(usb_class_AC_interface_descriptor_t),
    .bDescriptorType     = USB_CS_DESCRIPTOR,
    .bDescriptorSubType  = 0x01, // header
    .bcdADC              = 0x0001, // 1.0
    .wTotalLength        = 0,
    .bInCollection       = 0x01,
    .baInterfaceNr       = 0x01,
  },

  .input_terminal =
  {
    .bLength             = sizeof(usb_audio_input_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // input terminal
    .bTerminalID         = 0x01,
    .wTerminalType       = 0x1007, // radio reciever? --> check options
    .bAssocTerminal      = 0x00,
    .bNrChannels         = 0x02,
    .wChannelConfig      = 0x0300, // left, right? --> change to mono
    .iChannelNames       = 0x00,
    .iTerminal           = 0x00,
  },

  .feature_unit =
  {
    .bLength             = sizeof(usb_audio_input_feature_descriptor_t),
    .bDescriptorType     = USB_CS_DESCRIPTOR,
    .bDescriptorSubtype  = 0x06, // feature unit
    .bUnitID             = 0x02,
    .bSourceID           = 0x01, // link to input terminal
    .bControlSize        = 0x02, // bytes
    .bmControls0         = 0x0000,
    .iFeature            = 0,
  },

  .output_terminal =
  {
    .bLength             = sizeof(usb_audio_output_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_DESCRIPTOR,
    .bDescriptorSubtype  = 0x03,  // output terminal
    .bTerminalID         = 0x03,
    .wTerminalType       = 0x101, // streaming
    .bAssocTerminal      = 0x00,
    .bSourceID           = 0x02,  // link to feature unit
    .iTerminal           = 0x00,
  },

  .stream_interface =
  {
    .bLength             = sizeof(usb_audio_stream_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01,
    .bAlternateSetting   = 0x00,
    .bNumEndpoints       = 0x00,
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .alternate_interface =
  {
    .bLength             = sizeof(usb_alternate_audio_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01,
    .bAlternateSetting   = 0x01,
    .bNumEndpoints       = 0x01,
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .stream_class_detail =
  {
    .bLength             = sizeof(usb_audio_stream_class_descriptor_t),
    .bDescriptorType     = USB_CS_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01, // general
    .bTerminalLink       = 0x03,
    .bDelay              = 0x00,
    .wFormatTag          = 0x100, // PCM
  },

  .format_type =
  {
    .bLength             = sizeof(usb_audio_format_descriptor_t),
    .bDescriptorType     = USB_CS_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // format
    .bFormatType         = 0x01, // type
    .bNrChannels         = 0x02, // --> change to 1 for Mono?
    .bSubframeSize       = 0x02, // --> change to 1 for PCM8?
    .bBitResolution      = 0x10, // 16bit --> change to 8bit
    .bSamFreqType        = 0x01, // 1 sampling frequency
    .bSamFreq0_byte0     = 0x80,
    .bSamFreq0_byte1     = 0xBB,
    .bSamFreq0_byte2     = 0x00, // 48 kHz --> needs to match ADC?
  },

  .iso_ep =
  {
    .bLength             = sizeof(usb_iso_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x83, // ep[3].in
    .bmAttributes        = 0x05, // asynchronous
    .wMaxPacketSize      = 0x0002, // 512 --> change to 1023
    .bInterval           = 0x01, // 1 ms
    .bRefresh            = 0x00,
    .bSynchAddress       = 0x00, // no sync
  },

  .iso_ep_class_detail =
  {
    .bLength             = sizeof(usb_audio_iso_ep_descriptor_t),
    .bDescriptorType     = USB_CS_ENDPOINT,
    .bDescriptorSubtype  = 0x01, //general
    .bmAttributes        = 0x00,
    .bLockDelayUnits     = 0x02, // PCM samples
    .wLockDelay          = 0x0000,
  }
};