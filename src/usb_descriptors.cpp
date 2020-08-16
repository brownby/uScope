#include "usb_descriptors.h"
#include "usb_enums.h"

alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy = {
  
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 2,
    .bConfigurationValue = 1,
    .iConfiguration      = USB_STR_CONFIGURATION,
    .bmAttributes        = 0x80,
    .bMaxPower           = 200, // 400 mA
  },

  .standard_AC_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
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
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubType  = 0x01, // header
    .bcdADC              = 0x0100, // 1.0
    .wTotalLength        = sizeof(usb_class_AC_interface_descriptor_t) + sizeof(usb_audio_input_terminal_descriptor_t) + sizeof(usb_audio_output_terminal_descriptor_t) + sizeof(usb_audio_input_feature_descriptor_t),
    .bInCollection       = 0x01,
    .baInterfaceNr       = 0x01,
  },

  .input_terminal =
  {
    .bLength             = sizeof(usb_audio_input_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // input terminal
    .bTerminalID         = 0x01,
    .wTerminalType       = 0x0201, // 0x1007 = radio reciever, 0x0201 = microphone
    .bAssocTerminal      = 0x00,
    .bNrChannels         = 0x01,
    .wChannelConfig      = 0x0000, // 0x0000 = mono, 0x0300 = left, right
    .iChannelNames       = 0x00,
    .iTerminal           = 0x00,
  },

//  .feature_unit =
//  {
//    .bLength             = sizeof(usb_audio_input_feature_descriptor_t),
//    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
//    .bDescriptorSubtype  = 0x06, // feature unit
//    .bUnitID             = 0x02,
//    .bSourceID           = 0x01, // link to input terminal
//    .bControlSize        = 0x02, // bytes
//    .bmMasterControls    = 0x0001, // mute enabled
//    .bmControls1         = 0x0000,
//    .bmControls2         = 0x0000,
//    .iFeature            = 0,
//  },

  .output_terminal =
  {
    .bLength             = sizeof(usb_audio_output_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x03,  // output terminal
    .bTerminalID         = 0x02,
    .wTerminalType       = 0x0101, // streaming
    .bAssocTerminal      = 0x00,
    .bSourceID           = 0x01,  // link to input terminal
    .iTerminal           = 0x00,
  },

  .stream0_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01,
    .bAlternateSetting   = 0x00,
    .bNumEndpoints       = 0x00,
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .stream1_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01,
    .bAlternateSetting   = 0x01,
    .bNumEndpoints       = 0x01,
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .stream1_class_detail =
  {
    .bLength             = sizeof(usb_audio_stream_class_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01, // general
    .bTerminalLink       = 0x02, // output terminal
    .bDelay              = 0x00,
    .wFormatTag          = 0x0001, // PCM
  },

  .format_type =
  {
    .bLength             = sizeof(usb_audio_format_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // format
    .bFormatType         = 0x01, // type I
    .bNrChannels         = 0x01, // --> change to 1 for Mono?
    .bSubframeSize       = 0x02, // 2 bytes per audio subframe
    .bBitResolution      = 0x10, // 16bit --> change to 8bit
    .bSamFreqType        = 0x01, // 1 sampling frequency
    .bSamFreq0_byte0     = 0x44,
    .bSamFreq0_byte1     = 0xAC,
    .bSamFreq0_byte2     = 0x00, // 44.1 kHz --> needs to match ADC?
  },

  .iso_ep =
  {
    .bLength             = sizeof(usb_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x81, // ep[1].in
    .bmAttributes        = 0x01, // isochronous, not shared
    .wMaxPacketSize      = 0x0200, // 512 --> change to 1023
    .bInterval           = 0x01, // 1 ms
    .bRefresh            = 0x00,
    .bSynchAddress       = 0x00, // no sync
  },

  .iso_ep_class_detail =
  {
    .bLength             = sizeof(usb_audio_iso_ep_descriptor_t),
    .bDescriptorType     = USB_CS_ENDPOINT_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01, //general
    .bmAttributes        = 0x00,
    .bLockDelayUnits     = 0x00, // PCM samples
    .wLockDelay          = 0x0000,
  }
};
