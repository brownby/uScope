#include "usb_descriptors.h"
#include "usb_enums.h"

alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy = {
  
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 0x02, // *flag
    .bConfigurationValue = 0x01, // *flag 
    .iConfiguration      = USB_STR_CONFIGURATION,
    .bmAttributes        = 0x80,
    .bMaxPower           = 200, // 400 mA
  },

  .standard_AC_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x00, // *flag
    .bAlternateSetting   = 0x00,
    .bNumEndpoints       = 0x00,
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x01, // audio control
    .bInterfaceProtocol  = 0x00,
    .iInterface          = 0x00,
  },

  .class_AC_interface =
  {
    .bLength             = sizeof(usb_class_AC_interface_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubType  = 0x01,   // header subtype
    .bcdADC              = 0x0100, // 1.0
    .wTotalLength        = sizeof(usb_class_AC_interface_descriptor_t) + sizeof(usb_audio_input_terminal_descriptor_t) + sizeof(usb_audio_output_terminal_descriptor_t) + sizeof(usb_audio_input_feature_descriptor_t), // *flag --> include feature?
    .bInCollection       = 0x01,   // number of streaming interfaces
    .baInterfaceNr       = 0x01,   // interface number of the first AudioStreaming interface, possibility for continued list
  },

  .input_terminal =
  {
    .bLength             = sizeof(usb_audio_input_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02,   // input terminal
    .bTerminalID         = 0x01,   // *flag
    .wTerminalType       = 0x0101, // 0x0101 = USB streaming, 0x0602 = digital audio interface *flag
    .bAssocTerminal      = 0x00,   // no assocaition
    .bNrChannels         = 0x01,   // one channel
    .wChannelConfig      = 0x0000, // 0x0000 = mono, 0x0300 = left, right
    .iChannelNames       = 0x00,   // unused
    .iTerminal           = 0x00,   // unused
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
    .bDescriptorSubtype  = 0x03,   // output terminal
    .bTerminalID         = 0x02,   // *flag
    .wTerminalType       = 0x0101, // streaming *flag
    .bAssocTerminal      = 0x00,   // no association
    .bSourceID           = 0x01,   // link to input terminal
    .iTerminal           = 0x00,   // unused
  },

  .stream0_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01, // *flag
    .bAlternateSetting   = 0x00, 
    .bNumEndpoints       = 0x00,
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00,
    .iInterface          = 0x00, 
  },

  .stream1_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01, // *flag
    .bAlternateSetting   = 0x01, 
    .bNumEndpoints       = 0x01, // x1 endpoint
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .stream1_class_detail =
  {
    .bLength             = sizeof(usb_audio_stream_class_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01,   // general
    .bTerminalLink       = 0x02,   // ID for output terminal
    .bDelay              = 0x00,   // 0x01 in example, interface delay *flag
    .wFormatTag          = 0x0001, // 0x0001 = PCM, 0x0002 = PCM8 *flag --> macOS recognizes PCM, but not PCM8? 
  },

  .format_type =
  {
    .bLength             = sizeof(usb_audio_format_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // format
    .bFormatType         = 0x01, // type I
    .bNrChannels         = 0x01, // 1 channel
    .bSubframeSize       = 0x01, // 1 byte per audio subframe *flag
    .bBitResolution      = 0x08, // 8bit *flag
    .bSamFreqType        = 0x01, // 1 sampling frequency
    .bSamFreq0_byte0     = 0x44,
    .bSamFreq0_byte1     = 0xAC,
    .bSamFreq0_byte2     = 0x00, // 44.1 kHz
  },

  .iso_ep =
  {
    .bLength             = sizeof(usb_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x81,   // ep[1].in
    .bmAttributes        = 0x01,   // isochronous, not shared
    .wMaxPacketSize      = 0x0200, // 512 --> change to 1023? *flag
    .bInterval           = 0x01,   // 1 ms, one packet per frame
    .bRefresh            = 0x00,   // unused
    .bSynchAddress       = 0x00,   // no sync
  },

  .iso_ep_class_detail =
  {
    .bLength             = sizeof(usb_audio_iso_ep_descriptor_t),
    .bDescriptorType     = USB_CS_ENDPOINT_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01,   // general
    .bmAttributes        = 0x00,   // no sampling frequency control, no pitch control, no packet padding
    .bLockDelayUnits     = 0x00,   // unused
    .wLockDelay          = 0x0000, // unused
  }
};
