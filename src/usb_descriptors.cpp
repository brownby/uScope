#include "usb_descriptors.h"
#include "usb_enums.h"

alignas(4) usb_device_descriptor_t usb_device_descriptor =
{
  .bLength            = 18,     // bytes
  .bDescriptorType    = 0x01,   // for device
  .bcdUSB             = 0x0200, // version of USB spec
  .bDeviceClass       = 0x00,   // 0x00 = none / defined at interface level
  .bDeviceSubClass    = 0x00,
  .bDeviceProtocol    = 0x00, 
  .bMaxPacketSize0    = 64,     // *flag --> relation to iso_ep.wMaxPacketSize?
  .idVendor           = 0x6666, // Arduino 0x2341
  .idProduct          = 0x6666, // MKZero 0x804f
  .bcdDevice          = 0x0100, // release number of the device
  .iManufacturer      = USB_STR_MANUFACTURER,
  .iProduct           = USB_STR_PRODUCT,
  .iSerialNumber      = USB_STR_SERIAL_NUMBER,
  .bNumConfigurations = 1,
};

alignas(4) usb_configuration_hierarchy_t usb_configuration_hierarchy = {
  
  .configuration =
  {
    .bLength             = sizeof(usb_configuration_descriptor_t),
    .bDescriptorType     = USB_CONFIGURATION_DESCRIPTOR,
    .wTotalLength        = sizeof(usb_configuration_hierarchy_t),
    .bNumInterfaces      = 0x02, // AC and AudioStreaming with two alternative settings (0 = OFF, 1 = ON)
    .bConfigurationValue = 0x01, // used to select this configuration, not important given bNumConfigurations = 1
    .iConfiguration      = USB_STR_CONFIGURATION,
    .bmAttributes        = 0x80,
    .bMaxPower           = 250, // 400 mA
  },

  .standard_AC_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x00, // first interface of two
    .bAlternateSetting   = 0x00, // no alternate for this interface
    .bNumEndpoints       = 0x00, 
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x01, // audio control
    .bInterfaceProtocol  = 0x00, // unused
    .iInterface          = 0x00, // unused
  },

  .class_AC_interface =
  {
    .bLength             = sizeof(usb_class_AC_interface_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubType  = 0x01,   // header subtype
    .bcdADC              = 0x0100, // 1.0
    .wTotalLength        = sizeof(usb_class_AC_interface_descriptor_t) + sizeof(usb_audio_input_terminal_descriptor_t) + sizeof(usb_audio_feature_unit_descriptor_t)+ sizeof(usb_audio_output_terminal_descriptor_t), 
    .bInCollection       = 0x01,   // number of streaming interfaces --> AlternateSetting = 0x01 for bInterfaceNumber = 0x01
    .baInterfaceNr       = 0x01,   // interface number of the first AudioStreaming interface, possibility for continued list
  },

  .input_terminal =
  {
    .bLength             = sizeof(usb_audio_input_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02,   // input terminal
    .bTerminalID         = 0x01,   // unique ID, chosen by developer
    .wTerminalType       = 0x0201, // 0x0200 = input undefined (oscilloscope), 0x0201 is microphone
    .bAssocTerminal      = 0x00,   // no association
    .bNrChannels         = 0x01,   // one channel (possibility to extend to x2 channel scope over stereo?)
    .wChannelConfig      = 0x0000, // 0x0000 = mono, 0x0300 = left, right
    .iChannelNames       = 0x00,   // unused
    .iTerminal           = 0x00,   // unused
  },

  .feature_unit = 
  {
    .bLength             = sizeof(usb_audio_feature_unit_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x06,  // feature unit
    .bUnitID             = 0x02,  // chosen by programmer, between input and output terminals
    .bSourceID           = 0x01,  // for input_terminal
    .bControlSize        = 0x01,  // size in bytes of an element of the bmaControls() array
    .bmaControls         = {0x00, 0x03},
    .iFeature            = 0x00,  // unused
  },

  .output_terminal =
  {
    .bLength             = sizeof(usb_audio_output_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x03,   // output terminal (from device to host over isochronous endpoint)
    .bTerminalID         = 0x03,   // unique ID, chosen by developer
    .wTerminalType       = 0x0101, // 0x0101 = USB streaming
    .bAssocTerminal      = 0x00,   // no association
    .bSourceID           = 0x02,   // source is from feature unit
    .iTerminal           = 0x00,   // unused
  },

  .stream0_interface =
  {

    // this is the default AlternateSetting for the AudioStreaming interface (OFF)

    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01, // the second of two interfaces
    .bAlternateSetting   = 0x00, // OFF, zero-bandwidth setting
    .bNumEndpoints       = 0x00, // none 
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00, // unused
    .iInterface          = 0x00, // unused
  },

  .stream1_interface =
  {

    // this is the secondary AlternateSetting for the AudioStreaming interface (ON)

    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01, // the second of two interfaces
    .bAlternateSetting   = 0x01, // ON, operational setting
    .bNumEndpoints       = 0x01, // x1 endpoint
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00, // unused
    .iInterface          = USB_STR_INTERFACE,
  },

  .stream1_class_detail =
  {
    .bLength             = sizeof(usb_audio_stream_class_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01,   // general
    .bTerminalLink       = 0x03,   // ID for output terminal
    .bDelay              = 0x00,   // 0x01 in example, interface delay *flag
    .wFormatTag          = 0x0002, // 0x0002 for Windows, 0x0001 for Mac
    // .wFormatTag          = 0x0001, // 0x0001 = PCM, 0x0002 = PCM8 *flag 
                                   // macOS recognizes PCM, but not PCM8 (legacy = unsupported)? 
  },

  .format_type =
  {
    .bLength             = sizeof(usb_audio_format_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // format
    .bFormatType         = 0x01, // type I
    .bNrChannels         = 0x01, // 1 channel
    .bSubframeSize       = 0x01, // 1 byte per audio subframe 
    .bBitResolution      = 0x08, // 8 bits 
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
    .wMaxPacketSize      = 0x03ff, // 512 --> change to 1023, should match NBEATs for ADC buffer size? *flag
    .bInterval           = 0x01,   // 1 ms, one packet per frame
    .bRefresh            = 0x00,   // unused
    .bSynchAddress       = 0x00,   // unused, no sync
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

alignas(4) usb_string_descriptor_zero_t usb_string_descriptor_zero =
{
  .bLength         = sizeof(usb_string_descriptor_zero_t),
  .bDescriptorType = USB_STRING_DESCRIPTOR, 
  .wLANGID         = 0x0409, // US English
};