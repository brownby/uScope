#include "usb_descriptors.h"
#include "usb_enums.h"

alignas(4) usb_device_descriptor_t usb_device_descriptor =
{
  .bLength            = sizeof(usb_device_descriptor_t),     // bytes
  .bDescriptorType    = 0x01,   // for device
  .bcdUSB             = 0x0200, // version of USB spec
  .bDeviceClass       = 0xef,   // 0xef = multi-interface function
  .bDeviceSubClass    = 0x02,   // 0x02 = common class
  .bDeviceProtocol    = 0x01,   // 0x01 = IAD, interface association descriptor
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
    .bNumInterfaces      = 0x04, // AC and AudioStreaming interface, CDC comm and data interface
    .bConfigurationValue = 0x01, // used to select this configuration, not important given bNumConfigurations = 1
    .iConfiguration      = 0x00, //unused
    .bmAttributes        = 0x80,
    .bMaxPower           = 200, // 400 mA
  },

  .audio_IAD =
  {
    .bLength             = sizeof(usb_interface_association_descriptor_t),
    .bDescriptorType     = USB_IAD_DESCRIPTOR,
    .bFirstInterface     = 0x00, // interface number of AC interface
    .bInterfaceCount     = 0x02, // AC and AS interface
    .bFunctionClass      = 0x01, // audio
    .bFunctionSubClass   = 0x01, // should match subclass of first interface
    .bFunctionProtocol   = 0x00, // unused
    .iFunction           = 0x00, // unused
  },

  .standard_AC_interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x00, // first interface of three
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
    .wTotalLength        = sizeof(usb_class_AC_interface_descriptor_t) + sizeof(usb_audio_input_terminal_descriptor_t) + sizeof(usb_audio_feature_unit_descriptor_t) + sizeof(usb_audio_output_terminal_descriptor_t),
    .bInCollection       = 0x01,
    .baInterfaceNr1      = 0x01,   // interface number of the first AudioStreaming interface, possibility for continued list
  },

  .input_terminal_scope =
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

  .output_terminal_scope =
  {
    .bLength             = sizeof(usb_audio_output_terminal_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x03,   // output terminal (from device to host over isochronous endpoint)
    .bTerminalID         = 0x02,   // unique ID, chosen by developer
    .wTerminalType       = 0x0101, // 0x0101 = USB streaming
    .bAssocTerminal      = 0x00,   // no association
    .bSourceID           = 0x01, // source is scope input terminal
    .iTerminal           = 0x00,   // unused
  },

  .stream0_scope_interface =
  {

    // this is the default AlternateSetting for the AudioStreaming interface (OFF)

    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01, // the second of three interfaces
    .bAlternateSetting   = 0x00, // OFF
    .bNumEndpoints       = 0x00, // none, zero-bandwidth setting
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00, // unused
    .iInterface          = 0x00, // unused
  },

  .stream1_scope_interface =
  {

    // this is the secondary AlternateSetting for the AudioStreaming interface (ON)

    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x01, // the second of three interfaces
    .bAlternateSetting   = 0x01, // ON, operational setting
    .bNumEndpoints       = 0x01, // x1 endpoint
    .bInterfaceClass     = 0x01, // audio
    .bInterfaceSubClass  = 0x02, // streaming
    .bInterfaceProtocol  = 0x00, // unused
    .iInterface          = 0x00, // unused
  },

  .stream1_scope_class_detail =
  {
    .bLength             = sizeof(usb_audio_stream_class_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01,   // general
    .bTerminalLink       = 0x02,   // ID for output scope terminal
    .bDelay              = 0x00,   // 0x01 in example, interface delay *flag
    .wFormatTag          = 0x0001, // 0x0001 = PCM, 0x0002 = PCM8 *flag 
                                   // macOS recognizes PCM, but not PCM8 (legacy = unsupported)? 
  },

  .scope_format_type =
  {
    .bLength             = sizeof(usb_audio_format_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = 0x02, // format
    .bFormatType         = 0x01, // type I
    .bNrChannels         = 0x01, // 1 channel
    .bSubframeSize       = 0x02, // 2 bytes per audio subframe 
    .bBitResolution      = 0x0C, // 12 bits
    .bSamFreqType        = 0x01, // 1 sampling frequency
    .bSamFreq0_byte0     = 0x44,
    .bSamFreq0_byte1     = 0xAC,
    .bSamFreq0_byte2     = 0x00, // 44.1kHz (sample rate)
  },

  .scope_iso_ep =
  {
    .bLength             = sizeof(usb_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x81,   // ep[1].in
    .bmAttributes        = 0x01,   // isochronous, not shared
    .wMaxPacketSize      = 0x03ff, // 1023 bytes
    .bInterval           = 0x01,   // 1 ms, one packet per frame
    .bRefresh            = 0x00,   // unused
    .bSynchAddress       = 0x00,   // unused, no sync
  },

  .scope_iso_ep_class_detail =
  {
    .bLength             = sizeof(usb_audio_iso_ep_descriptor_t),
    .bDescriptorType     = USB_CS_ENDPOINT_DESCRIPTOR,
    .bDescriptorSubtype  = 0x01,   // general
    .bmAttributes        = 0x00,   // no sampling frequency control, no pitch control, no packet padding
    .bLockDelayUnits     = 0x00,   // unused
    .wLockDelay          = 0x0000, // unused
  },

  .cdc_IAD =
  {
    .bLength             = sizeof(usb_interface_association_descriptor_t),
    .bDescriptorType     = USB_IAD_DESCRIPTOR,
    .bFirstInterface     = 0x02,
    .bInterfaceCount     = 0x02,
    .bFunctionClass      = USB_CDC_COMM_CLASS,
    .bFunctionSubClass   = USB_CDC_ACM_SUBCLASS, // should match subclass of first interface
    .bFunctionProtocol   = 0x00,
    .iFunction           = 0x00,
  },

  .interface_comm =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x02,
    .bAlternateSetting   = 0x00,
    .bNumEndpoints       = 0x01,
    .bInterfaceClass     = USB_CDC_COMM_CLASS,
    .bInterfaceSubClass  = USB_CDC_ACM_SUBCLASS,
    .bInterfaceProtocol  = 0x00,
    .iInterface          = 0x00,    
  },

  .cdc_header =
  {
    .bFunctionalLength   = sizeof(usb_cdc_header_functional_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = USB_CDC_HEADER_SUBTYPE,
    .bcdCDC              = 0x0110,
  },

  .cdc_acm =
  {
    .bFunctionalLength   = sizeof(usb_cdc_abstract_control_managment_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = USB_CDC_ACM_SUBTYPE,
    .bmCapabilities      = USB_CDC_ACM_SUPPORT_LINE_REQUESTS, // Arduino COM port also supports send break requests
  },

  .cdc_call_mgmt =
  {
    .bFunctionalLength   = sizeof(usb_cdc_call_managment_functional_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = USB_CDC_CALL_MGMT_SUBTYPE,
    .bmCapabilities      = USB_CDC_CALL_MGMT_SUPPORTED, // Arduino COM port uses USB_CDC_CALL_MGMT_SUPPPORTED
    .bDataInterface      = 0x03, // interface number of data interface
  },

  .cdc_union =
  {
    .bFunctionalLength   = sizeof(usb_cdc_union_functional_descriptor_t),
    .bDescriptorType     = USB_CS_INTERFACE_DESCRIPTOR,
    .bDescriptorSubtype  = USB_CDC_UNION_SUBTYPE,
    .bMasterInterface    = 0x00,
    .bSlaveInterface0    = 0x03, // slave interface number, same as data interface to match ataradov's VCP and Arduino COM
  },

  .ep_comm =
  {
    .bLength             = sizeof(usb_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x83, // ep[3].in
    .bmAttributes        = USB_INTERRUPT_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 0x10, // to match Arduino COM, set to 1 for ataradov
  },

  .interface_data =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0x03,
    .bAlternateSetting   = 0x00,
    .bNumEndpoints       = 0x02,
    .bInterfaceClass     = USB_CDC_DATA_CLASS,
    .bInterfaceSubClass  = 0x00,
    .bInterfaceProtocol  = 0x00,
    .iInterface          = 0x00,
  },

  .ep_in =
  {
    .bLength             = sizeof(usb_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x84, // ep[4].in
    .bmAttributes        = USB_BULK_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 0x00,
  },

  .ep_out =
  {
    .bLength             = sizeof(usb_ep_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = 0x05, // ep[5].out
    .bmAttributes        = USB_BULK_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 0x00,
  },
  
};

alignas(4) usb_string_descriptor_zero_t usb_string_descriptor_zero =
{
  .bLength         = sizeof(usb_string_descriptor_zero_t),
  .bDescriptorType = USB_STRING_DESCRIPTOR, 
  .wLANGID         = 0x0409, // US English
};
