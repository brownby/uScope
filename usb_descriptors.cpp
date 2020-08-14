#include "usb_descriptors.h"
#include "usb_enums.h"

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

  .interface =
  {
    .bLength             = sizeof(usb_interface_descriptor_t),
    .bDescriptorType     = USB_INTERFACE_DESCRIPTOR,
    .bInterfaceNumber    = 0,
    .bAlternateSetting   = 0,
    .bNumEndpoints       = 2,
    .bInterfaceClass     = 0x03, // HID
    .bInterfaceSubClass  = 0x00,
    .bInterfaceProtocol  = 0x00,
    .iInterface          = USB_STR_INTERFACE,
  },

  .hid =
  {
    .bLength             = sizeof(usb_hid_descriptor_t),
    .bDescriptorType     = 0x21, // HID descriptor
    .bcdHID              = 0x0111,
    .bCountryCode        = 0,
    .bNumDescriptors     = 1,
    .bDescriptorType1    = 0x22, // HID_REPORT descriptor
    .wDescriptorLength   = sizeof(usb_hid_report_descriptor),
  },
  
  .ep_in =
  {
    .bLength             = sizeof(usb_endpoint_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = USB_IN_ENDPOINT | 1,
    .bmAttributes        = USB_INTERRUPT_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 1,
  },

  .ep_out =
  {
    .bLength             = sizeof(usb_endpoint_descriptor_t),
    .bDescriptorType     = USB_ENDPOINT_DESCRIPTOR,
    .bEndpointAddress    = USB_OUT_ENDPOINT | 2,
    .bmAttributes        = USB_INTERRUPT_ENDPOINT,
    .wMaxPacketSize      = 64,
    .bInterval           = 1,
  },
};

alignas(4) uint8_t usb_hid_report_descriptor[33] =
{
  0x06, 0x00, 0xff,  // Usage Page (Vendor Defined 0xFF00)
  0x09, 0x01,        // Usage (0x01)
  0xa1, 0x01,        // Collection (Application)
  0x15, 0x00,        //   Logical Minimum (0)
  0x26, 0xff, 0x00,  //   Logical Maximum (255)
  0x75, 0x08,        //   Report Size (8)
  0x95, 0x40,        //   Report Count (64)
  0x09, 0x01,        //   Usage (0x01)
  0x81, 0x02,        //   Input (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position)
  0x95, 0x40,        //   Report Count (64)
  0x09, 0x01,        //   Usage (0x01)
  0x91, 0x02,        //   Output (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
  0x95, 0x01,        //   Report Count (1)
  0x09, 0x01,        //   Usage (0x01)
  0xb1, 0x02,        //   Feature (Data,Var,Abs,No Wrap,Linear,Preferred State,No Null Position,Non-volatile)
  0xc0,              // End Collection
};
