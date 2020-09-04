#ifndef _USB_ENUMS_H_
#define _USB_ENUMS_H_

enum{
  USB_OUT_TRANSFER         = 0,
  USB_IN_TRANSFER          = 1,
};

enum{
  USB_STANDARD_REQUEST     = 0,
  USB_CLASS_REQUEST        = 1,
  USB_VENDOR_REQUEST       = 2,
};

enum{
  USB_IN_ENDPOINT          = 0x80,
  USB_OUT_ENDPOINT         = 0x00,
  USB_INDEX_MASK           = 0x7f,
  USB_DIRECTION_MASK       = 0x80,
};

enum{
  USB_STR_ZERO,
  USB_STR_MANUFACTURER,
  USB_STR_PRODUCT,
  USB_STR_SERIAL_NUMBER,
  USB_STR_CONFIGURATION,
  USB_STR_INTERFACE,
  USB_STR_COUNT,
};

enum
{
  USB_CONTROL_ENDPOINT     = 0 << 0,
  USB_ISOCHRONOUS_ENDPOINT = 1 << 0,
  USB_BULK_ENDPOINT        = 2 << 0,
  USB_INTERRUPT_ENDPOINT   = 3 << 0,

  USB_NO_SYNCHRONIZATION   = 0 << 2,
  USB_ASYNCHRONOUS         = 1 << 2,
  USB_ADAPTIVE             = 2 << 2,
  USB_SYNCHRONOUS          = 3 << 2,
};

enum{
  USB_DEVICE_EPCFG_EPTYPE_DISABLED    = 0,
  USB_DEVICE_EPCFG_EPTYPE_CONTROL     = 1,
  USB_DEVICE_EPCFG_EPTYPE_ISOCHRONOUS = 2,
  USB_DEVICE_EPCFG_EPTYPE_BULK        = 3,
  USB_DEVICE_EPCFG_EPTYPE_INTERRUPT   = 4,
  USB_DEVICE_EPCFG_EPTYPE_DUAL_BANK   = 5,
};

enum{
  USB_DEVICE_RECIPIENT     = 0,
  USB_INTERFACE_RECIPIENT  = 1,
  USB_ENDPOINT_RECIPIENT   = 2,
  USB_OTHER_RECIPIENT      = 3,
};

enum{
  USB_DEVICE_PCKSIZE_SIZE_8    = 0,
  USB_DEVICE_PCKSIZE_SIZE_16   = 1,
  USB_DEVICE_PCKSIZE_SIZE_32   = 2,
  USB_DEVICE_PCKSIZE_SIZE_64   = 3,
  USB_DEVICE_PCKSIZE_SIZE_128  = 4,
  USB_DEVICE_PCKSIZE_SIZE_256  = 5,
  USB_DEVICE_PCKSIZE_SIZE_512  = 6,
  USB_DEVICE_PCKSIZE_SIZE_1023 = 7,
};

enum{
  USB_GET_STATUS        = 0,
  USB_CLEAR_FEATURE     = 1,
  USB_SET_FEATURE       = 3,
  USB_SET_ADDRESS       = 5,
  USB_GET_DESCRIPTOR    = 6,
  USB_SET_DESCRIPTOR    = 7,
  USB_GET_CONFIGURATION = 8,
  USB_SET_CONFIGURATION = 9,
  USB_GET_INTERFACE     = 10,
  USB_SET_INTERFACE     = 11,
  USB_SYNCH_FRAME       = 12,
};

enum{
  USB_DEVICE_DESCRIPTOR         = 1,
  USB_CONFIGURATION_DESCRIPTOR  = 2,
  USB_STRING_DESCRIPTOR         = 3,
  USB_INTERFACE_DESCRIPTOR      = 4,
  USB_ENDPOINT_DESCRIPTOR       = 5,
  USB_CS_INTERFACE_DESCRIPTOR   = 0x24,
  USB_CS_ENDPOINT_DESCRIPTOR    = 0x25,
  USB_IAD_DESCRIPTOR            = 11,
};

enum{
  SET_CUR   = 0x01,
  GET_CUR   = 0x81,
  SET_MIN   = 0x02,
  GET_MIN   = 0x82,
  SET_MAX   = 0x03,
  GET_MAX   = 0x83,
  SET_RES   = 0x04,
  GET_RES   = 0x84,
  SET_MEM   = 0x05,
  GET_MEM   = 0x85,
  GET_STAT  = 0xFF,
};

enum{
  SET_LINE_CODING = 0x20,
  GET_LINE_CODING = 0x21,
  SET_CONTROL_LINE_STATE = 0x22,
  NOTIFY_SERIAL_STATE = 0x20,
};

enum
{
  USB_CDC_DEVICE_CLASS  = 2,  // USB Communication Device Class
  USB_CDC_COMM_CLASS    = 2,  // CDC Communication Class Interface
  USB_CDC_DATA_CLASS    = 10, // CDC Data Class Interface
};

enum
{
  USB_CDC_DLCM_SUBCLASS = 1, // Direct Line Control Model
  USB_CDC_ACM_SUBCLASS  = 2, // Abstract Control Model
  USB_CDC_TCM_SUBCLASS  = 3, // Telephone Control Model
  USB_CDC_MCCM_SUBCLASS = 4, // Multi-Channel Control Model
  USB_CDC_CCM_SUBCLASS  = 5, // CAPI Control Model
  USB_CDC_ETH_SUBCLASS  = 6, // Ethernet Networking Control Model
  USB_CDC_ATM_SUBCLASS  = 7, // ATM Networking Control Model
};

enum
{
  USB_CDC_HEADER_SUBTYPE    = 0, // Header Functional Descriptor
  USB_CDC_CALL_MGMT_SUBTYPE = 1, // Call Management
  USB_CDC_ACM_SUBTYPE       = 2, // Abstract Control Management
  USB_CDC_UNION_SUBTYPE     = 6, // Union Functional Descriptor
};

enum
{
  USB_CDC_SERIAL_STATE_DCD     = (1 << 0),
  USB_CDC_SERIAL_STATE_DSR     = (1 << 1),
  USB_CDC_SERIAL_STATE_BREAK   = (1 << 2),
  USB_CDC_SERIAL_STATE_RING    = (1 << 3),
  USB_CDC_SERIAL_STATE_FRAMING = (1 << 4),
  USB_CDC_SERIAL_STATE_PARITY  = (1 << 5),
  USB_CDC_SERIAL_STATE_OVERRUN = (1 << 6),
};

// USB CDC Call Management Capabilities
enum
{
  USB_CDC_CALL_MGMT_SUPPORTED = (1 << 0),
  USB_CDC_CALL_MGMT_OVER_DCI  = (1 << 1),
};

// USB CDC ACM Capabilities
enum
{
  // Device supports the request combination of Set_Comm_Feature,
  // Clear_Comm_Feature, and Get_Comm_Feature.
  USB_CDC_ACM_SUPPORT_FEATURE_REQUESTS   = (1 << 0),

  // Device supports the request combination of Set_Line_Coding, Set_Control_Line_State,
  // Get_Line_Coding, and the notification Serial_State.
  USB_CDC_ACM_SUPPORT_LINE_REQUESTS      = (1 << 1),

  // Device supports the request Send_Break.
  USB_CDC_ACM_SUPPORT_SENDBREAK_REQUESTS = (1 << 2),

  // Device supports the notification Network_Connection.
  USB_CDC_ACM_SUPPORT_NOTIFY_REQUESTS    = (1 << 3),
};

enum
{
  USB_CDC_CTRL_SIGNAL_DTE_PRESENT        = (1 << 0), // DTR
  USB_CDC_CTRL_SIGNAL_ACTIVATE_CARRIER   = (1 << 1), // RTS
};

#endif
