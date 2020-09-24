#ifndef _USCOPE_H_
#define _USCOPE_H_

#include "Arduino.h"          // required before wiring_private.h, also includes USBDesc.h, USBCore.h, USBAPI.h, and USB_host.h
#include "wiring_private.h"   // for pinPeripheral in adc_init()
#include "USB/PluggableUSB.h"
#include "USB/CDC.h"
#include "USB/SAMD21_USBDevice.h"
#include "usb_descriptors.h"
#include "usb_enums.h"
#include "uart_u.h"

#define ADCPIN A6           // selected arbitrarily, consider moving away from DAC / A0
#define NBEATS 232        // number of beats for adc transfer, MUST be < 512 (?)
#define NPTS 1022           // number of points within waveform definition

#define CONTROL_ENDPOINT  0
#define ISO_ENDPOINT_IN   1
#define ISO_ENDPOINT_OUT  2
#define CDC_ENDPOINT_COMM 3
#define CDC_ENDPOINT_IN   4
#define CDC_ENDPOINT_OUT  5

#endif