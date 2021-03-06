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
#include "adc_u.h"
#include "dmac_u.h"
#include "usb_u.h"

#define NBEATS 232        // number of beats for adc transfer, MUST be < 512 (?)
#define NPTS 1022           // number of points within waveform definition

#define CONTROL_ENDPOINT  0
#define ISO_ENDPOINT_IN   1
#define ISO_ENDPOINT_OUT  2
#define CDC_ENDPOINT_COMM 3
#define CDC_ENDPOINT_IN   4
#define CDC_ENDPOINT_OUT  5

enum type {sine, pulse, square, sawtooth}; // supported waveform types

void adc_to_sram_dma();

void DMAC_Handler();

void USB_Handler();

void fngenerator();

void uScope_init();

#endif