#ifndef _ADC_H_
#define _ADC_H_

#include "Arduino.h"
#include "wiring_private.h"

#define ADCPIN A6           // selected arbitrarily, consider moving away from DAC / A0

void adc_init();

#endif