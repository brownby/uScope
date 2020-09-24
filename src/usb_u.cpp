#include "usb_u.h"

void usb_init() {

  // reset and wait for reset to complete
  // Arduino core enables SOF interrupts by default, causing ISR to be entered a lot

  USB->DEVICE.CTRLA.bit.SWRST = 1;
  while(USB->DEVICE.SYNCBUSY.bit.SWRST);

  memset((uint8_t *)EP, 0, sizeof(EP));
  USB->DEVICE.DESCADD.reg = (uint32_t)EP;

  // pad calibration, copied from SAMD21_USBDevice.h

  uint32_t *pad_transn_p = (uint32_t *) USB_FUSES_TRANSN_ADDR;
  uint32_t *pad_transp_p = (uint32_t *) USB_FUSES_TRANSP_ADDR;
  uint32_t *pad_trim_p   = (uint32_t *) USB_FUSES_TRIM_ADDR;

  uint32_t pad_transn = (*pad_transn_p & USB_FUSES_TRANSN_Msk) >> USB_FUSES_TRANSN_Pos;
  uint32_t pad_transp = (*pad_transp_p & USB_FUSES_TRANSP_Msk) >> USB_FUSES_TRANSP_Pos;
  uint32_t pad_trim   = (*pad_trim_p   & USB_FUSES_TRIM_Msk  ) >> USB_FUSES_TRIM_Pos;

  if (pad_transn == 0x1F)  // maximum value (31)
    pad_transn = 5;
  if (pad_transp == 0x1F)  // maximum value (31)
    pad_transp = 29;
  if (pad_trim == 0x7)     // maximum value (7)
    pad_trim = 3;

  USB->DEVICE.PADCAL.bit.TRANSN = pad_transn;
  USB->DEVICE.PADCAL.bit.TRANSP = pad_transp;
  USB->DEVICE.PADCAL.bit.TRIM   = pad_trim;
  
  USB->DEVICE.CTRLA.bit.MODE = USB_CTRLA_MODE_DEVICE_Val;
  USB->DEVICE.CTRLA.bit.RUNSTDBY = 1;
  USB->DEVICE.CTRLB.bit.SPDCONF = USB_DEVICE_CTRLB_SPDCONF_FS_Val;
  USB->DEVICE.CTRLB.bit.DETACH = 0;
  
  USB->DEVICE.INTENSET.reg = USB_DEVICE_INTENSET_EORST;  
  // USB->DEVICE.DeviceEndpoint[CONTROL_ENDPOINT].EPINTENSET.bit.RXSTP = 1; // do this upon reset in handlerf
  
  USB->DEVICE.CTRLA.reg |= USB_CTRLA_ENABLE;

//   audio_stream_interface.interface_num = 1;
  
  NVIC_SetPriority(USB_IRQn, 0x01); // second priority
  NVIC_EnableIRQ(USB_IRQn); // will trigger USB_Handler

}