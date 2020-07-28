# uScope
## Revision history
- **v1.0: use EVSYS to trigger alternating SRAM-UART DMA from circular ADC buffers**
- v0.9: transfer ADC buffers to UART (success), setup EVSYS for precision timing
- v0.8: attempt to SW trigger timely, alternating SRAM-UART transfers from DMAC_Handler, consider EVSYS
- v0.7: setup FTDI device output, switch SERCOM 5 to 0, working SRAM (dummy buffer) to UART DMA
- v0.6: add USART support, basic write wrapper/test
- v0.5: clean DAC requirements (CPU only), grab bufnum in DMAC ISR
- v0.4: alternating buffers for circular ADC descriptors, separate dma init from start
- v0.3: wrapper for ADC, evaluate USBSerial for offload
- v0.2: add DAC, with simple wrapper
- v0.1: clean code, ADC to SRAM DMA test
- v0.0: scrub project
