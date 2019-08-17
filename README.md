# neutrino-dmx
"Blue Pill" aka STM32F103C8 USB Device emulating DMXKing UltraDMX Pro.

Original board uses FTDI chip, so this firmware works only in OLA on Linux

USB-CDC Serial Port = dmx widget (1 in/ 2 out)
Needs extra ESTA_ID support in OLA to extend to full 3in/3out configuration.

UART1 TX, UART3 TX are 5V-tolerant, connected to 6N137 cathode.
UART2 RX connected to resistor-divided MAX485 driver output
PB3, PB4 repeat dmx data as DMA-driven WS8212, should be connected to 74HCT series
