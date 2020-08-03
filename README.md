# neutrino-dmx
"Blue Pill" aka STM32F103C8 USB Device emulating DMXKing UltraDMX Pro + WS2812 LED output.
Original board uses FTDI chip, so this firmware works only in OLA on Linux.

UART1 TX, UART3 TX are 5V-tolerant, connect them to 6N137 LED cathode, then to MAX 485 input.
UART2 RX is 3.3V input, connect it to resistor-divided MAX485 driver output.
Pins PB3, PB4 repeat DMX data as DMA-driven WS8212, should be connected to 74HCT series voltage-shifter.
