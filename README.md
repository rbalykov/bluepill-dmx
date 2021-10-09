# bluepill-dmx
USB-DMX Interface with WS2812 support.
(C) 2019, GNU GPL v3

This is clone of DMXKing UltraDMX Pro.
- 2x DMX outputs
- 1x DMX input
- 2x WS2812 pixel outputs

Requirements:
- STM32F103C8 aka Blue Pill Board
- Original board uses FTDI chip, so this firmware works only in OLA on Linux.
If you wish, replace USB VID/PID with FTDI's, it's going to be supported everywhere.

Hardware:
- UART1 TX, UART3 TX are 5V-tolerant, are used to drive output 5V optocouplers.
If using 6n137, connect them to LED cathode (open-drain).
- UART2 RX is 3.3V pin, used as input. 
Connect it to resistor-divided MAX485 output.
- Pins PB3, PB4 repeat DMX data in WS8212 format, assuming pixels are RGB,
using 170x3=510 bytes of each universe.
Pins should be connected to voltage-shifter, 74HCT series recommended.
