
#ifndef RTKM_MIDI_H_FILE
#define RTKM_MIDI_H_FILE

#include "stdint.h"

#define EVENT_NOTE_OFF			0x08
#define EVENT_NOTE_ON			0x09
#define	EVENT_POLYKEY			0x0A
#define EVENT_CONTROL_CHANGE	0x0B
#define EVENT_PRESSURE_CHANGE	0x0D


void 	midi_parse_USB_OMNI (uint8_t *message);
uint8_t midi_note_velocity	(uint8_t index);
uint8_t midi_control_value	(uint8_t index);

#endif

/**@}*/

