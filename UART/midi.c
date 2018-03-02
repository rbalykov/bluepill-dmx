
#include "midi.h"

uint8_t midi_notes[128] = {0};
uint8_t midi_control[128] = {0};


// ============================================================================
void midi_parse_USB_OMNI (uint8_t *message)
{
	uint8_t event = (message[0] & 0x0F);
	static uint8_t note_index = 0, note_value = 0;
	static uint8_t ctrl_index = 0, ctrl_value = 0;
	
	if (event == EVENT_NOTE_OFF)
	{
		note_index = (message[2] & 0x7F);
		midi_notes[note_index] = 0;
	}
	else if   ((event == EVENT_NOTE_ON)
			|| (event == EVENT_POLYKEY))
	{
		note_index = (message[2] & 0x7F);
		note_value = (message[3] & 0x7F);
		midi_notes[note_index] = note_value;
	}
	else if (event == EVENT_PRESSURE_CHANGE)
	{
		// use note_index from last note
		note_value = (message[2] & 0x7F);
		midi_notes[note_index] = note_value;
	}
	else if (event == EVENT_CONTROL_CHANGE)
	{
		ctrl_index = (message[2] & 0x7F);
		ctrl_value = (message[3] & 0x7F);
		midi_control[ctrl_index] = ctrl_value;
	}
}

// ============================================================================
uint8_t midi_note_velocity(uint8_t index)
{
	if (index > 127) return 0;
	return midi_notes[index];
}

// ============================================================================
uint8_t midi_control_value	(uint8_t index)
{
	if (index > 127) return 0;
	return midi_control[index];
}
// ============================================================================
