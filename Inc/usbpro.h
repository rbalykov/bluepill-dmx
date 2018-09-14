/*
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; either version 2 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU Library General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program; if not, write to the Free Software
 *  Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 *
 * usbpro.h
 * Copyright (C) 2011 Simon Newton
 * Copyright (C) 2018 
 * Contains the message labels used to identify packets.
 */

#ifndef STMSERIAL_USBPRO_DMX_HEADER_FILE
#define STMSERIAL_USBPRO_DMX_HEADER_FILE

#include <stdint.h>

#define DMXUSBPRO_MAX_MESSAGE	(605)
#define DMXUSBPRO_MESSAGE_START (0x7E)
#define DMXUSBPRO_MESSAGE_END 	(0xE7)
#define DMXUSBPRO_ESTA_ID		(0x6A6B)	// DMXKing
#define DMXUSBPRO_DEVICE_ID		(0x0002)	// UltraDMXPro


extern void usb_rx_handler 	(uint8_t *buf, uint32_t *size);
extern void message_handler (uint8_t label, uint8_t *buf, uint16_t size);

extern void usb_send (uint8_t label, uint8_t *data, uint16_t size);

// Message Label Codes
enum {
	PARAMETERS_LABEL 	= 3,
	LABEL_RECEIVED_DMX 	= 5,
	DMX_DATA_LABEL 		= 6,
	SERIAL_NUMBER_LABEL = 10,
	MANUFACTURER_LABEL 	= 77,
	NAME_LABEL 			= 78,
	RDM_LABEL 			= 82,
	LABEL_UNIVERSE_0	= 100,
	LABEL_UNIVERSE_1	= 101
};



typedef enum {
	PRE_SOM = 0,
	GOT_SOM = 1,
	GOT_LABEL = 2,
	GOT_DATA_LSB = 3,
	IN_DATA = 4,
	WAITING_FOR_EOM = 5,
} rx_state_t;

extern char 	DEVICE_NAME[];
extern uint8_t 	DEVICE_NAME_SIZE;
extern char 	MANUFACTURER_NAME[];
extern uint8_t 	MANUFACTURER_NAME_SIZE;


#endif // STMSERIAL_USBPRO_DMX_HEADER_FILE
