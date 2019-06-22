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

#include <string.h> 
#include "usbpro.h"
#include "usbd_cdc_if.h"

#include "dmx-dma.h"

static uint8_t usb_rxbuf[DMXUSBPRO_MAX_MESSAGE];
static uint8_t usb_txbuf[DMXUSBPRO_MAX_MESSAGE];

uint8_t DEVICE_PARAMS[] = {0, 1, 9, 1, 40};
uint8_t DEVICE_SERIAL[] = {0x01, 0x00, 0x00, 0x00};
char	DEVICE_PROVIDER[] 	= {0x6B, 0x6A, 'D', 'M', 'X', 'K', 'i', 'n', 'g'};
char	DEVICE_NAME[] 		= {0x02, 0x00, 'U', 'l', 't', 'r', 'a', 'D', 'M', 'X', ' ', 'P', 'r', 'o'};
uint8_t DEVICE_ID[] = {1, 0};


static rx_state_t	rx_state = PRE_SOM;
static uint16_t 	rx_data_offset = 0;
//uint8_t 		rx_dmxdata_0[513] = {0};

void usb_send (uint8_t label, uint8_t *data, uint16_t size)
{
	usb_txbuf[0] = DMXUSBPRO_MESSAGE_START;
	usb_txbuf[1] = label;
	usb_txbuf[2] = (size & 0xFF);
	usb_txbuf[3] = (size >> 8);
	memcpy (&usb_txbuf[4], data, size);
	usb_txbuf[4+size] = DMXUSBPRO_MESSAGE_END;
	CDC_Transmit_FS (usb_txbuf, size+5);
}

void usb_rx_handler (uint8_t *buf, uint32_t *size)
{
	uint32_t cnt = 0;
	uint8_t data = 0;
	static uint8_t 		label = 0;
	static uint16_t 	expected_size = 0;

	for (cnt=0; cnt<*size; cnt++)
	{
		data = buf[cnt];
		switch (rx_state) 
		{
			case PRE_SOM:
				if (data == DMXUSBPRO_MESSAGE_START) 
					{rx_state = GOT_SOM;}
				break;
			case GOT_SOM:
				label = data;
				rx_state = GOT_LABEL;
				break;
			case GOT_LABEL:
				rx_data_offset = 0;
				expected_size = data;
				rx_state = GOT_DATA_LSB;
				break;
			case GOT_DATA_LSB:
				expected_size += (data << 8);
				if (expected_size == 0)	{rx_state = WAITING_FOR_EOM;} 
				else {rx_state = IN_DATA;}
				break;
			case IN_DATA:
				usb_rxbuf[rx_data_offset] = data;
				rx_data_offset++;
				if (rx_data_offset == expected_size)
					{rx_state = WAITING_FOR_EOM;}
				break;
			case WAITING_FOR_EOM:
				if (data == DMXUSBPRO_MESSAGE_END)
					{
					message_handler(label, usb_rxbuf, rx_data_offset);
					rx_state = PRE_SOM;
					}
				break;
			}
		}
}


void message_handler (uint8_t label, uint8_t *buf, uint16_t size)
{
	switch (label)
	{
		case LABEL_PARAMS:
			usb_send(LABEL_PARAMS, 	DEVICE_PARAMS, sizeof(DEVICE_PARAMS));
		break;
		case LABEL_SERIAL:
			usb_send(LABEL_SERIAL, 	DEVICE_SERIAL, sizeof(DEVICE_SERIAL));
		break;

		case LABEL_VENDOR:
			usb_send(LABEL_VENDOR, 	(uint8_t*)DEVICE_PROVIDER, sizeof(DEVICE_PROVIDER));
		break;

		case LABEL_NAME:
			usb_send(LABEL_NAME, 	(uint8_t*)DEVICE_NAME, sizeof(DEVICE_NAME));
		break;

		case LABEL_DMXDATA:
		case LABEL_UNIVERSE_0:
		{
			dmx_handle_input_buffer(DMX_TX_PORT_A, buf, size);
		}
		break;
		case LABEL_UNIVERSE_1:
		{
			dmx_handle_input_buffer(DMX_TX_PORT_B, buf, size);
		}
		break;
		
		default:
		break;
	}
}

