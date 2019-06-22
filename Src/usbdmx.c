
#include <string.h>
#include "usbdmx.h"
#include "myprintf.h"

#define STM32_UUID ((uint32_t*) 0x1FFF7A10)
//uint8_t usbdmx_buf[USBDMX_MAXBUF];

uint8_t* 	usbdmx_reply;
uint16_t 	usbdmx_reply_len;

static uint8_t reply_vendor[] 
	= {USBDMX_DELIMITER_START, USBDMX_LABEL_GETVENDOR, 0x00, 0x00, 
		0x6B, 0x6A, 'L', 'i', 'g', 'h', 't', 'P', 'H', 'Y', 0x00, 
		USBDMX_DELIMITER_STOP};
		
static uint8_t reply_name []
	= {USBDMX_DELIMITER_START, USBDMX_LABEL_GETNAME, 
		0x00, 0x00, 0x00, 0x01, 'U', 'n', 'i', 'D', 'm', 'x', 'P', 'r', 'o', 0x00,
		USBDMX_DELIMITER_STOP};

static uint8_t reply_version[] 
	= {USBDMX_DELIMITER_START, USBDMX_LABEL_GETPARAM, 0x00, 0x00,
		0x00, 0x01, 0x09, 0x01, 0x01, 
		USBDMX_DELIMITER_STOP};

// -----------------------------------------------------------------------------
int usbdmx_handle_cdc_request (uint8_t *buffer, uint16_t len)
{
	my_printf("cdc req\n");
	usbdmx_cmd_header_t *header = (usbdmx_cmd_header_t *) buffer;
	if (header->delimiter != USBDMX_DELIMITER_START) return USBDMX_CDC_ERRDELIM_A;
	
	uint16_t msg_len = (header->len_lsb) +  (header->len_lsb)*0x100;
	if (msg_len > USBDMX_LEN_MAX) return USBDMX_CDC_ERRLEN;
	
	if (buffer[msg_len + 4*sizeof(uint8_t)] != USBDMX_DELIMITER_STOP) return USBDMX_CDC_ERRDELIM_B;

	usbdmx_reply 		= NULL;
	usbdmx_reply_len	= 0 ;
	
	switch (header->label)
	{
	case USBDMX_LABEL_GETPARAM:
		my_printf("get param\n");
		reply_version[2] = (sizeof(reply_version) & 0xFF);
		reply_version[3] = (sizeof(reply_version) >> 8);
		usbdmx_reply = reply_version;
		usbdmx_reply_len = sizeof(reply_version);
	case USBDMX_LABEL_GETVENDOR:
		my_printf("vendor\n");
		reply_vendor[2] = (sizeof(reply_vendor) & 0xFF);
		reply_vendor[3] = (sizeof(reply_vendor) >> 8);
		usbdmx_reply = reply_vendor;
		usbdmx_reply_len = sizeof(reply_vendor);
		break;
	case USBDMX_LABEL_GETNAME:
		my_printf("name\n");
		reply_name[2] = (sizeof(reply_name) & 0xFF);
		reply_name[3] = (sizeof(reply_name) >> 8);
		usbdmx_reply = reply_name;
		usbdmx_reply_len = sizeof(reply_name);
		break;
	case USBDMX_LABEL_GETSERIAL:
		my_printf("serial\n");
		usbdmx_reply = (uint8_t*)STM32_UUID;
		usbdmx_reply_len = 4;
		break; 
	default:
		break;
	}
	return USBDMX_CDC_OK;
}

// -----------------------------------------------------------------------------

