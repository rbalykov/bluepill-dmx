#ifndef ___USER_USBDMX_512_HFILE___
#define ___USER_USBDMX_512_HFILE___

#include <stdint.h>

#define USBDMX_DELIMITER_START	(0x7E)
#define USBDMX_DELIMITER_STOP	(0xE7)
#define USBDMX_LEN_MAX			(600)

#define USBDMX_MAXBUF			(513)
#define USBDMX_SC_DMXDATA		(0x00)


#define USBDMX_CDC_ERRDELIM_A	(-1)
#define USBDMX_CDC_ERRDELIM_B	(-2)
#define USBDMX_CDC_ERRLEN		(-3)
#define USBDMX_CDC_OK			(1)

#define USBDMX_LABEL_REPROGRAM	(1)
#define USBDMX_LABEL_FLASHPAGE	(2)
#define USBDMX_LABEL_SETPARAM	(4)
#define USBDMX_LABEL_RXDATA		(5)
#define USBDMX_LABEL_RDMREQUEST	(7)
#define USBDMX_LABEL_RXOPTIONS  (8)
#define USBDMX_LABEL_RXCHDATA	(9)
#define USBDMX_LABEL_RDMDISCOVERY	(11)

#define USBDMX_LABEL_GETPARAM	(3)
#define USBDMX_LABEL_TRANSMIT	(6)
#define USBDMX_LABEL_GETSERIAL	(10)
#define USBDMX_LABEL_GETVENDOR	(77)
#define USBDMX_LABEL_GETNAME	(78)
#define USBDMX_LABEL_RDM		(82)

typedef struct usbdmx_cmd_header
{
	uint8_t	delimiter;
	uint8_t label;
	uint8_t len_lsb;
	uint8_t len_msb;
} usbdmx_cmd_header_t;


int usbdmx_handle_cdc_request (uint8_t *buffer, uint16_t len);

extern uint8_t* 	usbdmx_reply;
extern uint16_t 	usbdmx_reply_len;


#endif
