#if 0

#include "usb_lib.h"
#include "usb_desc.h"

// ============================================================================
// Table B-1 : MIDI Adapter Device Descriptor =================================
// ============================================================================

const uint8_t Midi_DeviceDescriptor[] =
  {
    0x12,          	/* bLength */
    0x01,        	/* bDescriptorType */
    0x00,          	/* 2.00 */             /* bcdUSB */
    0x02,
    0x00,           /* bDeviceClass */
    0x00,           /* bDeviceSubClass */
    0x00,           /* bDeviceProtocol */
    0x40,           /* bMaxPacketSize */
    0xE4,           /* idVendor */
    0x05,
    0xC0,           /* idProduct  = 0x5730*/
    0x16,
    0x00,          	/* 1.00 */             /* bcdDevice */
    0x01,
    1,              /* iManufacturer */
    2,              /* iProduct */
    3,              /* iSerialNumber */
    0x01            /* bNumConfigurations */
  };

// ============================================================================
// Table B-2: MIDI Adapter Configuration Descriptor ===========================
// ============================================================================
const uint8_t Midi_ConfigDescriptor[] =
  {
    /* Configuration 1 								9 bytes */
    0x09,	/* bLength */
    0x02,   /* bDescriptorType */
    0x65,   /* wTotalLength  101 bytes*/ 
    0x00,
    0x02,   /* bNumInterfaces */
    0x01,   /* bConfiguration ID */
    0x00,   /* iConfiguration - unused */
    0x80,   /* bmAttributes bus Powered*/
    0x80,   /* bMaxPower = 250 mA*/
    
// ============================================================================
// Table B-3: MIDI Adapter Standard AC Interface Descriptor ===================
// ============================================================================
												
    0x09, 	/* bLength */						/*  9 bytes */
    0x04,	/* bDescriptorType */
    0x00,   /* bInterfaceNumber */
    0x00,   /* bAlternateSetting */
    0x00,  	/* bNumEndpoints - no need here */
    0x01,   /* AUDIO bInterfaceClass */
    0x01,   /* AUDIO CONTROL bInterfaceSubClass */
    0x00,   /* unused - bInterfaceProtocol */
    0x00,   /* unused - iInterface */
	
// ============================================================================
// Table B-4: MIDI Adapter Class-specific AC Interface Descriptor =============
// ============================================================================

	0x09, 	/* bLength */						/* 09 byte*/
    0x24,   /* SC_INTERFACE bDescriptorType */
    0x01,   /* HEADER       bDescriptorSubtype */
    0x00,   /* bcdADC  - revision 1.0 */
    0x01,
    0x09,   /* wTotalLength */			
    0x00,
    0x01,   /* bInCollection - Number of streaming interfaces*/
    0x01,   /* baInterfaceNr -  interface 1 belongs to this AudioControl interface*/
    
// ============================================================================
// Table B-5: MIDI Adapter Standard MS Interface Descriptor ===================
// ============================================================================

	0x09,	// lenght								9 bytes
	0x04,	// INTERFACE descriptor
	0x01,	// index of interface
	0x00,	// index of alternate settings
	0x02,	// 2 endpoints
	0x01,	// AUDIO
	0x03,	// MIDI STREAMING
	0x00,	// unused
	0x00,	// unused
	
// ============================================================================
// Table B-6: MIDI Adapter Class-specific MS Interface Descriptor =============
// ============================================================================

	0x07,	// lenght								7 bytes
	0x24,	// CS_INTERFACE
	0x01,	// MS HEADER
	0x00,	// revision 1.0
	0x01,
	0x41,	// total 65 bytes
	0x00,
	
// ============================================================================
// Table B-7: MIDI Adapter MIDI IN Jack Descriptor (Embedded) =================
// Table B-8: MIDI Adapter MIDI IN Jack Descriptor (External) =================
// ============================================================================

	0x06, 0x24, 0x02, 0x01, 0x01, 0x00,			// 	12 bytes
	0x06, 0x24, 0x02, 0x02, 0x02, 0x00,
	
// ============================================================================
// Table B-9: MIDI Adapter MIDI OUT Jack Descriptor (Embedded) ================
// Table B-10: MIDI Adapter MIDI OUT Jack Descriptor (External) ===============
// ============================================================================
	
	0x09, 0x24, 0x03, 0x01, 0x03, 0x01, 0x02, 0x01, 0x00,	// 18 bytes
	0x09, 0x24, 0x03, 0x02, 0x04, 0x01, 0x01, 0x01, 0x00,

// #define		MIDI_OUT_EP					0x01
// #define		MIDI_IN_EP					0x81

	0x09, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00,	// 28 bytes
	0x05, 0x25, 0x01, 0x01, 0x01,

	0x09, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00, 0x00, 0x00,
	0x05, 0x25, 0x01, 0x01, 0x03
  };

/* == USB String Descriptor (optional) */
const uint8_t Midi_StringLangID[MIDI_SIZ_STRING_LANGID] =
  {
    0x04,
    0x03,
    0x09,
    0x04
  }
  ; /* LangID = 0x0409: U.S. English */

const uint8_t Midi_StringVendor[MIDI_SIZ_STRING_VENDOR] =
  {
    MIDI_SIZ_STRING_VENDOR, /* Size of manufacturer string */
    USB_STRING_DESCRIPTOR_TYPE,  /* bDescriptorType*/
    'R', 0, 'u', 0, 's', 0, 'l', 0, 'a', 0, 'n', 0, ' ', 0, 'T', 0,
    'o', 0, 'k', 0, 'm', 0, 'a', 0, 'g', 0, 'a', 0, 'm', 0, 'b', 0,
    'e', 0, 't', 0, 'o', 0, 'v', 0
  };

const uint8_t Midi_StringProduct[MIDI_SIZ_STRING_PRODUCT] =
  {
    MIDI_SIZ_STRING_PRODUCT,  /* bLength */
    USB_STRING_DESCRIPTOR_TYPE,        /* bDescriptorType */
    'M', 0, 'i', 0, 'd', 0, 'i', 0, ' ', 0, 'G', 0,
    'a', 0, 't', 0, 'e', 0, 'w', 0, 'a', 0, 'y', 0
  };

uint8_t Midi_StringSerial[MIDI_SIZ_STRING_SERIAL] =
  {
    MIDI_SIZ_STRING_SERIAL,  /* bL  ength */
    USB_STRING_DESCRIPTOR_TYPE,         /* bDescriptorType */
    '0', 0, 'S', 0, 'X', 0, 'A', 0, '1', 0
  };


#endif
