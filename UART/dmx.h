#ifndef ___DMX_512_HFILE___
#define ___DMX_512_HFILE___

// DMX RX Status

#define			DMX_RXSM_STATE_UNKNOWN 	(0)
#define			DMX_RXSM_STATE_AWAIT 	(1)
#define			DMX_RXSM_STATE_ARMED	(2)
#define			DMX_RXSM_STATE_IDLE		(3)
#define			DMX_RXSM_STATE_DATA 	(4)
#define			DMX_RXSM_STATE_COMPLETE	(5)
#define			DMX_RXSM_STATE_ERROR	(6)
#define			DMX_RXSM_STATE_FAULT	(7)

#define			DMX_RXSM_STATE_SIZE		(8)

#define			DMX_RXSM_TRANSITION_UNKNOWN (0)
#define			DMX_RXSM_TRANSITION_FE 		(1)
#define			DMX_RXSM_TRANSITION_IDLE 	(2)
#define			DMX_RXSM_TRANSITION_RXNE 	(3)
#define			DMX_RXSM_TRANSITION_BREAK   (4)
#define			DMX_RXSM_TRANSITION_FAULT   (5)

#define			DMX_RXSM_TRANSITION_SIZE    (6)
//#define			DMX_RXSM_TRANSITION_ORE     (0xFF)
//#define			DMX_RXSM_TRANSITION_NE      (0xFE)

// indicator blink masks
#define			DMX_RXSM_INDICATOR_ON		(0xFF)		// ON
#define			DMX_RXSM_INDICATOR_OFF		(0x00)		// OFF
#define			DMX_RXSM_INDICATOR_SLOW		(0x0F)		// Slow pulse
#define			DMX_RXSM_INDICATOR_FAST		(0x53)		// pause, two short pulses
#define			DMX_RXSM_INDICATOR_ULTRA	(0x55)		// fast pulses

#define			DMX_RXSM_DATA_TIMEOUT	(10)	// TIM4 UPD 10 Hz -> timeout 1 sec
#define 		DMX_RXSM_IM_PSC		    (0xFFFF)		//
#define			DMX_RXSM_TIM_10HZ		(109)			// 

#define			DMX_MAX_BUFFER_SIZE (513)			// Buffer length
#define			DMX_RXSM_BUF_A			(0)
#define			DMX_RXSM_BUF_B			(1)

#define			DMX_RXSM_FLAG_DATA_UPDATED	(1)		// set to 1 when receive dmx byte
#define			DMX_RXSM_FLAG_DATA_STILL	(0)		// set to 1 when receive dmx byte
#define			DMX_STARTCODE		(0x00)			//

#define F_CPU 		(72000000UL)
#define BAUDRATE 	(250000)
#define UART_MULTDIV (16)
#define MANTISSA_BITS (4)

void dmx_init (void);


#endif
