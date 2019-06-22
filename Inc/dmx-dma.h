#ifndef __DMX_DMA_HEADER_FILE__

#define __DMX_DMA_HEADER_FILE__

#include "stm32f1xx_hal.h"

#define DMX_MAX_FRAME_SIZE (513)
#define DMX_MIN_FRAME_SIZE (25)

#define DMX_TX_PORT_COUNT (2)
#define DMX_TX_PORT_A (0)
#define DMX_TX_PORT_B (1)

#define DMX_TX_DOUBLEBUFFERED (2)
#define DMX_TX_BUFFER_A (0)
#define DMX_TX_BUFFER_B (1)

#define DMX_TX_DATA_UNCHANGED (0)
#define DMX_TX_DATA_UPDATED   (1)

#define DMX_RX_FRAME_SIZE (514)
#define DMX_RX_PORT_COUNT (1)
#define DMX_RX_PORT_A (0)

#define DMX_RX_DATA_UNCHANGED (0)
#define DMX_RX_DATA_UPDATED   (1)

#define DMX_RX_FE_BREAK_DETECT_MASK (0x03)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_tx;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;

#define UART_DMX_A	huart1
#define UART_DMX_B	huart3
#define UART_RX_A	huart2

#define UART_TIMER_A	htim2
#define UART_TIMER_B	htim3

#define DMXTIMING_BREAK_MIN_BITS	(22)
#define DMXTIMING_MAB_MIN_BITS		(11)

#define			DMX_RXSM_STATE_NONE 	(0)
#define			DMX_RXSM_STATE_AWAIT 	(1)
#define			DMX_RXSM_STATE_ARMED	(2)
#define			DMX_RXSM_STATE_IDLE		(3)
#define			DMX_RXSM_STATE_DATA 	(4)
#define			DMX_RXSM_STATE_COMPLETE	(5)
#define			DMX_RXSM_STATE_ERROR	(6)
#define			DMX_RXSM_STATE_FAULT	(7)

//#define ___DMX_BREAK_START_A()  (HAL_GPIO_WritePin(GPIOB, DMX_EN_A_Pin, GPIO_PIN_SET))

typedef struct
{
	uint8_t port_rx_FE;
	uint8_t port_rx_state;
	uint8_t * data_rx;
	UART_HandleTypeDef * huart;
	uint8_t data_updated;
	uint16_t data_offset;
	uint8_t	 indication;
} DMX_RX_Port_Handler_t;

#define	DMX_RX_INDICATION_DATA_ON	(0xFF)		// ON
#define	DMX_RX_INDICATION_DATA_OFF	(0x00)		// OFF
#define	DMX_RX_INDICATION_FAULT		(0x53)		// pause, two short pulses

void dmx_handle_input_buffer (uint8_t buffer_id, uint8_t *data, uint16_t len);
void dmx_tx_start (void);

void dmx_rx_start (void);
void dmx_rx_irq_handler (UART_HandleTypeDef *huart);
void dmx_rx_handler 	(DMX_RX_Port_Handler_t *hport);
void dmx_rx_complete 	(DMX_RX_Port_Handler_t *hport);
void dmx_rx_fault 		(DMX_RX_Port_Handler_t *hport);

#endif
