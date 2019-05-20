#ifndef __DMX_DMA_HEADER_FILE__

#define __DMX_DMA_HEADER_FILE__

#include "stm32f1xx_hal.h"

#define DMX_MAX_BUFFER_SIZE (513)

#define DMX_TX_PORT_COUNT (2)
#define DMX_TX_PORT_A (0)
#define DMX_TX_PORT_B (1)

#define DMX_TX_DOUBLEBUFFERED (2)
#define DMX_TX_BUFFER_A (0)
#define DMX_TX_BUFFER_B (1)

#define DMX_TX_DATA_UNCHANGED (0)
#define DMX_TX_DATA_UPDATED   (1)

extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart1_tx;
extern DMA_HandleTypeDef hdma_usart2_tx;
extern DMA_HandleTypeDef hdma_usart3_tx;

void dmx_handle_input_buffer (uint8_t buffer_id, uint8_t *data, uint16_t len);
#endif
