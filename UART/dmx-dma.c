
#include "dmx-dma.h"
#include <string.h>
#include "stm32f1xx_hal_gpio.h"


#define LED_ON()		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)  
#define LED_OFF()		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)  

static uint8_t data_tx [DMX_TX_PORT_COUNT][DMX_TX_DOUBLEBUFFERED][DMX_MAX_BUFFER_SIZE] = {0};
static uint8_t port_active_buffer[DMX_TX_PORT_COUNT] = {DMX_TX_BUFFER_A};
static uint8_t port_data_updated [DMX_TX_PORT_COUNT] = {DMX_TX_DATA_UNCHANGED};

void dmx_handle_input_buffer (uint8_t port_id, uint8_t *data, uint16_t len)
{
	uint8_t buffer_write_to;
	len = (len > DMX_MAX_BUFFER_SIZE) ? DMX_MAX_BUFFER_SIZE : len;
		
	if ((port_id == DMX_TX_PORT_A) || (port_id == DMX_TX_PORT_B))
	{
		buffer_write_to = (port_active_buffer[port_id] == DMX_TX_BUFFER_A)? \
				DMX_TX_BUFFER_B: DMX_TX_BUFFER_A;
		memcpy(data_tx[port_id][buffer_write_to], data, len);
		port_data_updated [port_id] = DMX_TX_DATA_UPDATED;
		
	if (data_tx[0][0][1] > 1) {LED_ON();}
	else {LED_OFF();}
	}
}


void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	
}
