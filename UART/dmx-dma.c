
#include "dmx-dma.h"
#include <string.h>
#include "stm32f1xx_hal_gpio.h"
#include "main.h"

#define LED_ON()		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET)  
#define LED_OFF()		  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET)  

static uint8_t data_tx [DMX_TX_PORT_COUNT][DMX_TX_DOUBLEBUFFERED][DMX_MAX_BUFFER_SIZE] = {0};
static uint8_t port_active_buffer[DMX_TX_PORT_COUNT] = {DMX_TX_BUFFER_A};
static uint8_t port_data_updated [DMX_TX_PORT_COUNT] = {DMX_TX_DATA_UNCHANGED};

// -----------------------------------------------------------------------------
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

//	if (data_tx[0][1][1] > 100) {LED_ON();}
//	else {LED_OFF();}
	}
}

// -----------------------------------------------------------------------------
void dmx_transmit_start (void)
{
	HAL_UART_Transmit_DMA (&UART_DMX_A, \
		data_tx[DMX_TX_PORT_A][port_active_buffer[DMX_TX_PORT_A]], DMX_MAX_BUFFER_SIZE);
	HAL_UART_Transmit_DMA (&UART_DMX_B, \
		data_tx[DMX_TX_PORT_B][port_active_buffer[DMX_TX_PORT_B]], DMX_MAX_BUFFER_SIZE);
}
// -----------------------------------------------------------------------------
void dmx_transmit_FE (TIM_HandleTypeDef *htim)
{
    __HAL_TIM_SET_COUNTER(htim, 0);
    HAL_TIM_Base_Start(htim);
    __HAL_TIM_ENABLE_IT(htim, TIM_IT_UPDATE);
}
// -----------------------------------------------------------------------------
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
	// BREAK end, MAB start
	uint8_t pin_id = (htim == &UART_TIMER_A) ? DMX_EN_A_Pin: DMX_EN_B_Pin;

	// turn LED VCC on->off
	HAL_GPIO_WritePin(GPIOB, pin_id, GPIO_PIN_RESET);
}
// -----------------------------------------------------------------------------
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	UART_HandleTypeDef *huart;
	uint8_t port_id;
	
	HAL_TIM_OnePulse_DeInit(htim);

	if (htim == &UART_TIMER_A)
	{
		huart = &UART_DMX_A;
		port_id = DMX_TX_PORT_A;
	}
	else
	{
		huart = &UART_DMX_B;
		port_id = DMX_TX_PORT_B;
	}

	HAL_UART_Transmit_DMA (huart, \
		data_tx[port_id][port_active_buffer[port_id]], DMX_MAX_BUFFER_SIZE);
}

// -----------------------------------------------------------------------------
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t port_id;
	uint16_t pin_id;
	TIM_HandleTypeDef *htim;
	if (huart == &UART_DMX_A)
	{
		port_id  = DMX_TX_PORT_A;
		htim     = &UART_TIMER_A;
		pin_id   = DMX_EN_A_Pin;
	}
	else
	{
		port_id  = DMX_TX_PORT_B;
		htim     = &UART_TIMER_B;
		pin_id   = DMX_EN_B_Pin;
	}
	
	if (port_data_updated [port_id] == DMX_TX_DATA_UPDATED)
	{
		port_active_buffer[port_id] = \
			(port_active_buffer[port_id] == DMX_TX_BUFFER_A) ? \
			DMX_TX_BUFFER_B : DMX_TX_BUFFER_A;
		port_data_updated [port_id]  = DMX_TX_DATA_UNCHANGED;
	}

	//	BREAK LINE (VCC off->on)
	HAL_GPIO_WritePin(GPIOB, pin_id, GPIO_PIN_SET);
	dmx_transmit_FE(htim);
}
// -----------------------------------------------------------------------------
