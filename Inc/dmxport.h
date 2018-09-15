#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_uart.h"

namespace dmx
{
	class dataframe_t {};

	namespace io
	{
		void TIM4_IRQHandler	(void);
		void USART1_IRQHandler	(void);

		class result_t {};
		class direction_t {};
		
		class port
		{
			public:
				result_t setDirection (direction_t d);
				result_t sendData (const dataframe_t&);
				dataframe_t receivedData ();
				
				void hookUart (UART_HandleTypeDef*);
			private:
				direction_t dataFlowDirection;
				UART_HandleTypeDef * uartHal;
		};


	} //io
} // dmx


 

