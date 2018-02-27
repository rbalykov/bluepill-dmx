#include "stm32f10x_usart.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"

#include "dmx.h"
#include <stdint.h> 

void TIM4_IRQHandler	(void);
void USART1_IRQHandler	(void);

#define DMX_DEBUG

// ----------------------------------------------------------------------------
// FAULT CALLBACKS
// ----------------------------------------------------------------------------
typedef int (*RXSM_FaultHandler_t)(int line,char *file, char *message);	

int dmx_rxsm_fault_overrun (int l, char* f, char* m)
	{ return 0;}

int dmx_rxsm_fault_noise    (int l, char* f, char* m)
	{ return 0;}

int dmx_rxsm_fault_assert    (int l, char* f, char* m)
	{ return 0;}

RXSM_FaultHandler_t dmx_rxsm_FAULT_ORE 		= dmx_rxsm_fault_overrun;
RXSM_FaultHandler_t dmx_rxsm_FAULT_NE  		= dmx_rxsm_fault_noise;
RXSM_FaultHandler_t dmx_rxsm_FAULT_ASSERT 	= dmx_rxsm_fault_assert;

// ----------------------------------------------------------------------------
// STATE-TRANSITION CALLBACKS
// ----------------------------------------------------------------------------
typedef int (*RXSM_StateHandler_t)(uint16_t DR, uint16_t SR, uint8_t BR_detected);	

int dmx_rxsm_handler_assert (uint16_t DR, uint16_t SR, uint8_t BR_detected)
	{
	return  dmx_rxsm_FAULT_ASSERT ? 
		dmx_rxsm_FAULT_ASSERT(__LINE__, __FILE__, "RXSM illegal transition")
		: 0 ;
	}

int dmx_rxsm_hNOP	 	(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
int dmx_rxsm_hBREAK 	(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
int dmx_rxsm_hMAB 		(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
int dmx_rxsm_hDEMAB 	(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
int dmx_rxsm_hSC 		(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
int dmx_rxsm_hDATA 		(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
int dmx_rxsm_hCOMPLETE 	(uint16_t DR, uint16_t SR, uint8_t BR_detected) { return 0; }
	
// ----------------------------------------------------------------------------
// STATE-TRANSITION MAP
// ----------------------------------------------------------------------------
static RXSM_StateHandler_t 
dmx_rxsm_state_map [DMX_RXSM_STATE_SIZE][DMX_RXSM_TRANSITION_SIZE] = {0};

void dmx_rxsm_mapinit (void)
{
	int i, j;
	for (i=0; i< DMX_RXSM_STATE_SIZE; i++)
	{
		for (j=0; j< DMX_RXSM_TRANSITION_SIZE; j++)
		{
			dmx_rxsm_state_map[i][j] = dmx_rxsm_handler_assert;
		}
	}

	dmx_rxsm_state_map[DMX_RXSM_STATE_AWAIT 	][DMX_RXSM_TRANSITION_FE]   = dmx_rxsm_hNOP;
	dmx_rxsm_state_map[DMX_RXSM_STATE_AWAIT 	][DMX_RXSM_TRANSITION_IDLE] = dmx_rxsm_hNOP;
	dmx_rxsm_state_map[DMX_RXSM_STATE_AWAIT 	][DMX_RXSM_TRANSITION_RXNE] = dmx_rxsm_hNOP;

	dmx_rxsm_state_map[DMX_RXSM_STATE_AWAIT 	][DMX_RXSM_TRANSITION_BREAK] = dmx_rxsm_hBREAK;
	dmx_rxsm_state_map[DMX_RXSM_STATE_ERROR		][DMX_RXSM_TRANSITION_BREAK] = dmx_rxsm_hBREAK;
	dmx_rxsm_state_map[DMX_RXSM_STATE_COMPLETE	][DMX_RXSM_TRANSITION_BREAK] = dmx_rxsm_hBREAK;

	dmx_rxsm_state_map[DMX_RXSM_STATE_ARMED		][DMX_RXSM_TRANSITION_IDLE] = dmx_rxsm_hMAB;
	dmx_rxsm_state_map[DMX_RXSM_STATE_ARMED		][DMX_RXSM_TRANSITION_FE] 	= dmx_rxsm_hNOP;

	dmx_rxsm_state_map[DMX_RXSM_STATE_IDLE		][DMX_RXSM_TRANSITION_FE]   = dmx_rxsm_hDEMAB;
	dmx_rxsm_state_map[DMX_RXSM_STATE_IDLE		][DMX_RXSM_TRANSITION_IDLE] = dmx_rxsm_hNOP;
	dmx_rxsm_state_map[DMX_RXSM_STATE_IDLE		][DMX_RXSM_TRANSITION_RXNE] = dmx_rxsm_hSC;

	dmx_rxsm_state_map[DMX_RXSM_STATE_DATA 		][DMX_RXSM_TRANSITION_RXNE] = dmx_rxsm_hDATA;
	dmx_rxsm_state_map[DMX_RXSM_STATE_DATA 		][DMX_RXSM_TRANSITION_IDLE] = dmx_rxsm_hNOP;
	dmx_rxsm_state_map[DMX_RXSM_STATE_DATA 		][DMX_RXSM_TRANSITION_FE]   = dmx_rxsm_hCOMPLETE;

}

// -----------------------------------------------------------------------------
// RX double-buffered
// -----------------------------------------------------------------------------
static uint8_t	dmx_rxsm_buf [2][DMX_MAX_BUFFER_SIZE] = {0};
static volatile	uint8_t		dmx_rxsm_active_buffer = DMX_RXSM_BUF_A;

static volatile uint16_t	dmx_rxsm_byte_counter;			// rx bytes
static volatile	uint8_t		dmx_rxsm_frame_counter = 0;		// rx frames
static volatile	uint8_t		dmx_rxsm_active_state;			// 

static __inline uint8_t* rx_active_buffer()
{
	return dmx_rxsm_buf[dmx_rxsm_active_buffer];
}

static __inline void buffer_swap (void)
{
	if (dmx_rxsm_active_buffer == DMX_RXSM_BUF_A)
		{ dmx_rxsm_active_buffer = DMX_RXSM_BUF_B; }
	else
		{ dmx_rxsm_active_buffer = DMX_RXSM_BUF_A; }	
}

static __inline int data_updated (void)
{
	static uint16_t last_frame = 0xFFFF;
	int result = (dmx_rxsm_frame_counter != last_frame);
	last_frame = dmx_rxsm_frame_counter;
	return result;
}

// -----------------------------------------------------------------------------
// RX STATE MACHINE IRQ HANDLER
// -----------------------------------------------------------------------------
void USART1_IRQHandler(void)
{
#ifdef DMX_DEBUG
	GPIOA->BSRR = GPIO_BSRR_BS3;	// measure IRQ handler time
	// debug: provoke data overrun
	// for (uint32_t i = 0xFFFF; i; ) {i--;}
#endif

	static uint8_t BREAK_buf      = 0;
	static uint8_t BREAK_detected = 0;
	static uint8_t transition = DMX_RXSM_TRANSITION_UNKNOWN;
	RXSM_StateHandler_t tr_handler = 0;
	
	// DR Data Register
	// SR Status Register { RXNE, IDLE, ORE, NE, FE}
	// READ SR + READ DR = CLEAR SR
	uint16_t rxdata, rxflags;
	rxflags 	= USART1->SR;
	rxdata 		= USART1->DR;

#ifdef DMX_DEBUG
	// debug pins for logic analyser
	GPIOA->BSRR = (rxflags & USART_SR_FE) 	? GPIO_BSRR_BS0 : GPIO_BSRR_BR0;
	GPIOA->BSRR = (rxflags & USART_SR_IDLE)	? GPIO_BSRR_BS1 : GPIO_BSRR_BR1;
#endif

	BREAK_buf = (BREAK_buf << 1) | (rxflags & USART_SR_FE) ? 1 : 0;	
	BREAK_detected = (BREAK_buf & 0x03) ? 1 : 0;

	if (rxflags & USART_SR_ORE)
	{
		transition = DMX_RXSM_TRANSITION_FAULT;
		if (dmx_rxsm_FAULT_ORE != 0)
		 { dmx_rxsm_FAULT_ORE(__LINE__, __FILE__, "Your MCU is too slow");	}
	}
	else if (rxflags & USART_SR_NE)
	{
		transition = DMX_RXSM_TRANSITION_FAULT;
		if (dmx_rxsm_FAULT_NE != 0)
		 { dmx_rxsm_FAULT_NE(__LINE__, __FILE__, "UART Noise, check your cables");	}	
	}
	else if (BREAK_detected)
		{ transition = DMX_RXSM_TRANSITION_BREAK; }
	else if (rxflags & USART_SR_FE)
		{ transition = DMX_RXSM_TRANSITION_FE; }
	else if (rxflags & USART_SR_IDLE)
		{ transition = DMX_RXSM_TRANSITION_IDLE; }
	else 
		{ transition = DMX_RXSM_TRANSITION_RXNE; }

	tr_handler = dmx_rxsm_state_map[dmx_rxsm_active_state][transition];
	if (tr_handler != 0)
		{ tr_handler(rxdata, rxflags, BREAK_detected);}
	else
		{
		if (dmx_rxsm_FAULT_ASSERT) 
			{ dmx_rxsm_FAULT_ASSERT(__LINE__, __FILE__, "RXSM map not initialised");}
		}

#ifdef DMX_DEBUG	
	// debug pins for logic analyser
	GPIOA->BSRR = (dmx_rxsm_active_state == DMX_RXSM_STATE_ARMED) 	? GPIO_BSRR_BS2 : GPIO_BSRR_BR2;
	GPIOA->BSRR = (dmx_rxsm_active_state == DMX_RXSM_STATE_DATA) 	? GPIO_BSRR_BS4 : GPIO_BSRR_BR4;
	GPIOA->BSRR = (dmx_rxsm_active_state == DMX_RXSM_STATE_ERROR) 	? GPIO_BSRR_BS5 : GPIO_BSRR_BR5;
	GPIOA->BSRR = GPIO_BSRR_BR3; 	// measure IRQ handler time
#endif
}

// -----------------------------------------------------------------------------
// RX STATE MACHINE INDICATION
// LED at PC13
// -----------------------------------------------------------------------------

void TIM4_IRQHandler(void)
{
	static uint8_t 	indicator 	= 0; 						// indicator pattern pointer
	static uint8_t	pattern		= DMX_RXSM_INDICATOR_OFF; 	//
	static uint8_t	still_counter = 0; 	
	
	// same as TIM_ClearITPendingBit (TIM4, TIM_IT_Update);
	TIM4->SR = (uint16_t)~TIM_IT_Update;

	if (data_updated())
	{	
		pattern = DMX_RXSM_INDICATOR_ON;
		still_counter = 0;
	}
	else
		{ still_counter++; }

	if (still_counter >= DMX_RXSM_DATA_TIMEOUT)
		{ pattern = DMX_RXSM_INDICATOR_SLOW; }
	else if (dmx_rxsm_active_state == DMX_RXSM_STATE_FAULT)
		{ pattern = DMX_RXSM_INDICATOR_SLOW; }
	else if(dmx_rxsm_active_state == DMX_RXSM_STATE_ERROR)
		{ pattern = DMX_RXSM_INDICATOR_SLOW; }

	GPIOC->BSRR = ((pattern) & (1<<indicator)) ? GPIO_BSRR_BR13 : GPIO_BSRR_BS13;
	indicator++;	
	indicator = (indicator > 7) ? 0 : indicator;	// range [0 to 7]
}
// ----------------------------------------------------------------------------
// STATE MACHINE INIT
// ----------------------------------------------------------------------------
void dmx_rxsm_initsm (void)
{
	dmx_rxsm_mapinit();
	dmx_rxsm_active_state 	= DMX_RXSM_STATE_AWAIT;	
	dmx_rxsm_active_buffer	= DMX_RXSM_BUF_A;
	dmx_rxsm_frame_counter 	= 0;
	dmx_rxsm_byte_counter	= 0;
}
// ----------------------------------------------------------------------------
// HARDWARE INIT (STM32F103Cx) 
// ----------------------------------------------------------------------------
void dmx_rxsm_inithw_STM32F103Cx()
{
	// ------------------------------------------------------
	// LED pin PC13 output open-drain
	// ------------------------------------------------------
	// Port C
	RCC->APB2ENR |= (RCC_APB2ENR_IOPCEN);	
	// PC13
	GPIOC->CRH &= ~(GPIO_CRH_CNF13 | GPIO_CRH_MODE13);
	GPIOC->CRH |= (GPIO_CRH_CNF13_0 | GPIO_CRH_MODE13_1);
	// ------------------------------------------------------
	// Timer4 (Indication)
	// ------------------------------------------------------
	RCC->APB1ENR |= (RCC_APB1ENR_TIM4EN);
	TIM4->ARR = DMX_RXSM_TIM_10HZ; 		// 109
	TIM4->PSC = DMX_RXSM_IM_PSC;		// 0xFFFF
	TIM4->DIER |= TIM_DIER_UIE;			// enable update IRQ
	TIM4->CR1 |= TIM_CR1_CEN;			// enable timer
	NVIC_EnableIRQ(TIM4_IRQn); 
	// ------------------------------------------------------------
	// UART1 RX, PB
	// ------------------------------------------------------------
	// Port A
	RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN);
	// PA10 input floating
	GPIOA->CRH &= ~(GPIO_CRH_CNF10 | GPIO_CRH_MODE10);
	GPIOA->CRH |= (GPIO_CRH_CNF10_0);
	// UART
	RCC->APB2ENR  |= (RCC_APB2ENR_USART1EN);
	USART1->CR1 |= USART_CR1_UE;		// enable usart
	USART1->BRR = (F_CPU/(UART_MULTDIV * BAUDRATE )) << MANTISSA_BITS; // 4-bit mantissa
	USART1->CR1 &= ~(USART_CR1_M);		// 8-bit word
	USART1->CR2 &= ~(USART_CR2_STOP);	// 2 stop bits
	USART1->CR2 |= USART_CR2_STOP_1; 	// 2 stop bits
	USART1->CR1 |= USART_CR1_RXNEIE;	// rx interrupt enable
	// IRQ
	NVIC_EnableIRQ (USART1_IRQn); 
	USART1->CR1 |= USART_CR1_RE;		// receive enable

// ---------------------------------------------
// 		LOGIC ANALYSER PINS
// ---------------------------------------------	
#ifdef DMX_DEBUG	
	GPIOA->CRL &= ~(GPIO_CRL_CNF0 | GPIO_CRL_MODE0);
	GPIOA->CRL |= (GPIO_CRL_MODE0);
	GPIOA->CRL &= ~(GPIO_CRL_CNF1 | GPIO_CRL_MODE1);
	GPIOA->CRL |= (GPIO_CRL_MODE1);
	GPIOA->CRL &= ~(GPIO_CRL_CNF2 | GPIO_CRL_MODE2);
	GPIOA->CRL |= (GPIO_CRL_MODE2);
	GPIOA->CRL &= ~(GPIO_CRL_CNF3 | GPIO_CRL_MODE3);
	GPIOA->CRL |= (GPIO_CRL_MODE3);
	GPIOA->CRL &= ~(GPIO_CRL_CNF4 | GPIO_CRL_MODE4);
	GPIOA->CRL |= (GPIO_CRL_MODE4);
	GPIOA->CRL &= ~(GPIO_CRL_CNF5 | GPIO_CRL_MODE5);
	GPIOA->CRL |= (GPIO_CRL_MODE5);
#endif

}
