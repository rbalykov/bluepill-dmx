#ifndef _TIMER_H
#define _TIMER_H
#include "stm32f10x.h"
#include "stm32f10x_conf.h"
 
void TIM6_Init(u16 arr,u16 psc);
void TIM6_IRQHandler(void);

#endif
