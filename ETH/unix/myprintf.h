/*
File: printf.h
 
Copyright (c) 2004,2012 Kustaa Nyholm / SpareTimeLabs

All rights reserved.
-----------------------------------------------------------------------
To use the printf you need to supply your own character output function, 
something like :

void putc ( void* p, char c)
	{
	while (!SERIAL_PORT_EMPTY) ;
	SERIAL_PORT_TX_REGISTER = c;
	}

init_printf(NULL,putc);

regs Kusti, 23.10.2004
*/


#ifndef MYPRINTF_H
#define MYPRINTF_H

#include <stdarg.h>

#define PRINTF_LONG_SUPPORT

void Myprintf_Init(void* putp,void (*putf) (void*,char));
void my_printf(char *fmt, ...);
void my_sprintf(char* s,char *fmt, ...);
void tfp_format(void* putp,void (*putf) (void*,char),char *fmt, va_list va);
void myputc ( void* p, char c);
#define printf tfp_printf 
#define sprintf tfp_sprintf 

#endif

