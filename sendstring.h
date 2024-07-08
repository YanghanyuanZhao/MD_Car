#ifndef _SENDSTRING_H_
#define _SENDSTRING_H_

#include "stm32f1xx.h"

void USART1_TX_Byte(unsigned char data)
{
	USART1->TDR = data;
	while((USART1->ISR & 0x40) == 0);
}

void USART1_TX_String(unsigned char *str)
{
	while(*str!='\0')
	{
		USART1_TX_Byte(*str++);
	}
}