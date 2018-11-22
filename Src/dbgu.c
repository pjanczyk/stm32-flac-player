/*
* The MIT License (MIT)
* Copyright (c) 2017 Robert Brzoza-Woch
* Permission is hereby granted, free of charge, to any person obtaining 
* a copy of this software and associated documentation files (the "Software"), 
* to deal in the Software without restriction, including without limitation
* the rights to use, copy, modify, merge, publish, distribute, sublicense,
* and/or sell copies of the Software, and to permit persons to whom the
* Software is furnished to do so, subject to the following conditions:
* 
* The above copyright notice and this permission notice shall be
* included in all copies or substantial portions of the Software.
* 
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES
* OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.
* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
* DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR
* OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR
* THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

#include "dbgu.h"
#include "term_io.h"
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "stm32f7xx_hal_uart.h"

#define USE_HAL		0

UART_HandleTypeDef* pUart = NULL;

void debug_shdn(uint32_t shdn_on)
{
	if(shdn_on)
	{
	}
	else
	{
	}
	
	
}

void debug_init(UART_HandleTypeDef* handler)
{
	pUart = handler;
}

//send chr via UART (platform dependent)
void debug_chr(char chr)
{
	#if USE_HAL
	HAL_UART_Transmit(pUart,
	(uint8_t*)&chr, 1, 1000);
	#else
	while(__HAL_UART_GET_FLAG(pUart, UART_FLAG_TXE) == RESET) { ; }
	pUart->Instance->TDR = (uint16_t)chr;
	#endif
}


//returns ascii value of last char received
//returns 0 if no char was received since last debug_inkey call
//(platform dependent)
char debug_inkey(void)
{
	uint32_t flags = pUart->Instance->ISR;
	
	if((flags & UART_FLAG_RXNE) || (flags & UART_FLAG_ORE))
	{
		__HAL_UART_CLEAR_OREFLAG(pUart);
		return (pUart->Instance->RDR);
	}
	else
		return 0;
}

//halts program/task execution until char is received
//(platform dependent)
char debug_waitkey(void)
{
	uint8_t rxed;
	HAL_StatusTypeDef res;
	
	do
	{
		res = HAL_UART_Receive(pUart,&rxed,1,HAL_MAX_DELAY);
	}
	while(res != HAL_OK);
	return rxed;
}


//platform independent funcs


//prints text starting at str
//adds new line at end
void debug_msg(const char *str)
{
	debug_txt(str);
	debug_chr('\r');
	debug_chr('\n');
}

//prints text starting at str
void debug_txt(const char *str)
{
	while(*str) debug_chr(*str++);
}

//prints text starting at str
//prints exactly len chars
void debug_txt_limit(const char *str, uint8_t len)
{
	while(len)
	{
		debug_ascii(*str);
		str++;
		len--;
	}
}


//sends char b over pDbgu UART. Replaces values that can change cursor pos. on terminal
void debug_ascii(uint8_t b)
{
	switch(b)
	{
		case 0:
		{
			debug_chr('.');	//replace 0 with dot
			break;
		}
		case 8:
		case 9:
		case 10:
		case 13:
		{
			//avoid other chars that modify terminal cursor
			//replace them with space
			debug_chr(' ');
			break;
		}
		default:
		{
			debug_chr(b);
		}
	}//switch(chr)
}





void debug_dump(void *address, uint16_t len)
{
	uint8_t *buf = address;
	const uint16_t bytesInLine = 16;
	const uint16_t spaceBetweenDumpAndASCII = 4;
	uint16_t i, counter=len;
	
	xprintf("Debug dump @ %08X\n",(unsigned int)address);
	
	while(1)
	{
		//insert last line (may be shorter than full line)
		if(counter < bytesInLine)
		{
			xprintf("%08X %04X: ",(unsigned int)buf,(unsigned int)(len-counter));
			
			//contents in hex
			for(i=0;i<bytesInLine;i++)
			{
				if(i<counter)
				{
					xprintf("%02X ",(unsigned int)(buf[i]));
				}
				else
				{
					xprintf("   ");
				}
				if(i%8==7) xprintf(" ");
			}
			
			//space
			for(i=0;i<spaceBetweenDumpAndASCII;i++)
			{
				xprintf(" ");
			}
			
			//contents in ASCII
			for(i=0;i<bytesInLine;i++)
			{
				if(i<counter)
				{
					debug_ascii(buf[i]);
				}
				else
				{
					debug_chr(' ');
				}
			}
			
			debug_chr('\n');
			
			break;
		}
		
		xprintf("%08X %04X:   ",(unsigned int)buf,(unsigned int)(len-counter));
		
		
		for(i=0;i<bytesInLine;i++)
		{
			xprintf("%02X ",(unsigned int)(buf[i]));
			if(i%8==7) debug_chr(' ');
		}
		
		//space
		for(i=0;i<spaceBetweenDumpAndASCII;i++)
		{
			debug_chr(' ');
		}
		
		//contents in ASCII
		for(i=0;i<bytesInLine;i++)
		{
			debug_ascii(buf[i]);
		}
		
		buf += bytesInLine;
		if(counter >= bytesInLine)
		{
			counter -= bytesInLine;
		}
		
		debug_chr('\n');
			
		if(counter == 0) break;
		
	}	//while(counter)
	//footer
	xprintf("End of dump");
	
}

