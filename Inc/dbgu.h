/*
* The MIT License (MIT)
* Copyright (c) 2016 Robert Brzoza-Woch
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
#ifndef __DBGU_H__
#define __DBGU_H__

#include <inttypes.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_uart.h"
#include "ansi.h"

void debug_init(UART_HandleTypeDef* handler);
void debug_chr(char chr);
int debug_test(void);
char debug_inkey(void);
char debug_waitkey(void);
void debug_ascii(uint8_t b);
void debug_msg(const char *str);
void debug_txt(const char *str);
void debug_txt_limit(const char *str, uint8_t len);
void debug_dump(void *address, uint16_t len);
void debug_shdn(uint32_t shdn_on);

#endif


