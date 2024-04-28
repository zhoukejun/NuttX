/****************************************************************************
 * arch/risc-v/src/sg2002/sg2002_lowputc.c
 *
 * Licensed to the Apache Software Foundation (ASF) under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.  The
 * ASF licenses this file to you under the Apache License, Version 2.0 (the
 * "License"); you may not use this file except in compliance with the
 * License.  You may obtain a copy of the License at
 *
 *   http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.  See the
 * License for the specific language governing permissions and limitations
 * under the License.
 *
 ****************************************************************************/

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>

#include <stdint.h>

#include <arch/board/board.h>

#include "riscv_internal.h"
#include "sg2002_config.h"

#include "hardware/sg2002_memorymap.h"
#include "hardware/sg2002_uart.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Select UART parameters for the selected console */

#ifdef HAVE_SERIAL_CONSOLE
#  if defined(CONFIG_UART0_SERIAL_CONSOLE)
#    define HAVE_UART
#  endif
#endif /* HAVE_CONSOLE */

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * Name: riscv_lowputc
 *
 * Description:
 *   Output one byte on the serial console
 *
 ****************************************************************************/

int sg2002_stub(void)
{
   int i;
   i = 100;
   return i;
}

void riscv_lowputc(char ch)
{
#ifdef HAVE_SERIAL_CONSOLE
  /* Wait until the TX data register is empty */

  while (!(getreg32(UART0_LSR) & LSR_TRANS_HOLDING_REG_EMPTY))
    {
    }

  /* Then send the character */

  putreg32(ch, UART0_THR);

#endif /* HAVE_CONSOLE */
}

/****************************************************************************
 * Name: sg2002_lowsetup
 *
 * Description:
 *   This performs basic initialization of the UART used for the serial
 *   console.  Its purpose is to get the console output available as soon
 *   as possible.
 *
 ****************************************************************************/

void sg2002_lowsetup(void)
{
  /* Software Reset */
  putreg32(0x7, UART0_SRR);

  /* Enable and configure the selected console device */
  putreg32(0, UART0_DLH);
  putreg32(0, UART0_MCR);
  putreg32(UART_FCR_DEFVAL, UART0_FCR);
  putreg32(UART_LCR_BKSE | UART_LCRVAL, UART0_LCR);

  /* Configure the UART 115200 Baud Rate */
  putreg32(14 & 0xFF, UART0_THR);
  putreg32(14 >> 8, UART0_DLH);

  /* 8n1 no parity */
  putreg32(UART_LCRVAL, UART0_LCR);


  //========
//  sg2002_print_str(">>>IER:\n");
//  sg2002_print_hex(getreg32(UART0_IER));

  putreg32(IER_DLH_RECV_DATA | IER_DLH_RECV_LINE_STA , UART0_IER);
//  sg2002_print_str("<<<IER:\n");
//  sg2002_print_hex(getreg32(UART0_IER));
  //====

  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('s');
  riscv_lowputc('g');
  riscv_lowputc('2');
  riscv_lowputc('0');
  riscv_lowputc('0');
  riscv_lowputc('2');
  riscv_lowputc('\r');
  riscv_lowputc('\n');

}

void sg2002_test1(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('1');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test2(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('2');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test3(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('3');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test4(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('4');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test5(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('5');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test6(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('6');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test7(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('t');
  riscv_lowputc('e');
  riscv_lowputc('s');
  riscv_lowputc('t');
  riscv_lowputc('7');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_test_irq(void)
{
  riscv_lowputc('\r');
  riscv_lowputc('\n');
  riscv_lowputc('I');
  riscv_lowputc('R');
  riscv_lowputc('Q');
  riscv_lowputc('\r');
  riscv_lowputc('\n');
}

void sg2002_print_hex(uintptr_t hex)
{
	int i;
	unsigned char tmp;
	riscv_lowputc('\r');
	riscv_lowputc('\n');
 	riscv_lowputc('[');
	riscv_lowputc('0');
	riscv_lowputc('x');
	for (i = sizeof(uintptr_t) * 2 - 1; i >= 0; i--) {
		tmp = (hex >> (i * 4)) & 0xF;
		if(tmp >= 0 && tmp <= 9) {
			riscv_lowputc('0' + tmp);
		} else if (tmp >= 0xa && tmp <= 0xf) {
			riscv_lowputc('a' + tmp - 0xa);
		} else {
			riscv_lowputc('e');
			riscv_lowputc('r');
			riscv_lowputc('r');
			riscv_lowputc('o');
			riscv_lowputc('r');
			riscv_lowputc(' ');
		}	
	}
 	riscv_lowputc(']');
	riscv_lowputc('\r');	
	riscv_lowputc('\n');	

}

void sg2002_print_str(char *str)
{
  int i;
  riscv_lowputc('{');
  for(i = 0; i < strlen(str); i++)
  {
    if(str[i] == '\n') {
      riscv_lowputc('}');
      riscv_lowputc('\r');
    }
    riscv_lowputc(str[i]);

  }
}


