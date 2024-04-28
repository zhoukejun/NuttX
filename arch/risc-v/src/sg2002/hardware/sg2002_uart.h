/****************************************************************************
 * arch/risc-v/src/sg2002/hardware/sg2002_uart.h
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

#ifndef ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_UART_H
#define ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_UART_H


/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

//Name				Address Offset	Description
#define UART_RBR_THR_DLL		0x000 // Receive Buffer,Transmit Holding or Divisor Latch Low byte Register
#define UART_IER_DLH			0x004 // Interrupt Enable or Divisor Latch high byte Register
#define UART_FCR_IIR			0x008 // FIFO Control or Interrupt Identification Register
#define UART_LCR			0x00c // Line Control Register
#define UART_MCR			0x010 // Modem Control Register
#define UART_LSR			0x014 // Line Status Register
#define UART_MSR			0x018 // Modem Status Register
#define UART_LPDLL			0x020 // Low Power Divisor Latch (Low) Register
#define UART_LPDLH			0x024 // Low Power Divisor Latch (High) Register
#define UART_SRBR_STHR			0x030 // Shadow Receive/Trasnmit Buffer Register
#define UART_FAR			0x070 // FIFO Access Register
#define UART_TFR			0x074 // Transmit FIFO Read
#define UART_RFW			0x078 // Receive FIFO Write
#define UART_USR			0x07c // UART Status Register
#define UART_TFL			0x080 // Transmit FIFO Level
#define UART_RFL			0x084 // Receive FIFO Level
#define UART_SRR			0x088 // Software Reset Register
#define UART_SRTS			0x08c // Shadow Request to Send
#define UART_SBCR			0x090 // Shadow Break Control Register
#define UART_SDMAM			0x094 // Shadow DMA Mode
#define UART_SFE			0x098 // Shadow FIFO Enable
#define UART_SRT			0x09c // Shadow RCVR Trigger
#define UART_STET			0x0a0 // Shadow TX Empty Trigger
#define UART_HTX			0x0a4 // Halt TX
#define UART_DMASA			0x0a9 // DMA Software Acknowledge


//IER_DLH
//LCR[7] bit = 0
#define IER_DLH_RECV_DATA		(0x1 << 0)
#define IER_DLH_TRANS_HOLDING_REG_EMPTY	(0x1 << 1)
#define IER_DLH_RECV_LINE_STA		(0x1 << 2)
#define IER_DLH_MODEM_STA		(0x1 << 3)
#define IER_DLH_PROG_THRE		(0x1 << 7)


//FCR_IIR
#define FCR_IIR_MODEM_STA		0b0000
#define FCR_IIR_NO_INT_PENDING		0b0001
#define FCR_IIR_THR_EMPTY		0b0010
#define	FCR_IIR_RECV_DATA		0b0100
#define FCR_IIR_RECV_LINE_STA		0b0110
#define FCR_IIR_BUSY_DETECT		0b0111
#define FCR_IIR_CHAR_TIMEOUT		0b1100



//LCR
#define LCR_DLS_MASK			(0x3)
#define LCR_DLS_8BITS			(0x3)
#define LCR_NUM_OF_STOP_MASK		(0x1 << 2)
#define LCR_STOP_1BIT			(0x0)
#define LCR_STOP_BIT_SHIFT		(2)
#define LCR_PARITY_ENABLE_MASK		(0x1 << 3)
#define LCR_EVEN_PARITY_SEL		(0x1 << 4)
#define LCR_STICK_PARITY		(0x1 << 5)
#define LCR_BREAK_CTRL_BIT		(0x1 << 6)
#define LCR_DLAB			(0x1 << 7)

//LSR
#define LSR_DATA_READY			(0x1 << 0)
#define LSR_TRANS_HOLDING_REG_EMPTY	(0x1 << 5)

//USR
#define USR_UART_BUSY			(0x1 << 0)
#define USR_TRANS_FIFO_NOT_FULL		(0x1 << 1)
#define USR_TRANS_FIFO_EMPTY		(0x1 << 2)
#define USR_RECV_FIFO_NOT_FULL		(0x1 << 3)
#define USR_RECV_FIFO_EMPTY		(0x1 << 4)

//-----------
#define UART_FCR_FIFO_EN		0x01 /* Fifo enable */
#define UART_FCR_RXSR			0x02 /* Receiver soft reset */
#define UART_FCR_TXSR			0x04 /* Transmitter soft reset */

/* Clear & enable FIFOs */
#define UART_FCR_DEFVAL			(UART_FCR_FIFO_EN | \
						UART_FCR_RXSR | \
						UART_FCR_TXSR)

//---------------
/* useful defaults for LCR */
#define UART_LCR_8N1			0x03
#define UART_LCR_BKSE			0x80 /* Bank select enable */
#define UART_LCR_DLAB			0x80 /* Divisor latch access bit */

#define UART_LCRVAL			UART_LCR_8N1 /* 8 data, 1 stop, no parity */


//=====================================
//UART0
#define UART0_THR			(SG2002_UART0_BASE + UART_RBR_THR_DLL)
#define UART0_IER			(SG2002_UART0_BASE + UART_IER_DLH)
#define UART0_DLH			(SG2002_UART0_BASE + UART_IER_DLH)
#define UART0_FCR			(SG2002_UART0_BASE + UART_FCR_IIR)
#define UART0_LCR			(SG2002_UART0_BASE + UART_LCR)
#define UART0_MCR			(SG2002_UART0_BASE + UART_MCR)
#define UART0_LSR			(SG2002_UART0_BASE + UART_LSR)
#define UART0_SRR			(SG2002_UART0_BASE + UART_SRR)

#endif /* ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_UART_H */

