/****************************************************************************
 * arch/risc-v/include/sg2002/irq.h
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

#ifndef __ARCH_RISCV_INCLUDE_SG2002_IRQ_H
#define __ARCH_RISCV_INCLUDE_SG2002_IRQ_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Map RISC-V exception code to NuttX IRQ */

#define SG2002_IRQ_TEMPSENS			(RISCV_IRQ_MEXT + 16)
#define SG2002_IRQ_RTC_ALARM			(RISCV_IRQ_MEXT + 17)
#define SG2002_IRQ_RTC_LONGPRESS		(RISCV_IRQ_MEXT + 18)
#define SG2002_IRQ_VBAT_DET			(RISCV_IRQ_MEXT + 19)
#define SG2002_IRQ_JPEG				(RISCV_IRQ_MEXT + 20)
#define SG2002_IRQ_H264				(RISCV_IRQ_MEXT + 21)
#define SG2002_IRQ_H265				(RISCV_IRQ_MEXT + 22)
#define SG2002_IRQ_VC_SBM			(RISCV_IRQ_MEXT + 23)
#define SG2002_IRQ_ISP				(RISCV_IRQ_MEXT + 24)
#define SG2002_IRQ_SC_TOP			(RISCV_IRQ_MEXT + 25)
#define SG2002_IRQ_CSI_MAC0			(RISCV_IRQ_MEXT + 26)
#define SG2002_IRQ_CSI_MAC1			(RISCV_IRQ_MEXT + 27)
#define SG2002_IRQ_LDC				(RISCV_IRQ_MEXT + 28)
#define SG2002_IRQ_SYSTEM_DMA			(RISCV_IRQ_MEXT + 29)
#define SG2002_IRQ_USB				(RISCV_IRQ_MEXT + 30)
#define SG2002_IRQ_ETHNET0			(RISCV_IRQ_MEXT + 31)
#define SG2002_IRQ_ETHNET0_1			(RISCV_IRQ_MEXT + 32)
#define SG2002_IRQ_EMMC_WAKUP			(RISCV_IRQ_MEXT + 33)
#define SG2002_IRQ_EMMC				(RISCV_IRQ_MEXT + 34)
#define SG2002_IRQ_SD0_WAKUP			(RISCV_IRQ_MEXT + 35)
#define SG2002_IRQ_SD0				(RISCV_IRQ_MEXT + 36)
#define SG2002_IRQ_SD1_WAKUP			(RISCV_IRQ_MEXT + 37)
#define SG2002_IRQ_SD1				(RISCV_IRQ_MEXT + 38)
#define SG2002_IRQ_SPI_NAND			(RISCV_IRQ_MEXT + 39)
#define SG2002_IRQ_I2S0				(RISCV_IRQ_MEXT + 40)
#define SG2002_IRQ_I2S1				(RISCV_IRQ_MEXT + 41)
#define SG2002_IRQ_I2S2				(RISCV_IRQ_MEXT + 42)
#define SG2002_IRQ_I2S3				(RISCV_IRQ_MEXT + 43)
#define SG2002_IRQ_UART0			(RISCV_IRQ_MEXT + 44)
#define SG2002_IRQ_UART1			(RISCV_IRQ_MEXT + 45)
#define SG2002_IRQ_UART2			(RISCV_IRQ_MEXT + 46)
#define SG2002_IRQ_UART3			(RISCV_IRQ_MEXT + 47)
#define SG2002_IRQ_UART4			(RISCV_IRQ_MEXT + 48)
#define SG2002_IRQ_I2C0				(RISCV_IRQ_MEXT + 49)
#define SG2002_IRQ_I2C1				(RISCV_IRQ_MEXT + 50)
#define SG2002_IRQ_I2C2				(RISCV_IRQ_MEXT + 51)
#define SG2002_IRQ_I2C3				(RISCV_IRQ_MEXT + 52)
#define SG2002_IRQ_I2C4				(RISCV_IRQ_MEXT + 53)
#define SG2002_IRQ_SPI1				(RISCV_IRQ_MEXT + 54)
#define SG2002_IRQ_SPI2				(RISCV_IRQ_MEXT + 55)
#define SG2002_IRQ_SPI3				(RISCV_IRQ_MEXT + 56)
#define SG2002_IRQ_SPI4				(RISCV_IRQ_MEXT + 57)
#define SG2002_IRQ_WATCHDOG1			(RISCV_IRQ_MEXT + 58)
#define SG2002_IRQ_KEYSCAN			(RISCV_IRQ_MEXT + 59)
#define SG2002_IRQ_GPIO0			(RISCV_IRQ_MEXT + 60)
#define SG2002_IRQ_GPIO1			(RISCV_IRQ_MEXT + 61)
#define SG2002_IRQ_GPIO2			(RISCV_IRQ_MEXT + 62)
#define SG2002_IRQ_GPIO3			(RISCV_IRQ_MEXT + 63)
#define SG2002_IRQ_WIEGAND0			(RISCV_IRQ_MEXT + 64)
#define SG2002_IRQ_WIEGAND1			(RISCV_IRQ_MEXT + 65)
#define SG2002_IRQ_WIEGAND2			(RISCV_IRQ_MEXT + 66)
#define SG2002_IRQ_RTC_MBOX			(RISCV_IRQ_MEXT + 67)
//#define SG2002_IRQ_N/A			(RISCV_IRQ_MEXT + 68)
#define SG2002_IRQ_RTC_IRRX			(RISCV_IRQ_MEXT + 69)
#define SG2002_IRQ_RTC_GPIO			(RISCV_IRQ_MEXT + 70)
#define SG2002_IRQ_RTC_UART			(RISCV_IRQ_MEXT + 71)
#define SG2002_IRQ_RTC_SPI_NOR			(RISCV_IRQ_MEXT + 72)
#define SG2002_IRQ_RTC_I2C			(RISCV_IRQ_MEXT + 73)
#define SG2002_IRQ_RTC_WDG			(RISCV_IRQ_MEXT + 74)
#define SG2002_IRQ_TPU				(RISCV_IRQ_MEXT + 75)
#define SG2002_IRQ_TDMA				(RISCV_IRQ_MEXT + 76)
#define SG2002_IRQ_RESERVED			(RISCV_IRQ_MEXT + 77)
#define SG2002_IRQ_RESERVED			(RISCV_IRQ_MEXT + 78)
#define SG2002_IRQ_TIMER0			(RISCV_IRQ_MEXT + 79)
#define SG2002_IRQ_TIMER1			(RISCV_IRQ_MEXT + 80)
#define SG2002_IRQ_TIMER2			(RISCV_IRQ_MEXT + 81)
#define SG2002_IRQ_TIMER3			(RISCV_IRQ_MEXT + 82)
#define SG2002_IRQ_TIMER4			(RISCV_IRQ_MEXT + 83)
#define SG2002_IRQ_TIMER5			(RISCV_IRQ_MEXT + 84)
#define SG2002_IRQ_TIMER6			(RISCV_IRQ_MEXT + 85)
#define SG2002_IRQ_TIMER7			(RISCV_IRQ_MEXT + 86)
#define SG2002_IRQ_PERI_FIREWALL		(RISCV_IRQ_MEXT + 87)
#define SG2002_IRQ_HSPERI_FIREWALL		(RISCV_IRQ_MEXT + 88)
#define SG2002_IRQ_DDR_FW			(RISCV_IRQ_MEXT + 89)
#define SG2002_IRQ_ROM_FIREWALL			(RISCV_IRQ_MEXT + 90)
#define SG2002_IRQ_SPACC			(RISCV_IRQ_MEXT + 91)
#define SG2002_IRQ_TRNG				(RISCV_IRQ_MEXT + 92)
#define SG2002_IRQ_DDR_AXI_MON			(RISCV_IRQ_MEXT + 93)
#define SG2002_IRQ_DDR_PI_PHY			(RISCV_IRQ_MEXT + 94)
#define SG2002_IRQ_SPI_NOR			(RISCV_IRQ_MEXT + 95)
#define SG2002_IRQ_EPHY				(RISCV_IRQ_MEXT + 96)
#define SG2002_IRQ_IVE				(RISCV_IRQ_MEXT + 97)
//#define SG2002_IRQ_RESERVED			(RISCV_IRQ_MEXT + 98)
//#define SG2002_IRQ_RESERVED			(RISCV_IRQ_MEXT + 99)
#define SG2002_IRQ_SARADC			(RISCV_IRQ_MEXT + 100)
#define SG2002_IRQ_MBOX				(RISCV_IRQ_MEXT + 101)

/* Total number of IRQs */

#define NR_IRQS					(SG2002_IRQ_MBOX + 1)

#endif /* __ARCH_RISCV_INCLUDE_SG2002_IRQ_H */
