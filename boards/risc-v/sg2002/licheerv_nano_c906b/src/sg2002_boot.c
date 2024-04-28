/****************************************************************************
 * boards/risc-v/sg200x/licheerv_nano_c906b/src/sg2002_boot.c
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

#include <debug.h>

#include <nuttx/board.h>
#include <arch/board/board.h>

#include "riscv_internal.h"

#include "hardware/sg2002_pinmux.h"
/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Private Functions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/
void sg2002_board_pinmux_setup(void)
{
  //user_led
  PINMUX_CONFIG(FMUX_REG_SD0_PWR_EN, SD0_PWR_EN__XGPIOA_14);

  //user_key
  PINMUX_CONFIG(FMUX_REG_AUX0, AUX0__XGPIOA_30);

  //UART0
  PINMUX_CONFIG(FMUX_REG_UART0_TX, UART0_TX__UART0_TX);
  PINMUX_CONFIG(FMUX_REG_UART0_RX, UART0_RX__UART0_RX);
}


/****************************************************************************
 * Name: sg2002_boardinitialize
 *
 * Description:
 *   All SG2002 architectures must provide the following entry point.
 *   This entry point is called early in the initialization -- after all
 *   memory has been configured and mapped but before any devices have been
 *   initialized.
 *
 ****************************************************************************/

void sg2002_boardinitialize(void)
{
  board_autoled_initialize();
}
