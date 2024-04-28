/****************************************************************************
 * arch/risc-v/src/sg2002/sg2002_irq_dispatch.c
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
#include <assert.h>

#include <nuttx/irq.h>
#include <nuttx/arch.h>

#include "riscv_internal.h"
#include "group/group.h"

#include "sg2002_memorymap.h"

#include <debug.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/****************************************************************************
 * Public Functions
 ****************************************************************************/

/****************************************************************************
 * riscv_dispatch_irq
 ****************************************************************************/

void *riscv_dispatch_irq(uintptr_t vector, uintptr_t *regs)
{
  int32_t irq = vector & 0x1f;

  /* If current is interrupt */

  if ((vector & RISCV_IRQ_BIT) != 0)
    {

      irq += RISCV_IRQ_ASYNC;

      switch(irq)
        {
          case RISCV_IRQ_MTIMER:
            {
              /* Deliver the IRQ */

              regs = riscv_doirq(irq, regs);

              putreg32(irq - RISCV_IRQ_ASYNC, SG2002_PLIC_MCLAIM);
              break;
	    }

          case RISCV_IRQ_MEXT:
            {
              uint32_t val = getreg32(SG2002_PLIC_MCLAIM);

              /* Add the value to nuttx irq which is offset to the mext */

              irq += val;

              /* Deliver the IRQ */

              regs = riscv_doirq(irq, regs);

	      /* From MCAUSE, vectro contains the MEXT(11), then plus the IRQ_ASYNC.
	       *
	       * After that, get value from MCLAIN. 
	       *
	       * To clean MCLAIM should minues RISCV_IRQ_MEXT */
	       
              putreg32(irq - RISCV_IRQ_MEXT, SG2002_PLIC_MCLAIM);
              break;
            }
           default:
             break;
         }
    }
  else
    {
      /* Deliver the IRQ */

      regs = riscv_doirq(irq, regs);
    }

  return regs;
}

