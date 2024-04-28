/****************************************************************************
 * arch/risc-v/src/sg2002/sg2002_memorymap.h
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

#ifndef __ARCH_RISCV_SRC_SG2002_SG2002_MEMORYMAP_H
#define __ARCH_RISCV_SRC_SG2002_SG2002_MEMORYMAP_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include "riscv_common_memorymap.h"
#include "hardware/sg2002_memorymap.h"
#include "hardware/sg2002_clint.h"
#include "hardware/sg2002_plic.h"
#include "hardware/sg2002_sysctl.h"

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Idle thread stack starts from _ebss */

#ifndef __ASSEMBLY__
#define SG2002_IDLESTACK_BASE  (uintptr_t)_ebss
#else
#define SG2002_IDLESTACK_BASE  _ebss
#endif

#define SG2002_IDLESTACK0_TOP  (SG2002_IDLESTACK_BASE + CONFIG_IDLETHREAD_STACKSIZE)
#define SG2002_IDLESTACK_TOP   (SG2002_IDLESTACK0_TOP)

#endif /* __ARCH_RISCV_SRC_SG2002_SG2002_MEMORYMAP_H */
