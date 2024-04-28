/****************************************************************************
 * arch/risc-v/src/sg2002/hardware/sg2002_plic.h
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

#ifndef __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_PLIC_H
#define __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_PLIC_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define SG2002_PLIC_PRIORITY    (SG2002_PLIC_BASE + 0x00000000)
#define SG2002_PLIC_IP0         (SG2002_PLIC_BASE + 0x00001000)
#define SG2002_PLIC_IP1         (SG2002_PLIC_BASE + 0x00001004)
#define SG2002_PLIC_MIE0        (SG2002_PLIC_BASE + 0x00002000)
#define SG2002_PLIC_MIE1        (SG2002_PLIC_BASE + 0x00002004)
#define SG2002_PLIC_SIE0        (SG2002_PLIC_BASE + 0x00002080)
#define SG2002_PLIC_SIE1        (SG2002_PLIC_BASE + 0x00002084)
#define SG2002_PLIC_CTRL        (SG2002_PLIC_BASE + 0x001FFFFC)
#define SG2002_PLIC_MTHRESHOLD  (SG2002_PLIC_BASE + 0x00200000)
#define SG2002_PLIC_MCLAIM      (SG2002_PLIC_BASE + 0x00200004)
#define SG2002_PLIC_STHRESHOLD  (SG2002_PLIC_BASE + 0x00201000)
#define SG2002_PLIC_SCLAIM      (SG2002_PLIC_BASE + 0x00201004)

#endif /* __ARCH_RISCV_SRC_SG2002_HARDWARE_SG2002_PLIC_H */
