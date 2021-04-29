/****************************************************************************
 * arch/lambdasoc/src/common/hw/timer.h
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

#ifndef __ARCH_LAMBDASOC_SRC_COMMON_HW_TIMER_H
#define __ARCH_LAMBDASOC_SRC_COMMON_HW_TIMER_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define TIMER_RELOAD_OFFSET     0x00
#define TIMER_EN_OFFSET         0x04
#define TIMER_CTR_OFFSET        0x08

#define TIMER_EV_STATUS_OFFSET  0x10
#define TIMER_EV_PENDING_OFFSET 0x1c
#define TIMER_EV_ENABLE_OFFSET  0x20

#ifdef CONFIG_LAMBDASOC_TIMER
#  define LAMBDASOC_TIMER_BASE 0x6000
#  define LAMBDASOC_TIMER_IRQNO 0
#  define LAMBDASOC_TIMER_RELOAD     (LAMBDASOC_TIMER_BASE + TIMER_RELOAD_OFFSET)
#  define LAMBDASOC_TIMER_EN         (LAMBDASOC_TIMER_BASE + TIMER_EN_OFFSET)
#  define LAMBDASOC_TIMER_CTR        (LAMBDASOC_TIMER_BASE + TIMER_CTR_OFFSET)
#  define LAMBDASOC_TIMER_EV_STATUS  (LAMBDASOC_TIMER_BASE + TIMER_EV_STATUS_OFFSET)
#  define LAMBDASOC_TIMER_EV_PENDING (LAMBDASOC_TIMER_BASE + TIMER_EV_PENDING_OFFSET)
#  define LAMBDASOC_TIMER_EV_ENABLE  (LAMBDASOC_TIMER_BASE + TIMER_EV_ENABLE_OFFSET)
#endif

#define TIMER_EV_ZERO 0x1

#endif /* __ARCH_LAMBDASOC_SRC_COMMON_HW_TIMER_H */
