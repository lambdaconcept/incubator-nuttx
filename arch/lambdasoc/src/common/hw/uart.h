/****************************************************************************
 * arch/lambdasoc/src/common/hw/uart.h
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

#ifndef __ARCH_LAMBDASOC_SRC_COMMON_HW_UART_H
#define __ARCH_LAMBDASOC_SRC_COMMON_HW_UART_H

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define UART_DIVISOR_OFFSET     0x00
#define UART_RX_DATA_OFFSET     0x04
#define UART_RX_RDY_OFFSET      0x08
#define UART_RX_ERR_OFFSET      0x0c
#define UART_TX_DATA_OFFSET     0x10
#define UART_TX_RDY_OFFSET      0x14

#define UART_EV_STATUS_OFFSET   0x20
#define UART_EV_PENDING_OFFSET  0x24
#define UART_EV_ENABLE_OFFSET   0x28

#ifdef CONFIG_LAMBDASOC_UART1
#  define LAMBDASOC_UART1_BASE 0x5000
#  define LAMBDASOC_UART1_IRQNO 1
#  define LAMBDASOC_UART1_RXBUFSIZE 16
#  define LAMBDASOC_UART1_TXBUFSIZE 16
#  define LAMBDASOC_UART1_DIVISOR    (LAMBDASOC_UART1_BASE + UART_DIVISOR_OFFSET)
#  define LAMBDASOC_UART1_RX_DATA    (LAMBDASOC_UART1_BASE + UART_RX_DATA_OFFSET)
#  define LAMBDASOC_UART1_RX_RDY     (LAMBDASOC_UART1_BASE + UART_RX_RDY_OFFSET)
#  define LAMBDASOC_UART1_RX_ERR     (LAMBDASOC_UART1_BASE + UART_RX_ERR_OFFSET)
#  define LAMBDASOC_UART1_TX_DATA    (LAMBDASOC_UART1_BASE + UART_TX_DATA_OFFSET)
#  define LAMBDASOC_UART1_TX_RDY     (LAMBDASOC_UART1_BASE + UART_TX_RDY_OFFSET)
#  define LAMBDASOC_UART1_EV_STATUS  (LAMBDASOC_UART1_BASE + UART_EV_STATUS_OFFSET)
#  define LAMBDASOC_UART1_EV_PENDING (LAMBDASOC_UART1_BASE + UART_EV_PENDING_OFFSET)
#  define LAMBDASOC_UART1_EV_ENABLE  (LAMBDASOC_UART1_BASE + UART_EV_ENABLE_OFFSET)
#endif

#define UART_EV_RX_RDY 0x1
#define UART_EV_RX_ERR 0x2
#define UART_EV_TX_MTY 0x4

#endif /* __ARCH_LAMBDASOC_SRC_COMMON_HW_UART_H */
