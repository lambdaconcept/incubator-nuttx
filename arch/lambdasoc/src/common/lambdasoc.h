/****************************************************************************
 * arch/lambdasoc/src/common/lambdasoc.h
 *
 *   Copyright (C) 2016 Gregory Nutt. All rights reserved.
 *   Author: Gregory Nutt <gnutt@nuttx.org>
 *           Ramtin Amin <keytwo@gmail.com>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name NuttX nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#ifndef __ARCH_LAMBDASOC_SRC_COMMON_LAMBDASOC_H
#define __ARCH_LAMBDASOC_SRC_COMMON_LAMBDASOC_H

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <stdint.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

/* Low-level register access */

#define getreg8(a)            (*(volatile uint8_t *)(a))
#define putreg8(v,a)          (*(volatile uint8_t *)(a) = (v))
#define getreg16(a)           (*(volatile uint16_t *)(a))
#define putreg16(v,a)         (*(volatile uint16_t *)(a) = (v))
#define getreg32(a)           (*(volatile uint32_t *)(a))
#define putreg32(v,a)         (*(volatile uint32_t *)(a) = (v))

/****************************************************************************
 * Public Functions Prototypes
 ****************************************************************************/

#ifndef __ASSEMBLY__

/****************************************************************************
 * Name: flush_cpu_dcache
 *
 * Description:
 *  flush cpu cache Data cache
 *
 ****************************************************************************/

void flush_cpu_dcache(void);

/****************************************************************************
 * Name: up_serialinit
 *
 * Description:
 *   Register serial console and serial ports.  This assumes that
 *   lambdasoc_earlyserialinit was called previously.
 *
 ****************************************************************************/

void lambdasoc_serial_initialize(void);

/****************************************************************************
 * Name: up_net_initialize
 *
 * Description:
 *   Register Network
 *
 ****************************************************************************/

int lambdasoc_net_initialize(int intf);

/****************************************************************************
 * Name: lambdasoc_puts
 *
 * Description:
 *   This is a low-level helper function used to support debug.
 *
 ****************************************************************************/

void lambdasoc_puts(const char *str);

/****************************************************************************
 * Name: lambdasoc_lowputc
 *
 * Description:
 *   Low-level, blocking character output the serial console.
 *
 ****************************************************************************/

void lambdasoc_lowputc(char ch);

/****************************************************************************
 * Name: lambdasoc_lowputs
 *
 * Description:
 *   This is a low-level helper function used to support debug.
 *
 ****************************************************************************/

void lambdasoc_lowputs(const char *str);

/****************************************************************************
 * Name: modifyreg[N]
 *
 * Description:
 *   Atomic modification of registers.
 *
 ****************************************************************************/

void modifyreg8(unsigned int addr, uint8_t clearbits, uint8_t setbits);
void modifyreg16(unsigned int addr, uint16_t clearbits, uint16_t setbits);
void modifyreg32(unsigned int addr, uint32_t clearbits, uint32_t setbits);

/****************************************************************************
 * Name: lambdasoc_flush_dcache
 *
 * Description:
 *   Flush the data cache of the cpu
 *
 ****************************************************************************/

void lambdasoc_flush_dcache(void);

/****************************************************************************
 * Name: lambdasoc_flush_icache
 *
 * Description:
 *   Flush the instruction cache of the cpu
 *
 ****************************************************************************/

void lambdasoc_flush_icache(void);

#endif /* __ASSEMBLY__ */
#endif /* __ARCH_LAMBDASOC_SRC_COMMON_LAMBDASOC_H */
