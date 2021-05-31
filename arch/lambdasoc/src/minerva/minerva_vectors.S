/****************************************************************************
 * arch/lambdasoc/src/minerva/minerva_vectors.S
 *
 *   Copyright (C) 2019 Gregory Nutt. All rights reserved.
 *   Author: Ramtin Amin <keytwo@gmail.com>
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

/****************************************************************************
 * Included Files
 ****************************************************************************/

#include <nuttx/config.h>
#include <arch/irq.h>

/****************************************************************************
 * Pre-processor Definitions
 ****************************************************************************/

#define MINERVA_STACK_BASE    _ebss
#define MINERVA_STACK_TOP     _ebss+CONFIG_IDLETHREAD_STACKSIZE
#define MINERVA_HEAP_BASE     MINERVA_STACK_TOP

#define MIE_MEIE              0x800
#define CSR_IRQ_MASK          0x330

/****************************************************************************
 * Macros
 ****************************************************************************/

	.section	.text, "ax", @progbits
	.global		g_idle_topstack
	.global		__start

.macro SAVE_REGS
	addi sp, sp, -XCPTCONTEXT_SIZE

	sw  ra, REG_X1(sp)
	sw  gp, REG_X3(sp)
	sw  tp, REG_X4(sp)
	sw  t0, REG_X5(sp)
	sw  t1, REG_X6(sp)
	sw  t2, REG_X7(sp)
	sw  s0, REG_X8(sp)
	sw  s1, REG_X9(sp)
	sw  a0, REG_X10(sp)
	sw  a1, REG_X11(sp)
	sw  a2, REG_X12(sp)
	sw  a3, REG_X13(sp)
	sw  a4, REG_X14(sp)
	sw  a5, REG_X15(sp)
	sw  a6, REG_X16(sp)
	sw  a7, REG_X17(sp)
	sw  s2, REG_X18(sp)
	sw  s3, REG_X19(sp)
	sw  s4, REG_X20(sp)
	sw  s5, REG_X21(sp)
	sw  s6, REG_X22(sp)
	sw  s7, REG_X23(sp)
	sw  s8, REG_X24(sp)
	sw  s9, REG_X25(sp)
	sw s10, REG_X26(sp)
	sw s11, REG_X27(sp)
	sw  t3, REG_X28(sp)
	sw  t4, REG_X29(sp)
	sw  t5, REG_X30(sp)
	sw  t6, REG_X31(sp)

	addi t0, sp, XCPTCONTEXT_SIZE
	sw t0, REG_X2(sp) /* original SP */

	csrr t0, mstatus
	sw t0, REG_CSR_MSTATUS(sp)

	csrr t0, mepc
	sw t0, REG_CSR_MEPC(sp)

	csrr t0, mbadaddr
	sw t0, REG_CSR_MBADADDR(sp)

	csrr t0, mcause
	sw t0, REG_CSR_MCAUSE(sp)
 .endm

/* restore regs */

 .macro RESTORE_REGS
	lw  ra, REG_X1(a0)
	lw  sp, REG_X2(a0)
	lw  gp, REG_X3(a0)
	lw  tp, REG_X4(a0)
	lw  t1, REG_X6(a0)
	lw  t2, REG_X7(a0)
	lw  s0, REG_X8(a0)
	lw  s1, REG_X9(a0)
	lw  a1, REG_X11(a0)
	lw  a2, REG_X12(a0)
	lw  a3, REG_X13(a0)
	lw  a4, REG_X14(a0)
	lw  a5, REG_X15(a0)
	lw  a6, REG_X16(a0)
	lw  a7, REG_X17(a0)
	lw  s2, REG_X18(a0)
	lw  s3, REG_X19(a0)
	lw  s4, REG_X20(a0)
	lw  s5, REG_X21(a0)
	lw  s6, REG_X22(a0)
	lw  s7, REG_X23(a0)
	lw  s8, REG_X24(a0)
	lw  s9, REG_X25(a0)
	lw s10, REG_X26(a0)
	lw s11, REG_X27(a0)
	lw  t3, REG_X28(a0)
	lw  t4, REG_X29(a0)
	lw  t5, REG_X30(a0)
	lw  t6, REG_X31(a0)

	lw t0, REG_CSR_MEPC(a0)
	csrw mepc, t0

	lw t0, REG_CSR_MSTATUS(a0)
	csrw mstatus, t0

	lw t0, REG_X5(a0)
	lw a0, REG_X10(a0)
 .endm

/****************************************************************************
 * Reset Handler
 ****************************************************************************/

__start:
_reset_handler:
	/* Set stack pointer */

	la sp, _fstack
	la t0, trap_entry
	csrw mtvec, t0

	/* Initialize .bss */

	la t0, _sbss
	la t1, _ebss
1:	beq t0, t1, 2f
	sw zero, 0(t0)
	addi t0, t0, 4
	j 1b
2:

#if 0 /* REVISIT */
	/* Enable external interrupts */

	li t0, MIE_MEIE
	csrs mie, t0
#endif

	call nx_start
5:	j 5b

trap_entry:
	SAVE_REGS

	mv a0, sp
	call minerva_doexception

	RESTORE_REGS
	mret

	/* This global variable is unsigned long g_idle_topstack and is
	 * exported from here only because of its coupling to other
	 * uses of _ebss in this file
	 */

	.data
	.align  4
	.type   g_idle_topstack, object

g_idle_topstack:
	.long	MINERVA_STACK_TOP
	.size	g_idle_topstack, .-g_idle_topstack
	.end
