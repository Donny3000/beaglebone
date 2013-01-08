/*
 * Copyright (C) 2007 Philippe Gerum <rpm@xenomai.org>.
 *
 * Xenomai is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published
 * by the Free Software Foundation; either version 2 of the License,
 * or (at your option) any later version.
 *
 * Xenomai is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Xenomai; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA
 * 02111-1307, USA.
 */

#ifndef _XENO_ASM_X86_WRAPPERS_64_H
#define _XENO_ASM_X86_WRAPPERS_64_H
#define _XENO_ASM_X86_WRAPPERS_H

#ifndef __KERNEL__
#error "Pure kernel header included from user-space!"
#endif

#include <asm-generic/xenomai/wrappers.h> /* Read the generic portion. */
#include <linux/interrupt.h>

#define wrap_strncpy_from_user(dstP, srcP, n)	rthal_strncpy_from_user(dstP, srcP, n)

#define wrap_phys_mem_prot(filp,pfn,size,prot)  (prot)

#define rthal_irq_desc_status(irq)	(rthal_irq_descp(irq)->status)

#if !defined(CONFIG_GENERIC_HARDIRQS) \
	|| LINUX_VERSION_CODE < KERNEL_VERSION(2,6,37)
#define rthal_irq_chip_enable(irq)   ({ rthal_irq_descp(irq)->chip->unmask(irq); 0; })
#define rthal_irq_chip_disable(irq)  ({ rthal_irq_descp(irq)->chip->mask(irq); 0; })
#endif
#define rthal_irq_chip_end(irq)      ({ rthal_irq_descp(irq)->ipipe_end(irq, rthal_irq_descp(irq)); 0; })

typedef irq_handler_t rthal_irq_host_handler_t;

#define DECLARE_LINUX_IRQ_HANDLER(fn, irq, dev_id)	\
	irqreturn_t fn(int irq, void *dev_id)

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,25)
#define x86reg_origax	orig_ax
#define x86reg_ax	ax
#define x86reg_bx	bx
#define x86reg_cx	cx
#define x86reg_dx	dx
#define x86reg_si	si
#define x86reg_di	di
#define x86reg_sp	sp
#define x86reg_bp	bp
#define x86reg_ip	ip
#else
#define x86reg_origax	orig_rax
#define x86reg_ax	rax
#define x86reg_bx	rbx
#define x86reg_cx	rcx
#define x86reg_dx	rdx
#define x86reg_si	rsi
#define x86reg_di	rdi
#define x86reg_sp	rsp
#define x86reg_bp	rbp
#define x86reg_ip	rip
#endif

#if LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,25)
typedef union i387_union x86_fpustate;
#define x86_fpustate_ptr(t) (&(t)->i387)
#elif LINUX_VERSION_CODE <= KERNEL_VERSION(2,6,34)
typedef union thread_xstate x86_fpustate;
#define x86_fpustate_ptr(t) ((t)->xstate)
#else /* 2.6.35 and above */
typedef union thread_xstate x86_fpustate;
#define x86_fpustate_ptr(t) ((t)->fpu.state)
#endif

#ifdef TS_USEDFPU
#define wrap_test_fpu_used(task)  \
   (task_thread_info(task)->status & TS_USEDFPU)
#define wrap_set_fpu_used(task)   \
do { \
   task_thread_info(task)->status |= TS_USEDFPU; \
} while(0)
#define wrap_clear_fpu_used(task) \
do { \
   task_thread_info(task)->status &= ~TS_USEDFPU; \
} while(0)
#else /* !defined(TS_USEDFPU) */
#define wrap_test_fpu_used(task) ((task)->thread.has_fpu)
#define wrap_set_fpu_used(task)			\
	do {					\
		(task)->thread.has_fpu = 1;	\
	} while(0)
#define wrap_clear_fpu_used(task)		\
	do {					\
		(task)->thread.has_fpu = 0;	\
	} while(0)
#endif /* !defined(TS_USEDFPU) */

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,34)
#define per_cpu_var(var) (var)
#endif /* Linux >= 2.6.34 */

#endif /* _XENO_ASM_X86_WRAPPERS_64_H */
