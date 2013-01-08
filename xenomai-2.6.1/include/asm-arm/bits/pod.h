/*
 * Copyright (C) 2001,2002,2003,2004 Philippe Gerum <rpm@xenomai.org>.
 *
 * ARM port
 *   Copyright (C) 2005 Stelian Pop
 *
 * Xenomai is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
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

#ifndef _XENO_ASM_ARM_BITS_POD_H
#define _XENO_ASM_ARM_BITS_POD_H

#include <asm-generic/xenomai/bits/pod.h>

void xnpod_welcome_thread(struct xnthread *, int);

void xnpod_delete_thread(struct xnthread *);

#ifdef CONFIG_GENERIC_CLOCKEVENTS
#define xnarch_start_timer(tick_handler, cpu)	\
	rthal_timer_request(tick_handler, xnarch_switch_htick_mode, xnarch_next_htick_shot, cpu)
#else
#define xnarch_start_timer(tick_handler, cpu)	\
	rthal_timer_request(tick_handler, cpu)
#endif

#define xnarch_stop_timer(cpu)	rthal_timer_release(cpu)

static inline void xnarch_leave_root(xnarchtcb_t * rootcb)
{
	rthal_mute_pic();
	/* Remember the preempted Linux task pointer. */
	rootcb->user_task = rootcb->active_task = current;
	rootcb->mm = rootcb->active_mm = rthal_get_active_mm();
	rootcb->tip = current_thread_info();
#ifdef CONFIG_XENO_HW_FPU
#ifdef CONFIG_VFP
	rootcb->fpup = rthal_get_fpu_owner();
#else /* !CONFIG_VFP */
	rootcb->user_fpu_owner = rthal_get_fpu_owner(rootcb->user_task);
	/* So that xnarch_save_fpu() will operate on the right FPU area. */
	rootcb->fpup = (rootcb->user_fpu_owner
			? rthal_task_fpenv(rootcb->user_fpu_owner) : NULL);
#endif /* !CONFIG_VFP */
#endif /* CONFIG_XENO_HW_FPU */
}

static inline void xnarch_enter_root(xnarchtcb_t * rootcb)
{
#ifdef TIF_MMSWITCH_INT
	if (!rootcb->mm)
		set_ti_thread_flag(rootcb->tip, TIF_MMSWITCH_INT);
#endif /* TIF_MMSWITCH_INT */
	rthal_unmute_pic();
}

static inline void xnarch_switch_to(xnarchtcb_t *out_tcb, xnarchtcb_t *in_tcb)
{
	struct task_struct *prev = out_tcb->active_task;
	struct mm_struct *prev_mm = out_tcb->active_mm;
	struct task_struct *next = in_tcb->user_task;

	if (likely(next != NULL)) {
		in_tcb->active_task = next;
		in_tcb->active_mm = in_tcb->mm;
		rthal_clear_foreign_stack(&rthal_domain);
	} else {
		in_tcb->active_task = prev;
		in_tcb->active_mm = prev_mm;
		rthal_set_foreign_stack(&rthal_domain);
	}

	if (prev_mm != in_tcb->active_mm) {
		/* Switch to new user-space thread? */
		if (in_tcb->active_mm)
			switch_mm(prev_mm, in_tcb->active_mm, next);
		if (!next->mm)
			enter_lazy_tlb(prev_mm, next);
	}

	/* Kernel-to-kernel context switch. */
	rthal_thread_switch(prev, out_tcb->tip, in_tcb->tip);
}

asmlinkage static void xnarch_thread_trampoline(xnarchtcb_t * tcb)
{
	xnpod_welcome_thread(tcb->self, tcb->imask);
	tcb->entry(tcb->cookie);
	xnpod_delete_thread(tcb->self);
}

static inline void xnarch_init_thread(xnarchtcb_t * tcb,
				      void (*entry) (void *),
				      void *cookie,
				      int imask,
				      struct xnthread *thread, char *name)
{
	unsigned long flags;
	struct cpu_context_save *regs;

	rthal_local_irq_flags_hw(flags);

	regs = &tcb->ti.cpu_context;
	memset(regs, 0, sizeof(*regs));
	regs->pc = (unsigned long)&rthal_thread_trampoline;
	regs->r4 = (unsigned long)&xnarch_thread_trampoline;
	regs->r5 = (unsigned long)tcb;
	regs->sp = (unsigned long)tcb->stackbase + tcb->stacksize;

	tcb->entry = entry;
	tcb->cookie = cookie;
	tcb->self = thread;
	tcb->imask = imask;
	tcb->name = name;
}

/* No lazy FPU init on ARM. */
#define xnarch_fpu_init_p(task) (1)

static inline void xnarch_enable_fpu(xnarchtcb_t *tcb)
{
#ifdef CONFIG_XENO_HW_FPU
#ifdef CONFIG_VFP
	/* If we are restoring the Linux current thread which does not own the
	   FPU context, we keep FPU disabled, so that a fault will occur if the
	   newly switched thread uses the FPU, to allow the kernel handler to
	   pick the correct FPU context.
	*/
	if (likely(!tcb->is_root)) {
		rthal_enable_fpu();
		/* No exception should be pending, since it should have caused
		   a trap earlier.
		*/
	} else if (tcb->fpup && tcb->fpup == rthal_task_fpenv(tcb->user_task)) {
		unsigned fpexc = rthal_enable_fpu();
		unsigned cpu;
#ifndef CONFIG_SMP
		if (likely(!(fpexc & RTHAL_VFP_ANY_EXC)
			   && !(rthal_vfp_fmrx(FPSCR) & FPSCR_IXE)))
			return;
		/*
		   If current process has pending exceptions it is
		   illegal to restore the FPEXC register with them, we must
		   save the fpu state and disable them, to get linux
		   fpu fault handler take care of them correctly.
		*/
#endif
		/*
		   On SMP systems, if we are restoring the root
		   thread, running the task holding the FPU context at
		   the time when we switched to real-time domain,
		   forcibly save the FPU context. It seems to fix SMP
		   systems for still unknown reasons.
		*/
		rthal_save_fpu(tcb->fpup, fpexc);

		cpu = rthal_processor_id();
		vfp_current_hw_state[cpu] = NULL;
		rthal_disable_fpu();
	}
#else /* !CONFIG_VFP */
	if (!tcb->user_task)
		rthal_enable_fpu();
#endif /* !CONFIG_VFP */
#endif /* CONFIG_XENO_HW_FPU */
}

static inline void xnarch_init_fpu(xnarchtcb_t * tcb)
{
#ifdef CONFIG_XENO_HW_FPU
	/* Initialize the FPU for an emerging kernel-based RT thread. This
	   must be run on behalf of the emerging thread. */
	memset(&tcb->fpuenv, 0, sizeof(tcb->fpuenv));
	rthal_init_fpu(&tcb->fpuenv);
#ifdef CONFIG_VFP
	rthal_enable_fpu();
	rthal_restore_fpu(&tcb->fpuenv);
#endif /* CONFIG_VFP */
#endif /* CONFIG_XENO_HW_FPU */
}

static inline void xnarch_save_fpu(xnarchtcb_t * tcb)
{
#ifdef CONFIG_XENO_HW_FPU
#ifdef CONFIG_VFP
	if (tcb->fpup)
		rthal_save_fpu(tcb->fpup, rthal_enable_fpu());
#else /* !CONFIG_VFP */
	if (tcb->fpup) {
		rthal_save_fpu(tcb->fpup);

		if (tcb->user_fpu_owner && task_thread_info(tcb->user_fpu_owner)) {
			task_thread_info(tcb->user_fpu_owner)->used_cp[1] = 0;
			task_thread_info(tcb->user_fpu_owner)->used_cp[2] = 0;
		}
	}
#endif /* !CONFIG_VFP */
#endif /* CONFIG_XENO_HW_FPU */
}

static inline void xnarch_restore_fpu(xnarchtcb_t * tcb)
{
#ifdef CONFIG_XENO_HW_FPU
#ifdef CONFIG_VFP
	if (likely(!tcb->is_root)) {
		rthal_enable_fpu();
		rthal_restore_fpu(tcb->fpup);
	} else {
	/* We are restoring the Linux current thread which does not own the FPU
	   context, so the FPU must be disabled, so that a fault will occur if
	   the newly switched thread uses the FPU, to allow the kernel handler
	   to pick the correct FPU context.

	   Further set vfp_current_hw_state to NULL to avoid the Linux kernel to
	   save, when the fault occur, the current FPU context, the one of an RT
	   task, into the FPU area of the last non RT task which used the FPU
	   before the preemption by Xenomai.
	*/
		unsigned cpu = rthal_processor_id();
		vfp_current_hw_state[cpu] = NULL;
		rthal_disable_fpu();
	}
#else /* !CONFIG_VFP */
	if (tcb->fpup) {
		rthal_restore_fpu(tcb->fpup);

		if (tcb->user_fpu_owner && task_thread_info(tcb->user_fpu_owner)) {
			task_thread_info(tcb->user_fpu_owner)->used_cp[1] = 1;
			task_thread_info(tcb->user_fpu_owner)->used_cp[2] = 1;
		}
	}

	/* FIXME: We restore FPU "as it was" when Xenomai preempted Linux,
	   whereas we could be much lazier. */
	if (tcb->user_task)
		rthal_disable_fpu();
#endif /* !CONFIG_VFP */
#endif /* CONFIG_XENO_HW_FPU */
}

static inline int xnarch_escalate(void)
{
	extern int xnarch_escalation_virq;

	if (rthal_current_domain == rthal_root_domain) {
		rthal_trigger_irq(xnarch_escalation_virq);
		return 1;
	}

	return 0;
}

#endif /* !_XENO_ASM_ARM_BITS_POD_H */
