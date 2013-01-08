/**
 *   @ingroup hal
 *   @file
 *
 *   Generic Real-Time HAL.
 *   Copyright &copy; 2005 Philippe Gerum.
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation, Inc., 675 Mass Ave, Cambridge MA 02139,
 *   USA; either version 2 of the License, or (at your option) any later
 *   version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

/**
 * @defgroup hal HAL.
 *
 * Generic Adeos-based hardware abstraction layer.
 *
 *@{*/

#include <linux/version.h>
#include <linux/slab.h>
#include <linux/errno.h>
#include <linux/module.h>
#include <linux/console.h>
#include <linux/kallsyms.h>
#include <linux/bitops.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,0)
#include <linux/hardirq.h>
#else
#include <asm/hardirq.h>
#endif
#include <asm/system.h>
#include <asm/irq.h>
#include <asm/xenomai/hal.h>
#include <stdarg.h>

MODULE_LICENSE("GPL");

unsigned long rthal_cpufreq_arg;
module_param_named(cpufreq, rthal_cpufreq_arg, ulong, 0444);

unsigned long rthal_timerfreq_arg;
module_param_named(timerfreq, rthal_timerfreq_arg, ulong, 0444);

unsigned long rthal_clockfreq_arg;
module_param_named(clockfreq, rthal_clockfreq_arg, ulong, 0444);

#ifdef CONFIG_SMP
static unsigned long supported_cpus_arg = -1;
module_param_named(supported_cpus, supported_cpus_arg, ulong, 0444);

cpumask_t rthal_supported_cpus;
EXPORT_SYMBOL_GPL(rthal_supported_cpus);
#endif /* CONFIG_SMP */

static int rthal_init_done;

static rthal_spinlock_t rthal_apc_lock = RTHAL_SPIN_LOCK_UNLOCKED;

static atomic_t rthal_sync_count = ATOMIC_INIT(1);

rthal_pipeline_stage_t rthal_domain;

struct rthal_calibration_data rthal_tunables;

rthal_trap_handler_t rthal_trap_handler;

unsigned rthal_realtime_faults[RTHAL_NR_CPUS][RTHAL_NR_FAULTS];
EXPORT_SYMBOL_GPL(rthal_realtime_faults);

unsigned long rthal_apc_map;
EXPORT_SYMBOL_GPL(rthal_apc_map);

struct rthal_apc_desc rthal_apc_table[RTHAL_NR_APCS];
EXPORT_SYMBOL_GPL(rthal_apc_table);

volatile int rthal_sync_op;

unsigned long rthal_apc_pending[RTHAL_NR_CPUS];

unsigned int rthal_apc_virq;

unsigned long rthal_critical_enter(void (*synch) (void))
{
    unsigned long flags = rthal_grab_superlock(synch);

    if (atomic_dec_and_test(&rthal_sync_count))
	rthal_sync_op = 0;
    else if (synch != NULL)
	printk(KERN_WARNING "Xenomai: Nested critical sync will fail.\n");

    return flags;
}

void rthal_critical_exit(unsigned long flags)
{
    atomic_inc(&rthal_sync_count);
    rthal_release_superlock(flags);
}

/**
 * @fn int rthal_irq_request(unsigned irq, rthal_irq_handler_t handler, rthal_irq_ackfn_t ackfn, void *cookie)
 *
 * @brief Install a real-time interrupt handler.
 *
 * Installs an interrupt handler for the specified IRQ line by
 * requesting the appropriate Adeos virtualization service. The
 * handler is invoked by Adeos on behalf of the Xenomai domain
 * context.  Once installed, the HAL interrupt handler will be called
 * prior to the regular Linux handler for the same interrupt source.
 *
 * @param irq The hardware interrupt channel to install a handler on.
 * This value is architecture-dependent.
 *
 * @param handler The address of a valid interrupt service routine.
 * This handler will be called each time the corresponding IRQ is
 * delivered, and will be passed the @a cookie value unmodified.
 *
 * @param ackfn The address of an optional interrupt acknowledge
 * routine, aimed at replacing the one provided by Adeos. Only very
 * specific situations actually require to override the default Adeos
 * setting for this parameter, like having to acknowledge non-standard
 * PIC hardware. If @a ackfn is NULL, the default Adeos routine will
 * be used instead.
 *
 * @param cookie A user-defined opaque cookie the HAL will pass to the
 * interrupt handler as its sole argument.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EBUSY is returned if an interrupt handler is already installed.
 * rthal_irq_release() must be issued first before a handler is
 * installed anew.
 *
 * - -EINVAL is returned if @a irq is invalid or @a handler is NULL.
 *
 * - Other error codes might be returned in case some internal error
 * happens at the Adeos level. Such error might caused by conflicting
 * Adeos requests made by third-party code.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Any domain context.
 */

int rthal_irq_request(unsigned irq,
		      rthal_irq_handler_t handler,
		      rthal_irq_ackfn_t ackfn, void *cookie)
{
    if (handler == NULL || irq >= IPIPE_NR_IRQS)
	return -EINVAL;

    return rthal_virtualize_irq(&rthal_domain,
				irq,
				handler,
				cookie,
				ackfn,
				IPIPE_HANDLE_MASK | IPIPE_WIRED_MASK |
				IPIPE_EXCLUSIVE_MASK);
}

/**
 * @fn int rthal_irq_release(unsigned irq)
 *
 * @brief Uninstall a real-time interrupt handler.
 *
 * Uninstalls an interrupt handler previously attached using the
 * rthal_irq_request() service.
 *
 * @param irq The hardware interrupt channel to uninstall a handler
 * from.  This value is architecture-dependent.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a irq is invalid.
 *
 * - Other error codes might be returned in case some internal error
 * happens at the Adeos level. Such error might caused by conflicting
 * Adeos requests made by third-party code.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Any domain context.
 */

int rthal_irq_release(unsigned irq)
{
    if (irq >= IPIPE_NR_IRQS)
	return -EINVAL;

    return rthal_virtualize_irq(&rthal_domain,
				irq, NULL, NULL, NULL, IPIPE_PASS_MASK);
}

/**
 * @fn int rthal_irq_host_request(unsigned irq,rthal_irq_host_handler_t handler,char *name,void *dev_id)
 *
 * @brief Install a shared Linux interrupt handler.
 *
 * Installs a shared interrupt handler in the Linux domain for the
 * given interrupt source.  The handler is appended to the existing
 * list of Linux handlers for this interrupt source.
 *
 * @param irq The interrupt source to attach the shared handler to.
 * This value is architecture-dependent.
 *
 * @param handler The address of a valid interrupt service routine.
 * This handler will be called each time the corresponding IRQ is
 * delivered, as part of the chain of existing regular Linux handlers
 * for this interrupt source. The handler prototype is the same as the
 * one required by the request_irq() service provided by the Linux
 * kernel.
 *
 * @param name is a symbolic name identifying the handler which will
 * get reported through the /proc/interrupts interface.
 *
 * @param dev_id is a unique device id, identical in essence to the
 * one requested by the request_irq() service.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a irq is invalid or @a handler is NULL.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Linux domain context.
 */

/**
 * @fn int rthal_irq_host_release (unsigned irq,void *dev_id)
 *
 * @brief Uninstall a shared Linux interrupt handler.
 *
 * Uninstalls a shared interrupt handler from the Linux domain for the
 * given interrupt source.  The handler is removed from the existing
 * list of Linux handlers for this interrupt source.
 *
 * @param irq The interrupt source to detach the shared handler from.
 * This value is architecture-dependent.
 *
 * @param dev_id is a valid device id, identical in essence to the one
 * requested by the free_irq() service provided by the Linux
 * kernel. This value will be used to locate the handler to remove
 * from the chain of existing Linux handlers for the given interrupt
 * source. This parameter must match the device id. passed to
 * rthal_irq_host_request() for the same handler instance.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a irq is invalid.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Linux domain context.
 */

/**
 * @fn void rthal_irq_host_pend (unsigned irq)
 *
 * @brief Propagate an IRQ event to Linux.
 *
 * Causes the given IRQ to be propagated down to the Adeos pipeline to
 * the Linux kernel. This operation is typically used after the given
 * IRQ has been processed into the Xenomai domain by a real-time
 * interrupt handler (see rthal_irq_request()), in case such interrupt
 * must also be handled by the Linux kernel.
 *
 * @param irq The interrupt number to propagate.  This value is
 * architecture-dependent.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Xenomai domain context.
 */

/**
 * @fn int rthal_trap_catch (rthal_trap_handler_t handler)
 *
 * @brief Installs a fault handler.
 *
 * The HAL attempts to invoke a fault handler whenever an uncontrolled
 * exception or fault is caught at machine level. This service allows
 * to install a user-defined handler for such events.
 *
 * @param handler The address of the fault handler to call upon
 * exception condition. The handler is passed the address of the
 * low-level information block describing the fault as passed by
 * Adeos. Its layout is implementation-dependent.
 *
 * @return The address of the fault handler previously installed.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Any domain context.
 */

rthal_trap_handler_t rthal_trap_catch(rthal_trap_handler_t handler)
{
    return (rthal_trap_handler_t) xchg(&rthal_trap_handler, handler);
}

static void rthal_apc_handler(unsigned virq, void *arg)
{
    void (*handler) (void *), *cookie;
    int cpu;

    rthal_spin_lock(&rthal_apc_lock);

    cpu = rthal_processor_id();

    /* <!> This loop is not protected against a handler becoming
       unavailable while processing the pending queue; the software
       must make sure to uninstall all apcs before eventually
       unloading any module that may contain apc handlers. We keep the
       handler affinity with the poster's CPU, so that the handler is
       invoked on the same CPU than the code which called
       rthal_apc_schedule(). */

    while (rthal_apc_pending[cpu] != 0) {
	int apc = ffnz(rthal_apc_pending[cpu]);
	clear_bit(apc, &rthal_apc_pending[cpu]);
	handler = rthal_apc_table[apc].handler;
	cookie = rthal_apc_table[apc].cookie;
	rthal_apc_table[apc].hits[cpu]++;
	rthal_spin_unlock(&rthal_apc_lock);
	handler(cookie);
	rthal_spin_lock(&rthal_apc_lock);
    }

    rthal_spin_unlock(&rthal_apc_lock);
}

#ifdef CONFIG_PREEMPT_RT

/* On PREEMPT_RT, we need to invoke the apc handlers over a process
   context, so that the latter can access non-atomic kernel services
   properly. So the Adeos virq is only used to kick a per-CPU apc
   server process which in turns runs the apc dispatcher. A bit
   twisted, but indeed consistent with the threaded IRQ model of
   PREEMPT_RT. */

#include <linux/kthread.h>

static struct task_struct *rthal_apc_servers[RTHAL_NR_CPUS];

static int rthal_apc_thread(void *data)
{
    unsigned cpu = (unsigned)(unsigned long)data;

    set_cpus_allowed(current, cpumask_of_cpu(cpu));
    sigfillset(&current->blocked);
    current->flags |= PF_NOFREEZE;
    /* Use highest priority here, since some apc handlers might
       require to run as soon as possible after the request has been
       pended. */
    rthal_setsched_root(current, SCHED_FIFO, MAX_RT_PRIO - 1);

    while (!kthread_should_stop()) {
	set_current_state(TASK_INTERRUPTIBLE);
	schedule();
	rthal_apc_handler(0, NULL);
    }

    __set_current_state(TASK_RUNNING);

    return 0;
}

void rthal_apc_kicker(unsigned virq, void *cookie)
{
    wake_up_process(rthal_apc_servers[smp_processor_id()]);
}

#define rthal_apc_trampoline rthal_apc_kicker

#else /* !CONFIG_PREEMPT_RT */

#define rthal_apc_trampoline rthal_apc_handler

#endif /* CONFIG_PREEMPT_RT */

/**
 * @fn int rthal_apc_alloc (const char *name,void (*handler)(void *cookie),void *cookie)
 *
 * @brief Allocate an APC slot.
 *
 * APC is the acronym for Asynchronous Procedure Call, a mean by which
 * activities from the Xenomai domain can schedule deferred
 * invocations of handlers to be run into the Linux domain, as soon as
 * possible when the Linux kernel gets back in control. Up to
 * BITS_PER_LONG APC slots can be active at any point in time. APC
 * support is built upon Adeos's virtual interrupt support.
 *
 * The HAL guarantees that any Linux kernel service which would be
 * callable from a regular Linux interrupt handler is also available
 * to APC handlers.
 *
 * @param name is a symbolic name identifying the APC which will get
 * reported through the /proc/xenomai/apc interface. Passing NULL to
 * create an anonymous APC is allowed.
 *
 * @param handler The address of the fault handler to call upon
 * exception condition. The handle will be passed the @a cookie value
 * unmodified.
 *
 * @param cookie A user-defined opaque cookie the HAL will pass to the
 * APC handler as its sole argument.
 *
 * @return an valid APC id. is returned upon success, or a negative
 * error code otherwise:
 *
 * - -EINVAL is returned if @a handler is invalid.
 *
 * - -EBUSY is returned if no more APC slots are available.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Linux domain context.
 */

int rthal_apc_alloc(const char *name,
		    void (*handler) (void *cookie), void *cookie)
{
	unsigned long flags;
	int apc;

	if (handler == NULL)
		return -EINVAL;

	rthal_spin_lock_irqsave(&rthal_apc_lock, flags);

	if (rthal_apc_map == ~0) {
		apc = -EBUSY;
		goto out;
	}

	apc = ffz(rthal_apc_map);
	__set_bit(apc, &rthal_apc_map);
	rthal_apc_table[apc].handler = handler;
	rthal_apc_table[apc].cookie = cookie;
	rthal_apc_table[apc].name = name;
out:
	rthal_spin_unlock_irqrestore(&rthal_apc_lock, flags);

	return apc;
}

/**
 * @fn int rthal_apc_free (int apc)
 *
 * @brief Releases an APC slot.
 *
 * This service deallocates an APC slot obtained by rthal_apc_alloc().
 *
 * @param apc The APC id. to release, as returned by a successful call
 * to the rthal_apc_alloc() service.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Any domain context.
 */

void rthal_apc_free(int apc)
{
	BUG_ON(apc < 0 || apc >= RTHAL_NR_APCS);
	clear_bit(apc, &rthal_apc_map);
}

#ifdef CONFIG_PREEMPT_RT

static inline int setup_apc_handler(void)
{
	int cpu;

	for_each_online_cpu(cpu) {
		rthal_apc_servers[cpu] =
			kthread_create(rthal_apc_thread,
				       (void *)(unsigned long)cpu,
				       "apc/%d", cpu);
		if (rthal_apc_servers[cpu] == NULL)
			return -ENOMEM;
		wake_up_process(rthal_apc_servers[cpu]);
	}

	return 0;
}

static inline void cleanup_apc_handler(void)
{
	int cpu;

	for_each_online_cpu(cpu) {
		if (rthal_apc_servers[cpu])
			kthread_stop(rthal_apc_servers[cpu]);
	}
}

#else  /* !CONFIG_PREEMPT_RT */

static inline int setup_apc_handler(void)
{
	return 0;
}

static inline void cleanup_apc_handler(void) { }

#endif  /* !CONFIG_PREEMPT_RT */

int rthal_init(void)
{
    int err;

    err = rthal_arch_init();

    if (err)
	goto out;

#ifdef CONFIG_SMP
    {
	int cpu;
	cpus_clear(rthal_supported_cpus);
	for (cpu = 0; cpu < BITS_PER_LONG; cpu++)
	    if (supported_cpus_arg & (1 << cpu))
		cpu_set(cpu, rthal_supported_cpus);
    }
#endif /* CONFIG_SMP */

    /*
     * The arch-dependent support must have updated the various
     * frequency args as required.
     */

    /* check the CPU frequency first and abort if it's invalid */
    if (rthal_cpufreq_arg == 0) {
	printk(KERN_ERR "Xenomai has detected a CPU frequency of 0. Aborting.\n");
	return -ENODEV;
    }

    rthal_tunables.cpu_freq = rthal_cpufreq_arg;
    rthal_tunables.timer_freq = rthal_timerfreq_arg;
    rthal_tunables.clock_freq = rthal_clockfreq_arg;

    /*
     * Allocate a virtual interrupt to handle apcs within the Linux
     * domain.
     */
    rthal_apc_virq = rthal_alloc_virq();

    if (!rthal_apc_virq) {
	printk(KERN_ERR "Xenomai: No virtual interrupt available.\n");
	err = -EBUSY;
	goto out_arch_cleanup;
    }

    err = rthal_virtualize_irq(rthal_current_domain,
			       rthal_apc_virq,
			       rthal_apc_trampoline,
			       NULL, NULL, IPIPE_HANDLE_MASK);
    if (err) {
	printk(KERN_ERR "Xenomai: Failed to virtualize IRQ.\n");
	goto out_free_irq;
    }

    err = setup_apc_handler();
    if (err)
	    goto fail;

    err = rthal_register_domain(&rthal_domain,
				"Xenomai",
				RTHAL_DOMAIN_ID,
				RTHAL_XENO_PRIO, &rthal_domain_entry);
    if (!err)
	rthal_init_done = 1;
    else {
#ifdef __ipipe_pipeline_head
	if (err == -EAGAIN) {
	    printk(KERN_ERR
		   "Xenomai: the real-time domain cannot head the pipeline,\n");
	    printk(KERN_ERR
		   "         either unload domain %s or disable CONFIG_XENO_OPT_PIPELINE_HEAD.\n",
		   __ipipe_pipeline_head()->name);
	} else
#endif
	    printk(KERN_ERR "Xenomai: Domain registration failed (%d).\n", err);

	goto fail;
    }

    return 0;

  fail:
    rthal_virtualize_irq(rthal_current_domain, rthal_apc_virq, NULL, NULL, NULL,
			 0);

    cleanup_apc_handler();

  out_free_irq:
    rthal_free_virq(rthal_apc_virq);

  out_arch_cleanup:
    rthal_arch_cleanup();

  out:
    return err;
}

void rthal_exit(void)
{
    if (rthal_apc_virq) {
	rthal_virtualize_irq(rthal_current_domain, rthal_apc_virq, NULL, NULL,
			     NULL, 0);
	rthal_free_virq(rthal_apc_virq);
    }

    cleanup_apc_handler();

    if (rthal_init_done)
	rthal_unregister_domain(&rthal_domain);

    rthal_arch_cleanup();
}

unsigned long long __rthal_generic_full_divmod64(unsigned long long a,
						 unsigned long long b,
						 unsigned long long *rem)
{
	unsigned long long q = 0, r = a;
	int i;

	for (i = fls(a >> 32) - fls(b >> 32), b <<= i; i >= 0; i--, b >>= 1) {
		q <<= 1;
		if (b <= r) {
			r -= b;
			q++;
		}
	}

	if (rem)
		*rem = r;
	return q;
}

/**
 * @fn int rthal_irq_enable(unsigned irq)
 *
 * @brief Enable an interrupt source.
 *
 * Enables an interrupt source at PIC level. Since Adeos masks and
 * acknowledges the associated interrupt source upon IRQ receipt, this
 * action is usually needed whenever the HAL handler does not
 * propagate the IRQ event to the Linux domain, thus preventing the
 * regular Linux interrupt handling code from re-enabling said
 * source. After this call has returned, IRQs from the given source
 * will be enabled again.
 *
 * @param irq The interrupt source to enable.  This value is
 * architecture-dependent.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a irq is invalid.
 *
 * - Other error codes might be returned in case some internal error
 * happens at the Adeos level. Such error might caused by conflicting
 * Adeos requests made by third-party code.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Any domain context.
 */

/**
 * @fn int rthal_irq_disable(unsigned irq)
 *
 * @brief Disable an interrupt source.
 *
 * Disables an interrupt source at PIC level. After this call has
 * returned, no more IRQs from the given source will be allowed, until
 * the latter is enabled again using rthal_irq_enable().
 *
 * @param irq The interrupt source to disable.  This value is
 * architecture-dependent.
 *
 * @return 0 is returned upon success. Otherwise:
 *
 * - -EINVAL is returned if @a irq is invalid.
 *
 * - Other error codes might be returned in case some internal error
 * happens at the Adeos level. Such error might caused by conflicting
 * Adeos requests made by third-party code.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Any domain context.
 */

/**
 * \fn int rthal_timer_request(void (*tick_handler)(void),
 *             void (*mode_emul)(enum clock_event_mode mode, struct clock_event_device *cdev),
 *             int (*tick_emul)(unsigned long delay, struct clock_event_device *cdev), int cpu)
 * \brief Grab the hardware timer.
 *
 * rthal_timer_request() grabs and tunes the hardware timer in oneshot
 * mode in order to clock the master time base.
 *
 * A user-defined routine is registered as the clock tick handler.
 * This handler will always be invoked on behalf of the Xenomai domain
 * for each incoming tick.
 *
 * Hooks for emulating oneshot mode for the tick device are accepted
 * when CONFIG_GENERIC_CLOCKEVENTS is defined for the host
 * kernel. Host tick emulation is a way to share the clockchip
 * hardware between Linux and Xenomai, when the former provides
 * support for oneshot timing (i.e. high resolution timers and no-HZ
 * scheduler ticking).
 *
 * @param tick_handler The address of the Xenomai tick handler which will
 * process each incoming tick.
 *
 * @param mode_emul The optional address of a callback to be invoked
 * upon mode switch of the host tick device, notified by the Linux
 * kernel. This parameter is only considered whenever
 * CONFIG_GENERIC_CLOCKEVENTS is defined.
 *
 * @param tick_emul The optional address of a callback to be invoked
 * upon setup of the next shot date for the host tick device, notified
 * by the Linux kernel. This parameter is only considered whenever
 * CONFIG_GENERIC_CLOCKEVENTS is defined.
 *
 * @param cpu The CPU number to grab the timer from.
 *
 * @return a positive value is returned on success, representing the
 * duration of a Linux periodic tick expressed as a count of
 * nanoseconds; zero should be returned when the Linux kernel does not
 * undergo periodic timing on the given CPU (e.g. oneshot
 * mode). Otherwise:
 *
 * - -EBUSY is returned if the hardware timer has already been
 * grabbed.  rthal_timer_request() must be issued before
 * rthal_timer_request() is called again.
 *
 * - -ENODEV is returned if the hardware timer cannot be used.  This
 * situation may occur after the kernel disabled the timer due to
 * invalid calibration results; in such a case, such hardware is
 * unusable for any timing duties.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Linux domain context.
 */

/**
 * \fn void rthal_timer_release(int cpu)
 * \brief Release the hardware timer.
 *
 * Releases the hardware timer, thus reverting the effect of a
 * previous call to rthal_timer_request(). In case the timer hardware
 * is shared with Linux, a periodic setup suitable for the Linux
 * kernel will be reset.
 *
 * @param cpu The CPU number the timer was grabbed from.
 *
 * Environments:
 *
 * This service can be called from:
 *
 * - Linux domain context.
 */

/**
 * @fn int rthal_apc_schedule (int apc)
 *
 * @brief Schedule an APC invocation.
 *
 * This service marks the APC as pending for the Linux domain, so that
 * its handler will be called as soon as possible, when the Linux
 * domain gets back in control.
 *
 * When posted from the Linux domain, the APC handler is fired as soon
 * as the interrupt mask is explicitly cleared by some kernel
 * code. When posted from the Xenomai domain, the APC handler is
 * fired as soon as the Linux domain is resumed, i.e. after Xenomai has
 * completed all its pending duties.
 *
 * @param apc The APC id. to schedule.
 *
 * This service can be called from:
 *
 * - Any domain context, albeit the usual calling place is from the
 * Xenomai domain.
 */

/*@}*/

EXPORT_SYMBOL_GPL(rthal_irq_request);
EXPORT_SYMBOL_GPL(rthal_irq_release);
EXPORT_SYMBOL_GPL(rthal_irq_enable);
EXPORT_SYMBOL_GPL(rthal_irq_disable);
EXPORT_SYMBOL_GPL(rthal_irq_end);
EXPORT_SYMBOL_GPL(rthal_irq_host_request);
EXPORT_SYMBOL_GPL(rthal_irq_host_release);
EXPORT_SYMBOL_GPL(rthal_trap_catch);
EXPORT_SYMBOL_GPL(rthal_timer_request);
EXPORT_SYMBOL_GPL(rthal_timer_release);
EXPORT_SYMBOL_GPL(rthal_timer_calibrate);
EXPORT_SYMBOL_GPL(rthal_apc_alloc);
EXPORT_SYMBOL_GPL(rthal_apc_free);

EXPORT_SYMBOL_GPL(rthal_critical_enter);
EXPORT_SYMBOL_GPL(rthal_critical_exit);

EXPORT_SYMBOL_GPL(rthal_domain);
EXPORT_SYMBOL_GPL(rthal_tunables);

EXPORT_SYMBOL_GPL(rthal_init);
EXPORT_SYMBOL_GPL(rthal_exit);
EXPORT_SYMBOL_GPL(__rthal_generic_full_divmod64);
EXPORT_SYMBOL_GPL(rthal_apc_virq);
EXPORT_SYMBOL_GPL(rthal_apc_pending);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,27)
EXPORT_SYMBOL_GPL(kill_proc_info);
#endif
