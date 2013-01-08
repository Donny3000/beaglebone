#ifndef _XENO_ASM_ARM_BIND_H
#define _XENO_ASM_ARM_BIND_H

#include <stdio.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <limits.h>

#include <asm-generic/xenomai/bits/bind.h>
#include <asm/xenomai/syscall.h>

struct __xn_full_tscinfo __xn_tscinfo = {
	.kinfo = {
		.type = -1,
	},
};

static inline void xeno_arm_features_check(struct xnfeatinfo *finfo)
{
#ifdef CONFIG_XENO_ARM_TSC_TYPE
	unsigned long phys_addr;
	unsigned page_size;
	int err, fd;
	void *addr;

	if (__xn_tscinfo.kinfo.type != -1)
		return;

	err = XENOMAI_SYSCALL2(__xn_sys_arch,
			       XENOMAI_SYSARCH_TSCINFO, &__xn_tscinfo);
	if (err)
		goto error;

	fd = open("/dev/mem", O_RDONLY | O_SYNC);
	if (fd == -1) {
		perror("Xenomai init: open(/dev/mem)");
		exit(EXIT_FAILURE);
	}

	page_size = sysconf(_SC_PAGESIZE);

	switch(__xn_tscinfo.kinfo.type) {
#if CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_KUSER
	default:
		__xn_tscinfo.kuser_tsc_get =
			(rdtsc_t *)(0xffff1004 -
				    ((*(unsigned *)(0xffff0ffc) + 3) << 5));
		goto domap;

#elif CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_FREERUNNING		\
	|| CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_FREERUNNING_COUNTDOWN \
	|| CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_FREERUNNING_FAST_WRAP
	case __XN_TSC_TYPE_FREERUNNING:
	case __XN_TSC_TYPE_FREERUNNING_COUNTDOWN:
#if CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_FREERUNNING_FAST_WRAP
		if (__xn_tscinfo.kinfo.mask >= ((1 << 28) - 1)) {
			fprintf(stderr, "Hardware tsc is not a fast wrapping"
				" one, select the correct platform, or fix\n"
				"configure.in\n");
			exit(EXIT_FAILURE);
		}
#endif /* __XN_TSC_TYPE_FREERUNNING_FAST_WRAP */
		goto domap;

	default:
		fprintf(stderr,
			"Xenomai: kernel/user tsc emulation mismatch.\n");
		exit(EXIT_FAILURE);
		break;
#elif CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_DECREMENTER
	case __XN_TSC_TYPE_DECREMENTER:
		goto domap;

	default:
		fprintf(stderr,
			"Xenomai: kernel/user tsc emulation mismatch.\n");
		exit(EXIT_FAILURE);
		break;
#endif /* CONFIG_XENO_ARM_TSC_TYPE == __XN_TSC_TYPE_DECREMENTER */
	case __XN_TSC_TYPE_NONE:
	  error:
		fprintf(stderr, "Xenomai: Your board/configuration does not"
			" allow tsc emulation in user-space: %d\n", err);
		exit(EXIT_FAILURE);
	}

  domap:
	phys_addr = (unsigned long) __xn_tscinfo.kinfo.counter;

	addr = mmap(NULL, page_size, PROT_READ, MAP_SHARED,
		    fd, phys_addr & ~(page_size - 1));
	if (addr == MAP_FAILED) {
		perror("Xenomai init: mmap(/dev/mem)");
		exit(EXIT_FAILURE);
	}

	__xn_tscinfo.kinfo.counter =
		((volatile unsigned *)
		 ((char *) addr + (phys_addr & (page_size - 1))));

	if (close(fd)) {
		perror("Xenomai init: close(/dev/mem)");
		exit(EXIT_FAILURE);
	}
#endif /* CONFIG_XENO_ARM_TSC_TYPE */
}
#define xeno_arch_features_check(finfo) xeno_arm_features_check(finfo)

#endif /* _XENO_ASM_ARM_BIND_H */
