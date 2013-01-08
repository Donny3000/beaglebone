#ifndef _XENO_ASM_ARM_ARITH_H
#define _XENO_ASM_ARM_ARITH_H

#include <asm/xenomai/features.h>

#if __LINUX_ARM_ARCH__ >= 4
static inline __attribute__((__const__)) unsigned long long
rthal_arm_nodiv_ullimd(const unsigned long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ);

#define rthal_nodiv_ullimd(op, frac, integ) \
	rthal_arm_nodiv_ullimd((op), (frac), (integ))

static inline __attribute__((__const__)) long long
rthal_arm_nodiv_llimd(const long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ);

#define rthal_nodiv_llimd(op, frac, integ) \
	rthal_arm_nodiv_llimd((op), (frac), (integ))
#else /* arm <= v3 */
#define __rthal_add96and64(l0, l1, l2, s0, s1)		\
	do {						\
		__asm__ ("adds %2, %2, %4\n\t"		\
			 "adcs %1, %1, %3\n\t"		\
			 "adc %0, %0, #0\n\t"		\
			 : "+r"(l0), "+r"(l1), "+r"(l2)	\
			 : "r"(s0), "r"(s1): "cc");	\
	} while (0)
#endif /* arm <= v3 */

#include <asm-generic/xenomai/arith.h>

#if __LINUX_ARM_ARCH__ >= 4
#define rthal_arm_nodiv_ullimd_str			\
	"umull %[tl], %[rl], %[opl], %[fracl]\n\t"	\
	"umull %[rm], %[rh], %[oph], %[frach]\n\t"	\
	"adds %[rl], %[rl], %[tl], lsr #31\n\t"		\
	"adcs %[rm], %[rm], #0\n\t"			\
	"adc %[rh], %[rh], #0\n\t"			\
	"umull %[tl], %[th], %[oph], %[fracl]\n\t"	\
	"adds %[rl], %[rl], %[tl]\n\t"			\
	"adcs %[rm], %[rm], %[th]\n\t"			\
	"adc %[rh], %[rh], #0\n\t"			\
	"umull %[tl], %[th], %[opl], %[frach]\n\t"	\
	"adds %[rl], %[rl], %[tl]\n\t"			\
	"adcs %[rm], %[rm], %[th]\n\t"			\
	"adc %[rh], %[rh], #0\n\t"			\
	"umlal %[rm], %[rh], %[opl], %[integ]\n\t"	\
	"mla %[rh], %[oph], %[integ], %[rh]\n\t"

static inline __attribute__((__const__)) unsigned long long
rthal_arm_nodiv_ullimd(const unsigned long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ)
{
	register unsigned rl __asm__("r5");
	register unsigned rm __asm__("r0");
	register unsigned rh __asm__("r1");
	register unsigned fracl __asm__ ("r2");
	register unsigned frach __asm__ ("r3");
	register unsigned integ __asm__("r4") = rhs_integ;
	register unsigned opl __asm__ ("r6");
	register unsigned oph __asm__ ("r7");
	register unsigned tl __asm__("r8");
	register unsigned th __asm__("r9");

	__rthal_u64tou32(op, oph, opl);
	__rthal_u64tou32(frac, frach, fracl);

	__asm__ (rthal_arm_nodiv_ullimd_str
		 : [rl]"=r"(rl), [rm]"=r"(rm), [rh]"=r"(rh),
		   [tl]"=r"(tl), [th]"=r"(th)
		 : [opl]"r"(opl), [oph]"r"(oph),
		   [fracl]"r"(fracl), [frach]"r"(frach),
		   [integ]"r"(integ)
		 : "cc");

	return __rthal_u64fromu32(rh, rm);
}

static inline __attribute__((__const__)) long long
rthal_arm_nodiv_llimd(const long long op,
		       const unsigned long long frac,
		       const unsigned rhs_integ)
{
	register unsigned rl __asm__("r5");
	register unsigned rm __asm__("r0");
	register unsigned rh __asm__("r1");
	register unsigned fracl __asm__ ("r2");
	register unsigned frach __asm__ ("r3");
	register unsigned integ __asm__("r4") = rhs_integ;
	register unsigned opl __asm__ ("r6");
	register unsigned oph __asm__ ("r7");
	register unsigned tl __asm__("r8");
	register unsigned th __asm__("r9");
	register unsigned s __asm__("r10");

	__rthal_u64tou32(op, oph, opl);
	__rthal_u64tou32(frac, frach, fracl);

	__asm__ ("movs %[s], %[oph], lsr #30\n\t"
		 "beq 1f\n\t"
		 "rsbs  %[opl], %[opl], #0\n\t"
		 "sbc  %[oph], %[oph], %[oph], lsl #1\n"
		 "1:\t"
		 rthal_arm_nodiv_ullimd_str
		 "teq %[s], #0\n\t"
		 "beq 2f\n\t"
		 "rsbs  %[rm], %[rm], #0\n\t"
		 "sbc  %[rh], %[rh], %[rh], lsl #1\n"
		 "2:\t"
		 : [rl]"=r"(rl), [rm]"=r"(rm), [rh]"=r"(rh),
		   [tl]"=r"(tl), [th]"=r"(th), [s]"=r"(s)
		 : [opl]"r"(opl), [oph]"r"(oph),
		   [fracl]"r"(fracl), [frach]"r"(frach),
		   [integ]"r"(integ)
		 : "cc");

	return __rthal_u64fromu32(rh, rm);
}
#endif /* arm >= v4 */

#endif /* _XENO_ASM_ARM_ARITH_H */
