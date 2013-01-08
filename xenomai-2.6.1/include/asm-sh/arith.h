/*
 * Copyright (C) 2011 Philippe Gerum <rpm@xenomai.org>.
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

#ifndef _XENO_ASM_SH_ARITH_H
#define _XENO_ASM_SH_ARITH_H

#include <asm/xenomai/features.h>

#define __rthal_add96and64(l0, l1, l2, s0, s1)		\
	do {						\
		__asm__ ("clrt\n\t"			\
			 "addc %4, %2\n\t"		\
			 "addc %3, %1\n\t"		\
			 "addc %5, %0\n\t"		\
			 : "+r"(l0), "+r"(l1), "+r"(l2)	\
			 : "r"(s0), "r"(s1), "r" (0) : "t");	\
	} while (0)

#include <asm-generic/xenomai/arith.h>

#endif /* _XENO_ASM_SH_ARITH_H */
