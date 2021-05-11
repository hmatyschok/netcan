/*	$NetBSD: if_slvar.h,v 1.33 2007/07/14 21:02:41 ad Exp $	*/

/*-
 * Copyright (c) 1991, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)if_slvar.h	8.4 (Berkeley) 1/9/95
 */
 /*
 * Copyright (c) 2019 Henning Matyschok
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
#ifndef _NET_IF_SLCVAR_H_
#define _NET_IF_SLCVAR_H_

#include <sys/ctype_inline.h>

/* 
 * ASCII representation of can(4) frame:
 *
 *  <frm> ::= <type> <id> <dlc> <data> <hc>;
 *
 * encodes the
 *
 *  <type> ::= "R" | "T" | "r" | "t";
 *
 * by
 *
 *   "R" denotes remote frame, extended frame format
 *
 *   "T" denotes data frame, extendet frame format
 *
 *   "r" denotes remote frame, base frame format
 *
 *   "t" denotes data frame, base frame format
 *
 * with
 *
 *  <id> ::= <sff> | <eff>;
 *
 * where encodes the 11 identifier bits
 *
 *  <sff> ::= 3 * <byte>;
 *
 * or the 29 identifier bits
 *
 *  <eff> ::= 8 * <byte>;
 *
 * by
 *
 *  <byte> ::= <xdigit> <xdigit>;
 *
 * with
 *
 *  <xdigit> ::= <digit> | "A" | "B" | "C" | "D" | "E" | "F";
 *
 * and the
 *
 *  <dlc> ::= <digit> - "9";
 *
 * by
 *
 *  <digit> ::= "0" | "1" | "2" | "3" | "4" |
 *              "5" | "6" | "7" | "8" | "9";
 *
 * and
 *
 *  <data> ::= <dlc> * <byte>;
 *
 * with
 *
 *  <hc> ::= "CR" | "BEL";
 *
 * finally.
 */
#define SLC_CMD_LEN 	(sizeof(u_char))
#define SLC_SFF_ID_LEN 	(sizeof(u_char) * 3)
#define SLC_EFF_ID_LEN 	(sizeof(u_char) * 8)
#define SLC_DLC_LEN 	(sizeof(u_char))
#define SLC_HC_LEN 	(sizeof(u_char))

#define SLC_HC_SFF_DATA 	't'
#define SLC_HC_SFF_RTR 	'r'
#define SLC_HC_EFF_DATA 	'T'
#define SLC_HC_EFF_RTR 	'R'

#define SLC_HC_CR 		'\r'
#define SLC_HC_BEL 		'\a'

#define SLC_HC_DLC_INF 	'0'
#define SLC_HC_DLC_SUP 	'9'

/*
 * Definitions for SLCAN interface data structures.
 */
struct slc_softc {
	struct ifnet	*slc_if;		/* network-visible interface */
	int 	slc_unit;		/* XXX unit number */
	struct mbuf		*slc_ifbuf;
	struct ifqueue	slc_inq;
	struct ifqueue	slc_outq;		/* queue of outgoing data */
	struct	tty *slc_tp;		/* pointer to tty structure */	
	long	slc_outqlen;	/* number of bytes in outq */
	void	*slc_intr;			/* softintr handle */
#ifdef __NetBSD__
	int	slc_oldbufsize;		/* previous output buffer size */
	int	slc_oldbufquot;		/* previous output buffer quoting */
#endif
	LIST_ENTRY(slc_softc) slc_iflist;
};

/* internal flags */
#define	SLC_ERROR	0x0001		/* had an input error */

#endif /* _NET_IF_SLCVAR_H_ */
