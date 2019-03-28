/*	$NetBSD: can_var.h,v 1.2 2017/05/27 21:02:56 bouyer Exp $	*/

/*-
 * Copyright (c) 2003, 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Robert Swindells and Manuel Bouyer
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Copyright (c) 2018 Henning Matyschok
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
 
#ifndef _NETCAN_CAN_VAR_H_
#define _NETCAN_CAN_VAR_H_

/* statistics */
struct can_stat {
	uint64_t cans_total;		/* total frames received */
	uint64_t cans_tooshort;		/* frame too short */
	uint64_t cans_toosmall; 	/* not enough data */
	uint64_t cans_forward; 		/* frames forwarded */
	uint64_t cans_sff_rxd; 			/* rx'd SFF frames */
	uint64_t cans_eff_rxd; 			/* rx'd EFF frames */
	uint64_t cans_rtr_rxd; 			/* rx'd RTR frames */
	uint64_t cans_err_rxd; 			/* rx'd ERR frames */
	uint64_t cans_sff_txd; 			/* tx'd SFF frames */
	uint64_t cans_eff_txd; 			/* tx'd EFF frames */
	uint64_t cans_rtr_txd; 			/* tx'd RTR frames */
	uint64_t cans_err_txd; 			/* tx'd ERR frames */
	uint64_t cans_unknown; 	/* unknown or unsupported protocol */
	uint64_t cans_delivered; 	/* frames delivered to upper level*/
	uint64_t cans_localout; 	/* total frames generated here */
	uint64_t cans_odropped; 	/* lost frames due to nobufs, etc. */
	uint64_t cans_rawout; 	/* total raw CAN frames generated */
	uint64_t cans_badid; 	/* invalid address on header */
};

/*
 * XXX: incomplete, but it looks for me as possible
 * XXX: candidate of an implementation of a message
 * XXX: primitive for SOCK_DGRAM on CANPROTO_BCM?
 */
struct can_ifreq {
	char            cfr_name[IFNAMSIZ];	/* if name, e.g. "sja0" */
};

#ifdef _KERNEL

/*
 * Implements CAN filter on interface-layer.  
 */
struct can_ifaddr {
	struct	ifaddr cia_ifa;		/* protocol-independent info */
#define	cia_ifp		cia_ifa.ifa_ifp
#define cia_flags	cia_ifa.ifa_flags
	TAILQ_ENTRY(can_ifaddr) cia_link;
	struct sockaddr_can cia_addr;	/* reserve space for CAN Filter */
};

/* AF_CAN communication domain(9) */
extern struct domain candomain;

/* raw userreqs */
extern struct pr_usrreqs rcan_usrreqs;

/* CANPROTO_CAN */
void 	can_ctlinput(int, struct sockaddr *, void *);
int 	can_ctloutput(struct socket *, struct sockopt *);
void 	can_init(void);
void 	can_nh_input(struct mbuf *);
int 	can_output(struct mbuf *, struct canpcb *);
int 	can_control(struct socket *, u_long, caddr_t, 
	struct ifnet *, struct thread *);

/* CANPROTO_RAW */
void 	rcan_init(void);
int 	rcan_ctloutput(struct socket *, struct sockopt *);

#endif /* _KERNEL */
#endif /* _NETCAN_CAN_VAR_H_ */
