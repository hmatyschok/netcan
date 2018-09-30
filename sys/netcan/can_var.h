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

#include <sys/queue.h>
#include <sys/device.h>
#include <netcan/can_link.h>

struct can_ifreq {
	char            cfr_name[IFNAMSIZ];	/* if name, e.g. "sja0" */
};

#ifdef _KERNEL
#include <sys/socketvar.h>

/*
 * common structure for CAN interface drivers. Should be at the 
 * start ofeach driver's softc.
 * 
 * XXX: On the one hand this is generic, but on the other hand it 
 * XXX: is not in sight of the binding between its communication
 * XXX: domain(9) and interface-layer.
 */
struct canif_softc {
	device_t csc_dev;
	struct can_link_timecaps csc_timecaps; /* timing capabilities */
	struct can_link_timings csc_timings; /* operating timing values */
	uint32_t csc_linkmodes;
};

extern struct domain candomain;

/* raw userreqs */
extern const struct pr_usrreqs rcan_usrreqs;

/* interface-layer */
void 	can_ifattach(struct ifnet *);
void 	can_ifdetach(struct ifnet *);
void 	can_bpf_mtap(struct ifnet *, struct mbuf *, bool);
void 	can_ifinit_timings(struct canif_softc *);

/* common subr. */
void 	can_mbuf_tag_clean(struct mbuf *);

/* CANPROTO_CAN */
void *can_ctlinput(int, struct sockaddr *, void *);
int 	can_ctloutput(struct socket *, struct sockopt *);
void 	can_init(void);
int 	can_output(struct mbuf *, struct canpcb *);

/* CANPROTO_RAW */
int 	rcan_ctloutput(struct socket *, struct sockopt *);

#endif /* _KERNEL */
#endif /* _NETCAN_CAN_VAR_H_ */