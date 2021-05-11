/*	$NetBSD: can.c,v 1.2.2.1 2018/04/09 13:34:11 bouyer Exp $	*/

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
#include <sys/cdefs.h>

#include "opt_can.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_can.h>
#include <net/netisr.h>

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

/*
 * Declare netisr(4) handler for input queue.
 */ 
static struct netisr_handler	can_nh = {
	.nh_name = 	"can",
	.nh_handler = 	can_nh_input,
	.nh_proto = 	NETISR_CAN,
#ifdef	RSS
	.nh_m2cpuid =  	rss_soft_m2cpuid_v4,
	.nh_policy = 	NETISR_POLICY_CPU,
	.nh_dispatch = 	NETISR_DISPATCH_HYBRID,
#else
	.nh_policy = 	NETISR_POLICY_FLOW,
#endif
};

/* Superset of PCBs on PF_CAN with its protective lock */
struct rwlock can_pcbinfo_lock;
struct canpcbinfo_head can_pcbinfo_tbl;

/* 
 * Initialize.
 */
void
can_init(void)
{

	rw_init_flags(&can_pcbinfo_lock, "canpinfo", RW_RECURSE | RW_DUPOK);
	TAILQ_INIT(&can_pcbinfo_tbl);
	netisr_register(&can_nh);
}

/*
 * Handoff rx'd can(4) frame from protocol- into socket-layer.
 */ 
void
can_nh_input(struct mbuf *m)
{
	struct sockaddr_can from;
	struct m_tag	*sotag;
	struct canpcb	*sender_canp;
	struct ifnet 	*ifp;
	int		rcv_ifindex; /* XXX */
#ifdef DIAGNOSTIC
	struct can_hdr *ch; 
#endif /* DIAGNOSTIC */
	struct canpcbinfo *cani;

	M_ASSERTPKTHDR(m);
	KASSERT((m->m_pkthdr.rcvif != NULL),
		("%s: NULL interface pointer", __func__));
#if 0
	CANSTAT_INC(cans_total);
#endif 

	if (m->m_pkthdr.len < sizeof(struct can_frame)) {
#if 0
		CANSTAT_INC(cans_tooshort);
#endif	
		goto out;
	}

	if (m->m_len < sizeof(struct can_frame)) {
	    if ((m = m_pullup(m, sizeof(struct can_frame))) == NULL) {
#if 0
		CANSTAT_INC(cans_toosmall);
#endif
			goto out;
		}
	}

	if ((sotag = m_tag_find(m, PACKET_TAG_ND_OUTGOING, NULL)) != NULL) {
		sender_canp = *(struct canpcb **)(sotag + 1);
		m_tag_delete(m, sotag);

		KASSERT((sender_canp != NULL),
			("%s: sender_canp == NULL", __func__));

		/* if the sender doesn't want loopback, don't do it */
		if ((sender_canp->canp_flags & CANP_NO_LOOPBACK) != 0)
			goto out1;
			
	} else
		sender_canp = NULL;

	/* fetch interface index */
	ifp = m->m_pkthdr.rcvif;
	rcv_ifindex = ifp->if_index;

	(void)memset(&from, 0, sizeof(struct sockaddr_can));

	from.scan_ifindex = rcv_ifindex;
	from.scan_len = sizeof(struct sockaddr_can);
	from.scan_family = AF_CAN;

#ifdef DIAGNOSTIC
	ch = mtod(m, struct can_hdr *);
	(void)printf("%s: type 0x%01x id 0x%08x dlc 0x%02x\n",
		__func__, (ch->ch_id & CAN_FLAG_MASK) >> 28,
		(ch->ch_id & CAN_EFF_MASK), ch->ch_dlc);
#endif /* DIAGNOSTIC */

	rw_rlock(&can_pcbinfo_lock);

	/* XXX: Well, I'll change this, ...  */
	TAILQ_FOREACH(cani, &can_pcbinfo_tbl, cani_next) {
		struct canpcb   *canp;

		/* fetch PCB maps to interface by its index, if any */
		TAILQ_FOREACH(canp, &cani->cani_queue, canp_queue) {
			struct mbuf *mc;

			CANP_LOCK(canp);

			/* skip if we're detached */
			if (canp->canp_state == CANP_DETACHED) {
				CANP_UNLOCK(canp);
				continue;
			}

			/* don't loop back to sockets on other interfaces */
			if (canp->canp_ifp != NULL &&
				canp->canp_ifp->if_index != rcv_ifindex) {
				CANP_UNLOCK(canp);
				continue;
			}

			/* don't loop back to myself if I don't want it */
			if (canp == sender_canp && 
				(canp->canp_flags & CANP_RECEIVE_OWN) == 0) {
				CANP_UNLOCK(canp);
				continue;
			}

			/* skip if the accept filter doesn't match this pkt */
			if (can_pcbfilter(canp, m) == 0) {
				CANP_UNLOCK(canp);
				continue;
			}

			if (TAILQ_NEXT(canp, canp_queue) != NULL) {
				/*
				 * we can't be sure we won't need 
				 * the original mbuf later so copy 
				 */
				if ((mc = m_copypacket(m, M_NOWAIT)) == NULL) {
					/* deliver this mbuf and abort */
					mc = m;
					m = NULL;
				}
			} else {
				mc = m;
				m = NULL;
			}

			/* enqueue mbuf(9) */
			if (sbappendaddr(&canp->canp_so->so_rcv,
					(struct sockaddr *) &from, mc,
					(struct mbuf *) 0) == 0) {
				m_freem(mc);
			} else
				sorwakeup(canp->canp_so);

			CANP_UNLOCK(canp);

			if (m == NULL)
				goto out2;
		}
	}
out2:	/* XXX */
	rw_runlock(&can_pcbinfo_lock);
out1:
	if (sender_canp != NULL) {
		CANP_LOCK(sender_canp);
		canp_unref(sender_canp);
		CANP_UNLOCK(sender_canp);
	}

	/* If it didn't go anywhere just delete it */
out:
	if (m != NULL) 
		m_freem(m);
}

