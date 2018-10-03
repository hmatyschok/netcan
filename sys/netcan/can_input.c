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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/ioctl.h>
#include <sys/domain.h>
#include <sys/protosw.h>
#include <sys/errno.h>
#include <sys/socket.h>
#include <sys/socketvar.h>

#include <net/if.h>
#include <net/if_types.h>
#include <net/netisr.h>
#include <net/route.h>
#include <net/bpf.h> 

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

/*
 * Declare netisr(4) handler for input queue.
 */ 
static struct netisr_handler	can_nh = {
	.nh_name = "can",
	.nh_handler = can_nh_input,
	.nh_proto = NETISR_CAN,
#ifdef	RSS
	.nh_m2cpuid = rss_soft_m2cpuid_v4,
	.nh_policy = NETISR_POLICY_CPU,
	.nh_dispatch = NETISR_DISPATCH_HYBRID,
#else
	.nh_policy = NETISR_POLICY_FLOW,
#endif
};

struct canpcbinfo_head can_pcbinfo;

/* 
 * Initialize.
 */
void
can_init(void)
{
	
	TAILQ_INIT(&can_pcbinfohead);
	netisr_register(&can_nh);
}

/*
 * Process rx'd CAN frames by protocol-layer.
 */ 
static void 	
can_nh_input(struct mbuf *m);
{
	struct sockaddr_can from;
	struct canpcb   *canp;
	struct m_tag	*sotag;
	struct canpcb	*sender_canp;

	int		rcv_ifindex; /* XXX */	

	struct canpcbinfo *cani;

	M_ASSERTPKTHDR(m);
	KASSERT(m->m_pkthdr.rcvif != NULL,
	    ("%s: NULL interface pointer", __func__));
#if 0
	CANSTAT_INC(cans_total);
#endif 
	
	if (m->m_pkthdr.len < sizeof(struct can_frame)) {
#if 0
		CANSTAT_INC(cans_toosmall);
#endif	
		goto out:
	}
	
	if (m->m_len < sizeof (struct can_frame) {
		m = m_pullup(m, sizeof (struct can_frame));
	    if (m == NULL) {
#if 0
		CANSTAT_INC(cans_toosmall);
#endif	
		goto out:
	}
	
	sotag = m_tag_find(m, PACKET_TAG_SO, NULL);
	if (sotag != NULL) {
		sender_canp = *(struct canpcb **)(sotag + 1);
		m_tag_delete(m, sotag);
		
		KASSERT((sender_canp != NULL)
			("%s: sender_canp == NULL", __func__));
		
		/* if the sender doesn't want loopback, don't do it */
		if ((sender_canp->canp_flags & CANP_NO_LOOPBACK) != 0) 
			goto out1;
			
	} else 
		sender_canp = NULL;
	
	/* fetch interface index */
	rcv_ifindex = m->m_pkthdr.rcvif->if_index;
	
	(void)memset(&from, 0, sizeof(struct sockaddr_can));
		
	from.scan_ifindex = rcv_ifindex;
	from.scan_len = sizeof(struct sockaddr_can);
	from.scan_family = AF_CAN;
	
	rw_rlock(&can_pcbinfo_lock);
	
	TAILQ_FOREACH(cani, &can_pcbinfo, cani_queue) {
		/* fetch PCB maps to interface by its index, if any */
		TAILQ_FOREACH(canp, &cani->canpt_queue, canp_queue) {
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

			/* skip if the accept filter doen't match this pkt */
			if (!can_pcbfilter(canp, m)) {
				CANP_UNLOCK(canp);
				continue;
			}

			if (TAILQ_NEXT(canp, canp_queue) != NULL) {
				/*
				 * we can't be sure we won't need 
				 * the original mbuf later so copy 
				 */
				mc = m_copypacket(m, M_NOWAIT);
				if (mc == NULL) {
					/* deliver this mbuf and abort */
					mc = m;
					m = NULL;
				}
			} else {
				mc = m;
				m = NULL;
			}
		
			/* enqueue mbuf(9) */
			if (sbappendaddr(&canp->canp_socket->so_rcv,
					(struct sockaddr *) &from, mc,
					(struct mbuf *) 0) == 0) {
				m_freem(mc);
			} else
				sorwakeup(canp->canp_socket);
		
			CANP_UNLOCK(canp);
		
			if (m == NULL)
				goto out2;
		}
	}
out2:	/* XXX */
	rw_runlock(&can_pcbinfo_lock);
out1:	
	if (sender_canp != NULL) 
		canp_unref(sender_canp);
	
	/* If it didn't go anywhere just delete it */
out:
	if (m != NULL) 
		m_freem(m);
}

/*
 * XXX: Yeah, I'll refactor this. 
 */
#if 0
static void
can_notify(struct canpcb *canp, int errno)
{

	canp->canp_socket->so_error = errno;
	sorwakeup(canp->canp_socket);
	sowwakeup(canp->canp_socket);
}

/*
 * XXX: Yeah, I'll refactor this. 
 */
void 
can_ctlinput(int cmd, struct sockaddr *sa, void *v)
{
	struct ip *ip = v;
	struct udphdr *uh;
	void (*notify) __P((struct inpcb *, int)) = can_notify;
	int errno;

	if (sa->sa_family != AF_CAN
	 || sa->sa_len != sizeof(struct sockaddr_can))
		return NULL;
	if ((unsigned)cmd >= PRC_NCMDS)
		return NULL;
	errno = inetctlerrmap[cmd];
	if (PRC_IS_REDIRECT(cmd))
		notify = in_rtchange, ip = 0;
	else if (cmd == PRC_HOSTDEAD)
		ip = 0;
	else if (errno == 0)
		return NULL;
	if (ip) {
		uh = (struct canhdr *)((caddr_t)ip + (ip->ip_hl << 2));
		in_pcbnotify(&udbtable, satosin(sa)->sin_addr, uh->uh_dport,
		    ip->ip_src, uh->uh_sport, errno, notify);

		/* XXX mapped address case */
	} else
		can_pcbnotifyall(&cbtable, satoscan(sa)->scan_addr, errno,
		    notify);
	return NULL;
}
#endif
