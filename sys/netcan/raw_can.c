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
#include <sys/eventhandler.h>
#include <sys/module.h>
#include <sys/mbuf.h>
#include <sys/proc.h>
#include <sys/protosw.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_can.h>

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

extern int can_hashsize;

static struct canpcbinfo rcan_pcbinfo = {
	.cani_queue = TAILQ_HEAD_INITIALIZER(rcan_pcbinfo.cani_queue),
};

u_long	rcan_sendspace = 4096;		/* really max datagram size */
u_long	rcan_recvspace = 40 * (1024 + sizeof(struct sockaddr_can));
					/* 40 1K datagrams */
/*
 * Initialize raw connection block queue.
 */
 
static void
rcan_zone_change(void *tag)
{

	uma_zone_set_max(rcan_pcbinfo.cani_zone, maxsockets);
}

static int
rcan_pcb_init(void *mem, int size, int flags)
{
	struct canpcb *canp = mem;

	CANP_LOCK_INIT(canp, "canp", "rawcanp");
	return (0);
}

void
rcan_init(void)
{

	can_pcbinfo_init(&rcan_pcbinfo, "rawcan", "rawcanp", 
		rcan_pcb_init, NULL, can_hashsize, can_hashsize);
	EVENTHANDLER_REGISTER(maxsockets_change, rcan_zone_change, NULL,
	    EVENTHANDLER_PRI_ANY);	
}

/*
 * Common usrreqs.
 */

static int
rcan_attach(struct socket *so, int proto, struct thread *td)
{
	struct canpcb *canp;
	int error;

	canp = sotocanpcb(so);
	KASSERT((canp == NULL), 
		("%s: canp != NULL", __func__));

	if ((error = soreserve(so, rcan_sendspace, rcan_recvspace)) == 0) { 
		CANP_INFO_WLOCK(&rcan_pcbinfo);	
		error = can_pcballoc(so, &rcan_pcbinfo);
		CANP_INFO_WUNLOCK(&rcan_pcbinfo);
	}
	return (error);
}

static void
rcan_detach(struct socket *so)
{
	struct canpcb *canp;

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));
	
	CANP_INFO_WLOCK(&rcan_pcbinfo);	
	can_pcbdetach(canp);
	CANP_INFO_WUNLOCK(&rcan_pcbinfo);	
}

static int
rcan_bind(struct socket *so, struct sockaddr *nam, struct thread *td)
{
	struct sockaddr_can *scan;
	struct canpcb *canp;
	int error;

	scan = (struct sockaddr_can *)nam;

	if (scan->scan_len != sizeof(*scan)) {
		error = EINVAL;
		goto out;
	}
	
	if (TAILQ_EMPTY(&V_ifnet)) {
		error = EADDRNOTAVAIL;
		goto out;
	}
		
	if (scan->scan_family != AF_CAN) {
		error = EAFNOSUPPORT;
		goto out;
	}
	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	CANP_INFO_WLOCK(&rcan_pcbinfo);
	CANP_WLOCK(canp);
	error = can_pcbbind(canp, scan, td->td_ucred);
	CANP_WUNLOCK(canp);
	CANP_INFO_WUNLOCK(&rcan_pcbinfo);	
out:	
	return (error);
}

static int
rcan_connect(struct socket *so, struct sockaddr *nam, 
	struct thread *td)
{
	struct sockaddr_can *scan;
	struct canpcb *canp;
	int error;
	
	scan = (struct sockaddr_can *)nam;
	
	if (nam->sa_len != sizeof(*scan)) {
		error = EINVAL;
		goto out;
	}
		
	if (TAILQ_EMPTY(&V_ifnet)) {
		error = EADDRNOTAVAIL;
		goto out;
	}
		
	if (scan->scan_family != AF_CAN) {
		error = EAFNOSUPPORT;
		goto out;
	}
	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	CANP_INFO_WLOCK(&rcan_pcbinfo);
	error = can_pcbconnect(canp, scan);
	if (error != 0)
		soisconnected(so);
	CANP_INFO_WUNLOCK(&rcan_pcbinfo);	
out:
	return (error);
}

static int
rcan_disconnect(struct socket *so)
{
	struct canpcb *canp;
	int error;

	if (so->so_state & SS_ISCONNECTED) {
		canp = sotocanpcb(so);
		KASSERT((canp != NULL), 
			("%s: canp == NULL", __func__));

		CANP_INFO_WLOCK(&rcan_pcbinfo);
		SOCK_LOCK(so);
		so->so_state &= ~SS_ISCONNECTED;
		SOCK_UNLOCK(so);

		can_pcbdisconnect(canp);
		CANP_INFO_WUNLOCK(&rcan_pcbinfo);

		error = 0;
	} else 
		error = ENOTCONN;
				
	return (error);
}

static int
rcan_shutdown(struct socket *so)
{
	struct canpcb *canp;

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	CANP_WLOCK(canp);
	socantsendmore(so);
	CANP_WUNLOCK(canp);

	return (0);
}

static int
rcan_sockaddr(struct socket *so, struct sockaddr **nam)
{
	struct canpcb *canp;
			
	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	*nam = can_sockaddr(canp);
	
	return (0);
}

static int
rcan_send(struct socket *so, int flags, struct mbuf *m, 
	struct sockaddr *nam, struct mbuf *control, struct thread *td)
{
	struct sockaddr_can *scan;
	struct canpcb *canp;
	int error;

	scan = (struct sockaddr_can *)nam;
	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	if (control != NULL && control->m_len != 0) {
		error = EINVAL;
		goto bad;
	}

	if (m->m_len > sizeof(struct can_frame) ||
		m->m_len < offsetof(struct can_frame, can_dlc)) {
		error = EINVAL;
		goto bad;
	}

	if (m->m_len != m->m_pkthdr.len) {
		error = EINVAL;
		goto bad;
	}

	if (nam != NULL) {
		if ((so->so_state & SS_ISCONNECTED) != 0) 
			error = EISCONN;
		else {
			CANP_WLOCK(canp);
			error = can_pcbbind(canp, scan, td->td_ucred);
			CANP_WUNLOCK(canp);
		}
	} else {
		if ((so->so_state & SS_ISCONNECTED) == 0) 
			error =  EDESTADDRREQ;
		else
			error = 0;
	}
	
	if (error != 0) 
		goto bad;
	
	error = can_output(m, canp);
	
	if (nam != NULL) {
		struct sockaddr_can lscan;
		
		(void)memset(&lscan, 0, sizeof(lscan));

		lscan.scan_family = AF_CAN;
		lscan.scan_len = sizeof(lscan);

		CANP_WLOCK(canp);
		can_pcbbind(canp, &lscan, td->td_ucred);
		CANP_WUNLOCK(canp);
	}
out:
	return (error);
bad:
	m_freem(control);
	m_freem(m);
	goto out;
}

/*
 * Called by [gs]etsockopt(2). 
 *
 */
static int
rcan_getop(struct canpcb *canp, struct sockopt *sopt)
{
	int optval = 0;
	int error;

	switch (sopt->sopt_name) {
	case CAN_RAW_LOOPBACK:
		optval = (canp->canp_flags & CANP_NO_LOOPBACK) ? 0 : 1;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_RECV_OWN_MSGS: 
		optval = (canp->canp_flags & CANP_RECEIVE_OWN) ? 1 : 0;
		error = sooptcopyout(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_FILTER:
		error = sooptcopyout(sopt, canp->canp_filters, 
			sizeof(struct can_filter) * canp->canp_nfilters);
		break;
	default:
		error = ENOPROTOOPT;
		break;
	}
	return (error);
}

static int
rcan_setop(struct canpcb *canp, struct sockopt *sopt)
{
	int optval = 0;
	int error;

	switch (sopt->sopt_name) {
	case CAN_RAW_LOOPBACK:	
		error = sooptcopyin(sopt, &optval, sizeof(optval), 
			sizeof(optval));
		if (error == 0) {
			if (optval != 0) 
				canp->canp_flags &= ~CANP_NO_LOOPBACK;
			else 
				canp->canp_flags |= CANP_NO_LOOPBACK;
		}
		break;
	case CAN_RAW_RECV_OWN_MSGS: 
		error = sooptcopyin(sopt, &optval, sizeof(optval), 
			sizeof(optval));
		if (error == 0) {
			if (optval != 0) 
				canp->canp_flags |= CANP_RECEIVE_OWN;
			else 
				canp->canp_flags &= ~CANP_RECEIVE_OWN;
		}
		break;
	case CAN_RAW_FILTER:
		error = sooptcopyin(sopt, canp->canp_filters, 
			sizeof(struct can_filter) * canp->canp_nfilters, 
				sizeof(struct can_filter) * canp->canp_nfilters);
		if (error == 0) {		
			int nfilters = sopt->sopt_valsize / sizeof(struct can_filter);

			if (sopt->sopt_valsize % sizeof(struct can_filter) != 0) {
				error = EINVAL;
				break;
			}
			CANP_WLOCK(canp);
			error = can_pcbsetfilter(canp, sopt->sopt_val, nfilters);
			CANP_WUNLOCK(canp);
		}
		break;
	default:
		error = ENOPROTOOPT;
		break;
	}
	return (error);
}

int
rcan_ctloutput(struct socket *so, struct sockopt *sopt)
{
	struct canpcb *canp;
	int error;

	canp = sotocanpcb(so);

	if (sopt->sopt_level == so->so_proto->pr_protocol) { /* XXX: CANPROTO_RAW */
		switch (sopt->sopt_dir) {
		case SOPT_GET:
			error = rcan_getop(canp, sopt);
			break;
		case SOPT_SET:
			error = rcan_setop(canp, sopt);
			break;
		default:
			error = EINVAL;
			break;
		}
	} else
		error = can_ctloutput(so, sopt);
		
	return (error);
}

struct pr_usrreqs rcan_usrreqs = {
	.pru_attach = 		rcan_attach,
	.pru_detach = 		rcan_detach,
	.pru_bind = 		rcan_bind,
	.pru_connect = 		rcan_connect,
	.pru_control =		can_control,	
	.pru_disconnect = 		rcan_disconnect,
	.pru_shutdown = 		rcan_shutdown,
	.pru_sockaddr = 		rcan_sockaddr,
	.pru_send = 		rcan_send,
};
