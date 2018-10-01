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
#include <sys/mbuf.h>
#include <sys/errno.h>
#include <sys/protosw.h>
#include <sys/sockio.h>
#include <sys/socket.h>
#include <sys/socketvar.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_types.h>

#include <net/if.h>
#include <net/if_types.h>
#include <net/route.h>

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

extern int can_hashsize;

struct canpcbinfo rcan_pcbinfo;

u_long	rcan_sendspace = 4096;		/* really max datagram size */
SYSCTL_ULONG(_net_can_raw, OID_AUTO, maxdgram, CTLFLAG_RW,
    &rcan_sendspace, 0, "Maximum outgoing raw CAN frame size");

u_long	rcan_revcspace = 40 * (1024 + sizeof(struct sockaddr_can));
					/* 40 1K datagrams */
SYSCTL_ULONG(_net_can_raw, OID_AUTO, recvspace, CTLFLAG_RW,
    &rcan_recvspace, 0, "Maximum space for incoming raw CAN frames");

/*
 * Initialize raw connection block queue.
 */
 
static void
rcan_zone_change(void *tag)
{

	uma_zone_set_max(canpcbinfo.cani_zone, maxsockets);
}

static int
rcan_pcb_init(void *mem, int size, int flags)
{
	struct canpcb *canp = mem;

	CANP_LOCK_INIT(canp, "rawcanp");
	return (0);
}

void
rcan_init(void)
{

	can_pcbinit(&rcan_pcbinfo, "rawcan", "rawcanp", 
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
	struct inpcb *inp;
	int error;

	canp = sotocanpcb(so);
	KASSERT(canp == NULL, ("%s: canp != NULL", __func__));

	error = soreserve(so, rcan_sendspace, rcan_revcspace);
	if (error != 0) 
		goto out;
	
	CANP_INFO_LOCK(&rcan_pcbinfo);	
	error = can_pcballoc(so, &rcan_pcbinfo);
	CANP_INFO_UNLOCK(&rcan_pcbinfo);
out:
	return (error);
}

static void
rcan_detach(struct socket *so)
{
	struct canpcb *canp;

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	can_pcbdetach(canp);
}

static int
rcan_bind(struct socket *so, struct sockaddr *nam, struct thread *td)
{
	struct sockaddr_can *scan = (struct sockaddr_can *)nam;
	int error;

	if (nam->sa_len != sizeof(*scan)) {
		error = EINVAL;
		goto out;
	}

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	error = can_pcbbind(canp, scan, td->td_ucred);
out:	
	return (error);
}

static int
rcan_connect(struct socket *so, struct sockaddr *nam, 
	struct thread *td)
{
	struct canpcb *canp;
	int error;
	
	if (nam->sa_len != sizeof(*scan)) {
		error = EINVAL;
		goto out;
	}

	if (nam->sa_family != AF_CAN) {
		error = EAFNOSUPPORT;
		goto out;
	}
	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	error = can_pcbconnect(canp, (struct sockaddr_can *)nam);
	if (error != 0)
		soisconnected(so);
out:
	return (error);
}

static int
rcan_disconnect(struct socket *so)
{
	struct canpcb *canp;

	if ((so->so_state & SS_ISCONNECTED) == 0) 
		return (ENOTCONN);

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	so->so_state &= ~SS_ISCONNECTED;
	can_pcbdisconnect(canp);
	
	return (0);
}

static int
rcan_shutdown(struct socket *so)
{
	struct canpcb *canp;

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	CANP_LOCK(canp);
	socantsendmore(so);
	CANP_UNLOCK(canp);

	return (0);
}

static int
rcan_sockaddr(struct socket *so, struct sockaddr *nam)
{
	struct canpcb *canp;

	if (nam->sa_len != sizeof(*scan)) {
		return (EINVAL);

	if (nam->sa_family != AF_CAN) {
		return (EAFNOSUPPORT);
		
	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	can_setsockaddr(canp, (struct sockaddr_can *)nam);

	return (0);
}

static int
rcan_send(struct socket *so, struct mbuf *m, struct sockaddr *nam,
    struct mbuf *control, struct thread *td)
{
	struct canpcb *canp;
	int error;

	canp = sotocanpcb(so);
	KASSERT((canp != NULL), 
		("%s: canp == NULL", __func__));

	if (control != NULL && control->m_len != 0) {
		m_freem(control);
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
		else
			error = can_pcbbind(canp, (struct sockaddr_can *)nam, l);
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

		lscan.can_family = AF_CAN;
		lscan.can_len = sizeof(lscan);

		can_pcbbind(canp, &lscan, td->td_ucred);
	}
	
	if (error != 0)
		goto bad;
		
	return (error);
bad:
	m_freem(m);
	goto out;
}

/*
 * Called by getsockopt and setsockopt.
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
		error = sockopt_set(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_RECV_OWN_MSGS: 
		optval = (canp->canp_flags & CANP_RECEIVE_OWN) ? 1 : 0;
		error = sockopt_set(sopt, &optval, sizeof(optval));
		break;
	case CAN_RAW_FILTER:
		error = sockopt_set(sopt, canp->canp_filters,
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
		error = sockopt_getint(sopt, &optval);
		if (error == 0) {
			if (optval) {
				canp->canp_flags &= ~CANP_NO_LOOPBACK;
			} else {
				canp->canp_flags |= CANP_NO_LOOPBACK;
			}
		}
		break;
	case CAN_RAW_RECV_OWN_MSGS: 
		error = sockopt_getint(sopt, &optval);
		if (error == 0) {
			if (optval) {
				canp->canp_flags |= CANP_RECEIVE_OWN;
			} else {
				canp->canp_flags &= ~CANP_RECEIVE_OWN;
			}
		}
		break;
	case CAN_RAW_FILTER:
		{
		int nfilters = sopt->sopt_size / sizeof(struct can_filter);

		if (sopt->sopt_size % sizeof(struct can_filter) != 0) {
			error = EINVAL;
			break;
		}
		CANP_LOCK(canp);
		error = can_pcbsetfilter(canp, sopt->sopt_data, nfilters);
		CANP_UNLOCK(canp);
		break;
		}
	default:
		error = ENOPROTOOPT;
		break;
	}
	return (error);
}

int
rcan_ctloutput(struct socket *so, struct sockopt *sopt)
{
	struct canpcb *canp = sotocanpcb(so);
	int error;

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
	.pr_attach = 		rcan_attach,
	.pr_detach = 		rcan_detach,
	.pr_bind = 		rcan_bind,
	.pr_listen = 		rcan_listen,
	.pr_connect = 		rcan_connect,
	.pr_disconnect = 		rcan_disconnect,
	.pr_shutdown = 		rcan_shutdown,
	.pr_sockaddr = 		rcan_sockaddr,
	.pr_send = 		rcan_send,
	.pr_sendoob = 		rcan_sendoob,
	.pr_purgeif = 		rcan_purgeif,
};
