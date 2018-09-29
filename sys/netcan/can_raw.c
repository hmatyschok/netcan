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

struct canpcb canpcb;

struct	canpcbtable cbtable;

struct ifqueue	canintrq;
int	canqmaxlen = IFQ_MAXLEN;

int can_copy_output = 0;
int can_output_cnt = 0;
struct mbuf *can_lastout;

int	can_sendspace = 4096;		/* really max datagram size */
int	can_recvspace = 40 * (1024 + sizeof(struct sockaddr_can));
					/* 40 1K datagrams */
#ifndef CANHASHSIZE
#define	CANHASHSIZE	128
#endif
int	canhashsize = CANHASHSIZE;

static int can_output(struct mbuf *, struct canpcb *);

static int can_control(struct socket *, u_long, void *, struct ifnet *);


/*
 * Initialize raw connection block queue.
 * 
 * XXX: Wrong place.
 */
 
static void
can_zone_change(void *tag)
{

	uma_zone_set_max(canpcbinfo.cani_zone, maxsockets);
}

static int
can_canpcb_init(void *mem, int size, int flags)
{
	struct canpcb *canp = mem;

	CANP_LOCK_INIT(canp, "canp");
	return (0);
}

void
can_init(void)
{
	canintrq.ifq_maxlen = canqmaxlen;
	IFQ_LOCK_INIT(&canintrq);
	can_pcbinit(&cbtable, canhashsize, canhashsize);
}


static int
can_attach(struct socket *so, int proto)
{
	int error;

	KASSERT(sotocanpcb(so) == NULL);

	/* Assign the lock (must happen even if we will error out). */
	sosetlock(so);

#ifdef MBUFTRACE
	so->so_mowner = &can_mowner;
	so->so_rcv.sb_mowner = &can_rx_mowner;
	so->so_snd.sb_mowner = &can_tx_mowner;
#endif
	if (so->so_snd.sb_hiwat == 0 || so->so_rcv.sb_hiwat == 0) {
		error = soreserve(so, can_sendspace, can_recvspace);
		if (error) {
			return error;
		}
	}

	error = can_pcballoc(so, &cbtable);
	if (error) {
		return error;
	}
	KASSERT(solocked(so));

	return error;
}

static void
can_detach(struct socket *so)
{
	struct canpcb *canp;

	KASSERT(solocked(so));
	canp = sotocanpcb(so);
	can_pcbdetach(canp);
}

static int
can_accept(struct socket *so, struct sockaddr *nam)
{
	KASSERT(solocked(so));

	panic("can_accept");

	return EOPNOTSUPP;
}

static int
can_bind(struct socket *so, struct sockaddr *nam, struct lwp *l)
{
	struct canpcb *canp = sotocanpcb(so);
	struct sockaddr_can *scan = (struct sockaddr_can *)nam;

	KASSERT(solocked(so));
	KASSERT(nam != NULL);

	return can_pcbbind(canp, scan, l);
}

static int
can_listen(struct socket *so, struct lwp *l)
{
	KASSERT(solocked(so));

	return EOPNOTSUPP;
}

static int
can_connect(struct socket *so, struct sockaddr *nam, struct lwp *l)
{
	struct canpcb *canp = sotocanpcb(so);
	int error = 0;

	KASSERT(solocked(so));
	KASSERT(canp != NULL);
	KASSERT(nam != NULL);

	error = can_pcbconnect(canp, (struct sockaddr_can *)nam);
	if (! error)
		soisconnected(so);
	return error;
}

static int
can_connect2(struct socket *so, struct socket *so2)
{
	KASSERT(solocked(so));

	return EOPNOTSUPP;
}

static int
can_disconnect(struct socket *so)
{
	struct canpcb *canp = sotocanpcb(so);

	KASSERT(solocked(so));
	KASSERT(canp != NULL);

	/*soisdisconnected(so);*/
	so->so_state &= ~SS_ISCONNECTED;	/* XXX */
	can_pcbdisconnect(canp);
	return 0;
}

static int
can_shutdown(struct socket *so)
{
	KASSERT(solocked(so));

	socantsendmore(so);
	return 0;
}

static int
can_abort(struct socket *so)
{
	KASSERT(solocked(so));

	panic("can_abort");

	return EOPNOTSUPP;
}

static int
can_peeraddr(struct socket *so, struct sockaddr *nam)
{
	KASSERT(solocked(so));
	KASSERT(sotocanpcb(so) != NULL);
	KASSERT(nam != NULL);

	return EOPNOTSUPP;
}

static int
can_sockaddr(struct socket *so, struct sockaddr *nam)
{
	KASSERT(solocked(so));
	KASSERT(sotocanpcb(so) != NULL);
	KASSERT(nam != NULL);

	can_setsockaddr(sotocanpcb(so), (struct sockaddr_can *)nam);

	return 0;
}

static int
can_rcvd(struct socket *so, int flags, struct lwp *l)
{
	KASSERT(solocked(so));

	return EOPNOTSUPP;
}

static int
can_recvoob(struct socket *so, struct mbuf *m, int flags)
{
	KASSERT(solocked(so));

	return EOPNOTSUPP;
}

static int
can_send(struct socket *so, struct mbuf *m, struct sockaddr *nam,
    struct mbuf *control, struct lwp *l)
{
	struct canpcb *canp = sotocanpcb(so);
	int error = 0;
	int s;

	if (control && control->m_len) {
		m_freem(control);
		error = EINVAL;
		goto err;
	}
	if (m->m_len > sizeof(struct can_frame) ||
	   m->m_len < offsetof(struct can_frame, can_dlc)) {
		error = EINVAL;
		goto err;
	}

	/* we expect all data in the first mbuf */
	KASSERT((m->m_flags & M_PKTHDR) != 0);
	KASSERT(m->m_len == m->m_pkthdr.len);

	if (nam) {
		if ((so->so_state & SS_ISCONNECTED) != 0) {
			error = EISCONN;
			goto err;
		}
		s = splnet();
		error = can_pcbbind(canp, (struct sockaddr_can *)nam, l);
		if (error) {
			splx(s);
			goto err;
		}
	} else {
		if ((so->so_state & SS_ISCONNECTED) == 0) {
			error =  EDESTADDRREQ;
			goto err;
		}
	}
	error = can_output(m, canp);
	if (nam) {
		struct sockaddr_can lscan;
		memset(&lscan, 0, sizeof(lscan));
		lscan.can_family = AF_CAN;
		lscan.can_len = sizeof(lscan);
		can_pcbbind(canp, &lscan, l);
	}
	if (error)
		goto err;
	return 0;

err:
	m_freem(m);
	return error;
}

static int
can_sendoob(struct socket *so, struct mbuf *m, struct mbuf *control)
{
	KASSERT(solocked(so));

	m_freem(m);
	m_freem(control);

	return EOPNOTSUPP;
}

#if 0
int
can_usrreq(struct socket *so, int req, struct mbuf *m, struct mbuf *nam,
	   struct mbuf *control, struct lwp *l)
{
	struct canpcb *canp;
	int s;
	int error = 0;

	if (req == PRU_CONTROL)
		 return (can_control(so, (long)m, nam,
		     (struct ifnet *)control));

	if (req == PRU_PURGEIF) {
#if 0
		can_pcbpurgeif0(&udbtable, (struct ifnet *)control);
		can_purgeif((struct ifnet *)control);
		can_pcbpurgeif(&udbtable, (struct ifnet *)control);
#endif
		return (0);
	}

	s = splsoftnet();
	canp = sotocanpcb(so);
#ifdef DIAGNOSTIC
	if (req != PRU_SEND && req != PRU_SENDOOB && control)
		panic("can_usrreq: unexpected control mbuf");
#endif
	if (canp == 0 && req != PRU_ATTACH) {
		printf("can_usrreq: no pcb %p %d\n", canp, req);
		error = EINVAL;
		goto release;
	}

	/*
	 * Note: need to block can_input while changing
	 * the can pcb queue and/or pcb addresses.
	 */
	switch (req) {

	  case PRU_ATTACH:
	      if (canp != 0) {
			 error = EISCONN;
			 break;
		 }
#ifdef MBUFTRACE
		so->so_mowner = &can_mowner;
		so->so_rcv.sb_mowner = &can_rx_mowner;
		so->so_snd.sb_mowner = &can_tx_mowner;
#endif
		if (so->so_snd.sb_hiwat == 0 || so->so_rcv.sb_hiwat == 0) {
			error = soreserve(so, can_sendspace, can_recvspace);
			if (error)
				break;
		}
		error = can_pcballoc(so, &cbtable);
		if (error)
			break;
		canp = sotocanpcb(so);
#if 0
		inp->inp_ip.ip_ttl = ip_defttl;
#endif
		break;

	case PRU_DETACH:
		can_pcbdetach(canp);
		break;

	case PRU_BIND:
		error = can_pcbbind(canp, nam, l);
		break;

	case PRU_LISTEN:
		error = EOPNOTSUPP;
		break;

	case PRU_CONNECT:
		error = can_pcbconnect(canp, nam);
		if (error)
			break;
		soisconnected(so);
		break;

	case PRU_CONNECT2:
		error = EOPNOTSUPP;
		break;

	case PRU_DISCONNECT:
		/*soisdisconnected(so);*/
		so->so_state &= ~SS_ISCONNECTED;	/* XXX */
		can_pcbdisconnect(canp);
		can_pcbstate(canp, CANP_BOUND);		/* XXX */
		break;

	case PRU_SHUTDOWN:
		socantsendmore(so);
		break;

	case PRU_RCVD:
		error = EOPNOTSUPP;
		break;

	case PRU_SEND:
		break;

	case PRU_SENSE:
		/*
		 * stat: don't bother with a blocksize.
		 */
		splx(s);
		return (0);

	case PRU_RCVOOB:
		error =  EOPNOTSUPP;
		break;

	case PRU_SENDOOB:
		m_freem(control);
		m_freem(m);
		error =  EOPNOTSUPP;
		break;

	case PRU_SOCKADDR:

		break;

	case PRU_PEERADDR:
		error =  EOPNOTSUPP;
		break;

	default:
		panic("can_usrreq");
	}

release:
	splx(s);
	return (error);
}
#endif

/*
 * Called by getsockopt and setsockopt.
 *
 */
static int
can_raw_getop(struct canpcb *canp, struct sockopt *sopt)
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
can_raw_setop(struct canpcb *canp, struct sockopt *sopt)
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
		mutex_enter(&canp->canp_mtx);
		error = can_pcbsetfilter(canp, sopt->sopt_data, nfilters);
		mutex_exit(&canp->canp_mtx);
		break;
		}
	default:
		error = ENOPROTOOPT;
		break;
	}
	return (error);
}

int
can_raw_ctloutput(struct socket *so, struct sockopt *sopt)
{
	struct canpcb *canp = sotocanpcb(so);
	int error;

	if (sopt->sopt_level == so->so_proto->pr_protocol) { /* XXX: CANPROTO_RAW */
		switch (sopt->sopt_dir) {
		case SOPT_GET:
			error = can_raw_getop(canp, sopt);
			break;
		case SOPT_SET:
			error = can_raw_setop(canp, sopt);
			break;
		default:
			error = EINVAL;
			break;
		}
	} else
		error = can_ctloutput(so, sopt);
		
	return (error);
}

struct pr_usrreqs can_usrreqs = {
	.pr_attach	= can_attach,
	.pr_detach	= can_detach,
	.pr_accept	= can_accept,
	.pr_bind	= can_bind,
	.pr_listen	= can_listen,
	.pr_connect	= can_connect,
	.pr_connect2	= can_connect2,
	.pr_disconnect	= can_disconnect,
	.pr_shutdown	= can_shutdown,
	.pr_abort	= can_abort,
	.pr_stat	= can_stat,
	.pr_peeraddr	= can_peeraddr,
	.pr_sockaddr	= can_sockaddr,
	.pr_rcvd	= can_rcvd,
	.pr_recvoob	= can_recvoob,
	.pr_send	= can_send,
	.pr_sendoob	= can_sendoob,
	.pr_purgeif	= can_purgeif,
};
