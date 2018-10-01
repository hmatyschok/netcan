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
#if 0
struct canpcb canrawpcb;
#endif

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


/*
 * Generic control operations (ioctl's).
 */
static int
can_get_netlink(struct ifnet *ifp, struct ifdrv *ifd)
{
	struct canif_softc *csc = ifp->if_softc;

	if (ifp->if_dlt != DLT_CAN_SOCKETCAN || csc == NULL)
		return EOPNOTSUPP;

	switch(ifd->ifd_cmd) {
	case CANGLINKTIMECAP:
		if (ifd->ifd_len != sizeof(struct can_link_timecaps))
			return EINVAL;
		return copyout(&csc->csc_timecaps, ifd->ifd_data, ifd->ifd_len);
	case CANGLINKTIMINGS:
		if (ifd->ifd_len != sizeof(struct can_link_timings))
			return EINVAL;
		return copyout(&csc->csc_timings, ifd->ifd_data, ifd->ifd_len);
	case CANGLINKMODE:
		if (ifd->ifd_len != sizeof(uint32_t))
			return EINVAL;
		return copyout(&csc->csc_linkmodes, ifd->ifd_data, ifd->ifd_len);
	}
	return EOPNOTSUPP;
}

static int
can_set_netlink(struct ifnet *ifp, struct ifdrv *ifd)
{
	struct canif_softc *csc = ifp->if_softc;
	uint32_t mode;
	int error;

	if (ifp->if_dlt != DLT_CAN_SOCKETCAN || csc == NULL)
		return EOPNOTSUPP;

	error = kauth_authorize_network(curlwp->l_cred,
		    KAUTH_NETWORK_INTERFACE,
		    KAUTH_REQ_NETWORK_INTERFACE_SETPRIV, ifp,
	            (void *)SIOCSDRVSPEC, NULL);
	if (error != 0)
		return error;

	if ((ifp->if_flags & IFF_UP) != 0) {
		return EBUSY;
	}

	switch(ifd->ifd_cmd) {
	case CANSLINKTIMINGS:
		if (ifd->ifd_len != sizeof(struct can_link_timings))
			return EINVAL;
		return copyin(ifd->ifd_data, &csc->csc_timings, ifd->ifd_len);

	case CANSLINKMODE:
	case CANCLINKMODE:
		if (ifd->ifd_len != sizeof(uint32_t))
			return EINVAL;
		error = copyin(ifd->ifd_data, &mode, ifd->ifd_len);
		if (error)
			return error;
		if ((mode & csc->csc_timecaps.cltc_linkmode_caps) != mode)
			return EINVAL;
		/* XXX locking */
		if (ifd->ifd_cmd == CANSLINKMODE)
			csc->csc_linkmodes |= mode;
		else
			csc->csc_linkmodes &= ~mode;
		return 0;
	}
	return EOPNOTSUPP;
}

/* ARGSUSED */
static int
can_control(struct socket *so, u_long cmd, caddr_t data, struct ifnet *ifp,
    struct thread *td)
{
	struct can_ifreq *cfr = (struct can_ifreq *)data;
	int error = 0;

	if (ifp == NULL)
		error = EADDRNOTAVAIL;
		goto out;
	}

	switch (cmd) {
	case SIOCGDRVSPEC:
		return can_get_netlink(ifp, cfr);
	case SIOCSDRVSPEC:
		return can_set_netlink(ifp, cfr);
	default:
		if (ifp->if_ioctl != NULL)
			
			
			
			return (EOPNOTSUPP);
		return ((*ifp->if_ioctl)(ifp, cmd, data));
	}
out:	
	return (error);
}

static int
can_purgeif(struct socket *so, struct ifnet *ifp)
{
	return 0;
}

void
can_ifattach(struct ifnet *ifp)
{
	if_attach(ifp);
	ifp->if_mtu = sizeof(struct can_frame);
	ifp->if_type = IFT_OTHER;
	ifp->if_hdrlen = 0;
	ifp->if_addrlen = 0;
	ifp->if_dlt = DLT_CAN_SOCKETCAN;
	ifp->if_output = NULL; /* unused */
	IFQ_SET_READY(&ifp->if_snd);
	if_alloc_sadl(ifp);
	bpf_attach(ifp, DLT_CAN_SOCKETCAN, 0);
}

void
can_ifdetach(struct ifnet *ifp)
{
	bpf_detach(ifp);
	if_detach(ifp);
}

void
can_ifinit_timings(struct canif_softc *csc)
{
	/* uninitialized parameters is all-one */
	memset(&csc->csc_timings, 0xff, sizeof(struct can_link_timings));
}

/*
 * cleanup mbuf tag, keeping the PACKET_TAG_SO tag
 */
void
can_mbuf_tag_clean(struct mbuf *m)
{
	struct m_tag *sotag;

	sotag = m_tag_find(m, PACKET_TAG_SO, NULL);
	if (sotag)
		m_tag_unlink(m, sotag);

	m_tag_delete_nonpersistent(m);
	if (sotag)
		m_tag_prepend(m, sotag);
}

/*
 * Process a received CAN frame
 * the packet is in the mbuf chain m with
 * the CAN header.
 */
void
can_input(struct ifnet *ifp, struct mbuf *m)
{
	struct ifqueue *inq;

	if ((ifp->if_flags & IFF_UP) == 0) {
		m_freem(m);
		return;
	}

	inq = &canintrq;
	
	IFQ_LOCK(inq);
	if (IF_QFULL(inq)) {
		IF_DROP(inq);
		IFQ_UNLOCK(inq);
		m_freem(m);
	} else {
		IF_ENQUEUE(inq, m);
		IFQ_UNLOCK(inq);
		ifp->if_ipackets++;
		ifp->if_ibytes += m->m_pkthdr.len;
		schednetisr(NETISR_CAN);
	}
}

void
canintr(void)
{
	int		rcv_ifindex;
	struct mbuf    *m;

	struct sockaddr_can from;
	struct canpcb   *canp;
	struct m_tag	*sotag;
	struct canpcb	*sender_canp;

	mutex_enter(softnet_lock);
	for (;;) {
		IFQ_LOCK(&canintrq);
		IF_DEQUEUE(&canintrq, m);
		IFQ_UNLOCK(&canintrq);

		if (m == NULL)	/* no more queued packets */
			break;

#if 0
		m_claim(m, &can_rx_mowner);
#endif
		sotag = m_tag_find(m, PACKET_TAG_SO, NULL);
		if (sotag) {
			sender_canp = *(struct canpcb **)(sotag + 1);
			m_tag_delete(m, sotag);
			KASSERT(sender_canp != NULL);
			/* if the sender doesn't want loopback, don't do it */
			if ((sender_canp->canp_flags & CANP_NO_LOOPBACK) != 0) {
				m_freem(m);
				canp_unref(sender_canp);
				continue;
			}
		} else {
			sender_canp = NULL;
		}
		memset(&from, 0, sizeof(struct sockaddr_can));
		rcv_ifindex = m->m_pkthdr.rcvif_index;
		from.can_ifindex = rcv_ifindex;
		from.can_len = sizeof(struct sockaddr_can);
		from.can_family = AF_CAN;

		TAILQ_FOREACH(canp, &cbtable.canpt_queue, canp_queue) {
			struct mbuf *mc;

			mutex_enter(&canp->canp_mtx);
			/* skip if we're detached */
			if (canp->canp_state == CANP_DETACHED) {
				mutex_exit(&canp->canp_mtx);
				continue;
			}

			/* don't loop back to sockets on other interfaces */
			if (canp->canp_ifp != NULL &&
			    canp->canp_ifp->if_index != rcv_ifindex) {
				mutex_exit(&canp->canp_mtx);
				continue;
			}
			/* don't loop back to myself if I don't want it */
			if (canp == sender_canp &&
			    (canp->canp_flags & CANP_RECEIVE_OWN) == 0) {
				mutex_exit(&canp->canp_mtx);
				continue;
			}

			/* skip if the accept filter doen't match this pkt */
			if (!can_pcbfilter(canp, m)) {
				mutex_exit(&canp->canp_mtx);
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
			if (sbappendaddr(&canp->canp_socket->so_rcv,
					 (struct sockaddr *) &from, mc,
					 (struct mbuf *) 0) == 0) {
				soroverflow(canp->canp_socket);
				m_freem(mc);
			} else
				sorwakeup(canp->canp_socket);
			mutex_exit(&canp->canp_mtx);
			if (m == NULL)
				break;
		}
		if (sender_canp) {
			canp_unref(sender_canp);
		}
		/* If it didn't go anywhere just delete it */
		if (m) {
			m_freem(m);
		}
	}
	mutex_exit(softnet_lock);
}

void
can_bpf_mtap(struct ifnet *ifp, struct mbuf *m, bool do_softint)
{
	/* bpf wants the CAN id in network byte order */
	struct can_frame *cf;
	canid_t oid;

	cf = mtod(m, struct can_frame *);
	oid = cf->can_id;
	cf->can_id = htonl(oid);
	if (do_softint)
		bpf_mtap_softint(ifp, m);
	else
		bpf_mtap(ifp, m);
	cf->can_id = oid;
}

#if 0
static void
can_notify(struct canpcb *canp, int errno)
{

	canp->canp_socket->so_error = errno;
	sorwakeup(canp->canp_socket);
	sowwakeup(canp->canp_socket);
}

void *
can_ctlinput(int cmd, struct sockaddr *sa, void *v)
{
	struct ip *ip = v;
	struct canhdr *uh;
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
