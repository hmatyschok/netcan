/*	$NetBSD: can_pcb.c,v 1.5.2.1 2017/06/15 05:32:35 snj Exp $	*/

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
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/malloc.h>
#include <sys/mutex.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/socketvar.h>

#include <net/if.h>
#include <net/route.h>

#include <netcan/can.h>
#include <netcan/can_var.h>
#include <netcan/can_pcb.h>

#define	CANPCBHASH_BIND(cani, ifindex) \
	&(cani)->cani_bindhashtbl[ \
	    (ifindex) & (cani)->cani_bindhash]
#define	CANPCBHASH_CONNECT(cani, ifindex) \
	&(cani)->cani_connecthashtbl[ \
	    (ifindex) & (cani)->cani_bindhash]
	    
/*
 * Initialize protocol specific software-context.
 */ 
void 
can_pcbinfo_init(struct canpcbinfo *pcbinfo, const char *name, 
	const char *zone_name, uma_init canpcbzone_init, 
	uma_fini canpcbzone_fini, int bindhash_nelements, 
	int connecthash_nelements)
{
	CANP_INFO_LOCK_INIT(pcbinfo, name);
	
	pcbinfo->cani_zone = uma_zcreate(zone_name, sizeof(struct canpcb),
		NULL, canpcbzone_init, can_canpcb_init, canpcbzone_fini,
			 UMA_ALIGN_PTR, 0);
	uma_zone_set_max(pcbinfo->cani_zone, maxsockets);
	uma_zone_set_warning(pcbinfo->cani_zone,
			"kern.ipc.maxsockets limit reached");
	TAILQ_INIT(&pcbinfo->cani_queue);
	pcbinfo->cani_bindhashtbl = hashinit(bindhash_nelements, 
		M_PCB, &pcbinfo->cani_bindhash);
	pcbinfo->cani_connecthashtbl = hashinit(connecthash_nelements, 
		M_PCB, &pcbinfo->cani_connecthash);
}

int
can_pcballoc(struct socket *so, struct canpcbinfo *pcbinfo)
{
	struct canpcb *canp;
	struct can_filter *can_init_filter;

	can_init_filter = malloc(sizeof(struct can_filter), M_TEMP, 
		M_NOWAIT);
	if (can_init_filter == NULL)
		return (ENOMEM);

	can_init_filter->can_id = 0;
	can_init_filter->can_mask = 0; /* accept all by default */

	canp = uma_zalloc(pcbinfo->cani_zone, M_NOWAIT);
	if (canp == NULL) {
		free(can_init_filter, M_TEMP);
		return (ENOBUFS);
	}
	canp->canp_pcbinfo = pcbinfo;
	canp->canp_socket = so;
	canp->canp_filters = can_init_filter;
	canp->canp_nfilters = 1;
	
	mtx_init(&canp->canp_mtx, MUTEX_DEFAULT, IPL_NET);
	canp->canp_refcount = 1;

	so->so_pcb = canp;
	CANP_INFO_LOCK(pcbinfo);
	TAILQ_INSERT_HEAD(&pcbinfo->cani_queue, canp, canp_queue);
	can_pcbstate(canp, CANP_ATTACHED);
	CANP_INFO_UNLOCK(pcbinfo);

	return (0);
}

int
can_pcbbind(struct canpcb *canp, struct sockaddr_can *scan, 
	struct ucred *cred)
{
	
	if (scan->can_family != AF_CAN)
		return (EAFNOSUPPORT);

	CANP_LOCK(canp);

	if (scan->can_ifindex != 0) {
		canp->canp_ifp = if_byindex(scan->can_ifindex);
		if (canp->canp_ifp == NULL ||
		    canp->canp_ifp->if_dlt != DLT_CAN_SOCKETCAN) {
			canp->canp_ifp = NULL;
			CANP_UNLOCK(canp);
			return (EADDRNOTAVAIL);
		}
		soisconnected(canp->canp_socket);
	} else {
		canp->canp_ifp = NULL;
		canp->canp_socket->so_state &= ~SS_ISCONNECTED;	/* XXX: No! */
	}
	can_pcbstate(canp, CANP_BOUND);
	CANP_UNLOCK(canp);
	return 0;
}

/*
 * Connect from a socket to a specified address.
 * 
 * XXX: D'oh, I'll refactor this.
 */
int
can_pcbconnect(struct canpcb *canp, struct sockaddr_can *scan)
{
#if 0
	struct sockaddr_can *ifaddr = NULL;
	int error;
#endif

	if (scan->can_family != AF_CAN)
		return (EAFNOSUPPORT);
#if 0
	CANP_LOCK(canp);
	(void)memcpy(&canp->canp_dst, scan, sizeof(struct sockaddr_can));
	can_pcbstate(canp, CANP_CONNECTED);
	CANP_UNLOCK(canp);
	return 0;
#endif
	return EOPNOTSUPP;
}

void
can_pcbdisconnect(struct canpcb *canp)
{
	CANP_LOCK(canp);
	can_pcbstate(canp, CANP_DETACHED);
	CANP_UNLOCK(canp);
	if (canp->canp_socket->so_state & SS_NOFDREF)
		can_pcbdetach(canp);
}

/*
 * XXX: I'll refactor this.
 */
void
can_pcbdetach(struct canpcb *canp)
{
	struct socket *so;

	so = canp->canp_socket;
		
	so->so_pcb = NULL;

	CANP_LOCK(canp);
	can_pcbstate(canp, CANP_DETACHED);
	can_pcbsetfilter(canp, NULL, 0);
	CANP_UNLOCK(canp);
	TAILQ_REMOVE(&canp->canp_pcbinfo->cani_queue, canp, canp_queue);
	sofree(so); /* XXX */
	canp_unref(canp);
}

void
canp_ref(struct canpcb *canp)
{
	KASSERT(mutex_owned(&canp->canp_mtx));
	canp->canp_refcount++;
}

void
canp_unref(struct canpcb *canp)
{
	CANP_LOCK(canp);
	canp->canp_refcount--;
	KASSERT(canp->canp_refcount >= 0);
	if (canp->canp_refcount > 0) {
		CANP_UNLOCK(canp);
		return;
	}
	CANP_UNLOCK(canp);
	mtx_destroy(&canp->canp_mtx);
	pool_put(&canpcb_pool, canp);
}

void
can_setsockaddr(struct canpcb *canp, struct sockaddr_can *scan)
{

	CANP_LOCK(canp);
	memset(scan, 0, sizeof (*scan));
	scan->can_family = AF_CAN;
	scan->can_len = sizeof(*scan);
	if (canp->canp_ifp) 
		scan->can_ifindex = canp->canp_ifp->if_index;
	else
		scan->can_ifindex = 0;
	CANP_UNLOCK(canp);
}

int
can_pcbsetfilter(struct canpcb *canp, struct can_filter *fp, int nfilters)
{

	struct can_filter *newf;
	KASSERT(mutex_owned(&canp->canp_mtx));

	if (nfilters > 0) {
		newf = malloc(sizeof(struct can_filter) * nfilters, 
			M_TEMP, M_WAITOK);
		(void)memcpy(newf, fp, sizeof(struct can_filter) * nfilters);
	} else {
		newf = NULL;
	}
	if (canp->canp_filters != NULL) {
		free(canp->canp_filters, M_TEMP);
	}
	canp->canp_filters = newf;
	canp->canp_nfilters = nfilters;
	return 0;
}



#if 0
/*
 * Pass some notification to all connections of a protocol
 * associated with address dst.  The local address and/or port numbers
 * may be specified to limit the search.  The "usual action" will be
 * taken, depending on the ctlinput cmd.  The caller must filter any
 * cmds that are uninteresting (e.g., no error in the map).
 * Call the protocol specific routine (if any) to report
 * any errors for each matching socket.
 *
 * Must be called at splsoftnet. <-- XXX: NO!!11!
 */
int
can_pcbnotify(struct canpcbinfo *pcbinfo, u_int32_t faddr, u_int32_t laddr,
    int errno, void (*notify)(struct canpcb *, int))
{
	struct canpcbhead *head;
	struct canpcb *canp, *ncanp;
	int nmatch;

	if (faddr == 0 || notify == 0)
		return (0);

	nmatch = 0;
	head = CANPCBHASH_CONNECT(pcbinfo, faddr, laddr);
	for (canp = LIST_FIRST(head); canp != NULL; canp = ncanp) {
		ncanp = LIST_NEXT(canp, canp_hash);
		if (canp->canp_faddr == faddr &&
		    canp->canp_laddr == laddr) {
			(*notify)(canp, errno);
			nmatch++;
		}
	}
	return (nmatch);
}

void
can_pcbnotifyall(struct canpcbinfo *pcbinfo, u_int32_t faddr, int errno,
    void (*notify)(struct canpcb *, int))
{
	struct canpcb *canp, *ncanp;

	if (faddr == 0 || notify == 0)
		return;

	TAILQ_FOREACH_SAFE(canp, &pcbinfo->cani_queue, canp_queue, ncanp) {
		if (canp->canp_faddr == faddr)
			(*notify)(canp, errno);
	}
}
#endif

#if 0

/*
 * XXX: Wrong place for doing that.
 */
void
can_pcbpurgeif0(struct canpcbinfo *pcbinfo, struct ifnet *ifp)
{
	struct canpcb *canp, *ncanp;
	struct ip_moptions *imo;
	int i, gap;

}

/*
 * XXX: Wrong place for doing that.
 */
void
can_pcbpurgeif(struct canpcbinfo *pcbinfo, struct ifnet *ifp)
{
	struct canpcb *canp, *ncanp;


}
#endif



void
can_pcbstate(struct canpcb *canp, int state)
{
	int ifindex = canp->canp_ifp ? canp->canp_ifp->if_index : 0;

	CANP_LOCK_ASSERT(cani);

	if (canp->canp_state > CANP_ATTACHED)
		LIST_REMOVE(canp, canp_hash);

	switch (state) {
	case CANP_BOUND:
		LIST_INSERT_HEAD(CANPCBHASH_BIND(canp->canp_pcbinfo,
		    ifindex), canp, canp_hash);
		break;
	case CANP_CONNECTED:
		LIST_INSERT_HEAD(CANPCBHASH_CONNECT(canp->canp_pcbinfo,
		    ifindex), canp, canp_hash);
		break;
	}

	canp->canp_state = state;
}

/*
 * check mbuf against socket accept filter.
 * returns true if mbuf is accepted, false otherwise
 *
 * XXX: I'll refactor this.
 */
int
can_pcbfilter(struct canpcb *canp, struct mbuf *m)
{
	int i;
	struct can_frame *fmp;
	struct can_filter *fip;

	CANP_LOCK_ASSERT(cani);

	fmp = mtod(m, struct can_frame *);
	for (i = 0; i < canp->canp_nfilters; i++) {
		fip = &canp->canp_filters[i];
		if ((fmp->can_id & fip->can_mask) == fip->can_id)
			return 1;
	}
	/* no match */
	return 0;
}
