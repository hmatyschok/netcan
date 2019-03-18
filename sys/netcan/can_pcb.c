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
#include <net/if_types.h>
#include <net/if_can.h>

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

extern struct rwlock can_pcbinfo_lock;
extern struct canpcbinfo_head can_pcbinfo_tbl;

#define	CANPCBHASH_BIND(cani, ifindex) \
	&(cani)->cani_bindhashtbl[ \
	    (ifindex) & (cani)->cani_bindhash]
#define	CANPCBHASH_CONNECT(cani, ifindex) \
	&(cani)->cani_connecthashtbl[ \
	    (ifindex) & (cani)->cani_bindhash]

#ifndef CAN_HASHSIZE
#define	CAN_HASHSIZE	128
#endif
int	can_hashsize = CAN_HASHSIZE;
	    
/*
 * Initialize protocol specific software-context.
 */ 
void 
can_pcbinfo_init(struct canpcbinfo *pcbinfo, const char *name, 
	const char *zone_name, uma_ctor canpcb_ctor, 
	uma_dtor canpcb_dtor, int bindhash_nelements, 
	int connecthash_nelements)
{
	CANP_INFO_LOCK_INIT(pcbinfo, name);
	
	TAILQ_INIT(&pcbinfo->cani_queue);
	pcbinfo->cani_bindhashtbl = hashinit(bindhash_nelements, 
		M_PCB, &pcbinfo->cani_bindhash);
	pcbinfo->cani_connecthashtbl = hashinit(connecthash_nelements, 
		M_PCB, &pcbinfo->cani_connecthash);
	pcbinfo->cani_zone = uma_zcreate(zone_name, sizeof(struct canpcb),
		canpcb_ctor, canpcb_dtor, NULL, NULL, UMA_ALIGN_PTR, 0);
	uma_zone_set_max(pcbinfo->cani_zone, maxsockets);
	uma_zone_set_warning(pcbinfo->cani_zone,
			"kern.ipc.maxsockets limit reached");
	
	rw_wlock(&can_pcbinfo_lock);
	TAILQ_INSERT_HEAD(&can_pcbinfo_tbl, pcbinfo, cani_next);		
	rw_wunlock(&can_pcbinfo_lock);
}

int
can_pcballoc(struct socket *so, struct canpcbinfo *pcbinfo)
{
	struct can_filter *can_init_filter;
	struct canpcb *canp;
	int error = 0;

	can_init_filter = malloc(sizeof(struct can_filter), M_TEMP, M_NOWAIT);
	if (can_init_filter == NULL) {
		error = ENOMEM;
		goto out;
	}

	can_init_filter->cf_id = 0;
	can_init_filter->cf_mask = 0; /* accept all by default */

	if ((canp = uma_zalloc(pcbinfo->cani_zone, M_NOWAIT)) == NULL) {
		free(can_init_filter, M_TEMP);
		error = ENOBUFS;
		goto out;
	}
	canp->canp_pcbinfo = pcbinfo;
	canp->canp_so = so;
	canp->canp_filters = can_init_filter;
	canp->canp_nfilters = 1;
	canp->canp_refcount = 1;

	so->so_pcb = canp;
	
	CANP_INFO_LOCK(pcbinfo);
	TAILQ_INSERT_HEAD(&pcbinfo->cani_queue, canp, canp_queue);
	CANP_INFO_UNLOCK(pcbinfo);
	
	CANP_LOCK(canp);
	can_pcbstate(canp, CANP_ATTACHED);
	CANP_UNLOCK(canp);
out:
	return (error);
}

int
can_pcbbind(struct canpcb *canp, struct sockaddr_can *scan, 
	struct ucred *cred)
{
	int error = 0;

	CANP_LOCK_ASSERT(canp);

	if (scan->scan_ifindex != 0) {
		canp->canp_ifp = ifnet_byindex(scan->scan_ifindex);
		if (canp->canp_ifp == NULL || 
				canp->canp_ifp->if_type != IFT_CAN) { 
			canp->canp_ifp = NULL;
			error = EADDRNOTAVAIL;
			goto out;
		}
		soisconnected(canp->canp_so);
	} else {
		canp->canp_so->so_state &= ~SS_ISCONNECTED;
		canp->canp_ifp = NULL;
	}
	can_pcbstate(canp, CANP_BOUND);
out:		
	return (error);
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

	CANP_LOCK_ASSERT(canp);
	
	bcopy(scan, &canp->canp_dst, sizeof(struct sockaddr_can));
	can_pcbstate(canp, CANP_CONNECTED);
#endif
	return (EOPNOTSUPP);
}

void
can_pcbdisconnect(struct canpcb *canp)
{

	CANP_LOCK_ASSERT(canp);
	can_pcbstate(canp, CANP_DETACHED);
	
	if (canp->canp_so->so_state & SS_NOFDREF)
		can_pcbdetach(canp);
}

void
can_pcbdetach(struct canpcb *canp)
{
	struct socket *so;
 	
	KASSERT((canp->canp_so != NULL), 
		("%s: canp_so == NULL", __func__));
	so = canp->canp_so;
	so->so_pcb = NULL;

	CANP_LOCK_ASSERT(canp);

	can_pcbstate(canp, CANP_DETACHED);
	can_pcbsetfilter(canp, NULL, 0);
	
	CANP_INFO_LOCK(canp->canp_pcbinfo);
	TAILQ_REMOVE(&canp->canp_pcbinfo->cani_queue, canp, canp_queue);
	CANP_INFO_UNLOCK(canp->canp_pcbinfo);

	canp_unref(canp);
}

void
can_pcbfree(struct canpcb *canp)
{

	CANP_LOCK_DESTROY(canp);	
	uma_zfree(canp->canp_pcbinfo->cani_zone, canp);
}

void
canp_ref(struct canpcb *canp)
{
	
	CANP_LOCK_ASSERT(canp);
	canp->canp_refcount++;
}

void
canp_unref(struct canpcb *canp)
{
	
	CANP_LOCK_ASSERT(canp);
	canp->canp_refcount--;
	KASSERT((canp->canp_refcount >= 0),
		("%s: canp->canp_refcount < 0", __func__));
}

struct sockaddr *
can_sockaddr(struct canpcb *canp)
{
	struct sockaddr_can *scan;
	
	scan = malloc(sizeof(*scan), M_SONAME, M_WAITOK | M_ZERO);
	scan->scan_family = AF_CAN;
	scan->scan_len = sizeof(*scan);
	
	if (canp->canp_ifp != NULL) 
		scan->scan_ifindex = canp->canp_ifp->if_index;
	else
		scan->scan_ifindex = 0;
	
	return ((struct sockaddr *)scan);
}

int
can_pcbsetfilter(struct canpcb *canp, struct can_filter *fp, int nfilters)
{
	struct can_filter *newf;

	CANP_LOCK_ASSERT(canp);

	if (nfilters > 0) {
		newf = malloc(sizeof(struct can_filter) * nfilters, 
			M_TEMP, M_WAITOK);
		bcopy(fp, newf, sizeof(struct can_filter) * nfilters);
	} else 
		newf = NULL;
	
	if (canp->canp_filters != NULL) 
		free(canp->canp_filters, M_TEMP);

	canp->canp_filters = newf;
	canp->canp_nfilters = nfilters;

	return (0);
}


#if 0
/*
 * Pass some notification to all connections of a protocol associated 
 * with address dst.  
 * 
 * The local address and / or port numbers may be specified to limit 
 * the search. The "usual action" will be taken, depending on the 
 * ctlinput cmd. 
 * 
 * The caller must filter any cmds that are uninteresting (e.g., no 
 * error in the map). Call the protocol specific routine (if any) to 
 * report any errors for each matching socket.
 *
 * Must be called at splsoftnet. <-- XXX: NO!!11!
 */
int
can_pcbnotify(struct canpcbinfo *pcbinfo, uint32_t faddr, uint32_t laddr,
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
can_pcbnotifyall(struct canpcbinfo *pcbinfo, uint32_t faddr, int errno,
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

/*
 * XXX: I'll refactor this. Those code-sections are from
 * XXX: original implementation by the NetBSD project. 
 */
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
	int ifindex;

	CANP_LOCK_ASSERT(canp);

	ifindex = canp->canp_ifp ? canp->canp_ifp->if_index : 0;

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
	default:
		break;
	}
	canp->canp_state = state;
}

/*
 * Check mbuf(9) against socket(9) accept filter. 
 * 
 * It returns true if mbuf is accepted, false otherwise.
 *
 * XXX: I'll refactor this, e. g. by utilizing the accept_filter(9) KPI.
 */
int
can_pcbfilter(struct canpcb *canp, struct mbuf *m)
{
	int i;
	struct can_frame *fmp;
	struct can_filter *fip;

	CANP_LOCK_ASSERT(canp);

	fmp = mtod(m, struct can_frame *);
	for (i = 0; i < canp->canp_nfilters; i++) {
		fip = &canp->canp_filters[i];
		if ((fmp->can_id & fip->cf_mask) == fip->cf_id)
			return (1);
	}
	/* no match */
	return (0);
}
