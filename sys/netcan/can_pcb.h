/*	$NetBSD: can_pcb.h,v 1.2 2017/05/27 21:02:56 bouyer Exp $	*/

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
#ifndef _NETCAN_CAN_PCB_H_
#define _NETCAN_CAN_PCB_H_

#include <sys/queue.h>

#ifdef _KERNEL
#include <sys/lock.h>
#include <sys/mutex.h>
#include <vm/uma.h>
#endif /* _KERNEL */

/*
 * Common structure pcb for can protocol implementation.
 * Here are stored pointers to local and foreign host table
 * entries, local and foreign socket numbers, and pointers
 * up (to a socket structure) and down (to a protocol-specific)
 * control block.
 */
struct canpcbpolicy;
struct canpcbinfo; /* XXX */

struct canpcb {
	LIST_ENTRY(canpcb) canp_hash;
	LIST_ENTRY(canpcb) canp_lhash;
	TAILQ_ENTRY(canpcb) canp_queue;
	struct mtx	canp_lock;	/* protect states and refcount */
	int		canp_state;
	int		canp_flags;
	struct socket 	*canp_so;	/* back pointer to socket */
	struct ifnet 	*canp_ifp; /* interface this socket is bound to */

	struct canpcbinfo 	*canp_pcbinfo;
	struct can_filter 	*canp_filters; /* filter array */
	int 	canp_nfilters; /* size of canp_filters */

	int		canp_refcount;
};
#define	CANP_LOCK_INIT(canp, d) \
	mtx_init(&(canp)->canp_lock, (d), NULL, MTX_DEF)
#define	CANP_LOCK(canp) 	mtx_lock(&(canp)->canp_lock)
#define	CANP_UNLOCK(canp) 	mtx_unlock(&(canp)->canp_lock)
#define	CANP_LOCK_ASSERT(canp) \
	mtx_assert(&(canp)->canp_lock, MA_OWNED)
#define	CANP_LOCK_DESTROY(canp) 	mtx_destroy(&(canp)->canp_lock)
	
LIST_HEAD(canpcbhead, canpcb);
TAILQ_HEAD(canpcbqueue, canpcb);

/* 
 * Set contains canpcb{}s maps to PF_CAN family on AF_CAN domain(9). 
 */
struct canpcbinfo {
	TAILQ_ENTRY(canpcbinfo) cani_next;
	struct mtx 	cani_lock; 		/* protects PCB traversal */
	uma_zone_t 	cani_zone; 	/* uma(9) zone for slap allocator */
	struct canpcbqueue 	cani_queue;
	struct canpcbhead 	*cani_bindhashtbl;
	struct canpcbhead 	*cani_connecthashtbl;
	u_long	cani_bindhash;
	u_long	cani_connecthash;
};
#define	CANP_INFO_LOCK_INIT(cani, d) \
	mtx_init(&(cani)->cani_lock, (d), NULL, MTX_DEF)
#define	CANP_INFO_LOCK(cani) 	mtx_lock(&(cani)->cani_lock)
#define	CANP_INFO_UNLOCK(cani) 	mtx_unlock(&(cani)->cani_lock)
#define	CANP_INFO_LOCK_ASSERT(cani) \
	mtx_assert(&(cani)->cani_lock, MA_OWNED)
#define	CANP_INFO_LOCK_DESTROY(cani) 	mtx_destroy(&(cani)->cani_lock)

TAILQ_HEAD(canpcbinfo_head, canpcbinfo);
	
/* states in canp_state: */
#define	CANP_DETACHED		0
#define	CANP_ATTACHED		1
#define	CANP_BOUND		2
#define	CANP_CONNECTED		3

/* flags in canp_flags: */
#define CANP_NO_LOOPBACK	0x0001 /* local loopback disabled */
#define CANP_RECEIVE_OWN	0x0002 /* receive own message */

#define	sotocanpcb(so)		((struct canpcb *)(so)->so_pcb)

#ifdef _KERNEL
void 	can_losing(struct canpcb *);
int 	can_pcballoc(struct socket *, struct canpcbinfo *);
int 	can_pcbbind(struct canpcb *, struct sockaddr_can *, 
	struct ucred *);
int 	can_pcbconnect(struct canpcb *, struct sockaddr_can *);
void 	can_pcbdetach(struct canpcb *);
void 	can_pcbdisconnect(struct canpcb *);
void 	can_pcbfree(struct canpcb *);
void 	can_pcbinfo_init(struct canpcbinfo *, const char *, 
	const char *, uma_init, uma_fini, int, int);
int 	can_pcbnotify(struct canpcbinfo *, uint32_t, 
	uint32_t, int, void (*)(struct canpcb *, int));
void 	can_pcbnotifyall(struct canpcbinfo *, uint32_t, int, 
	void (*)(struct canpcb *, int));
void 	can_pcbpurgeif0(struct canpcbinfo *, struct ifnet *);
void 	can_pcbpurgeif(struct canpcbinfo *, struct ifnet *);
void 	can_pcbstate(struct canpcb *, int);
struct sockaddr * 	can_sockaddr(struct canpcb *);
int 	can_pcbsetfilter(struct canpcb *, struct can_filter *, int);
int 	can_pcbfilter(struct canpcb *, struct mbuf *);

/* refcount management */
void	canp_ref(struct canpcb *);
void	canp_unref(struct canpcb *);
#endif /* _KERNEL */

#endif /* _NETCAN_CAN_PCB_H_ */
