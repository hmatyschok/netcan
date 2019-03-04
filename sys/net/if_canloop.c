/*	$NetBSD: if_canloop.c,v 1.2.2.1 2018/01/02 10:20:34 snj Exp $	*/

/*-
 * Copyright (c) 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Manuel Bouyer.
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
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_clone.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_can.h>

struct canolo_softc {
	struct ifnet	*cs_ifp;
	TAILQ_ENTRY(canlo_softc) cs_next;
};

/*
 * Loopback interface driver for the can(4) protocol
 */

static int		canlo_ioctl(struct ifnet *, u_long, caddr_t);
static void 	canlo_start(struct ifnet *);
static void 	canlo_init(void *);

static void 	canlo_clone_destroy(struct ifnet *);
static int 	canlo_clone_create(struct if_clone *, int, caddr_t);

/*
 * Interface cloner and module(9) description.
 */ 

static struct mtx canlo_list_mtx;
static TAILQ_HEAD(canlo_head, canlo_softc) canlo_list = 
	TAILQ_HEAD_INITIALIZER(canlo_list);
	
static MALLOC_DEFINE(M_CANLO, "canlo", "can(4) Loopback Interface"); 

static struct if_clone *canlo_cloner;
static const char canlo_name[] = "canlo";

static void
canlo_clone_destroy(struct ifnet *ifp)
{
	struct canlo_softc *cs
	
	cs = (struct canlo_softc *)ifp->if_softc;
	
	mtx_lock(&canlo_list_mtx);
	TAILQ_REMOVE(&canlo_list, cs, cs_next);
	mtx_unlock(&canlo_list_mtx);
	
	ifp->if_drv_flags &= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);
	ifp->if_flags &= ~IFF_UP;

	can_ifdetach(ifp);
	if_free(ifp);
}

static int
canlo_clone_create(struct if_clone *ifc, int unit, caddr_t data)
{
	struct canlo_softc *cs;
	struct ifnet *ifp;

	cs = malloc(sizeof(*cs), M_CANLO, M_WAITOK | M_ZERO);
	if ((ifp = if_alloc(IFT_CAN)) == NULL)
		return (ENOSPC);

	/* attach */
	mtx_lock(&canlo_list_mtx);
	TAILQ_INSERT_TAIL(&canlo_list, cs, cs_next);
	mtx_unlock(&canlo_list_mtx);
	
	ifp->if_softc = cs;

	if_initname(ifp, canlo_name, unit);
	
	ifp->if_flags = IFF_LOOPBACK | IFF_MULTICAST;
	ifp->if_init = canlo_init;
	ifp->if_ioctl = canlo_ioctl;
	ifp->if_start = canlo_start;
	
	can_ifattach(ifp, 0);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	return (0);
}

static int
canlo_modevent(module_t mod, int type, void *data)
{
	int error;

	switch (type) {
	case MOD_LOAD:
		mtx_init(&canlo_list_mtx, "canlo_list_mtx", NULL, MTX_DEF);
		canlo_cloner = if_clone_simple(canlo_name, 
			canlo_clone_create, canlo_clone_destroy, 0);
		error = 0;
	case MOD_UNLOAD:
		if_clone_detach(canlo_cloner);
		mtx_destroy(&canlo_list_mtx);
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
}

static moduledata_t canlo_mod = {
	"if_canlo",
	canlo_modevent,
	0
};

static void
canlo_ifinit(void *xsc)
{
	struct canlo_softc *slc;
	struct ifnet *ifp;

	slc = (struct canlo_softc *)xsc;
	ifp = slc->canlo_ifp;
	
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;	
}

/*
 * Dequeue for transmission.
 */
static void
canlo_start(struct ifnet *ifp)
{
	struct mbuf *m;

	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;
		
	ifp->if_drv_flags |= IFF_DRV_OACTIVE;
	for (;;) {
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL) 
			break;

		/* IAP for tapping by bpf(4). */
		can_bpf_mtap(ifp, m);

		/* Do some statistics. */		
		if_inc_counter(ifp, IFCOUNTER_OBYTES, m->m_pkthdr.len);
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);		

		m->m_pkthdr.rcvif = ifp;	
		
#ifdef CAN
		(*ifp->if_input)(ifp, m);
#else
		(void)printf("%s: %s: can't handle can(4) frame\n", 
			__func__, ifp->if_xname);
		m_freem(m);
#endif 	/* ! CAN */
	}								
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

/*
 * Process an ioctl(2) request.
 */
/* ARGSUSED */
static int
canlo_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ifreq *ifr;
	int error;

	ifr = (struct ifreq *)data;
	error = 0;

	switch (cmd) {
	case SIOCGDRVSPEC:
	case SIOCSDRVSPEC:
		break;	
	case SIOCSIFFLAGS:
	
		if ((ifp->if_flags & IFF_UP) != 0)
			ifp->if_drv_flags |= IFF_DRV_RUNNING;
		else
			ifp->if_drv_flags &= ~IFF_DRV_RUNNING;	
		break;
	default:
		error = can_ioctl(ifp, cmd, data);
		break;
	}
	return (error);
}

DECLARE_MODULE(if_canlo, canlo_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);
MODULE_DEPEND(if_canlo, can, 1, 1, 1);
