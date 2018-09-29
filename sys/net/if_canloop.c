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
 
/*
 * Loopback interface driver for the CAN protocol
 */

#include "opt_can.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/kernel.h>
#include <sys/mbuf.h>
#include <sys/module.h>
#include <machine/bus.h>
#include <sys/rman.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_clone.h>
#include <net/if_types.h>

#include <net/route.h>

#ifdef	CAN
#include <netcan/can.h>
#endif

#include <security/mac/mac_framework.h>

#define CANLOMTU 	(sizeof(struct can_frame));

static int		canloop_ioctl(struct ifnet *, u_long, caddr_t);
static int		canloop_output(struct ifnet *ifp, struct mbuf *m,
		    const struct sockaddr *dst, struct route *ro);
static void 	canloop_start(struct ifnet *ifp);
static int	canloop_clone_create(struct if_clone *, int, caddr_t);
static void	canloop_clone_destroy(struct ifnet *);

static struct ifnet *canloop_if; /* XXX */

static struct if_clone *canloop_cloner;
static const char canloop_name[] = "canlo";

static void
canloop_clone_destroy(struct ifnet *ifp)
{
	ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	ifp->if_flags &= ~IFF_UP;

	can_ifdetach(ifp);
	if_free(ifp);
}

static int
canloop_clone_create(struct if_clone *ifc, int unit, caddr_t params)
{
	struct ifnet *ifp;

	ifp = if_alloc(IFT_OTHER);
	if (ifp == NULL)
		return (ENOSPC);

	if_initname(ifp, canloop_name, unit);
	ifp->if_flags = IFF_LOOPBACK;
	ifp->if_ioctl = canloop_ioctl;
	ifp->if_output = canloop_output;
	ifp->if_start = canloop_start;
	ifp->if_snd.ifq_maxlen = ifqmaxlen;
	
	can_ifattach(ifp);

	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	return (0);
}

static void
canloop_init(const void *v __unused)
{

	canloop_cloner = if_clone_simple(canloop_name, 
		canloop_clone_create, canloop_clone_destroy, 1);
}
SYSINIT(canloop_init, SI_SUB_PSEUDO, SI_ORDER_ANY, canloop_init, NULL);

static int
canloop_modevent(module_t mod, int type, void *data)
{
	int error;

	switch (type) {
	case MOD_LOAD:
	case MOD_UNLOAD:
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
}

static moduledata_t canloop_mod = {
	"if_canloop",
	canloop_modevent,
	0
};

DECLARE_MODULE(if_canloop, canloop_mod, SI_SUB_PROTO_IFATTACHDOMAIN, 
	SI_ORDER_ANY);

int
canloop_output(struct ifnet *ifp, struct mbuf *m, const struct sockaddr *dst,
    struct route *ro)
{
#ifdef MAC
	int error;
#endif 	/* MAC */
	
	M_ASSERTPKTHDR(m);

#ifdef MAC
	error = mac_ifnet_check_transmit(ifp, m);
	if (error) {
		m_freem(m);
		return (error);
	}
#endif 	/* MAC */
	return ((*ifp->if_transmit)(ifp, m));
}

static void
canloop_ifstart(struct ifnet *ifp)
{
	struct mbuf *m;

	ifp->if_drv_flags |= IFF_DRV_OACTIVE;
	for (;;) {
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL) 
			break;

		/* IAP for tapping by bpf(4). */
		can_bpf_mtap(ifp, m, 0);

		/* Do some statistics. */		
		if_inc_counter(ifp, IFCOUNTER_OBYTES, m->m_pkthdr.len);
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);		

		m->m_pkthdr.rcvif = ifp;	
		
#ifdef CAN
		can_mbuf_tag_clean(m);
		(*ifp->if_input)(ifp, m);
#else
		(void)printf("%s: can't handle CAN packet\n", ifp->if_xname);
		m_freem(m);
#endif 	/* CAN */
	}								
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

/*
 * Process an ioctl(2) request.
 */
/* ARGSUSED */
int
canloop_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ifreq *ifr = (struct ifreq *)data;
	int error = 0;

	switch (cmd) {
	case SIOCSIFADDR:
		ifp->if_flags |= IFF_UP;
		ifp->if_drv_flags |= IFF_DRV_RUNNING;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		if (ifr == NULL) {
			error = EAFNOSUPPORT;		/* XXX */
			break;
		}
		
		switch (ifr->ifr_addr.sa_family) {
#ifdef CAN
		case AF_CAN:
			break;
#endif 	/* CAN */
		default:
			error = EAFNOSUPPORT;
			break;
		}
		break;
	case SIOCSIFMTU:
		if (ifr->ifr_mtu == CANLOMTU)
			ifp->if_mtu = ifr->ifr_mtu;
		else
			error = EINVAL;
		break;
	case SIOCSIFFLAGS:
		break;
	default:
		error = EINVAL;
		break;
	}
	return (error);
}
