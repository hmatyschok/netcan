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
/*-
 * Copyright (c) 2003-2009 Silicon Graphics International Corp.
 * Copyright (c) 2012 The FreeBSD Foundation
 * Copyright (c) 2014-2017 Alexander Motin <mav@FreeBSD.org>
 * All rights reserved.
 *
 * Portions of this software were developed by Edward Tomasz Napierala
 * under sponsorship from the FreeBSD Foundation.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions, and the following disclaimer,
 *    without modification.
 * 2. Redistributions in binary form must reproduce at minimum a disclaimer
 *    substantially similar to the "NO WARRANTY" disclaimer below
 *    ("Disclaimer") and any redistribution must be conditioned upon
 *    including a substantially similar Disclaimer requirement for further
 *    binary redistribution.
 *
 * NO WARRANTY
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTIBILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * HOLDERS OR CONTRIBUTORS BE LIABLE FOR SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING
 * IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGES.
 *
 * $Id$
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
#include "opt_if_can.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_can.h>
#include <net/netisr.h>
#include <net/bpf.h>

#ifdef MAC
#include <security/mac/mac_framework.h>
#endif /* MAC */

/*
 * XXX: Rather incomplete, but work in progress.
 * XXX: 
 * XXX: Well, it's a good place for an generic 
 * XXX: implementaion of LLC and error control.
 */

static MALLOC_DEFINE(M_IFCAN, "IFCAN", "CAN interface internals");

static void 	can_input(struct ifnet *, struct mbuf *);
static int 	can_output(struct ifnet *, struct mbuf *, 
	const struct sockaddr *, struct route *);

/*
 * Process a received CAN frame, the packet is 
 * in the mbuf(9) chain with the CAN header.
 */
static void
can_input(struct ifnet *ifp, struct mbuf *m)
{

	M_ASSERTPKTHDR(m);
	KASSERT(m->m_pkthdr.rcvif != NULL,
	    ("%s: NULL interface pointer", __func__));
	
	if ((ifp->if_flags & IFF_UP) == 0) {
		if_printf(ifp, "discard CAN frame at !IFF_UP\n");
		goto bad;
	}
	
#ifdef DIAGNOSTIC
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
		if_printf(ifp, "discard CAN frame at !IFF_DRV_RUNNING\n");
		goto bad;
	}
#endif 	/* DIAGNOSTIC */
	
	if (m->m_pkthdr.len < sizeof(struct can_frame)) {
		if_printf(ifp, "discard CAN frame"
			" (len %u pkt len %u)\n",
			m->m_len, m->m_pkthdr.len);
		goto bad1;
	}
	
	if (m->m_len < sizeof(struct can_frame)) {
		m = m_pullup(m, sizeof(struct can_frame));
	    if (m == NULL) {
			if_printf(ifp, "m_pullup(9) failed, discard CAN frame.");
			goto bad1;
		}
	}
	
#ifdef MAC
	mac_ifnet_create_mbuf(ifp, m);
#endif 	/* MAC */

	if_inc_counter(ifp, IFCOUNTER_IBYTES, m->m_pkthdr.len);
	
	M_SETFIB(m, ifp->if_fib);
	netisr_dispatch(NETISR_CAN, m);
	return;
bad1:
	if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
bad:
	m_freem(m);
}

/*
 * Wrapper for tx CAN frame by interface-layer.
 */
static int
can_output(struct ifnet *ifp, struct mbuf *m, 
	const struct sockaddr *dst, struct route *ro)
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

void
can_ifattach(struct ifnet *ifp)
{
	struct canif_softc *csc;
		
	if_attach(ifp);
		
	ifp->if_mtu = CAN_MTU;
	ifp->if_input = can_input;	
	ifp->if_output = can_output; 
	
	bpfattach(ifp, DLT_CAN_SOCKETCAN, 0);
	
	KASSERT((ifp->if_l2com != NULL),
	    ("%s: ifp->if_l2com == NULL", __func__));
	csc = ifp->if_l2com; 
	mtx_init(&csc->csc_mtx, "csc_mtx", NULL, MTX_DEF);
}

void
can_ifdetach(struct ifnet *ifp)
{
	struct canif_softc *csc;

	KASSERT((ifp->if_l2com != NULL),
	    ("%s: ifp->if_l2com == NULL", __func__));
	csc = ifp->if_l2com; 
	mtx_destroy(&csc->csc_mtx);
	
	bpfdetach(ifp);
	if_detach(ifp);
}

void
can_ifinit_timings(struct canif_softc *csc)
{
	/* uninitialized parameters is all-one */
	(void)memset(&csc->csc_timings, 0xff, 
		sizeof(struct can_link_timings));
}

/*
 * Capture a CAN frame.
 */
void
can_bpf_mtap(struct ifnet *ifp, struct mbuf *m)
{
	/* bpf wants the CAN id in network byte order */
	struct can_frame *cf;
	canid_t oid;

	cf = mtod(m, struct can_frame *);
	oid = cf->can_id;
	cf->can_id = htonl(oid);
	bpf_mtap(ifp->if_bpf, m);
	cf->can_id = oid;
}

/*
 * Utility functions.
 *
 * See sys/cam/ctl/ctl.c [@ line #4486] and the licence 
 * information on top of this file for further details. 
 */

int
can_bin2hex(struct can_frame *cf, u_char *buf)
{
	int len;
	int i;
	u_char c;
	
	if (buf == NULL) /* XXX: buflen??? */
		return (-1);
	
	if (cf == NULL) 
		return (-1);
	
	if (cf->can_dlc >= CAN_MAX_DLC)
		return (-1);
	
	len = cf->can_dlc * 2;
	
	for (i = 0; cf->can_data[i] != 0 && i < len; i++) {
		c = cf->can_data[i];
	
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c)) 
			c -= (isupper(c)) ? 'A' - 10 : 'a' - 10;
	
		if ((i & 1) == 0)
			buf[i / 2] |= (c >> 4);
		else
			buf[i / 2] |= c;
	}
	return (0);
}

int
can_hex2bin(u_char *buf, struct can_frame *cf)
{
	int len;
	int i;
	u_char c;
	
	if (cf == NULL) 
		return (-1);
	
	if (buf == NULL)
		return (-1);
	
	if (cf->can_dlc >= CAN_MAX_DLC)
		return (-1);
	
	(void)memset(cf->can_data, 0, cf->can_dlc); /* XXX */
	
	len = cf->can_dlc * 2;
	
	for (i = 0; buf[i] != 0 && i < len; i++) {
		c = buf[i];
	
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c)) 
			c -= (isupper(c)) ? 'A' - 10 : 'a' - 10;
	
		if ((i & 1) == 0)
			cf->can_data[i / 2] |= (c << 4);
		else
			cf->can_data[i / 2] |= c;
	}
	return (0);
}

int
can_id2hex(struct can_frame *cf, u_char *buf)
{
	int len;
	u_char *ep;
	u_char c;
	
	if (buf == NULL)
		return (-1);
	
	if (cf == NULL)
		return (-1);
	
	if (cf->can_id & CAN_EFF_FLAG) {
		cf->can_id &= CAN_EFF_MASK;
		len = SLC_EFF_ID_LEN;
	} else {
		cf->can_id &= CAN_SFF_MASK;
		len = SLC_SFF_ID_LEN;
	}
	
	for (ep = buf + len - 1; ep >= buf; ep--, cf->can_id >>= 4) {
		c = (cf->can_id & 0x0f);
		
		if (isdigit(c))
			c -= '0';
		else if (isalpha(c)) 
			c -= (isupper(c)) ? 'A' - 10 : 'a' - 10;
		
		*ep = c;	
	}
	return (len);
}

int
can_hex2id(u_char *buf, struct can_frame *cf)
{
	int len;
	canid_t u;
	canid_t v;
	int i;
	u_char c;
	
	if (cf == NULL)
		return (-1);
	
	if (buf == NULL)
		return (-1);
	
	if (cf->can_id & CAN_EFF_FLAG) 
		len = SLC_EFF_ID_LEN;
	else 
		len = SLC_SFF_ID_LEN;
	
	for (u = v = 0, i = 0; i < len; i++, v <<= 4) { /* XXX */
		c = buf[i];

		if (isdigit(c))
			c -= '0';
		else if (isalpha(c)) 
			c -= (isupper(c)) ? 'A' - 10 : 'a' - 10;
		
		v |= (c & 0x0f);
		u = v;
	}	
	cf->can_id |= u;
	
	return (len);
}

/*
 * Subr. for common structure of CAN interface. 
 */

static void *
can_alloc(u_char type, struct ifnet *ifp)
{
	struct canif_softc *csc;
	
	csc = malloc(sizeof(struct canif_softc), M_IFCAN, M_WAITOK | M_ZERO);
	csc->csc_ifp = ifp;
	
	return (csc);
}

static void
can_free(void *com, u_char type)
{
	
	free(com, M_IFCAN);
}

/*
 * Module description.
 */
 
static int
can_modevent(module_t mod, int type, void *data)
{
	int error;

	switch (type) {
	case MOD_LOAD:
		if_register_com_alloc(IFT_CAN, can_alloc, can_free);
		error = 0;
		break;
	case MOD_UNLOAD:
		if_deregister_com_alloc(IFT_CAN);
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
} 

static moduledata_t can_mod = {
	"can",
	can_modevent,
	0
};

DECLARE_MODULE(can, can_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);
MODULE_VERSION(can, 1);
