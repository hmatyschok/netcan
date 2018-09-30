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

/*
 * XXX: incomplete, ...
 */

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

static void 	can_input(struct ifnet *ifp, struct mbuf *m);
static int 	can_output(struct ifnet *ifp, struct mbuf *m, 
	const struct sockaddr *dst, struct route *ro);

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
		if_printf(ifp, "discard frame at !IFF_UP\n");
		goto bad;
	}
#ifdef DIAGNOSTIC
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
		if_printf(ifp, "discard frame at !IFF_DRV_RUNNING\n");
		goto bad;
	}
#endif 	/* DIAGNOSTIC */
	
	if (m->m_pkthdr.len < sizeof(struct can_frame)) {
		if_printf(ifp, "discard frame w/o can frame"
			" (len %u pkt len %u)\n",
			m->m_len, m->m_pkthdr.len);
		goto bad1:
	}
	
	if (m->m_len < sizeof (struct can_frame) {
		m = m_pullup(m, sizeof (struct can_frame))
	    if (m == NULL) {
		if_printf(ifp, "m_pullup(9) failed,"
			" discard frame w/o can frame"
			" (len %u pkt len %u)\n",
			m->m_len, m->m_pkthdr.len);
		goto bad1:
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
 * Output can_frame {} on interface-layer.
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
	
	if_attach(ifp);
		
	ifp->if_mtu = CAN_MTU;
	ifp->if_hdrlen = 0;
	ifp->if_addrlen = 0;
	ifp->if_input = can_input;	
	ifp->if_output = can_output; 
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
	(void)memset(&csc->csc_timings, 0xff, 
		sizeof(struct can_link_timings));
}

/*
 * cleanup mbuf tag, keeping the PACKET_TAG_SO tag
 */
void
can_mbuf_tag_clean(struct mbuf *m)
{
	struct m_tag *sotag;

	sotag = m_tag_find(m, PACKET_TAG_SO, NULL);
	if (sotag != NULL)
		m_tag_unlink(m, sotag);

	m_tag_delete_nonpersistent(m);
	if (sotag != NULL)
		m_tag_prepend(m, sotag);
}

void
can_bpf_mtap(struct ifnet *ifp, struct mbuf *m)
{
	/* bpf wants the CAN id in network byte order */
	struct can_frame *cf;
	canid_t oid;

	cf = mtod(m, struct can_frame *);
	oid = cf->can_id;
	cf->can_id = htonl(oid);
	bpf_mtap(ifp, m);
	cf->can_id = oid;
}
