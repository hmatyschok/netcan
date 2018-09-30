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

/*
 * Process a received CAN frame
 * the packet is in the mbuf chain m with
 * the CAN header.
 */
static void
can_input(struct ifnet *ifp, struct mbuf *m)
{
	if ((ifp->if_flags & IFF_UP) == 0) {
		m_freem(m);
		return;
	}
#ifdef MAC
	mac_ifnet_create_mbuf(ifp, m);
#endif
	if_inc_counter(ifp, IFCOUNTER_IBYTES, m->m_pkthdr.len);
	
	netisr_dispatch(NETISR_CAN, m);
}

/*
 * XXX: I'll refactor this..
 */
void
can_ifattach(struct ifnet *ifp)
{
	if_attach(ifp);
	ifp->if_mtu = sizeof(struct can_frame);
	
	/* XXX */
	ifp->if_hdrlen = 0;
	ifp->if_addrlen = 0;
	
	/* ... */
	
	ifp->if_output = NULL; /* XXX: unused */
	ifp->if_input = can_input;

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
