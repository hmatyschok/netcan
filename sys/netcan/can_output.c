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

int can_output_cnt = 0;

/*
 * Transmit can(4) frames.
 */
int
can_output(struct mbuf *m, struct canpcb *canp)
{
	int error = 0;
	struct ifnet *ifp;
	struct can_ifsoftc *csc; 
	struct m_tag *sotag;
	struct sockaddr_can scan;
	struct sockaddr *gw;
#ifdef DIAGNOSTIC
	struct can_hdr *ch; 
#endif /* DIAGNOSTIC */
	
	M_ASSERTPKTHDR(m);
	
	if (canp == NULL) {
		(void)printf("%s: no pcb\n", __func__);
		error = EINVAL;
		goto bad;
	}
	
	if ((ifp = canp->canp_ifp) == NULL) {
		error = EDESTADDRREQ;
		goto bad;
	}
	
	if (ifp->if_type == IFT_CAN) {	
		if ((csc = ifp->if_l2com) != NULL) {
			if (csc->csc_linkmodes & CAN_LINKMODE_LISTENONLY) 
				error = ENETUNREACH;
		}
	} else
		error = ENETUNREACH;
	
	if (error != 0)
		goto bad;
	
	sotag = m_tag_get(PACKET_TAG_ND_OUTGOING, 
		sizeof(struct socket *), M_NOWAIT);
	if (sotag == NULL) {
		error = ENOMEM;
		goto bad;
	}
	CANP_LOCK(canp);
	canp_ref(canp);
	CANP_UNLOCK(canp);
	
	*(struct canpcb **)(sotag + 1) = canp;
	m_tag_prepend(m, sotag);

	(void)memset(&scan, 0, sizeof(scan));
	scan.scan_family = AF_CAN;
	scan.scan_len = sizeof(scan);
	scan.scan_ifindex = ifp->if_index;
	
	gw = (struct sockaddr *)&scan;

#ifdef DIAGNOSTIC
	ch = mtod(m, struct can_hdr *);
	(void)printf("%s: type 0x%01x id 0x%08x dlc 0x%02x\n", 
		__func__, (ch->ch_id & CAN_FLAG_MASK) >> 28, 
		(ch->ch_id & CAN_EFF_MASK), ch->ch_dlc);
#endif /* DIAGNOSTIC */

	if (m->m_len <= ifp->if_mtu) {
		can_output_cnt++;
		error = (*ifp->if_output)(ifp, m,
			    (const struct sockaddr *)gw, NULL);
	} else {
		error = EMSGSIZE;
		goto bad;
	}
done:	
	return (error);
bad:
	m_freem(m);
	goto done;
}

/*
 * Called by [gs]etsockopt(2).
 */
int
can_ctloutput(struct socket *so, struct sockopt *sopt)
{	
	int error;	

	if (sopt->sopt_level == CANPROTO_CAN) 
		error = ENOPROTOOPT;
	else
		error = EINVAL;

	return (error);
}
