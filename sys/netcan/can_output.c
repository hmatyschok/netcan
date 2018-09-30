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
#include <sys/mbuf.h>
#include <sys/errno.h>
#include <sys/protosw.h>
#include <sys/sockio.h>
#include <sys/socket.h>
#include <sys/socketvar.h>

#include <net/if.h>
#include <net/if_types.h>

#include <net/if.h>
#include <net/if_types.h>
#include <net/route.h>

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

/*
 * XXX: Some components should externalized into interface-layer.
 */

int can_output_cnt = 0;
struct mbuf *can_lastout;

/*
 * Transmit frames.
 */
int
can_output(struct mbuf *m, struct canpcb *canp)
{
	int error = 0;
	struct ifnet *ifp;
	struct canif_softc *csc; /* XXX: interface-layer */
	struct m_tag *sotag;
	struct sockaddr_can dst;
	const struct sockaddr_can *gw;
	
	M_ASSERTPKTHDR(m);
	
	if (canp == NULL) {
		(void)printf("%: no pcb\n", __func__);
		error = EINVAL;
		goto bad;
	}
	
	if ((ifp = canp->canp_ifp) == NULL) {
		error = EDESTADDRREQ;
		goto bad;
	}
	
	switch (ifp->if_type) {
	case IFT_OTHER:	
		if ((csc = ifp->if_softc) != NULL) {
			if (csc->csc_linkmodes & CAN_LINKMODE_LISTENONLY) {
				error = ENETUNREACH;
				goto bad:
			}
		}
		break;
	default:
		error = ENETUNREACH;
		goto bad:
	}
		
	sotag = m_tag_get(PACKET_TAG_ND_OUTGOING, 
		sizeof(struct socket *), M_NOWAIT);
	if (sotag == NULL) {
		ifp->if_oerrors++;
		error = ENOMEM;
		goto bad;
	}
	
	CANP_LOCK(canp);
	canp_ref(canp);
	CANP_UNLOCK(canp);
	
	*(struct canpcb **)(sotag + 1) = canp;
	m_tag_prepend(m, sotag);

	gw = (const struct sockaddr_can *)&dst;
	(void)memset(&dst, 0, sizeof(dst));
	dst.can_family = AF_CAN;
	dst.can_len = sizeof(dst);

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
 * Called by getsockopt and setsockopt.
 *
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
