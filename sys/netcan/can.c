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

#include <netcan/can.h>
#include <netcan/can_pcb.h>
#include <netcan/can_var.h>

/*
 * Generic control operations.
 */
 
static int
can_get_netlink(struct ifnet *ifp, struct ifdrv *ifd)
{
	struct canif_softc *csc;
	int error;

	/* XXX */
	
	if (ifp->if_type == IFT_OTHER) {
		if ((csc = ifp->if_softc) == NULL)
			error = EOPNOTSUPP;
		else
			error = 0;
	} else
		error = EOPNOTSUPP;

	if (error != 0)
		goto out;

	switch (ifd->ifd_cmd) {
	case CANGLINKTIMECAP:
		if (ifd->ifd_len != sizeof(struct can_link_timecaps)) 
			error = EINVAL;
		else
			error = copyout(&csc->csc_timecaps, 
				ifd->ifd_data, ifd->ifd_len);
		break;
	case CANGLINKTIMINGS:
		if (ifd->ifd_len != sizeof(struct can_link_timings)) 
			error = EINVAL;
		else	
			error = copyout(&csc->csc_timings, 
				ifd->ifd_data, ifd->ifd_len);
		break;
	case CANGLINKMODE:
		if (ifd->ifd_len != sizeof(uint32_t))
			error = EINVAL;
		else 
			error = copyout(&csc->csc_linkmodes, 
				ifd->ifd_data, ifd->ifd_len);
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
out:	
	return (error);
}

static int
can_set_netlink(struct ifnet *ifp, struct ifdrv *ifd)
{
	struct canif_softc *csc;
	uint32_t mode;
	int error;

	if ((ifp->if_flags & IFF_UP) != 0) {
		error = ENETDOWN;
		goto out;
	}

	/* XXX */	
	
	if (ifp->if_type == IFT_OTHER) {
		if ((csc = ifp->if_softc) == NULL)
			error = EOPNOTSUPP;
		else
			error = 0;
	} else
		error = EOPNOTSUPP;

	if (error != 0)
		goto out;
		
	switch(ifd->ifd_cmd) {
	case CANSLINKTIMINGS:
		if (ifd->ifd_len != sizeof(struct can_link_timings))
			error = EINVAL;
		else 
			error = copyin(ifd->ifd_data, 
				&csc->csc_timings, ifd->ifd_len);
		break;
	case CANSLINKMODE:
	case CANCLINKMODE: 	/* FALLTHROUGH */
	
		if (ifd->ifd_len != sizeof(uint32_t))
			error = EINVAL;
		else 	
			error = copyin(ifd->ifd_data, &mode, ifd->ifd_len);
		
		if (error != 0)
			break;
			
		if ((mode & csc->csc_timecaps.cltc_linkmode_caps) != mode) {
			error = EINVAL;
			break;	
		}
		/* XXX locking */
		if (ifd->ifd_cmd == CANSLINKMODE)
			csc->csc_linkmodes |= mode;
		else
			csc->csc_linkmodes &= ~mode;
		
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	
out:	
	return (error);
}

/* ARGSUSED */
static int
can_control(struct socket *so, u_long cmd, caddr_t data, struct ifnet *ifp,
    struct thread *td)
{
	struct can_ifreq *cfr = (struct can_ifreq *)data;
	int error = 0;

	if (ifp == NULL)
		error = EADDRNOTAVAIL;
		goto out;
	}

	switch (cmd) {
	case SIOCGDRVSPEC:
		error = can_get_netlink(ifp, cfr);
		break;
	case SIOCSDRVSPEC:
		error = can_set_netlink(ifp, cfr);
		break;
	default:
		if (ifp->if_ioctl != NULL)
			error = (*ifp->if_ioctl)(ifp, cmd, data)
		else	
			error = EOPNOTSUPP;
		
		break;
	}
out:	
	return (error);
}

/* 
 * XXX: Well, this should be reimplemented. 
 */
static int
can_purgeif(struct socket *so, struct ifnet *ifp)
{
	
	return (0);
}
