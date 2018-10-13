
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
#include <net/netisr.h>
#include <net/route.h>
#include <net/bpf.h>
#include <net/if_can.h>

#if CAN
#include <netcan/can_var.h>
#include <netcan/can_link.h>
#else
#error "can(4) communication domain(9) not installed"
#endif /* CAN */

#include <net/if_slcvar.h>

/*
 * Serial line CAN interface implemented as TTY hook.
 * 
 * XXX: Missing error control [ISO 11898-1] due the 
 * XXX: the case of transmission by uart(4), but it's  
 * XXX: a work in progress and should understood as a 
 * XXX: prototype for a TTY Device-driver class.
 */
 
static MALLOC_DEFINE(M_SLC, "slc", "SLCAN Interface");

static struct mtx slc_mtx;
static TAILQ_HEAD(slc_head, slc_softc) slc_list;
 
/* Subr. */ 
static void 	slc_destroy(struct slc_softc *);
static int 	slc_encap(struct slc_softc *, struct mbuf **);
static int 	slc_rxeof(struct slc_softc *); 
static int 	slc_gtty(struct slc_softc *, void *); 
static int 	slc_stty(struct slc_softc *, void *); 
 
/* Interface-level routines. */
static void 	slc_init(void *);
static int 	slc_ioctl(struct ifnet *, u_long, caddr_t);
static void 	slc_start_locked(struct ifnet *);
static void 	slc_start(struct ifnet *);


/* Bottom-level routines, */
static th_getc_inject_t 	slc_txeof;
static th_getc_poll_t 	slc_txeof_poll;
static th_rint_t 	slc_rint;
static th_rint_poll_t 	slc_rint_poll;

/* TTY hook */
static struct ttyhook slc_hook = {
	.th_getc_inject = 	slc_txeof,
	.th_getc_poll = 	slc_txeof_poll,
	.th_rint = 	slc_rint,
	.th_rint_poll = 	slc_rint_poll,
};

/* Interface cloner */
static void 	slc_clone_destroy(struct ifnet *); 
static int 	slc_clone_create(struct if_clone *, int, caddr_t);
static int 	slc_modevent(module_t, int, void *);

static struct if_clone *slc_cloner;
static const char slc_name[] = "slc";

/*
 * Interface-level routines.
 */
 
static void
slc_init(void *xsc)
{
	struct slc_softc *slc;
	struct ifnet *ifp;

	slc = (struct slc_softc *)xsc;
	ifp = SLC2IFP(slc);
	
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

static void
slc_start(struct ifnet *ifp)
{
	struct slc_softc *slc;
		
	slc = ifp->if_softc;
	
	mtx_lock(&slc->slc_mtx);
	slc_start_locked(ifp);
	mtx_unlock(&slc->slc_mtx);
}

static void
slc_start_locked(struct ifnet *ifp)
{
	struct slc_softc *slc;
	struct tty *tp;
	struct mbuf *m;
	
	slc = ifp->if_softc;
	tp = slc->slc_tp;
	
	mtx_assert(&slc->slc_mtx, MA_OWNED);
	
	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		return;
			
	ifp->if_drv_flags |= IFF_DRV_OACTIVE;
	for (;;) {
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL) 
			break;

		/* IAP on bpf(4). */
		can_bpf_mtap(ifp, m);

		if (tp == NULL) {
			m_freem(m);
			continue;
		}

		/* 
		 * Encode CAN frame in its 
		 *
		 *  <type> <id> <dlc> <data>*
		 * 
		 * ASCII representation.
		 */
		if (slc_encap(slc, &m) != 0) {
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			m_freem(m);
			continue;
		}
		
		/* do some statistics */		
		if_inc_counter(ifp, IFCOUNTER_OBYTES, m->m_pkthdr.len);
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);		
		
		/* notify the TTY */
		tty_lock(tp);
		if (tty_gone(tp) == 0)
			ttydevsw_outwakeup(tp);
		tty_unlock(tp);
	}								
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

/*
 * Process an ioctl(2) request.
 */
/* ARGSUSED */
static int
slc_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct ifreq *ifr = (struct ifreq *)data;
	struct ifdrv *ifd = (struct ifdrv *)data;
	struct slc_softc *slc;
	int error;

	if ((slc = ifp->if_softc) == NULL) {
		error = EINVAL;
		goto out;
	} 
	error = 0;

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
	case SIOCGDRVSPEC:
	
		switch (ifd->ifd_cmd) {
		case SLCGTTY:
			if (ifd->ifd_len != sizeof(dev_t))
				error = EINVAL;
			else
				error = slc_gtty(slc, ifd->ifd_data);
			break;
		default:
			error = EINVAL;
			break:
		}
		break;
	case SIOCSDRVSPEC:
	
		switch (ifd->ifd_cmd) {
		case SLCSTTY:
			if (ifd->ifd_len != sizeof(int))
				error = EINVAL;
			else
				error = slc_stty(slc, ifd->ifd_data);
			break;
		default:
			error = EINVAL;
			break:
		}
		break;
	case SIOCSIFMTU:
		if (ifr->ifr_mtu == CAN_MTU) /* XXX */
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
out:	
	return (error);
}

/*
 * Bottom-level subr.
 */

static int
slc_rint(struct tty *tp, char c, int flags)
{
	struct slc_softc *slc;
	struct mbuf *m;
	int error = 0;

	tty_lock_assert(tp, MA_OWNED);

	if ((slc = ttyhook_softc(tp)) == NULL)
		goto out;

	mtx_lock(&slc->slc_mtx);

#if 0
	if (slc->slc_flags & SLC_ERROR) 
		goto out1; /* XXX */
#endif 

	/* allocate mbuf(9) and initialize */
	if ((m = slc->slc_inb) == NULL) {
		m = m_gethdr(M_NOWAIT, MT_DATA);
		if (m == NULL) {
			error = ENOBUFS;
			goto out1;
		}
		m->m_len = m->m_pkthdr.len = 0;
		slc->slc_inb = m;
	}
	
	*mtod(m, u_char *) = c;
	
	m->m_data++;
	m->m_len++;
	m->m_pkthdr.len++;

	if (c == SLC_HC_BEL || c == SLC_HC_CR || m->m_len >= SLC_MTU) {
		m->m_data = m->m_pktdat;
		error = slc_rxeof(slc);
	}
out1:
	mtx_unlock(&slc->slc_mtx);
out:
	return (error);
}

static size_t
slc_rint_poll(struct tty *tp)
{
	
	return (1);
}

static size_t
slc_txeof(struct tty *tp, void *buf, size_t len)
{
	struct slc_softc *slc;
	struct mbuf *m;
	size_t off = 0;
	size_t m_len;

	slc = ttyhook_softc(tp);

	while (len > 0) {
		IF_DEQUEUE(&slc->slc_outq, m);
		if (m == NULL)
			break;

		while (m != NULL) {
			m_len = min(m->m_len, len);
			bcopy(mtod(m, caddr_t), (caddr_t)buf + off, m_len);

			m->m_data += m_len;
			m->m_len -= m_len;
			off += m_len;
			len -= m_len;

			if (m->m_len > 0)
				break;
			
			m = m_free(m);
		}

		if (m != NULL) {
			IF_PREPEND(&slc->slc_outq, m);
			break;
		}
	}
	IF_LOCK(&slc->slc_outq);
	slc->slc_outqlen -= off;
	IF_UNLOCK(&slc->slc_outq);
	MPASS(slc->slc_outqlen >= 0);

	return (off);
}

static size_t
ngt_txeof_poll(struct tty *tp)
{
	struct slc_softc *slc;

	slc = ttyhook_softc(tp);

	return (slc->slc_outqlen);
}

/*
 * Subr.
 */

static void
slc_destroy(struct slc_softc *slc)
{
	struct ifnet *ifp;
	struct tty *tp;
	
	ifp = SLC2IFP(slc);
	ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	ifp->if_flags &= ~IFF_UP;

	can_ifdetach(ifp);
	if_free(ifp);
	
	/* detach hook, if any and flush queue */
	if ((tp = slc->slc_tp) != NULL) {
		tty_lock(tp);
		ttyhook_unregister(tp);
	}
	IF_DRAIN(&slc->slc_outq);
	mtx_destroy(&slc->slc_outq.ifq_mtx);
	
	mtx_destroy(&slc->slc_mtx);
	free(slc, M_SLC);
}

/*
 * Serialize data and enqueue for trsnsmision.
 */ 
static int 
slc_encap(struct slc_softc *slc, struct mbuf **mp)
{
	u_char buf[MHLEN];
	u_char *bp;
	struct mbuf *m;
	struct can_frame *cf;
	int len;
	int error;
	
	(void)memset((bp = buf), 0, MHLEN);

	M_ASSERTPKTHDR((m = *mp));
	cf = mtod(m, struct can_frame *);
	
	/* determine CAN frame type */
	if (cf->can_id & CAN_RTR_FLAG) 
		*bp = SLC_HC_RTR_SFF;
	else
		*bp = SLC_HC_DATA_SFF;
	
	if (cf->can_id & CAN_EFF_FLAG) 
		*bp = toupper(*bp);
		
	bp += SLC_CMD_LEN;	

	/* map id */
	if ((len = can_id2hex(cf, bp)) < 0) {
		error = EINVAL;
		goto out;
	}
	bp += len;
	
	/* map dlc */
	*bp = cf->can_dlc + '0';
	bp += SLC_DLC_LEN;
	
	/* apply data, if any */
	if ((cf->can_id & CAN_RTR_FLAG) == 0) { /* XXX */
		if (can_bin2hex(cf->data, bp) < 0) {
			error = EINVAL;
			goto out;
		}
		bp += cf->can_dlc;	
	}

	/* finalize */
	*bp = SLC_HC_CR;
	bp += sizeof(u_char);
	
	/* re-initialize mbuf(9) and copy back */
	len = bp - buf; 
	
	m->m_len = m->m_pkthdr.len = len;
	m->m_data = m->m_pktdat;

	bcopy(buf, mtod(m, u_char *), len);
	
	/* enqueue */
	IF_LOCK(&slc->slc_outq);
	if (_IF_QFULL(&slc->slc_outq)) 
		error = ENOSPC; /* XXX: upcall??? */
	else {
		_IF_ENQUEUE(&slc->slc_outq, m);
		slc->slc_outqlen += m->m_pkthdr.len;
		error = 0;
	}
	IF_UNLOCK(&slc->slc_outq);
out:
	*mp = m;	
	return (error);
}

static int 
slc_gtty(struct slc_softc *slc, void *data)
{
	dev_t *d = (dev_t *)data;
	
	mtx_lock(&slc->slc_mtx);
	
	if (slc->slc_tp != NULL)
		*d = tty_udev(slc->slc_tp);	
	else
		*d = NODEV;

	mtx_unlock(&slc->slc_mtx);

	return (0);
}

static int
slc_rxeof(struct slc_softc *slc)
{
	int error = 0;
	struct ifnet *ifp;
	struct mbuf *m;
	u_char buf[MHLEN];
	u_char *bp;
	struct can_frame *cf;
	int len;
	
	mtx_assert(&slc->slc_mtx, MA_OWNED);
	
	ifp = SLC2IFP(slc);
	
	if ((m = slc->slc_inb) == NULL) {
		error = EINVAL;
		goto out;
	}
	slc->slc_inb = NULL;

	(void)memset((bp = buf), 0, MHLEN);
	cf = (struct can_frame *)bp;

	/* determine CAN frame type */
	switch (*mtod(m, u_char *)) {
	case SLC_HC_RTR_SFF:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */
	case SLC_HC_DATA_SFF:
		break;
	case SLC_HC_RTR_EFF:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */
	case SLC_HC_DATA_EFF:
		cf->can_id |= CAN_EFF_FLAG; 
		break;
	default:
		error = EINVAL;
		goto bad;
	}
	m_adj(m, sizeof(u_char));
	
	/* fetch id */
	if ((len = can_hex2id(mtod(m, u_char *), cf)) < 0) {
		error = EINVAL;
		goto bad;
	}
	m_adj(m, len);
	
	/* fetch dlc */
	cf->can_dlc = *mtod(m, u_char *);
	
	if (cf->can_dlc < SLC_HC_DLC_INF) {
		error = EINVAL;
		goto bad;
	}
	
	if (cf->can_dlc > SLC_HC_DLC_SUP) {
		error = EINVAL;
		goto bad;
	}
	cf->can_dlc -= SLC_HC_DLC_INF;
	m_adj(m, sizeof(u_char));
	
	/* fetch data, if any */
	if ((cf->can_id & CAN_RTR_FLAG) == 0) { 
		if (can_hex2bin(mtod(m, u_char *), cf) < 0) {
			error = EINVAL;
			goto bad;
		}
	}

	if (m->m_len < sizeof(struct can_frame))
		len = sizeof(struct can_frame);
	else
		len = sizeof(struct can_hdr) + cf->can_dlc;

	/* reinitialize mbuf(9) and copy back */
	m->m_len = m->m_pkthdr.len = len;
	m->m_data = m->m_pktdat;

	bcopy(buf, mtod(m, u_char *), len);
	
	/* pass CAN frame to layer above */
 	mtx_unlock(&slc->slc_mtx);
 	(*ifp->if_input)(ifp, m);
 	mtx_lock(&slc->slc_mtx);
out:
	return (error);			
bad:
	m_freem(m);
	goto out;
}

static int 
slc_stty(struct slc_softc *slc, void *data)
{
	int fd = *(int *)data;
	struct proc *p;
	int error;
	
	mtx_lock(&slc->slc_mtx);
	
	if (slc->slc_tp != NULL) {
		error = EBUSY;
		goto out;
	}
	
	if ((p = pfind(fd)) == NULL) {
		error = ESRCH;
		goto out;
	}
	
	if (p->p_flag & P_WEXIT) {
		error = ESRCH;
		goto out;
	}
	_PHOLD(p);
	PROC_UNLOCK(p);
	error = ttyhook_register(&slc->slc_tp, p, fd, &slc_hook, slc);
	PRELE(p);	
out:
	mtx_unlock(&slc->slc_mtx);
	return (error);
}

/* 
 * Interface cloner and module description. 
 */ 

static int
slc_clone_create(struct if_clone *ifc, int unit, caddr_t data)
{
	struct slc_softc *slc;
	struct ifnet *ifp;

	slc = malloc(sizeof(*slc), M_SLC, M_WAITOK | M_ZERO);
	ifp = SLC2IFP(slc) = if_alloc(IFT_OTHER);
	if (ifp == NULL) {
		free(slc, M_SLC);
		return (ENOSPC);
	}
	ifp->if_softc = slc;
	
	if_initname(ifp, slc_name, unit);
	
	ifp->if_flags = IFF_POINTOPOINT | IFF_MULTICAST;
	ifp->if_init = slc_init;
	ifp->if_start = slc_start;
	ifp->if_ioctl = slc_ioctl;
	
	can_ifattach(ifp);

	ifp->if_mtu = SLC_MTU;
	
	/* initialize its protective lock */
	mtx_init(&slc->slc_mtx, "slc_mtx", NULL, MTX_DEF)
	
	/* initialize queue for transmission */
	mtx_init(&slc->slc_outq.ifq_mtx, "slc_outq_mtx", NULL, MTX_DEF);
	IFQ_SET_MAXLEN(&slc->slc_outq, ifqmaxlen);
	
	/* attach */
	mtx_lock(&slc_list_mtx);
	TAILQ_INSERT_TAIL(&slc_list, slc, slc_next);
	mtx_unlock(&slc_list_mtx);
	mtx_lock(&slc->slc_mtx);
	slc->slc_flags |= SLC_ATTACHED;
	mtx_unlock(&slc->slc_mtx);

	return (0);
}

static void
slc_clone_destroy(struct ifnet *ifp)
{
	struct slc_softc *slc = ifp->if_softc;

	mtx_lock(&slc_list_mtx);
	TAILQ_REMOVE(&slc_list, slc, slc_list);
	mtx_unlock(&slc_list_mtx);
	slc_destroy(slc);
}

static int
slc_modevent(module_t mod, int type, void *data) 
{ 
	struct slc_softc *slc;
	int error;

	switch (type) {
	case MOD_LOAD:
		mtx_init(&slc_list_mtx, "slc_list_mtx", NULL, MTX_DEF);
		slc_cloner = if_clone_simple(slc_name, 
			slc_clone_create, slc_clone_destroy, 0);
		error = 0;
		break;
	case MOD_UNLOAD:
		mtx_lock(&slc_list_mtx);
		while ((slc = TAILQ_FIRST(&slc_list)) != NULL) {
			TAILQ_REMOVE(&slc_list, slc, slc_next);
			mtx_unlock(&slc_list_mtx);
			slc_destroy(slc);
			mtx_lock(&slc_list_mtx);
		}
		mtx_unlock(&slc_list_mtx);
		mtx_destroy(&slc_list_mtx);
		if_clone_detach(slc_cloner);
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
} 

static moduledata_t slc_mod = { 
	"if_slc", 
	slc_modevent, 
	0
}; 

DECLARE_MODULE(if_slc, slc_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);
