
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
#include <sys/filio.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/tty.h>
     
#include <net/if.h>
#include <net/if_clone.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_slcvar.h>

/*
 * Serial line CAN interface implemented by tty(4) hook.
 * 
 * XXX: It's a work in progress and should be understood as  
 * XXX: RAD prototype for the purpose of developing a tty(4) 
 * XXX: device-driver class.
 * XXX:
 * XXX: See sys/sys/ttydevsw.h for further details.
 * 
 * Example - using uart(4) for MAC:
 * 
 *  a) create process, open(2) uart(4) device(9) 
 * 
 *  b) tc[gs]etattr(3) [CS8, ...] and fork(2)
 * 
 *  c) ifconfig slc0 create
 * 
 *  d) ifconfig slc0 stty cuau0
 */

static struct if_clone *slc_cloner;
static const char slc_name[] = "slc"; 

static struct mtx slc_list_mtx;
static TAILQ_HEAD(slc_head, slc_softc) slc_list = 
	TAILQ_HEAD_INITIALIZER(slc_list);
	
static MALLOC_DEFINE(M_SLC, "slc", "Serial line CAN Interface"); 
 
/* Subr. */
static void 	slc_destroy(struct slc_softc *);
static int 	slc_encap(struct slc_softc *, struct mbuf **);
static int 	slc_rxeof(struct slc_softc *); 
static int 	slc_gtty(struct slc_softc *, void *); 
static int 	slc_stty(struct slc_softc *, void *, struct thread *); 
static int 	slc_dtty(struct slc_softc *);
 
/* Interface cloner */
static void 	slc_ifclone_destroy(struct ifnet *); 
static int 	slc_ifclone_create(struct if_clone *, int, caddr_t);

/* Interface-level routines. */
static void 	slc_ifinit(void *);
static int 	slc_ifioctl(struct ifnet *, u_long, caddr_t);
static void 	slc_ifstart(struct ifnet *);

/* Bottom-level routines */
static th_getc_inject_t 	slc_txeof;
static th_getc_poll_t 	slc_txeof_poll;
static th_rint_t 	slc_rint;
static th_rint_poll_t 	slc_rint_poll;

/* device(9)-level routines */
static d_open_t 	slc_open;
static d_close_t 	slc_close;
static d_ioctl_t 	slc_ioctl;

/* tty(4) hook */
static struct ttyhook slc_hook = {
	.th_getc_inject = 	slc_txeof,
	.th_getc_poll = 	slc_txeof_poll,
	.th_rint = 	slc_rint,
	.th_rint_poll = 	slc_rint_poll,
};

/* device(9) methods */
static struct cdevsw slc_cdevsw = {
	.d_version = 	D_VERSION,
	.d_open = 	slc_open,
	.d_close = 	slc_close,	
	.d_ioctl = 	slc_ioctl,
	.d_name = 	slc_name,
};

/*-
 * Interface-level routines.
 * 
 */
 
static void
slc_ifinit(void *xsc)
{
	struct slc_softc *slc;
	struct ifnet *ifp;

	slc = (struct slc_softc *)xsc;
	ifp = slc->slc_ifp;
	
	if (slc->slc_tp != NULL) 
		ifp->if_flags |= IFF_UP;
	else
		ifp->if_flags &= ~IFF_UP;
		
	if (ifp->if_flags & IFF_UP)
		ifp->if_drv_flags |= IFF_DRV_RUNNING;
	else
		ifp->if_drv_flags &= ~IFF_DRV_RUNNING;		
}

static void
slc_ifstart(struct ifnet *ifp)
{
	struct slc_softc *slc;
	struct tty *tp;
	struct mbuf *m;
	
	slc = ifp->if_softc;
	tp = slc->slc_tp;
		
	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
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
		
		/* notify the tty(4) */
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
slc_ifioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
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
	case SIOCGDRVSPEC:
		switch (ifd->ifd_cmd) {
		case TIOCGETD:
			if (ifd->ifd_len != sizeof(dev_t))
				error = EINVAL;
			else
				error = slc_gtty(slc, ifd->ifd_data);
			break;
		default:
			error = EINVAL;
			break;
		}
		break;
	case SIOCSIFMTU:
		if (ifr->ifr_mtu == CAN_MTU) /* XXX */
			ifp->if_mtu = ifr->ifr_mtu;
		else
			error = EINVAL;
		break;
	case SIOCSIFFLAGS:
		slc_ifinit(slc);
		break;
	default:
		error = EINVAL;
		break;
	}
out:	
	return (error);
}

/*-
 * Bottom-level subr.
 * 
 */

/*
 * Rx-interrupt.
 */
static int
slc_rint(struct tty *tp, char c, int flags)
{
	struct slc_softc *slc;
	struct mbuf *m;
	int error;

	tty_lock_assert(tp, MA_OWNED);

	if ((slc = ttyhook_softc(tp)) == NULL) {
		error = ENETDOWN;
		goto out;
	}

	/* allocate mbuf(9) and initialize */
	if ((m = slc->slc_inb) == NULL) {
		if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL) {
			error = ENOBUFS;
			goto out;
		}
		m->m_len = m->m_pkthdr.len = 0;
		mtx_lock(&slc->slc_mtx);
		slc->slc_inb = m;
		mtx_unlock(&slc->slc_mtx);
	}
	
	if (flags != 0) {
		error = ECONNABORTED;	
		goto bad;
	}
	
	*mtod(m, u_char *) = c;
	
	m->m_data++;
	m->m_len++;
	m->m_pkthdr.len++;

	if (m->m_len < MHLEN) {
		if (c == SLC_HC_BEL || c == SLC_HC_CR) {
			m->m_data = m->m_pktdat;
			error = slc_rxeof(slc);
		} else 
			error = 0;
	} else {
		error = EFBIG;
		goto bad;
	}
out:
	return (error);
bad:
	mtx_lock(&slc->slc_mtx);
	if (slc->slc_inb != NULL) 
		slc->slc_inb = NULL;	
	mtx_unlock(&slc->slc_mtx);
	m_freem(m);
	goto out;
}

static size_t
slc_rint_poll(struct tty *tp)
{
	
	return (1);
}

/*
 * Tx-interrupt.
 */
static size_t
slc_txeof(struct tty *tp, void *buf, size_t len)
{
	struct slc_softc *slc;
	struct mbuf *m;
	size_t off = 0;
	size_t m_len;

	if ((slc = ttyhook_softc(tp)) == NULL)
		goto out; 	/* XXX */

	mtx_lock(&slc->slc_mtx);

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
	mtx_unlock(&slc->slc_mtx);
	MPASS(slc->slc_outqlen >= 0);
out:
	return (off);
}

static size_t
slc_txeof_poll(struct tty *tp)
{
	struct slc_softc *slc;
	size_t outqlen;

	if ((slc = ttyhook_softc(tp)) != NULL)
		outqlen = slc->slc_outqlen;
	else
		outqlen = 0;

	return (outqlen);
}

/*-
 * Device-level routines.
 * 
 */

static int
slc_open(struct cdev *dev, int flag, int mode, struct thread *td)
{
	
	return (0);
}
 
static int
slc_close(struct cdev *dev, int flag, int mode, struct thread *td)
{
	
	return (0);
} 

static int
slc_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flags,
    struct thread *td)
{
	struct slc_softc *slc;
	int error;

	slc = dev->si_drv1;

	switch (cmd) {
	case TIOCSETD:
		error = slc_stty(slc, data, td);
		break;
	case TIOCGETD:
		error = slc_gtty(slc, data);
		break;
	case TIOCNOTTY:
		error = slc_dtty(slc);
		break;	
	default:
		error = ENOTTY;
		break;
	}
	return (error);
}

/*-
 * Subr.
 * 
 */

static void
slc_destroy(struct slc_softc *slc)
{
	struct ifnet *ifp;
	struct cdev *dev;
	
	/* destroy its ifnet(9) mapping */
	ifp = slc->slc_ifp;
	ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
	ifp->if_flags &= ~IFF_UP;

	can_ifdetach(ifp);
	if_free(ifp);
	
	slc->slc_ifp = NULL;
	
	/* detach hook, if any and flush queue */
	(void)slc_dtty(slc);
	
	mtx_destroy(&slc->slc_outq.ifq_mtx);
	mtx_destroy(&slc->slc_mtx);
	
	/* destroy its device(9) mapping */
	dev = slc->slc_dev;
	destroy_dev(dev);
	free(slc, M_SLC);
}

/* 
 * Serialize data and enqueue for transmission. 
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

	m = *mp;
	cf = mtod(m, struct can_frame *);
	
	/* determine CAN frame type */
	if (cf->can_id & CAN_RTR_FLAG) 
		*bp = SLC_HC_SFF_RTR;
	else
		*bp = SLC_HC_SFF_DATA;
	
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
		if (can_bin2hex(cf, bp) < 0) {
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
	mtx_lock(&slc->slc_mtx);
	IF_LOCK(&slc->slc_outq);
	if (_IF_QFULL(&slc->slc_outq)) 
		error = ENOSPC; /* XXX: upcall??? */
	else {
		_IF_ENQUEUE(&slc->slc_outq, m);
		slc->slc_outqlen += m->m_pkthdr.len;
		error = 0;
	}
	IF_UNLOCK(&slc->slc_outq);
	mtx_unlock(&slc->slc_mtx);
out:
	*mp = m;	
	return (error);
}

/* 
 * Detach tty(4) hook. 
 */
static int 
slc_dtty(struct slc_softc *slc)
{
	struct tty *tp;
	int error;
	
	if ((tp = slc->slc_tp) != NULL) {
		tty_lock(tp);
		ttyhook_unregister(tp);
		mtx_lock(&slc->slc_mtx);
		slc->slc_tp = NULL;	
		
		if (slc->slc_inb != NULL) {
			m_freem(slc->slc_inb);
			slc->slc_inb = NULL;
		}
		IF_DRAIN(&slc->slc_outq);
		mtx_unlock(&slc->slc_mtx);
		error = 0;
	} else
		error = ESRCH;
	
	if (slc->slc_ifp != NULL)
		slc_ifinit(slc);
	
	return (error);
}

/* 
 * Get id from hooked line. 
 */
static int 
slc_gtty(struct slc_softc *slc, void *data)
{
	dev_t *d = (dev_t *)data;
	
	if (slc->slc_tp != NULL)
		*d = tty_udev(slc->slc_tp);	
	else
		*d = NODEV;

	return (0);
}

/* 
 * De-serialize rx'd data, handoff into protocol-layer. 
 */ 
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
	
	mtx_lock(&slc->slc_mtx);
		
	if ((m = slc->slc_inb) == NULL) {
		mtx_unlock(&slc->slc_mtx);
		error = EINVAL;
		goto out;
	}
	slc->slc_inb = NULL;
	mtx_unlock(&slc->slc_mtx);

	if ((ifp = slc->slc_ifp) == NULL) {
		error = ENXIO;
		goto bad;
	}

	(void)memset((bp = buf), 0, MHLEN);
	cf = (struct can_frame *)bp;

	/* determine CAN frame type */
	switch (*mtod(m, u_char *)) {
	case SLC_HC_SFF_RTR:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */
	case SLC_HC_SFF_DATA:
		break;
	case SLC_HC_EFF_RTR:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */
	case SLC_HC_EFF_DATA:
		cf->can_id |= CAN_EFF_FLAG; 
		break;
	default:
		error = EINVAL;
		goto bad;
	}
	m_adj(m, SLC_CMD_LEN);
	
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
	m_adj(m, SLC_DLC_LEN);
	
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
	m->m_pkthdr.rcvif = ifp;
 	(*ifp->if_input)(ifp, m);
out:
	return (error);			
bad:
	m_freem(m);
	goto out;
}

/* 
 * Attach tty(4) hook on selected line 
 */
static int 
slc_stty(struct slc_softc *slc, void *data, struct thread *td)
{
	struct proc *p;
	int fd;
	int error;
	
	if (td == NULL) {
		error = ESRCH;
		goto out;
	}
	
	if ((p = td->td_proc) == NULL) {
		error = ESRCH;
		goto out;
	}
	
	if (p->p_flag & P_WEXIT) {
		error = ESRCH;
		goto out;
	}
	fd = *(int *)data;
	mtx_lock(&slc->slc_mtx);
	error = ttyhook_register(&slc->slc_tp, p, fd, &slc_hook, slc);
	mtx_unlock(&slc->slc_mtx);
out:
	slc_ifinit(slc);
	return (error);
}

/*- 
 * Interface cloner and module description. 
 * 
 */ 

/* 
 * Ctor. 
 */
static int
slc_ifclone_create(struct if_clone *ifc, int unit, caddr_t data)
{
	struct slc_softc *slc;
	struct ifnet *ifp;
	struct cdev *dev;
	
	slc = malloc(sizeof(*slc), M_SLC, M_WAITOK | M_ZERO);
	if ((ifp = slc->slc_ifp = if_alloc(IFT_CAN)) == NULL) {
		free(slc, M_SLC);
		return (ENOSPC);
	}
	
	/* initialize char-device */
	dev = make_dev(&slc_cdevsw, unit,
		    UID_ROOT, GID_WHEEL, 0600, "%s%d", slc_name, unit);
	dev->si_drv1 = slc;
	slc->slc_dev = dev;
		    
	/* initialize its protective lock */
	mtx_init(&slc->slc_mtx, "slc_mtx", NULL, MTX_DEF);
	
	/* initialize queue for transmission */
	mtx_init(&slc->slc_outq.ifq_mtx, "slc_outq_mtx", NULL, MTX_DEF);
	IFQ_SET_MAXLEN(&slc->slc_outq, ifqmaxlen);
	
	/* attach */
	mtx_lock(&slc_list_mtx);
	TAILQ_INSERT_TAIL(&slc_list, slc, slc_next);
	mtx_unlock(&slc_list_mtx);
	
	ifp->if_softc = slc;
	
	if_initname(ifp, slc_name, unit);
	
	ifp->if_flags = IFF_POINTOPOINT | IFF_MULTICAST;
	ifp->if_init = slc_ifinit;
	ifp->if_start = slc_ifstart;
	ifp->if_ioctl = slc_ifioctl;
	
	can_ifattach(ifp, NULL);

	ifp->if_mtu = SLC_MTU;
	
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	
	return (0);
}

/* 
 * Dtor. 
 */
static void
slc_ifclone_destroy(struct ifnet *ifp)
{
	struct slc_softc *slc;

	slc = ifp->if_softc;
	
	mtx_lock(&slc_list_mtx);
	TAILQ_REMOVE(&slc_list, slc, slc_next);
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
		slc_ifclone_create, slc_ifclone_destroy, 0);
		error = 0;
		break;
	case MOD_UNLOAD:
		if_clone_detach(slc_cloner);
		
		mtx_lock(&slc_list_mtx);
		while ((slc = TAILQ_FIRST(&slc_list)) != NULL) {
			TAILQ_REMOVE(&slc_list, slc, slc_next);
			mtx_unlock(&slc_list_mtx);
			slc_destroy(slc);
			mtx_lock(&slc_list_mtx);
		}
		mtx_unlock(&slc_list_mtx);
		mtx_destroy(&slc_list_mtx);
		error = 0;
		break;
	default:
		error = EOPNOTSUPP;
		break;
	}
	return (error);
}

DEV_MODULE(if_slc, slc_modevent, NULL);
MODULE_DEPEND(if_slc, can, 1, 1, 1);
