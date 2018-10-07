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
#include "opt_slc.h"

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/endian.h>
#include <sys/fcntl.h>
#include <sys/malloc.h>
#include <sys/serial.h>
#include <sys/tty.h>
#include <sys/ttycom.h>
#include <sys/conf.h>
#include <sys/kernel.h>
#include <machine/resource.h>
#include <machine/bus.h>
#include <sys/bus.h>
#include <sys/rman.h>

/*
 * XXX: incomplete, work in progress...
 */

#include <net/if.h>
#include <net/if_types.h>
#include <net/netisr.h>

#if CAN
#include <netcan/can.h>
#include <netinet/can_var.h>
#include <netinet/can_link.h>
#else
#error "can(4) communication domain(9) not installed"
#endif

#include <net/if_slcvar.h>

#include <net/bpf.h>

/*
 * ...
 */

static MALLOC_DEFINE(M_SLC, "slc", "SLCAN Interface");

static struct mtx slc_mtx;
static TAILQ_HEAD(slc_head, slc_softc) slc_list;

/*
 * ...
 */


/*
 * Bottom-level routines.
 */
 
static th_getc_inject_t 	slc_txeof;
static th_getc_poll_t 	slc_getc_poll;
static th_rint_t 	slc_rint;
static th_rint_poll_t 	slc_rint_poll;

static struct ttyhook slc_hook = {
	.th_getc_inject = 	slc_txeof,
	.th_getc_poll = 	slc_getc_poll,
	.th_rint = 	slc_rint,
	.th_rint_poll = 	slc_rint_poll,
};

/*
 * Interface cloner.
 */ 

static struct if_clone *slc_cloner;
static const char slc_name[] = "slc";

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

static void
slc_clone_destroy(struct ifnet *ifp)
{
	struct slc_softc *slc = ifp->if_softc;

	mtx_lock(&slc_list_mtx);
	TAILQ_REMOVE(&slc_list, slc, slc_list);
	mtx_unlock(&slc_list_mtx);
	slc_destroy(slc);
}


/*
 * Module description.
 */
 
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

DECLARE_MODULE(if_slc, sl_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);

/*
 * Interface.
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
		
		/* enqueue */
		IF_LOCK(&slc->slc_outq);
		if (_IF_QFULL(&slc->slc_outq)) {
			IF_UNLOCK(&slc->slc_outq);
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
			m_freem(m);
			continue;
		}
		
		_IF_ENQUEUE(&slc->slc_outq, m);
		slc->slc_outqlen += m->m_pkthdr.len;
		IF_UNLOCK(&slc->slc_outq);
	
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
 * ...
 */

#if 0
static int 
slc_encap(struct slc_softc *slc, struct mbuf **mp)
{
	struct mbuf *m, *n;
	struct can_frame *cf;
	

}
#endif 

static int
slc_rint(struct tty *tp, char c, int flags)
{
	struct slc_softc slc;
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
	}
	
	*mtod(m, u_char *) = c;
	
	m->m_data++;
	m->m_len++;
	m->m_pkthdr.len++;

	if (c == SLC_HC_BEL || c == SLC_HC_CR || m->m_len >= SLC_MTU) {
		m->m_data = m->m_pktdat;
		slc_rxeof(slc);
	}
out1:
	mtx_unlock(&slc->slc_mtx);
out:
	return (error);
}

static void
slc_rxeof(struct slc_softc *slc)
{
	struct mbuf *m;
	u_char type;
	struct can_frame cf;
	int frame_len;
	int id_len;
	int data_len;
	uint32_t id;
	
	mtx_assert(&slc->slc_mtx, MA_OWNED);
	KASSERT((slc->slc_inb != NULL), 
		("%s: slc->slc_inb == NULL", __func__));
	m = slc->slc_inb;
	
	/* determine CAN frame type */
	type = *mtod(m, u_char *);

	(void)memset(&cf, 0, sizeof(cf));
	
	switch (type) {
	case SLC_RTR_SFF:
		cf->can_id |= CAN_RTR_FLAG;
	case SLC_DATA_SFF:
		frame_len = CAN_MTU;
		id_len = SLC_SFF_ID_LEN;
		break;
	case SLC_RTR_EFF:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */ 		
	case SLC_DATA_EFF:
		cf->can_id |= CAN_EFF_FLAG;
		fame_len = SLC_MTU; /* XXX */
		id_len = SLC_EFF_ID_LEN; 
		break;
	default:
		goto out;
	}
	m_adj(m, sizeof(u_char));
	
	/* fetch id */
	id = strtoul(mtod(m, u_char *), NULL, 16);
	cf->can_id |= id;
	m_adj(m, id_len);
	
	/* fetch dlc */
	cf->can_dlc = *mtod(m, u_char *);
	
	if (cf->can_dlc < SLC_HC_DLC_INF) 
		goto out;
	
	if (cf->can_dlc > SLC_HC_DLC_SUP) 
		goto out;

	cf->can_dlc -= SLC_HC_DLC_INF;
	m_adj(m, sizeof(u_char));
	
	/* fetch data */

/*
 * ...
 */	
	
	if ((m = m_gethdr(M_NOWAIT|M_ZERO, MT_DATA)) == NULL)
		goto out;

/*
 * ...
 */		
				
out:	
	m_freem(slc->slc_inb);
	slc->slc_inb = NULL;
}


static size_t
slc_rint_poll(struct tty *tp)
{
	
	return (1);
}