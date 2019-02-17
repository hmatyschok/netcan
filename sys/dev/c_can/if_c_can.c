/*
 * Copyright (c) 2018, 2019 Henning Matyschok
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

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/socket.h>
#include <sys/malloc.h>
#include <sys/module.h>
#include <sys/bus.h>

#include <net/if.h>
#include <net/if_can.h>
#include <net/if_var.h>
#include <net/if_types.h>

/*
 * XXX: Well, work in progess ...
 */

#include <dev/c_can/if_c_canreg.h>

#include "c_can_if.h"

/*
 * Subr.
 */
static void	c_can_rxeof(struct c_can_softc *);
static void	c_can_txeof(struct c_can_softc *);
static void	c_can_error(struct c_can_softc *, uint8_t);
static int	c_can_intr(void *);
static void	c_can_start(struct ifnet *);
static void	c_can_start_locked(struct ifnet *);
static void	c_can_encap(struct c_can_softc *, struct mbuf **); 
static int	c_can_ioctl(struct ifnet *, u_long, caddr_t data);
static void	c_can_init(void *);
static void	c_can_init_locked(struct c_can_softc *);
static int	c_can_reset(struct c_can_softc *);
static int	c_can_normal_mode(struct c_can_softc *);
static int 	c_can_set_link_timings(struct can_ifsoftc *);

/*
 * can(4) link timing capabilities 
 */
static const struct can_link_timecaps c_can_timecaps = {
	.cltc_ps1_min =		2,
	.cltc_ps1_max =		16,
	.cltc_ps2_min =		1,
	.cltc_ps2_max =		8,
	.cltc_sjw_max =		4,
	.cltc_brp_min =		1,
	.cltc_brp_max =		1024,
	.cltc_brp_inc =		1,
};

/*
 * ...
 */

static int
c_can_probe(device_t dev)
{
	
	device_set_desc(dev, "Bosch C_CAN network interface");
	
	return (BUS_PROBE_SPECIFIC);
}

static int 
c_can_attach(device_t dev)
{
	struct c_can_softc *cc;
	struct ifnet *ifp;
	struct canif_softc *csc;
	int rid, i, error;
	uint16_t status;
		
	cc = device_get_softc(dev);
	cc->cc_dev = dev; 
	
	cc->cc_freq = *(uint32_t *)device_get_ivar(dev);

	mtx_init(&cc->cc_mtx, device_get_nameunit(dev), 
		MTX_NETWORK_LOCK, MTX_DEF);	
	
	C_CAN_RESET(cc->cc_dev);
	
	/* allocate interrupt */
	rid = 0;
	cc->cc_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, 
		&rid, RF_SHAREABLE | RF_ACTIVE);

	if (cc->cc_irq == NULL) {
		device_printf(dev, "couldn't map interrupt\n");
		error = ENXIO;
		goto fail;
	}
	
	/* allocate and initialize ifnet(9) structure */
	if ((ifp = cc->cc_ifp = if_alloc(IFT_CAN)) == NULL) {
		device_printf(dev, "couldn't if_alloc(9)\n");
		error = ENOSPC;
		goto fail;
	}
	ifp->if_softc = cc;
	
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	
	ifp->if_flags = (IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	
	ifp->if_init = c_can_init;
	ifp->if_start = c_can_start;
	ifp->if_ioctl = c_can_ioctl;
	
	can_ifattach(ifp, cc->cc_freq);
	
	IFQ_SET_MAXLEN(&ifp->if_snd, C_CAN_IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = C_CAN_IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);

	/* initialize taskqueue(9) */
	TASK_INIT(&cc->cc_intr_task, 0, c_can_intr_task, cc);

	/* enable automatic retransmission */	
	status = C_CAN_READ_2(dev, C_CAN_CR);
	status &= ~C_CAN_CR_DAR;
	
	C_CAN_WRITE_2(cc->cc_dev, C_CAN_CR, status);

	/* configure message objects */
	for (i = 0; i < 32; i++) {	/* XXX */
		C_CAN_WRITE_2(cc->cc_dev, C_CAN_IF1_ID0, 0x0000);
		C_CAN_WRITE_2(cc->cc_dev, C_CAN_IF1_ID1, 0x0000);
		
		C_CAN_WRITE_2(cc->cc_dev, C_CAN_IF1_MCR, 0x0000);
		
		c_can_msg_obj_upd(cc, i, 
			(C_CAN_IFX_CMMR_MO_INVAL | C_CAN_IFX_CMMR_WR_RD), 
				C_CAN_IF_RX);
	}
	
	for (i = 0; i < 16; i++) {	/* XXX */
		C_CAN_WRITE_4(cc->cc_dev, C_CAN_IF1_MASK0, 0x20000000);
		C_CAN_WRITE_4(cc->cc_dev, C_CAN_IF1_ID0, C_CAN_IFX_ARB_MSG_VAL);
		
		C_CAN_WRITE_2(cc->cc_dev, C_CAN_IF1_MCR, C_CAN_IFX_MCR_RX); 
	
		c_can_msg_obj_upd(cc, i, C_CAN_IFX_CMMR_RX_INIT, C_CAN_IF_RX);
	}	

	/* set-up status register */
	C_CAN_WRITE_2(cc->cc_dev, C_CAN_SR, C_CAN_SR_UNUSED_ERR);

	error = c_can_set_link_timings(cc);

	if (error != 0) {
		device_printf(dev, "couldn't set up Bit Timing Register\n");
		goto fail1;
	}
	

	/* XXX: trigger led(4)s ... */
	
	/* hook interrupts */
	error = bus_setup_intr(dev, cc->cc_irq, 
		INTR_TYPE_NET | INTR_MPSAFE, cc_intr, 
			NULL, sja, &cc->cc_intr);

	if (error != 0) {
		device_printf(dev, "couldn't set up irq\n");
		goto fail1;
	}

	/* enable interrupts */
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
	status |= C_CAN_CR_INTR_MASK;
	C_CAN_WRITE_2(cc->cc_dev, C_CAN_CR, status);
out: 
	return (error);
fail1:
	can_ifdetach(ifp);
fail:
	(void)c_can_detach(dev);
	goto out;
}

static int
c_can_detach(device_t dev)
{
	struct c_can_softc *cc;
	struct ifnet *ifp;

	cc = device_get_softc(dev);
	ifp = cc->cc_ifp;

	if (device_is_attached(dev) != 0) {
		C_CAN_LOCK(cc);
		c_can_stop(cc);
		C_CAN_UNLOCK(cc);
		taskqueue_drain(taskqueue_fast, &cc->cc_intr_task);
		can_ifdetach(ifp);
	}
	
	if (cc->cc_intr != NULL) {
		(void)bus_teardown_intr(dev, cc->cc_irq, cc->cc_intr);
		cc->cc_intr = NULL;
	}
	
	if (cc->cc_irq != NULL)
		(void)bus_release_resource(dev, SYS_RES_IRQ, cc->cc_irq);
	
	if (ifp != NULL)
		if_free(ifp);
	
	mtx_destroy(&cc->cc_mtx);
	
	C_CAN_RESET(dev);
	
	return (0);
}

/*
 * ...
 */

static void 
c_can_msg_obj_upd(struct c_can_softc *cc, int n, int cmd, int ifx)
{
	int port, val, i;
	uint16_t status;

	port = C_CAN_IF1_CMR + ifx * (C_CAN_IF2_CMR - C_CAN_IF1_CMR);
	val = (cmd << 0x10) | n;

	C_CAN_WRITE_4(cc->cc_dev, port, val);
	
	for (i = 0; i < 6; i++) {	/* XXX */
		status = C_CAN_READ_2(cc->cc_dev, port);
		
		if ((status & C_CAN_IFX_CMR_BUSY) == 0)
			break;
		
		DELAY(1);
	}
}
/*
 * ...
 */
 
static int
c_can_intr(void *arg)
{
	struct c_can_softc *cc;
	uint16_t status;
	
	cc = (struct c_can_softc *)arg;

	status = C_CAN_READ_2(cc->cc_dev, C_CAN_IR);
	if (status == 0)
		return (FILTER_STRAY);

	/* disable interrupts */
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
	status &= ~C_CAN_CR_INTR_MASK

	C_CAN_WRITE_2(cc->cc_dev, C_CAN_CR, status);

	taskqueue_enqueue(taskqueue_fast, &cc->cc_inttr_task);
	
	return (FILTER_HANDLED);
}

static void
c_can_intr_task(void *arg, int npending)
{
	struct c_can_softc *cc;	
	uint16_t status;
	
	cc = (struct c_can_softc *)arg; 
	
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_SR);
	C_CAN_WRITE_2(cc->cc_dev, C_CAN_SR, C_CAN_SR_UNUSED_ERR);
	
	/*
	 * ...
	 */
	
	c_can_rxeof(cc);
	
	c_can_txeof(cc); 
	 
	/* enable interrupts */
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
	status |= C_CAN_CR_INTR_MASK;
	C_CAN_WRITE_2(cc->cc_dev, C_CAN_CR, status);
} 

static void
c_can_rxeof(struct c_can_softc *cc)
{
	struct ifnet *ifp;
	struct mbuf *m;
	uint16_t rxd, i, j, mask, status, k;
	uint32_t arb;
	uint8_t addr, maddr;
	
	C_CAN_LOCK_ASSERT(cc);
	ifp = cc->cc_ifp;
	
	rxd = C_CAN_READ_2(cc->cc_dev, C_CAN_NEW_DAT0);
	
	for (i = 0, j = 1; rxd != 0 || i < 16; i++, j++) {	/* XXX */
		
		mask = (1 << i);

		if ((rxd & mask) == 0) 
			continue;
		
		rxd &= ~mask;			
		
		c_can_msg_obj_upd(cc, j, C_CAN_IFX_CMMR_RX_LOW, C_CAN_IFX_RX);
		
		status = C_CAN_READ_2(cc->cc_dev, C_CAN_IF1_MCR);
		
		if ((status & IF_MCONT_NEWDAT) == 0)
			continue;
		
		if ((m = m_gethdr(M_NOWAIT, MT_DATA) == NULL)) {
			if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
			continue;
		}
	 
		(void)memset(mtod(m, caddr_t), 0, MHLEN);
		cf = mtod(m, struct can_frame *);
		
		if (status & C_CAN_IFX_MCR_MSG_LST) {
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
			
			status &= ~(C_CAN_IFX_MCR_INT_PND
				| C_CAN_IFX_MCR_MSG_LST
				| C_CAN_IFX_MCR_NEW_DAT);
			C_CAN_WRITE_2(cc->cc_dev, C_CAN_IF1_MCR, status);
			
			c_can_obj_upd(cc, j, 
				(C_CAN_IFX_CMMR_CONTROL | C_CAN_IFX_CMMR_WR_RD), 
					C_CAN_IF_RX);

			cf->can_id |= (CAN_ERR_FLAG|CAN_ERR_DEV);
			cf->can_data[CAN_ERR_DF_DEV] = CAN_ERR_DEV_RX_OVF;
		} else {
		
			/* determine frame type and map id */
			arb = C_CAN_READ_4(cc->cc_dev, C_CAN_IF1_ARB0);
		
			if (arb &C_CAN_IFX_ARBX_MSG_XTD) {
				cf->can_id = CAN_EFF_FLAG;
				cf->can_id |= (arb & CAN_EFF_MASK);
			} else 
				cf->can_id |= ((arb >> 18) & CAN_SFF_MASK);
		
			/* map dlc */
			cf->can_dlc = status & C_CAN_IFX_MCR_DLC_MASK;
		
			/* map data region */
			if (arb & C_CAN_IFX_ARBX_TX) {
				cf->can_id |= CAN_RTR_FLAG;
				maddr = addr = 0;
			} else {
				addr = C_CAN_IF1_DATA0;
				maddr = addr + cf->can_dlc;
			}
			
			for (k = 0; addr < maddr; addr++, k++) 
				cf->can_data[k] = C_CAN_READ_1(cc->cc_dev, addr);
		}
		m->m_len = m->m_pkthdr.len = sizeof(*cf);
		m->m_pkthdr.rcvif = ifp;

		/* pass can(4) frame to upper layer */
		C_CAN_UNLOCK(cc);
		(*ifp->if_input)(ifp, m);
		C_CAN_LOCK(cc);	
		
		if ((status & C_CAN_IFX_MCR_MSG_LST) == 0)
			c_can_msg_obj_upd(cc, j, C_CAN_IFX_CMMR_NEW_DAT, C_CAN_IF_RX);	
	}
}

 
/*
 * ...
 */

static int
c_can_cr_wait(struct c_can_softc *cc, uint16_t control, int iswitch)
{
	struct timeval tv0, tv;
	uint16_t status, eval;
	int error;
	
	getmicrotime(&tv0);
	getmicrotime(&tv);

	C_CAN_WRITE_2(cc->cc_dev, C_CAN_CR, control);	
	
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
	eval = (iswitch != 0) ? C_CAN_CR_INIT : 0;
	
	for (error = EIO; c_can_timercmp(&tv0, &tv, 10000);) {
		DELAY(10);
		
		if ((status & C_CAN_CR_INIT) == eval) {
			error = 0;
			break;
		}
		status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
		getmicrotime(&tv);
	}
	
	return (error);
}

static int 
c_can_set_link_timings(struct c_can_softc *cc)
{
	struct can_ifsoftc *csc
	struct can_link_timings *clt;
	uint16_t btr, brpe, status;
	int error;
	
	csc = cc->cc_ifp->if_l2com;
	clt = &csc->csc_timings;
	
	btr = ((clt->clt_brp - 1) & C_CAN_BTR_BRP_MASK);
	btr |= ((clt->clt_sjw - 1) << 6);
	btr |= ((clt->clt_prop + clt->clt_ps1 - 1) << 8);
	btr |= ((clt->clt_ps2 - 1) << 12);
	
	brpe = (((clt->clt_brp - 1) & C_CAN_BTR_BRP_MASK) >> 6);
	
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
	status &= ~C_CAN_CR_INIT;

	error = c_can_cr_wait(cc, (C_CAN_CR_CCE|C_CAN_CR_INIT), 1);

	if (error == 0) {
		C_CAN_WRITE_2(cc->cc_dev, C_CAN_BTR, btr);
		C_CAN_WRITE_2(cc->cc_dev, C_CAN_BRPER, brpe);
	
		error = c_can_cr_wait(cc, status, 0);
	}
	return (error);
}	
	
/*
 * ...
 */
 
static void 
c_can_stop(struct c_can_softc *cc)
{
	struct ifnet *ifp;
	uint16_t status;
	
	C_CAN_LOCK_ASSERT(cc);
	ifp = cc->cc_ifp;

	ifp->if_drv_flags |= ~(IFF_DRV_RUNNNING | IFF_DRV_OACTIVE);
	
	status = C_CAN_READ_2(cc->cc_dev, C_CAN_CR);
	status &= ~C_CAN_CR_INTR_MASK

	C_CAN_WRITE_2(cc->cc_dev, C_CAN_CR, status);
}
	
/*
 * ...
 */

static int
c_can_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct c_can_softc *cc;
	struct ifreq *ifr;
	struct ifdrv *ifd;
	int error;

	sja = ifp->if_softc;
	ifr = (struct ifreq *)data;
	ifd = (struct ifdrv *)data;
	error = 0;

	switch (cmd) {
	case SIOCGDRVSPEC:
	case SIOCSDRVSPEC:
		switch (ifd->ifd_cmd) {
		case CANSLINKTIMINGS:
			C_CAN_LOCK(cc);
			error = c_can_set_link_timings(cc);
			C_CAN_UNLOCK(cc);
			break;
		default:
			break;
		}
		break;
	case SIOCSIFFLAGS:
		C_CAN_LOCK(cc);
		if (ifp->if_flags & IFF_UP) 
			c_can_init_locked(cc);
		else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				c_can_stop(cc);
		}
		C_CAN_UNLOCK(cc);
		break;
	default:
		error = can_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

/*
 * Common I/O subr.
 */
static uint8_t
c_can_read_1(device_t dev, int port)
{
	
	return (C_CAN_READ_1(device_get_parent(dev), port));	
}

static uint16_t
c_can_read_2(device_t dev, int port)
{
	
	return (C_CAN_READ_2(device_get_parent(dev), port));	
}

static uint32_t
c_can_read_4(device_t dev, int port)
{
	
	return (C_CAN_READ_4(device_get_parent(dev), port));	
}

static void
c_can_write_1(device_t dev, int port, uint8_t val)
{
	
	C_CAN_WRITE_1(device_get_parent(dev), port, val);	
}

static void
c_can_write_2(device_t dev, int port, uint16_t val)
{
	
	C_CAN_WRITE_2(device_get_parent(dev), port, val);	
}

static void
c_can_write_4(device_t dev, int port, uint32_t val)
{
	
	C_CAN_WRITE_4(device_get_parent(dev), port, val);	
}

/*
 * Software reset.
 */
static void
c_can_reset(device_t dev)
{
	
	C_CAN_RESET(device_get_parent(dev));	
}

/* 
 * Hooks for the operating system.
 */
static device_method_t c_can_methods[] = {
	/* device(9) interface */
	DEVMETHOD(device_probe, 	c_can_probe),
	DEVMETHOD(device_attach,	c_can_attach),
	DEVMETHOD(device_detach,	c_can_detach),
	
	/* c_can(4) interface */
	DEVMETHOD(c_can_read_1,	c_can_read_1),
	DEVMETHOD(c_can_read_2,	c_can_read_2),
	DEVMETHOD(c_can_read_4,	c_can_read_4),

	DEVMETHOD(c_can_write_1,	c_can_write_1),
	DEVMETHOD(c_can_write_2,	c_can_write_2),
	DEVMETHOD(c_can_write_4,	c_can_write_4),
		
	DEVMETHOD(c_can_reset),		c_can_reset),	
		
	DEVMETHOD_END
};

static driver_t c_can_driver = {
	"c_can",
	c_can_methods,
	sizeof(struct c_can_softc)
};

devclass_t c_can_devclass;

MODULE_VERSION(c_can, 1);

