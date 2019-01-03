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

#include <dev/sja/if_sjareg.h>

/* accessor-macros */
#define CSR_WRITE_1(sja, reg, val) \
	bus_write_1((sja)->sja_res, (reg << (sja)->sja_shift), val)
#define CSR_READ_1(sja, reg) \
	bus_read_1((sja)->sja_res, (reg << (sja)->sja_shift))
#define SJA_SETBIT(sja, reg, x) \
	CSR_WRITE_1(sja, reg, CSR_READ_1(sja, reg) | (x))
#define SJA_CLRBIT(sc, reg, x) \
	CSR_WRITE_1(sja, reg, CSR_READ_1(sja, reg) & ~(x))

#define CSR_WRITE_2(sja, reg, val) \
	bus_write_2((sja)->sja_res, (reg << (sja)->sja_shift), val)
#define CSR_READ_2(sja, reg) \
	bus_read_2((sja)->sja_res, (reg << (sja)->sja_shift))

#define CSR_WRITE_4(sja, reg, val) \
	bus_write_4((sja)->sja_res, (reg << (sja)->sja_shift), val)
#define CSR_READ_4(sja, reg) \
	bus_read_4((sja)->sja_res, (reg << (sja)->sja_shift))

/* utility-macros */
#define	sja_timercmp(tvp, uvp, val)	\
	(((uvp)->tv_sec - (tvp)->tv_sec) < (val))

static int	sja_probe(device_t);
static int	sja_attach(device_t);
static int	sja_detach(device_t);

static void	sja_rxeof(struct sja_softc *);
static void	sja_txeof(struct sja_softc *);
static void	sja_error(struct sja_softc *, uint8_t);
static int	sja_intr(void *);
static void	sja_int_task(void *, int);
static void	sja_start(struct ifnet *);
static void	sja_start_locked(struct ifnet *);
static void	sja_encap(struct sja_softc *, struct mbuf **); 
static int	sja_ioctl(struct ifnet *, u_long, caddr_t data);
static void	sja_init(void *);
static void	sja_init_locked(struct sja_softc *);
static int	sja_reset(struct sja_softc *);
static int	sja_normal_mode(struct sja_softc *);
static int 	sja_set_link_timings(struct can_ifsoftc *);

/*
 * can(4) link timing capabilities 
 */
static const struct can_link_timecaps sja_timecaps = {
	.cltc_ps1_min =		1,
	.cltc_ps1_max =		16,
	.cltc_ps2_min =		1,
	.cltc_ps2_max =		8,
	.cltc_sjw_max =		4,
	.cltc_brp_min =		1,
	.cltc_brp_max =		64,
	.cltc_brp_inc =		1,
};

/*
 * kobj(9) method-table
 */
static device_method_t sja_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	sja_probe),
	DEVMETHOD(device_attach,	sja_attach),
	DEVMETHOD(device_detach,	sja_detach),
	DEVMETHOD_END
};

static driver_t sja_driver = {
	"sja",
	sja_methods,
	sizeof(struct sja_softc)
};

static devclass_t sja_devclass;

/*
 * Force controller into reset mode.
 */
static int 
sja_reset(struct sja_softc *sja)
{
	struct ifnet *ifp;
	struct can_ifsoftc *csc;
	struct timeval tv0, tv;
	uint8_t status;
	int error;

	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;

	getmicrotime(&tv0);
	getmicrotime(&tv);

	/* disable interrupts, if any */
	CSR_WRITE_1(sja, SJA_IER, SJA_IER_OFF);

	/* fetch contents of status register */
	status = CSR_READ_1(sja, SJA_MOD);
	
	/* set reset mode, until break-condition takes place */
	for (error = EIO; sja_timercmp(&tv0, &tv, 100); ) {

		if (status & SJA_MOD_RM) {
			csc->csc_flags = CAN_STATE_SUSPENDED;
			error = 0;
			break;
		}

		CSR_WRITE_1(sja, SJA_MOD, SJA_MOD_RM);
		DELAY(10);
		
		status = CSR_READ_1(sja, SJA_MOD);
		getmicrotime(&tv);
	}
	
	return (error);
}

/*
 * Force controller in normal mode.
 */
static int 
sja_normal_mode(struct sja_softc *sja)
{
	struct ifnet *ifp;
	struct can_ifsoftc *csc;
	struct timeval tv0, tv;
	uint8_t status;
	int error;

	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;

	/* flush error counters and error code capture */
	CSR_WRITE_1(sja, SJA_TEC, 0x00);
	CSR_WRITE_1(sja, SJA_REC, 0x00);
	
	status = CSR_READ_1(sja, SJA_ECC);
	
	/* clear interrupt flags */
	status = CSR_READ_1(sja, SJA_IR);
	
	getmicrotime(&tv0);
	getmicrotime(&tv);
	
	/* fetch contents of status register */
	status = CSR_READ_1(sja, SJA_MOD);
	
	/* set normal mode and enable interrupts, if any */
	for (error = EIO; sja_timercmp(&tv0, &tv, 100); ) {

		if ((status & SJA_MOD_RM) == 0) {
			status = SJA_IRE_ALL;
			status &= ~SJA_IER_BE;
#if 0
			if ((csc->csc_linkmodes & 0x10) == 0)
				status &= ~SJA_IER_BE;
#endif			
			CSR_WRITE_1(sja, SJA_IER, status);
			
			csc->csc_flags = CAN_STATE_ERROR_ACTIVE;
			error = 0;
			break;
		}

		status = SJA_MOD_RM;
	
		if (csc->csc_linkmodes & CAN_LINKMODE_LISTENONLY)
			status |= SJA_MOD_LOM;
			
		if (csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK)
			status |= SJA_MOD_STM;
		
		CSR_WRITE_1(sja, SJA_MOD, status);
		DELAY(10);
		
		status = CSR_READ_1(sja, SJA_MOD);
		getmicrotime(&tv);
	}
	
	return (error);
}

/*
 * Ctor implements device(9) driver(9) hook.
 */
static int 
sja_attach(device_t dev)
{
	struct sja_softc *sja;
	struct sja_data *sjad;
	struct ifnet *ifp;
	int rid, error;
	uint8_t addr;
		
	sja = device_get_softc(dev);
	sja->sja_dev = dev; 
	
	sja = device_get_ivar(dev);
	sja->sja_res = sjad->sjad_res;
	sja->sja_cdr = sjad->sjad_cdr;
	sja->sja_ocr = sjad->sjad_ocr;
	
	mtx_init(&sja->sja_mtx, device_get_nameunit(dev), 
		MTX_NETWORK_LOCK, MTX_DEF);	
	
	/* allocate and initialize ifnet(9) structure */
	if ((ifp = sja->sja_ifp = if_alloc(IFT_CAN)) == NULL) {
		device_printf(dev, "couldn't if_alloc(9)\n");
		error = ENOSPC;
		goto fail;
	}
	ifp->if_softc = sja;
	
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	
	ifp->if_flags = (IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);
	
	ifp->if_init = sja_init;
	ifp->if_start = sja_start;
	ifp->if_ioctl = sja_ioctl;
	
	can_ifattach(ifp, sja_set_link_timings);

	ifp->if_mtu = CAN_MTU;
	
	IFQ_SET_MAXLEN(&ifp->if_snd, SJA_IFQ_MAXLEN);
	ifp->if_snd.ifq_drv_maxlen = SJA_IFQ_MAXLEN;
	IFQ_SET_READY(&ifp->if_snd);

	/* force into reset mode */
	if ((error = sja_reset(sja)) != 0) {
		device_printf(dev, "couldn't reset\n");
		goto fail1;
	}
	
	/* set clock divider */
	sja->sja_cdr |= SJA_CDR_PELICAN;
	CSR_WRITE_1(sja, SJA_CDR, sja->sja_cdr);

	/* set acceptance filter (accept all) */
	for (addr = SJA_AC0; addr < SJA_AM0; addr++)
		CSR_WRITE_1(sja, addr, 0x00);

	for (addr = SJA_AM0; addr > SJA_AM3; addr++)
		CSR_WRITE_1(sja, addr, 0xff);

	/* set output control register */
	sja->sja_ocr |= SJA_OCR_MODE_NORMAL;
	CSR_WRITE_1(sja, SJA_OCR, sja->sja_ocr);

	/* set normal mode */
	if ((error = sja_normal_mode(sja)) != 0) {
		device_printf(dev, "couldn't set normal mode\n");
		goto fail1;
	}

	/* hook interrupts */
	TASK_INIT(&sja->sja_intr_task, 0, sja_intr_task, sja);

	error = bus_setup_intr(dev, sja->sja_res, 
		INTR_TYPE_NET | INTR_MPSAFE, sja_intr, 
			NULL, sja, &sja->sja_intr_hand);

	if (error != 0) {
		device_printf(dev, "couldn't set up irq\n");
		goto fail1;
	}
out: 
	return (error);
fail1:
	can_ifdetach(ifp);
fail:
	(void)sja_detach(dev);
	goto out;
}

/*
 * Dtor implements device(9) driver(9) hook.
 */
static int
sja_detach(device_t dev)
{
	struct sja_softc *sja;
	struct ifnet *ifp;

	sja = device_get_softc(dev);
	ifp = sja->sja_ifp;

	if (device_is_attached(dev) != 0) {
		SJA_LOCK(sja);
		sja_stop(sja);
		SJA_UNLOCK(sja);
		mtx_destroy(&sja->sja_mtx);
		taskqueue_drain(taskqueue_fast, 
			&sja->sja_intr_task);
		can_ifdetach(ifp);
	}
	
	if (sja->sja_intr_hand != NULL) {
		(void)bus_teardown_intr(dev, sja->sja_res, 
			sja->sja_intr_hand);
		sja->sja_intr_hand = NULL;
	}
	
	if (ifp != NULL)
		if_free(ifp);
	
	return (0);
}


/*
 * Copy can(4) frame from RX buffer into mbuf(9).
 */
static void 	
sja_rxeof(struct sja_softc *sja)
{
	struct ifnet *ifp;
	struct mbuf *m;
	struct can_frame *cf;
	uint8_t addr;
	uint8_t status;
	uint16_t maddr;
	int i;
	
	SJA_LOCK_ASSERT(sja);
	ifp = sja->sja_ifp;
	
again:	
	if ((m = m_gethdr(M_NOWAIT, MT_DATA) == NULL)) {
		if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		goto done;
	}
	 
	(void)memset(mtod(m, caddr_t), 0, MHLEN);
	cf = mtod(m, struct can_frame *);

	/* fetch frame information */	
	status = CSR_READ_1(sja, SJA_FI);

	/* map id */
	if (status & SJA_FI_FF) {
		cf->can_id = CAN_EFF_FLAG;
		cf->can_id |= CSR_READ_4(sja, SJA_ID) >> 3;
		addr = SJA_DATA_EFF;
	} else {
		cf->can_id |= CSR_READ_2(sja, SJA_ID) >> 5;
		addr = SJA_DATA_SFF;
	}	

	/* map dlc */
	cf->can_dlc = status & SJA_FI_DLC;

	/* map data region */
	if (status & SJA_FI_RTR) {
		cf->can_id |= CAN_RTR_FLAG;
		maddr = 0;
	} else
		maddr = addr + cf->can_dlc;
	
	for (i = 0; addr < maddr; addr++, i++) 
		cf->can_data[i] = CSR_READ_1(sja, addr);

	m->m_len = m->m_pkthdr.len = sizeof(*cf);
	m->m_pkthdr.rcvif = ifp;
	
	/* pass can(4) frame to layer above */
	SJA_UNLOCK(sja);
 	(*ifp->if_input)(ifp, m);
	SJA_LOCK(sja);
	
done:	/* SJA1000, 6.4.4, note 4 */
	CSR_WRITE_1(sja, SJA_CMR, SJA_CMR_RRB);
	status = CSR_READ_1(sja, SJA_SR);
	
	if (status & SJA_SR_RBS)
		goto again;
}

/*
 * TX buffer released or TX complete.
 */
static void 
sja_txeof(struct sja_softc *sja)
{
	struct ifnet *ifp;
	struct can_ifsoftc *csc;
	uint8_t status;
	
	SJA_LOCK_ASSERT(sja);

	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;
	
	status = CSR_READ_1(sja, SJA_SR);
	
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
		
	if (csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK 
		&& (status & SJA_SR_TCS) == 0) {
		if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		(*ifp->if_start)(ifp);
	} else {
		status = CSR_READ_1(sja, SJA_FI) & 0x0f; 
		if_inc_counter(ifp, IFCOUNTER_OBYTES, status);
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
	}
}

/*
 * Service primitive on interrupt.
 */ 
static int
sja_intr(void *arg)
{
	struct sja_softc *sja;
	uint8_t status;
	int error;
	
	sc = (struct sja_softc *)arg;
	
	status = CSR_READ_1(sja, SJA_IR);
	
	if (status == SJA_IR_OFF)  
		error = FILTER_STRAY;
	else {
		taskqueue_enqueue(taskqueue_fast, &sja->sja_intr_task);
		error = FILTER_HANDLED;
	}
	return (error);
}

static void
sja_intr_task(void *arg, int pending)
{
	struct sja_softc *sja;
	struct ifnet *ifp;
	uint8_t status;
	int n;
	
	sja = arg;

	SJA_LOCK(sja);

	ifp = sja->sja_ifp;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		goto done;
	
	status = CSR_READ_1(sja, SJA_IR);

	for (n = 0; status != SJA_IR_OFF && n < 6; n++) { /* XXX */
		
		if (status & SJA_IR_RX)
			sja_rxeof(sja);
			
		if (status & SJA_IR_TX)
			sja_txeof(sja);
			
		if (status & SJA_IR_ERR) {
			if (sja_error(sja, status) != 0)
				break;
		}
		intr = CSR_READ_1(sja, SJA_IR);	
	}
done:	
	SJA_UNLOCK(sja);
}

/*
 * Service primitive for exception handling.
 */
static int 
sja_error(struct sja_softc *sja, uint8_t intr)
{
	struct ifnet *ifp;
	struct can_ifsoftc *csc;
 	struct mbuf *m;
 	struct can_frame *cf;
	uint8_t status;
	uint8_t flags;
	int error = 0;
	
	SJA_LOCK_ASSERT(sja);
	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;
	
	if ((m = m_gethdr(M_NOWAIT, MT_DATA) == NULL)) {
		error = ENOBUFS;
		goto done;
	}
	 
	(void)memset(mtod(m, caddr_t), 0, MHLEN);
	cf = mtod(m, struct can_frame *);
	cf->can_id |= CAN_ERR_FLAG;

	/* fetch status information */	
	status = CSR_READ_1(sja, SJA_SR);	
	
	/*  error passive / warning condition */
	if (intr & (SJA_IR_EP | SJA_IR_EW)) {	
	
		if (status & SJA_SR_BS)
			flags = CAN_STATE_BUS_OFF;
		else if (status & SJA_SR_ES)
			flags = CAN_STATE_ERROR_WARNING;
		else
			flags = CAN_STATE_ERROR_ACTIVE;
		
		if (flags != csc->can_flags) {
			csc->can_flags = flags;
		
			if (csc->can_flags == CAN_STATE_BUS_OFF) {
				ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
				sja_init_locked(sc);
			}
	
			cf->can_id |= CAN_ERR_DEV;
/*
 * ...
 */		
		}	
	}
	
	/* data overrun condition */ 	
	if (intr & SJA_IR_DO) {
		cf->can_id |= CAN_ERR_DEV;
		cf->can_data[CAN_ERR_DEV_DF] |= CAN_ERR_DEV_RX_OVF; 
	
		CSR_WRITE_1(sja, SJA_CMR, SJA_CMR_CDO);
		status = CSR_READ_1(sja, SJA_SR);
/*
 * ...
 */	
	}
	
	/* bus error condition */
	if (intr & SJA_IR_BE) {
		flags = CSR_READ_1(sja, SJA_ECC);

		cf->can_id |= (CAN_ERR_PROTO | CAN_ERR_BE);

		/* map error condition, if any */
		if ((flags & SJA_ECC_ERR_MASK) == SJA_ECC_BE)
			cf->can_data[CAN_ERR_PROTO_DF] |= CAN_ERR_PROTO_BIT;
		else if ((flags & SJA_ECC_ERR_MASK) == SJA_ECC_FMT)
			cf->can_data[CAN_ERR_PROTO_DF] |= CAN_ERR_PROT_FORM;
		else if ((flags & SJA_ECC_ERR_MASK) == ECC_STUFF)
			cf->can_data[CAN_ERR_PROTO_DF] |= CAN_ERR_PROT_STUFF;
			
		/* map tx error condition, if any */ 
		if ((flags & ECC_DIR) == 0)
			cf->can_data[CAN_ERR_PROTO_DF] |= CAN_ERR_PROT_TX;	
	
		/* map error location */
		cf->can_data[CAN_ERR_PROTO_LOC_DF] |= flags & SJA_ECC_SEG;
/*
 * ...
 */	
	}
	
	/* arbitriation lost condition */
	if (intr & SJA_IR_AL) {
		flags = CSR_READ_1(sja, SJA_ALC);

		cf->can_id |= CAN_ERR_AL;
		cf->can_data[CAN_ERR_AL_DF] |= flags & SJA_ALC_MASK;
	} 
	
	/* map error count */
	cf->can_data[CAN_ERR_RX_DF] = CSR_READ_1(sja, SJA_REC);
	cf->can_data[CAN_ERR_TX_DF] = CSR_READ_1(sja, SJA_TEC);

	m->m_len = m->m_pkthdr.len = sizeof(*cf);
	m->m_pkthdr.rcvif = ifp;

	/* pass can(4) frame to upper layer */
	SJA_UNLOCK(sja);
	(*ifp->if_input)(ifp, m);
	SJA_LOCK(sja);
done:
	return (error);
}

/*
 * Copy can(4) frame into TX buffer.
 */
static void 
sja_encap(struct sja_softc *sja, struct mbuf **mp)
{
	struct mbuf *m;
	struct can_frame *cf;
	uint8_t status;
	uint8_t addr;
	uint16_t maddr;
	int i, error = 0;
	
	SJA_LOCK_ASSERT(sja);
	
	/* get a writable copy, if any */
	if (M_WRITABLE(*mp) == 0) {
		m = m_dup(*mp, M_NOWAIT);
		if (m == NULL) {
			error = ENOBUFS;
			goto out;
		}
		m_freem(*mp);
	} else
		m = *mp;
		
	cf = mtod(m, struct can_frame *);
	
	/* determine can(4) frame type and map dlc */
	if (cf->can_id & CAN_EFF_FLAG) { 
		status = SJA_FI_FF;  
	
		if (cf->can_id & CAN_RTR_FLAG) 
			status |= SJA_FI_RTR;		
	} else 
		status = 0x00;

	status |= cf->can_dlc & SJA_FI_DLC;

	CSR_WRITE_1(sja, SJA_FI, status);
	
	/* map id */
	if (cf->can_id & CAN_EFF_FLAG) { 
		cf->can_id &= CAN_EFF_MASK;
		cf->can_id <<= 3;
		
		CSR_WRITE_4(sja, SJA_ID, cf->can_id);
		
		addr = SJA_DATA_EFF;
	} else {
		cf->can_id &= CAN_SFF_MASK;
		cf->can_id <<= 5;
		
		CSR_WRITE_2(sja, SJA_ID, cf->can_id);
		
		addr = SJA_DATA_SFF;
	}
	
	maddr = addr + cf->can_dlc;
	
	for (i = 0; addr < maddr; addr++, i++) 
		CSR_WRITE_1(sja, addr, cf->can_data[i]);

	m_freem(m); /* XXX */
	*mp = NULL;
out:	
	return (error);
}

/*
 * Transmit can(4) frame.
 */
static void
sja_start(struct ifnet *ifp)
{
	struct sja_softc *sja;

	sja = ifp->if_softc;
	SJA_LOCK(sja);
	sja_start_locked(ifp);
	SJA_UNLOCK(sja);
}

static void
sja_start_locked(struct ifnet *ifp)
{
	struct sja_softc *sja;
	struct can_ifsoftc *csc;
	struct mbuf *m;
	uint8_t status;
	
	sja = ifp->if_softc;	
	SJA_LOCK_ASSERT(sja);
		
	csc = ifp->if_l2com;
			
	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;
			
	for (;;) {
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL) 
			break;

		if (sja_encap(sja, &m) != 0) {
			if (m == NULL)
				break;
			
			IFQ_DRV_PREPEND(&ifp->if_snd, m);
			ifp->if_drv_flags |= IFF_DRV_OACTIVE;
			break;
		}	
		
		/* IAP on bpf(4) */
		can_bpf_mtap(ifp, &m);		
	
		/* notify controller for transmission */
		if (csc->csc_linkmodes & CAN_LINKMODE_ONE_SHOT)
			status = SJA_CMR_AT;
		else 
			status = 0x00;

		if (csc->csc_linkmodes & CAN_LINKMODE_LOOPBACK)
			status |= SJA_CMR_SRR;
		else
			ststus |= SJA_CMR_TR;

		CSR_WRITE_1(sja, SJA_CMR, status);
		status = CSR_READ_1(sja, SJA_SR);
	}						
}

/*
 * Initialize sja(4) +controller. 
 */ 
static void 
sja_init(void *xsc)
{
	struct sja_softc *sja;
	
	sja = xsc;
	
	SJA_LOCK(sja);
	sja_init_locked(sja);
	SJA_UNLOCK(sja);
}

static void 
sja_init_locked(struct sja_softc *sja)
{
	struct ifnet *ifp;
	struct can_ifsoftc *csc;
	struct timeval tv0, tv;
	uint8_t status, addr;
	
	SJA_LOCK_ASSERT(sja);
	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING)
		return;

	/* disable interrupts and abort pending transmission, if any */
	sja_stop(sja);

	/* force controller into reset mode */
	getmicrotime(&tv0);
	getmicrotime(&tv);
	
	status = CSR_READ_1(sja, SJA_MOD);
		
	for (; sja_timercmp(&tv0, &tv, 100);) {

		CSR_WRITE_1(sja, SJA_MOD, SJA_MOD_RM);
		DELAY(10);
		
		status = CSR_READ_1(sja, SJA_MOD);
		getmicrotime(&tv);
	}
	
	if (status & SJA_MOD_RM) {
		csc->csc_flags = CAN_STATE_SUSPENDED;
	
		/* set clock divider */
		sja->sja_cdr |= SJA_CDR_PELICAN;
		CSR_WRITE_1(sja, SJA_CDR, sja->sja_cdr);

		/* set acceptance filter (accept all) */
		for (addr = SJA_AC0; addr < SJA_AM0; addr++)
			CSR_WRITE_1(sja, addr, 0x00);

		for (addr = SJA_AM0; addr > SJA_AM3; addr++)
			CSR_WRITE_1(sja, addr, 0xff);

		/* set output control register */
		sja->sja_ocr |= SJA_OCR_MODE_NORMAL;
		CSR_WRITE_1(sja, SJA_OCR, sja->sja_ocr);

		/* flush error counters and error code capture */
		CSR_WRITE_1(sja, SJA_TEC, 0x00);
		CSR_WRITE_1(sja, SJA_REC, 0x00);
	
		status = CSR_READ_1(sja, SJA_ECC);
		
		/* clear interrupt flags */
		status = CSR_READ_1(sja, SJA_IR);

		/* leave reset mode */
		getmicrotime(&tv0);
		getmicrotime(&tv);

		status = CSR_READ_1(sja, SJA_MOD);
	
		/* force controller into normal mode */ 
		for (; sja_timercmp(&tv0, &tv, 100);) {
			status = SJA_MOD_RM;
	
			if (csc->csc_linkmodes & CAN_LINKMODE_LISTENONLY)
				status |= SJA_MOD_LOM;
			
			if (csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK)
				status |= SJA_MOD_STM;
		
			CSR_WRITE_1(sja, SJA_MOD, status);
			DELAY(10);
		
			status = CSR_READ_1(sja, SJA_MOD);
			getmicrotime(&tv);
		}
	}
	
	/* enable interrupts, if any */
	if ((status & SJA_MOD_RM) == 0) {
		status = SJA_IRE_ALL;
		status &= ~SJA_IER_BE;
#if 0
		if ((csc->csc_linkmodes & 0x10) == 0)
			status &= ~SJA_IER_BE;
#endif			
		CSR_WRITE_1(sja, SJA_IER, status);
			
		csc->csc_flags = CAN_STATE_ERROR_ACTIVE;
	
		ifp->if_drv_flags |= IFF_DRV_RUNNING;
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	}
}

/*
 * Disable interrupts and abort pending transmission, if any.
 */
static void 
sja_stop(struct sja_softc *sja)
{
	struct ifnet *ifp;
	
	SJA_LOCK_ASSERT(sja);
	ifp = sja->sja_ifp;

	ifp->if_drv_flags |= ~(IFF_DRV_RUNNNING | IFF_DRV_OACTIVE);

	CSR_WRITE_1(sja, SJA_IER, SJA_IER_OFF);
	CSR_WRITE_1(sja, SJA_CMR, SJA_CMR_AT);
}



/*
 * ...
 */

static int
sja_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct sja_softc *sja;
	struct ifreq *ifr;
	int error;

	sja = ifp->if_softc;
	ifr = (struct ifreq *)data;
	error = 0;

	switch (cmd) {
	case SIOCSIFFLAGS:
		SJA_LOCK(sja);
		if (ifp->if_flags & IFF_UP) 
			sja_init_locked(sja);
		else {
			if (ifp->if_drv_flags & IFF_DRV_RUNNING)
				sja_stop(sja);
		}
		SJA_UNLOCK(sja);
		break;
	default:
		error = can_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

/*
 * Maps to generic software-context, called by can_set_netlink(9).
 */
static int 
sja_set_link_timings(struct can_ifsoftc *csc)
{
	struct sja_softc *sja;
	struct can_link_timings *clt;
	uint8_t btr0, btr1;
	
	sja = csc->csc_ifp->if_softc;
	clt = &csc->csc_timings;
	
	/* baud rate prescalar and synchroniziation jump */
	btr0 = ((clt->clt_brp - 1) & SJA_BTR0_BRP_MASK);
	btr0 |= (((clt->clt_sjw - 1) & SJA_BTR0_CLT_MASK) << 6);
	
	CSR_WRITE_1(sja, SJA_BTR0, btr0);
	
	/* time segments and sampling, if any */
	btr1 = ((clt->clt_prop + clt->clt_ps1 - 1) & 0x0f);
	btr1 |= (((clt->clt_sg2 - 1) & 0x07) << 4);

	if (csc->csc_linkmodes & CAN_CTRLMODE_3_SAMPLES)
		btr1 |= SJA_BTR1_SAM;
	
	CSR_WRITE_1(sja, SJA_BTR1, btr1);

	return (0);
}

MODULE_VERSION(sja, 1);
