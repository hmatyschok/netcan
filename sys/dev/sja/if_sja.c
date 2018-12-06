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
 * XXX: Well, work on progess ...
 */

#include <dev/sja/if_sjareg.h>

MODULE_VERSION(sja, 1);

#include "sja_if.h"

/*
 * kobj(9) method-table
 */
static device_method_t sja_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	sja_probe),
	DEVMETHOD(device_attach,	sja_attach),
	DEVMETHOD(device_detach,	sja_detach),
	DEVMETHOD(device_shutdown,	bus_generic_shutdown),

	/* sja(4) interface */
	DEVMETHOD(sja_readreg,	sja_readreg),
	DEVMETHOD(sja_writereg,	sja_writereg),

	DEVMETHOD_END
};

/*
 * ...
 */

static driver_t sja_driver = {
	"sja",
	sja_methods,
	sizeof(struct sja_softc)
};

static devclass_t sja_devclass;


static int 
sja_attach(device_t dev)
{
	struct sja_softc *sja;
	struct ifnet *ifp;
	int error, rid;
	
	sja = device_get_softc(dev);
	sja->sja_dev = dev;
	
	mtx_init(&sja->sja_mtx, device_get_nameunit(dev), 
		MTX_NETWORK_LOCK, MTX_DEF);
	
	error = 0;
	
/*
 * ...
 */	
	rid = 0;
	sja->sja_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ, &rid,
	    RF_SHAREABLE | RF_ACTIVE);

	if (sja->sja_irq == NULL) {
		device_printf(dev, "couldn't map interrupt\n");
		error = ENXIO;
		goto bad;
	}
/*
 * ...
 */	 
 
	error = bus_setup_intr(dev, sja->sja_irq, 
		INTR_TYPE_NET | INTR_MPSAFE, sja_intr, 
			NULL, sja, &sja->sja_intr_hand);

	if (error != 0) {
		device_printf(dev, "couldn't set up irq\n");
		can_ifdetach(ifp);
		goto bad;
	} 
	
out: 
	return (error);
bad:
	sja_detach(dev);
	goto out;
}

/*
 * ...
 */

static int
sja_intr(void *arg)
{
	struct sja_softc *sja;
	uint8_t status;
	int error;
	
	sc = (struct sja_softc *)arg;
	
	intr = CSR_READ_1(sja, SJA_IR);
	
	if (intr == SJA_IR_OFF)  
		error = FILTER_STRAY;
	else {
		taskqueue_enqueue(taskqueue_fast, &sja->sja_intr_task);
		error = FILTER_HANDLED;
	}
	return (error);
}

static void
sja_intr_task(void *arg)
{
	struct sja_softc *sja;
	struct ifnet *ifp;
	uint8_t intr;
	int n;
	
	sja = arg;

	SJA_LOCK(sja);

	ifp = sja->sja_ifp;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		goto done;
	
	intr = CSR_READ_1(sja, SJA_IR);

	for (n = 0; intr != SJA_IR_OFF && n < 6; n++) { 
		
		if (intr & SJA_IR_RX)
			sja_rxeof(sja);
			
		if (intr & SJA_IR_TX)
			sja_txeof(sja);
			
		if (intr & SJA_IR_ERR) {
			if (sja_error(sja, intr) != 0)
				break;
		}
		intr = CSR_READ_1(sja, SJA_IR);	
	}
done:	
	SJA_UNLOCK(sja);
}

static int 
sja_error(struct sja_softc *sja, uint8_t intr)
{
	int error = 0;
	struct ifnet *ifp;
	struct can_ifsoftc *csc;
 	struct mbuf *m;
 	struct can_frame *cf;
	uint8_t status;
	uint8_t flags;
	
	SJA_LOCK_ASSERT(sja);
	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;
	
	if ((m = m_gethdr(M_NOWAIT, MT_DATA) == NULL)) {
		error = ENOBUFS;
		goto done;
	}
	 
	(void)memset(mtod(m, caddr_t), 0, MHLEN);
	cf = mtod(m, struct can_frame *);

	/* fetch status information */	
	status = CSR_READ_1(sja, SJA_SR);	
	
	if (intr & (SJA_IR_EP|SJA_IR_EW)) {	
	
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
		
	if (intr & SJA_IR_DO) {
		cf->can_id |= CAN_ERR_DEV;
		cf->can_data[CAN_ERR_DEV_DF] |= CAN_ERR_DEV_RX_OVF; 
	
		CSR_WRITE_1(sja, SJA_CMR, SJA_CMR_CDO);
		status = CSR_READ_1(sja, SJA_SR);
/*
 * ...
 */	
	}
	
	if (intr & SJA_IR_BE) {
		flags = CSR_READ_1(sja, SJA_ECC);

		cf->can_id |= (CAN_ERR_PROTO|CAN_ERR_BE);

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

	/* pass CAN frame to upper layer */
	SJA_UNLOCK(sja);
	(*ifp->if_input)(ifp, m);
	SJA_LOCK(sja)
done:
	return (error);
}

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
	
	/* pass CAN frame to layer above */
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
 * ...
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
		if (csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK)
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

static void 
sja_encap(struct sja_softc *sja, struct mbuf *mp)
{
	int error = 0;
	struct mbuf *m;
	struct can_frame *cf;
	uint8_t status;
	uint8_t addr;
	uint16_t maddr;
	int i;
	
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
	
	/* determine CAN frame type and map dlc */
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
		cf->can_id &= CAN_EFF_MASK;
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
 * ...
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
	} else 
		if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);

/*
 * ...
 */

}

/*
 * ...
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
	uint8_t addr;
	
	SJA_LOCK_ASSERT(sja);
	ifp = sja->sja_ifp;
	csc = ifp->if_l2com;

	/* disable interrupt, if any */
	CSR_WRITE_1(sja, SJA_IER, SJA_IER_OFF);
	
	/* abort pending transmission, if any */
	CSR_WRITE_1(sja, SJA_CMR, SJA_CMR_AT);

/*
 * ...
 */ 		
	/* set clock divider */
	addr = SJA_CDR;
	sja->sja_cdr |= SJA_CDR_PELICAN;
	CSR_WRITE_1(sja, addr, sja->sja_cdr);

	/* set acceptance filter (accept all) */
	for (addr = SJA_AC0; addr < SJA_AM0; addr++)
		CSR_WRITE_1(sja, addr, 0x00);

	for (; addr <  0x18; addr++)
		CSR_WRITE_1(sja, addr, 0xff);

	/* set output control register */
	addr = SJA_ODR;
	sja->sja_ocr |= SJA_OCR_MODE_NORMAL;
	CSR_WRITE_1(sja, addr, sja->sja_ocr);

/*
 * ...
 */ 		

}



/*
 * ...
 */


