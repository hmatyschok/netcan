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
sja_intr_task(void *arg)
{
	struct sja_softc *sja;
	struct ifnet *ifp;
	uint8_t status;
	int n;
	
	sja = arg;

	SJA_LOCK(sja);

	ifp = sja->sja_ifp;

	if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
	
		status = CSR_READ_1(sja, SJA_IR);

		for (n = 0; status != SJA_IR_OFF && n < 6; n++) {
		
			if (status & SJA_IR_RX)
				sja_rxeof(sja);
			else if (status & SJA_IR_TX)
				sja_txeof(sja);
			else
				sja_error(sja);
	
			status = CSR_READ_1(sja, SJA_IR);
		}	
	}
	SJA_UNLOCK(sja);
}

static void 	
sja_rxeof(struct sja_softc *sja)
{
	struct ifnet *ifp;
	struct mbuf *m;
	struct can_frame *cf;
	uint8_t status;
	uint8_t addr;
	uint8_t maddr;
	uint8_t i;
	
	ifp = sja->sja_ifp;
	
	if ((m = m_gethdr(M_NOWAIT, MT_DATA) == NULL)) {
		if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
		addr = SJA_CMR; 
		goto done;
	}
	 
	(void)memset(mtod(m, caddr_t), 0, MHLEN);
	cf = mtod(m, struct can_frame *);

	/* fetch frame information */	
	status = CSR_READ_1(sja, SJA_FI);

	/* map addr */
	addr = SJA_ID;

	/* map id */
	if (status & SJA_FI_FF) {
		cf->can_id = CAN_EFF_FLAG;
		cf->can_id |= CSR_READ_4(sja, addr) >> 3;
		addr = SJA_DATA_EFF;
	} else {
		cf->can_id |= CSR_READ_2(sja, addr) >> 5;
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
	
	addr = SJA_CMR;
done:
	CSR_WRITE_1(s ja, addr, SJA_CMR_RRB);
	status = CSR_READ_1(sja, addr);
}
