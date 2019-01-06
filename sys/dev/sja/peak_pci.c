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
#include <sys/endian.h>
#include <sys/systm.h>
#include <sys/sockio.h>
#include <sys/mbuf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/socket.h>
#include <sys/sysctl.h>

#include <net/if.h>
#include <net/if_can.h>
#include <net/if_var.h>
#include <net/if_types.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

#include <dev/sja/if_sjareg.h>
#include <dev/sja/peak_pcireg.h>

#define CSR_WRITE_1(sc, reg, val) \
	bus_write_1((sc)->pk_res, reg, val)
#define CSR_READ_1(sja, reg) \
	bus_read_1((sc)->pk_res, reg)

#define CSR_WRITE_2(sc, reg, val) \
	bus_write_2((sc)->pk_res, reg, val)
#define CSR_READ_2(sja, reg) \
	bus_read_2((sc)->pk_res, reg)

#define CSR_WRITE_4(sc, reg, val) \
	bus_write_4((sc)->pk_res, reg, val)
#define CSR_READ_4(sja, reg) \
	bus_read_4((sc)->pk_res, reg)

/*
 * Device driver(9) for PEAK PCAN PCI family 
 * cards implements proxy pattern on pci(4) 
 * bus for instances of the sja(4) contoller.
 *
 * XXX: Well, work on progess ...
 */

MODULE_DEPEND(peak_pci, pci, 1, 1, 1);
MODULE_DEPEND(peak_pci, sja, 1, 1, 1); 
MODULE_DEPEND(peak_pci, can, 1, 1, 1);

static const struct peak_type {
	uint16_t 	pk_vid;
	uint16_t 	pk_did;
	const char 	*pk_name;
} pk_devs[] = {
	{ PEAK_VENDORID, PEAK_DEVICEID_PCI, 
		"PCAN PCI/PCIe card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_PCIEC, 
		"PCAN PCI ExpressCard" },
	{ PEAK_VENDORID, PEAK_DEVICEID_PCIE, 
		"PCAN nextgen PCIe card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_CPCI, 
		"PCAN nextgen cPCI card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_MPCI, 
		"PCAN nextgen miniPCI card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_PC_104P, 
		"PCAN-PC/104+ card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_PCI_104E, 
		"PCAN-PCI/104 card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_MPCIE, 
		"PCAN miniPCIe card" },
	{ PEAK_VENDORID, PEAK_DEVICEID_PCIE, 
		"PCAN-PCI Express OEM" },
	{ PEAK_VENDORID, PEAK_DEVICEID_PCIEC34, 
		"PCAN-PCI Express 34 card (one channel)" },
	{ 0 }
};

/* 
 * Hooks for the operating system.
 */
static int	peak_pci_probe(device_t dev);
static int	peak_pci_detach(device_t dev);
static int	peak_pci_attach(device_t dev);

/*
 * kobj(9) method-table
 */
static device_method_t peak_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	peak_pci_probe),
	DEVMETHOD(device_attach,	peak_pci_attach),
	DEVMETHOD(device_detach,	peak_pci_detach),
	DEVMETHOD_END
};

static driver_t peak_pci_driver = {
	"peak_pci",
	peak_pci_methods,
	sizeof(struct peak_softc)
};

static devclass_t peak_pci_devclass;

DRIVER_MODULE(peak_pci, pci, peak_pci_driver, peak_pci_devclass, 0, 0);

static int
peak_pci_probe(device_t dev)
{
	const struct peak_type	*t;
	uint16_t devid, vendor;
	int	i, error = ENXIO;
	
	vendor = pci_get_vendor(dev);
	devid = pci_get_device(dev);
	
	for (t = pk_devs, i = 0; i < nitems(pk_devs); i++, t++) {
		if (vendor == t->pk_vid && devid == t->pk_did) {
			device_set_desc(dev, t->pk_name);
			error = BUS_PROBE_DEFAULT;
			break;
		}
	}
	return (error);
}

static int
peak_pci_attach(device_t dev)
{
	struct peak_softc *sc;
	struct sja_chan *sjac;
	struct sja_data *sjad;
	uint32_t csid, cnt;
	uint16_t icr;
	int i, error = 0;
	
	sc = device_get_softc(dev);
	sc->pk_dev = dev;
	
	(void)pci_enable_busmaster(dev);
	
	/* determine cardinality of given channels */
	csid = pci_read_config(dev, PCIR_SUBDEV_0, 4);

	if (csid < PEAK_SUBDEV_DUAL_CHAN)	 /* 0x0004 */
		sc->pk_chan_cnt = PEAK_UNI_CHAN;
	else if (csid < PEAK_SUBDEV_TRIPLE_CHAN) 	/* 0x0010 */
		sc->pk_chan_cnt = PEAK_DUAL_CHAN;
	else if (csid < PEAK_SUBDEV_QUAD_CHAN) 	/* 0x0012 */
		sc->pk_chan_cnt = PEAK_TRIPLE_CHAN;
	else 
		sc->pk_chan_cnt = PEAK_QUAD_CHAN;
	
	sc->pk_chan = malloc(sizeof(struct sja_chan) * pk->pk_chan_cnt, 
		M_DEVBUF, M_WAITOK | M_ZERO);	
		
	pci_write_config(dev, PCIR_COMMAND, 4, 2);
	pci_write_config(dev, PCIR_PCCARDIF_2, 4, 0);	

	/* allocate resources for control registers and ports */
	sc->pk_res_id = PCIR_BAR(0); 
	sc->pk_res_type = SYS_RES_MEMORY;
	sc->pk_res = bus_alloc_resource_anywhere(dev, 
		sc->pk_res_type, &sc->pk_res_id, 
			PEAK_CFG_SIZE, RF_ACTIVE);
	if (sc->pk_res == NULL) {
		device_printf(dev, "couldn't map CSR\n");
		error = ENXIO;
		goto fail;
	}
	
	for (i = 0; i < sc->pk_chan_cnt; i++) { 
		sjac = &sc->pk_chan[i];
		sjad = &sjac->sjac_data;

		sjad->sjad_res_id = PCIR_BAR(1) + i * PEAK_CHAN_SIZE;
		sjad->sjad_res_type = SYS_RES_IRQ;
		sjad->sjad_res = bus_alloc_resource_anywhere(dev, 
			sjad->sjad_res_type, &sjad->sjad_res_id, 
			PEAK_CHAN_SIZE, RF_ACTIVE | RF_SHAREABLE);
		if (sjad->sjad_res == NULL) {
			device_printf(dev, "couldn't map port\n");
			error = ENXIO;
			goto fail;
		}
		
		sjad->sjad_shift = 2;
	}	
	
	/* set-up GPIO control register, if any */
	CSR_WRITE_2(sc, PEAK_GPIO_ICCR, PEAK_GPIO_ICCR_INIT);
	
	/* enable all channels, if any */
	CSR_WRITE_1(sc, PEAK_GPIO_ICR, PEAK_GPIO_ICCR_START);
	
	/* toggle reset */
	CSR_WRITE_1(sc, PEAK_MISC_CR, PEAK_MISC_CR_TOG_RST);
	DELAY(60);
	
	/* leave parport mux mode */
	CSR_WRITE_1(sc, PEAK_MISC_CR, PEAK_MISC_CR_PP_EPP);
	icr = CSR_READ_2(sc, PEAK_ICCR);

	/* attach set of SJA1000 controller as its children */		
	for (i = 0; i < sc->pk_chan_cnt; i++) { 
		sjac = &sc->pk_chan[i];
		sjad = &sjac->sjac_data;
				
		sjac->sjac_dev = device_add_child(dev, "sja", -1); 
		if (sjad->sjad_dev == NULL) {
			device_printf(dev, "couldn't map channels");
			error = ENXIO;
			goto fail;
		}
		device_set_ivars(sjac->sjac_dev, sjad);
	}
	
	/* enable interrupts */
	CSR_WRITE_2(sc, PEAK_ICCR, icr);
out:	
	return (error);
fail:
	(void)peak_pci_detach(dev);
	goto out;
}

static int
peak_pci_detach(device_t dev)
{
	struct peak_softc *sc;
	struct sja_chan *sjac;
	struct sja_data *sjad;
	int i;
 
	sc = device_get_softc(dev);
 
	/* disable interrupts */
	CSR_WRITE_2(sc, PEAK_ICCR, 0x0000);
 
	/* detach each channel, if any */
	for (i = 0; i < sc->pk_chan_cnt; i++) {
		sjac = &sc->pk_chan[i];
		
		if (sjac->sjac_dev != NULL)
			(void)device_delete_child(dev, sjac->sjac_dev);
	}
	(void)bus_generic_detach(dev);
	
	/* release bound resources */
	for (i = 0; i < sc->pk_chan_cnt; i++) {
		sjac = &sc->pk_chan[i];
		sjad = &sjac->sjac_data;
			
		if (sjad->sjad_res != NULL) {
			(void)bus_release_resource(dev, sjad->sjad_res_type, 
				sjad->sjad_res);
		}
	}
	
	if (sc->pk_res != NULL)
		(void)bus_release_resource(dev, sc->pk_res_type, sc->pk_res);
	
	free(sc->pk_chan, M_DEVBUF);
	
	return (0);
}
