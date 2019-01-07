/*
 * Copyright (c) 2019 Henning Matyschok
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
#include <dev/sja/plx_pcireg.h>

/*
 * Device driver(9) for PLX90xx PCI-bridge 
 * cards implements proxy pattern on pci(4) 
 * bus for instances of the sja(4) contoller.
 *
 * XXX: Well, work on progess ...
 */

#define CSR_WRITE_1(sc, reg, val) \
	bus_write_1((sc)->plx_res, reg, val)
#define CSR_READ_1(sc, reg) \
	bus_read_1((sc)->plx_res, reg)

#define CSR_WRITE_2(sc, reg, val) \
	bus_write_2((sc)->plx_res, reg, val)
#define CSR_READ_2(sc, reg) \
	bus_read_2((sc)->plx_res, reg)

#define CSR_WRITE_4(sc, reg, val) \
	bus_write_4((sc)->plx_res, reg, val)
#define CSR_READ_4(sc, reg) \
	bus_read_4((sc)->plx_res, reg)

static void		plx_pci_reset(struct plx_softc *sc);
static void 	plx_pci_9056_reset(struct plx_softc *sc);

/* 
 * Adlink PCI-7841/cPCI-7841 [SE] cards. 
 */ 
static struct plx_data plx_adlink = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x80,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
	
	.plx_status_addr = PLX_TCR;
	.plx_status = 
};

/*
 * ESD Electronics CAN-PCI/CPCI/PCI104/200 cards.
 */
static struct plx_data plx_esd_200 = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x100,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLC_ICR_PINT_ENB),
};

/*
 * ESD Electronics CAN-PCI/PMC/266 cards.
 */
static struct plx_data plx_esd_266 = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x100,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 0,
	.plx_icr_addr = PLX_9056_ICR,
	.plx_icr = (PLX_9056_ICR_INT0_ENB | PLX_9056_ICR_PINT_ENB),
};

/*
 * ESD Electronics CAN-PCIe/2000 cards.
 */
static struct plx_data plx_esd_2000 = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x100,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 0,
	.plx_icr_addr = PLX_9056_ICR,
	.plx_icr = (PLX_9056_ICR_INT0_ENB | PLX_9056_ICR_PINT_ENB),
};

/*
 * IXXAT PC-I 04/PCI cards.
 */
static struct plx_data plx_ixxat = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x200,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
};

/*
 * Marathon CAN-bus-PCI cards.
 */
static struct plx_data plx_marathon_pci = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(4),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
};

/*
 * Marathon CAN-bus-PCIe cards.
 */
static struct plx_data plx_marathon_pcie = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(3),
		.plx_off =	0x80,
		.plx_cnt = 	0,
	},
	
	.plx_icr_read = 0,
	.plx_icr_addr = PLX_9056_ICR,
	.plx_icr = (PLX_9056_ICR_INT0_ENB | PLX_9056_ICR_PINT_ENB),
};

/*
 * TEWS Technologies TPMC810 cards.
 */
static struct plx_data plx_tews = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x100,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
};

/*
 * Connect Tech Inc. CANpro/104-Plus Opto (CRG001) cards.
 */
static struct plx_data plx_cti = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0x100,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
};

/*
 * Elcus CAN-200-PCI cards.
 */
static struct plx_data plx_elcus = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(1),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(2),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(3),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
};

/*
 * Moxa cards.
 */
static struct plx_data plx_moxa = {
	.plx_res = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0,
	},
	
	.plx_chan[0] = {
		.plx_bar =	PCIR_BAR(0),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_chan[1] = {
		.plx_bar =	PCIR_BAR(1),
		.plx_off =	0,
		.plx_cnt = 	0x80,
	},
	
	.plx_icr_read = 1,
	.plx_icr_addr = PLX_ICR,
	.plx_icr = (PLX_ICR_INT0_ENB | PLX_ICR_INT1_ENB | PLX_ICR_PINT_ENB),
};

/*
 * ...
 */

static const struct plx_type plx_devs[] = {
	{ PLX_VENDORID_ADLINK, PLX_DEVICEID_ADLINK,
		PLX_SUBVENDID_ANY, PLX_SUBDEVID_ANY,
		&plx_adlink, "Adlink PCI-7841" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9050,
		PLX_SUBVENDID_ESD, PLX_SUBDEVID_ESD_PCI200,
		&plx_esd_200, "ESD CAN-PCI/200" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9030,
		PLX_SUBVENDID_ESD, PLX_SUBDEVID_ESD_CPCI200,
		&plx_esd_200, "ESD CAN-CPCI/200" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9030,
		PLX_SUBVENDID_ESD, PLX_SUBDEVID_ESD_PCI104200,
		&plx_esd_200, "ESD CAN-PCI104/200" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9056,
		PLX_SUBVENDID_ESD, PLX_SUBDEVID_ESD_PCI266,
		&plx_esd_266, "ESD CAN-PCI/266" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9056,
		PLX_SUBVENDID_ESD, PLX_SUBDEVID_ESD_PMC266,
		&plx_esd_266, "ESD CAN-PMC/266" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9056,
		PLX_SUBVENDID_ESD, PLX_SUBDEVID_ESD_PCIE2000,
		&plx_esd_2000, "ESD CAN-PCIE/2000" },
	{ PLX_VENDORID_IXXAT, PLX_DEVICEID_IXXAT,
		PLX_SUBVENDID_ANY, PLX_SUBDEVID_IXXAT,
		&plx_ixxat, "IXXAT PC-I 04/PCI"},
	{ PLX_VENDORID_PLX, PLX_DEVICEID_MARATHON_PCI,
		PLX_SUBVENDID_ANY, PLX_SUBDEVID_ANY,
		&plx_marathon_pci, "Marathon CAN-bus-PCI" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_MARATHON_PCIE,
		PLX_SUBVENDID_ANY, PLX_SUBDEVID_ANY,
		&plx_marathon_pcie, "Marathon CAN-bus-PCIe" },
	{ PLX_VENDORID_TEWS, PLX_DEVICEID_TEWS_TMPC810,
		PLX_SUBVENDID_ANY, PLX_SUBDEVID_ANY,
		&plx_tews, "TEWS Technologies TPMC810" },
	{ PLX_VENDORID_PLX, PLX_DEVICEID_PLX_9030,
		PLX_VENDORID_CTI, PLX_DEVICEID_CTI_CRG001,
		&plx_cti, "Connect Tech Inc. CANpro/104-Plus Opto (CRG001)" },
	{ PLX_VENDORID_CAN200PCI, PLX_DEVICEID_CAN200PCI,
		PLX_SUBVENDID_CAN200PCI, PLX_SUBDEVID_CAN200PCI,
		&plx_elcus, "Elcus CAN-200-PCI" },
	{ PLX_VENDORID_MOXA, PLX_DEVICEID_MOXA,
		PLX_SUBVENDID_ANY, PLX_SUBDEVID_ANY,
		&plx_moxa, "Moxa" },
	{ 0, 0, 0, 0, NULL, NULL}	
};

static const struct plx_type *	plx_pci_match(device_ dev);

/* 
 * Hooks for the operating system.
 */
static int	plx_pci_probe(device_t dev);
static int	plx_pci_detach(device_t dev);
static int	plx_pci_attach(device_t dev);

/*
 * kobj(9) method-table
 */
static device_method_t plx_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	plx_pci_probe),
	DEVMETHOD(device_attach,	plx_pci_attach),
	DEVMETHOD(device_detach,	plx_pci_detach),
	DEVMETHOD_END
};

static driver_t plx_pci_driver = {
	"plx_pci",
	plx_pci_methods,
	sizeof(struct plx_softc)
};

static devclass_t plx_pci_devclass;

DRIVER_MODULE(plx_pci, pci, plx_pci_driver, plx_pci_devclass, 0, 0);

static int
plx_pci_probe(device_t dev)
{
	const struct plx_type	*t;
	int error;

	if ((t = plx_pci_match(dev)) != NULL) {
		device_set_desc(dev, t->plx_name);
		error = BUS_PROBE_DEFAULT;
	} else 
		error = ENXIO;

	return (error);
}

static int
plx_pci_attach(device_t dev)
{
	const struct plx_type	*t;
	struct plx_softc *sc;
	struct plx_desc *res;	
	struct sja_chan *sjac;
	struct sja_data *sjad;
	int i, error = 0;
	uint32_t status;

	sc = device_get_softc(dev);
	sc->plx_dev = dev;
	
	(void)pci_enable_busmaster(dev);

	/* determine its type */
	if ((t = plx_pci_match(dev)) == NULL) {
		device_printf(dev, "couldn't fetch device(9) info\n");
		error = ENXIO;
		goto fail;
	}

	sc->plx_id = t->plx_id;
	
	/* allocate resources for control registers and ports */
	res = &sc->plx_id->plx_res;
	
	sc->plx_res_id = res->plx_bar + res->plx_off; 
	sc->plx_res_type = SYS_RES_MEMORY;
	
	if (res->plx_cnt == 0) {
		sc->plx_res = bus_alloc_resource_any(dev, sc->plx_res_type, 
			&sc->plx_res_id, RF_ACTIVE);
	} else {
		sc->plx_res = bus_alloc_resource_anywhere(dev, 
			sc->plx_res_type, &sc->plx_res_id, 
				res->plx_cnt, RF_ACTIVE);
	}
	
	if (sc->plx_res == NULL) {
		device_printf(dev, "couldn't map ICR / TCR\n");
		error = ENXIO;
		goto fail;
	}
	
	for (i = 0; i < PLX_CHAN_MAX; i++) { 
		res = &sc->plx_id->plx_chan[i]; 
		sjac = &sc->plx_chan[i];
		sjad = &sjac->sjac_data;

		sjad->sjad_res_id = res->plx_bar + res->plx_off;
		sjad->sjad_res_type = SYS_RES_IRQ;
		
		if (res->plx_cnt == 0) {
			sjad->sjad_res = bus_alloc_resource_any(dev, 
				sjad->sjad_res_type, &sjad->sjad_res_id, 
					RF_ACTIVE | RF_SHAREABLE);
		} else {
			sjad->sjad_res = bus_alloc_resource_anywhere(dev, 
				sjad->sjad_res_type, &sjad->sjad_res_id, 
					res->plx_cnt, RF_ACTIVE | RF_SHAREABLE);
		}
		
		if (sjad->sjad_res == NULL) {
			device_printf(dev, "couldn't map port %i\n", i);
			error = ENXIO;
			goto fail;
		}
		
		sjad->sjad_cdr = PLX_CDR_DFLT;
		sjad->sjad_ocr = PLX_OCR_DFLT;
		sjad->sjad_freq = PLX_CLK_FREQ;
	}	
	
	/* attach set of SJA1000 controller as its children */		
	for (i = 0; i < PLX_MAX_CHAN; i++) { 
		sjac = &sc->plx_chan[i];
		sjad = &sjac->sjac_data;
				
		sjac->sjac_dev = device_add_child(dev, "sja", -1); 
		if (sjad->sjad_dev == NULL) {
			device_printf(dev, "couldn't map channels");
			error = ENXIO;
			goto fail;
		}
		device_set_ivars(sjac->sjac_dev, sjad);
	}
	
	if ((error = bus_generic_attach(dev)) != 0) {
		device_printf(dev, "failed to attach ports\n");
		goto fail;
	}
	
	/* enable interrupts */
	if ((status = sc->plx_id->plx_icr_read) != 0)
		status = CSR_READ_4(sc, sc->plx_id->plx_icr_addr);
		
	status |= sc->plx_id->plx_icr;
	
	CSR_WRITE_4(sc, sc->plx_id->plx_icr_addr, status);
out:	
	return (error);
fail:
	(void)plx_pci_detach(dev);
	goto out;
}

static int
plx_pci_detach(device_t dev)
{
	struct plx_softc *sc;
	struct sja_chan *sjac;
	struct sja_data *sjad;
	uint32_t status;
	int i;
 
	sc = device_get_softc(dev);

	/* local bus reset for PLX9056 and PLX9030/50/52 */
	status = CSR_READ_4(sc, sc->plx_id->plx_tcr_addr);
	status |= sc->plx_id->plx_tcr_rst;
	
	CSR_WRITE_4(sc, sc->plx_id->plx_tcr_addr, status);
	DELAY(100);
	
	status &= ~sc->plx_id->plx_tcr_rst;
	
	CSR_WRITE_4(sc, sc->plx_id->plx_tcr_addr, status);
	
	/* reload data from EEPROM on PLX9056, if any */
	if (sc->plx_id->plx_tcr_rcr != 0) {
		status |= sc->plx_id->plx_tcr_rcr;
		CSR_WRITE_4(sc, sc->plx_id->plx_tcr_addr, status);
	
		DELAY(10);	/* XXX: ... */

		status &= ~sc->plx_id->plx_tcr_rcr;
		CSR_WRITE_4(sc, sc->plx_id->plx_tcr_addr, status);	
	}
 
	/* disable interrupts */
	CSR_WRITE_4(sc, sc->plx_id->plx_icr_addr, 0x00000000);
 
	/* detach each channel, if any */
	for (i = 0; i < PLX_CHAN_MAX; i++) {
		sjac = &sc->plx_chan[i];
		
		if (sjac->sjac_dev != NULL)
			(void)device_delete_child(dev, sjac->sjac_dev);
	}
	(void)bus_generic_detach(dev);
 
	/* release bound resources */
	for (i = 0; i < PLX_CHAN_MAX; i++) {
		sjac = &sc->plx_chan[i];
		sjad = &sjac->sjac_data;
			
		if (sjad->sjad_res != NULL) {
			(void)bus_release_resource(dev, sjad->sjad_res_type, 
				sjad->sjad_res);
		}
	}
	
	if (sc->pk_res != NULL)
		(void)bus_release_resource(dev, sc->plx_res_type, sc->plx_res);

	return (0);
}

static const struct plx_type *
plx_pci_match(device_ dev)
{
	const struct plx_type	*t;
	uint16_t did, vid;
	uint16_t sub_did, sub_vid;
	int	i;
	
	vid = pci_get_vendor(dev);
	did = pci_get_device(dev);
	sub_vid = pci_get_subvendor(dev);
	sub_did = pci_get_subdevice(dev);
	
	for (t = plx_devs, i = 0; i < nitems(plx_devs); i++, t++) {
		if ((t->plx_vid == vid) && (t->plx_did == did) 
			&& ((t->plx_sub_vid == sub_vid) || 
				(t->plx_sub_vid == PLX_SUBVENDID_ANY)) 
			&& ((t->plx_sub_did == sub_did) || 
				(t->plx_sub_vid == PLX_SUBVENDID_ANY))) {
			return (t);
		}
	}
	return (NULL);	
}

/*
 * 
 */
static void 
plx_pci_reset(struct plx_softc *sc)
{
	
};

/*
 * PLX9056 software reset.
 */
static void 
plx_pci_9056_reset(struct plx_softc *sc)
{
	uint32_t status;

	status = CSR_READ_4(sc, PLX_9056_TCR);
	status |= PLX_TCR_RST;
	
	CSR_WRITE_4(sc, PLX_9056_TCR, status);
	DELAY(100);
	
	status &= ~PLX_TCR_RST;
	CSR_WRITE_4(sc, PLX_9056_TCR, status);

	
};

MODULE_DEPEND(plx_pci, pci, 1, 1, 1);
MODULE_DEPEND(plx_pci, sja, 1, 1, 1); 
MODULE_DEPEND(plx_pci, can, 1, 1, 1);
