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

/*
 * XXX: Well, work on progess ...
 */

#include <machine/bus.h>
#include <machine/resource.h>
#include <sys/bus.h>
#include <sys/rman.h>

#include <dev/pci/pcireg.h>
#include <dev/pci/pcivar.h>

/*
 * General constants.
 * 
 * PEAK Systems vendor ID.
 */
#define PEAK_VENDORID		0x001C	/* the PCI device and vendor IDs */

/*
 * PEAK Systems PCAN device IDs.
 */
#define PEAK_DEVICEID_PCI		0x0001
#define PEAK_DEVICEID_PCIEC		0x0002
#define PEAK_DEVICEID_PCIE		0x0003
#define PEAK_DEVICEID_CPCI		0x0004
#define PEAK_DEVICEID_MPCI		0x0005
#define PEAK_DEVICEID_PC_104P		0x0006
#define PEAK_DEVICEID_PCI_104E		0x0007
#define PEAK_DEVICEID_MPCIE		0x0008
#define PEAK_DEVICEID_PCIE		0x0009
#define PEAK_DEVICEID_PCIEC34		0x000A

#define PEAK_SUBDEVID_DUAL_CHAN		0x0004
#define PEAK_SUBDEVID_TRIPLE_CHAN		0x0010
#define PEAK_SUBDEVID_QUAD_CHAN		0x0012

#define PEAK_UNI_CHAN		1
#define PEAK_DUAL_CHAN		2
#define PEAK_TRIPLE_CHAN	3
#define PEAK_QUAD_CHAN		4

/*
 * ...
 */ 

#define PEAK_ICR		0x00		/* interrupt control register */
#define PEAK_GPIO_ICR		0x18	/* GPIO interface control register */
#define PEAK_MISC		0x1C		/* miscellaneous register */


#define PEAK_CSID		0x2e	


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
};

/*
 * Parent device(9) accessing e. g. PCI-BUS, etc.
 */
struct peak_softc {
	device_t 	pk_dev;
	device_t 	pk_sja;
	struct resource		*pk_res;
	int			pk_res_id;
	int			pk_res_type;
	uint8_t			pk_revid;	/* revision control */
/*
 * ...
 */
	struct mtx 	pk_mtx;
};

/*
 * ...
 */

MODULE_DEPEND(peak, pci, 1, 1, 1);
MODULE_DEPEND(peak, sja, 1, 1, 1); 
MODULE_DEPEND(peak, can, 1, 1, 1);

/*
 * kobj(9) metod-table
 */
static device_method_t peak_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	peak_probe),
	DEVMETHOD(device_attach,	peak_attach),
	DEVMETHOD(device_detach,	peak_detach),
	DEVMETHOD_END
};

/*
 * ...
 */

static driver_t peak_driver = {
	"peak",
	peak_methods,
	sizeof(struct peak_softc)
};

static devclass_t peak_devclass;

DRIVER_MODULE(peak, pci, peak_driver, peak_devclass, 0, 0);
DRIVER_MODULE(sja, peak, sja_driver, sja_devclass, 0, 0);

/*
 * ...
 */


static int
peak_attach(device_t dev)
{
	struct peak_softc *sc;
	uint32_t csid, chan;
	int error;
	
	sc = device_get_softc(dev);
	sc->pk_dev = dev;
		
	pci_enable_busmaster(dev);
	
	/* determine cardinality of given channels */
	csid = pci_read_config(dev, PCIR_SUBDEV_0, 4);

	if (csid < PEAK_SUBDEV_DUAL_CHAN)	 /* 0x0004 */
		chan = PEAK_UNI_CHAN;
	else if (csid < PEAK_SUBDEV_TRIPLE_CHAN) 	/* 0x0010 */
		chan = PEAK_DUAL_CHAN;
	else if (csid < PEAK_SUBDEV_QUAD_CHAN) 	/* 0x0012 */
		chan = PEAK_TRIPLE_CHAN;
	else 
		chan = PEAK_QUAD_CHAN;
		
	pci_write_config(dev, PCIR_COMMAND, 4, 2);
	pci_write_config(dev, PCIR_PCCARDIF_2, 4, 0);	

	/* map control / status registers */
	sc->pk_revid = pci_get_revid(dev); 
	device_printf(dev, "Revision: 0x%x\n", sc->pk_revid);

	sc->pk_res_id = PCIR_BAR(0);
	sc->pk_res_type = SYS_RES_IOPORT;
	sc->pk_res = bus_alloc_resource_any(dev, sc->pk_res_type,
	    &sc->pk_res_id, RF_ACTIVE);
	if (sc->pk_res == NULL) {
		device_printf(dev, "couldn't map ports\n");
		error = ENXIO;
		goto fail;
	}
/*
 * ...
 */
	mtx_init(&sc->pk_mtx, device_get_nameunit(dev), 
		MTX_NETWORK_LOCK, MTX_DEF);

out:	
	return (error);
fail:
/*
 * ...
 */ 
	goto out;
}

static int
peak_probe(device_t dev)
{
	const struct peak_type	*t;
	uint16_t devid, vendor;
	int	i, error;
	
	vendor = pci_get_vendor(dev);
	devid = pci_get_device(dev);
	
	error = ENXIO;
	
	for (t = pk_devs, i = 0; i < nitems(pk_devs); i++, t++) {
		if (vendor == t->pk_vid && devid == t->pk_did) {
			device_set_desc(dev, t->pk_name);
			error = BUS_PROBE_DEFAULT;
			break;
		}
	}
	return (error);
}

/*
 * ...
 */

