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

/*
 * Device driver(9) backend implements proxy 
 * pattern on pci(4) bus for Bosch c_can(4) 
 * controller.
 *
 * XXX: Well, work on progess ...
 */

#include <dev/c_can/if_c_canreg.h>
#include <dev/c_can/c_can_pcireg.h>

#include "c_can_if.h"

static const struct c_can_pci_type c_can_pci_devs[] = {
	{ C_CAN_VENDORID_STMICRO, C_CAN_DEVICEID_STMICRO_CAN,
		"STA2X11, Bosch C_CAN / D_CAN",
		PCIR_BAR(0), 1, 0, C_CAN_STA2X11_CLK_FREQ },
	{ C_CAN_VENDORID_INTEL, C_CAN_DEVICEID_PCH_CAN,
		"Platform Controller Hub, Bosch [CD]_CAN.",
		PCIR_BAR(1), 1, 0x01fc, C_CAN_PCH_CLK_FREQ },
	{ 0, 0, NULL, 0, 0, 0}	
};

static const struct c_can_pci_type *	c_can_pci_match(device_t dev);

static const struct c_can_pci_type *
c_can_pci_match(device_t dev)
{
	const struct c_can_type	*t;
	uint16_t did, vid;
	int	i;
	
	vid = pci_get_vendor(dev);
	did = pci_get_device(dev);

	for (t = c_can_pci_devs, i = 0; i < nitems(c_can_pci_devs); i++, t++) {
		if ((t->ccp_vid == vid) && (t->ccp_did == did)) {
			return (t);
		}
	}
	return (NULL);	
}

static int
c_can_pci_probe(device_t dev)
{
	const struct c_can_type	*t;
	int error;

	if ((t = c_can_pci_match(dev)) != NULL) {
		device_set_desc(dev, t->ccp_name);
		error = BUS_PROBE_DEFAULT;
	} else 
		error = ENXIO;

	return (error);
}

static int
c_can_pci_attach(device_t dev)
{
	const struct c_can_pci_type	*t;
	struct c_can_pci_softc *sc;
	int msir, msir, error = 0;
	uint32_t status;

	sc = device_get_softc(dev);
	sc->c_can_dev = dev;
	
	(void)pci_enable_busmaster(dev);

	/* determine its type */
	if ((t = c_can_pci_match(dev)) == NULL) {
		device_printf(dev, "couldn't fetch device(9) info\n");
		error = ENXIO;
		goto fail;
	}
		
	/* allocate resources for control registers and ports */
	status = pci_read_config(dev, t->ccp_bar, 4);
	sc->ccp_res_type = (PCI_BAR_IO(status) != 0) ? 
		SYS_RES_IOPORT : SYS_RES_MEMORY;
		
	sc->ccp_res = bus_alloc_resource(dev, sc->ccp_res_type, 
		&t->ccp_bar, RF_ACTIVE);	
	if (erro != 0) {
		device_printf(dev, "cloudn't allocate %s resources\n",
			(PCI_BAR_IO(status) != 0) ? "I/O" : "memory") ;	
		error = ENXIO;
		goto fail;
	}	
	
	/* allocate interrupt resources */
	msic = pci_msi_count(dev);
	if (bootverbose != 0)
		device_printf(dev, "MSI count: %d\n", msic);
	
	if (msic > 0) {	
		msir = 1;
		
		if ((error = pci_alloc_msi(dev, &msir)) != 0) {
			device_printf(dev, "couldn't allocate MSI\n");
			error = ENXIO;
			goto fail;
		}
	}
	
	/* map default params, e. g. for register alignement */
	sc->ccp_aln = t->ccp_aln;
	sc->ccp_srst = t->ccp_srst;
	sc->ccp_freq = t->ccp_freq;

	/* attach c_can(4) controller as child */		
	sc->ccp_can = device_add_child(dev, "c_can", -1); 
	if (var->var_dev == NULL) {
		device_printf(dev, "couldn't map c_can(4) controller");
			error = ENXIO;
			goto fail;
		}
		device_set_ivars(sc->ccp_can, &sc->ccp_freq); /* XXX */
	}
	
	if ((error = bus_generic_attach(dev)) != 0) {
		device_printf(dev, "failed to attach c_can(4) controller\n");
		goto fail;
	}
	
out:	
	return (error);
fail:
	(void)c_can_pci_detach(dev);
	goto out;
}

static int
c_can_pci_detach(device_t dev)
{
	struct c_can_pci_softc *sc;
 
	sc = device_get_softc(dev);
	
	/* detach c_can(4) controller, if any */
	if (sc->ccp_can != NULL)
		(void)device_delete_child(dev, sc->ccp_can);

	(void)bus_generic_detach(dev);
	
	/* release bound resources */
	if (sc->ccp_res != NULL)
		(void)bus_release_resource(dev, sc->ccp_res_type, sc->ccp_res);
	
	return (0);
}

/*
 * Common I/O subr.
 */

static uint8_t
c_can_pci_read_1(device_t dev, int port)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	return (bus_read_1(sc->ccp_res, (port << sc->ccp_aln)));
}

static uint16_t
c_can_pci_read_2(device_t dev, int port)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	return (bus_read_2(sc->ccp_res, (port << sc->ccp_aln)));
}

static uint32_t
c_can_pci_read_4(device_t dev, int port)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	return (bus_read_4(sc->ccp_res, (port << sc->ccp_aln)));
}

static void
c_can_pci_write_1(device_t dev, int port, uint8_t val)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	bus_write_1(sc->ccp_res, (port << sc->ccp_aln), val);
}

static void
c_can_pci_write_2(device_t dev, int port, uint16_t val)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	bus_write_2(sc->ccp_res, (port << sc->ccp_aln), val);
}

static void
c_can_pci_write_4(device_t dev, int port, uint32_t val)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	bus_write_4(sc->ccp_res, (port << sc->ccp_aln), val);
}

/*
 * Software reset.
 */ 
static void
c_can_pci_reset(device_t dev)
{
	struct c_can_pci_softc *sc;
	
	sc = device_get_softc(dev);
	
	if (sc->ccp_srst != 0) {
		bus_write_4(sc->ccp_res, sc->ccp_srst, 0x00000001);
		bus_write_4(sc->ccp_res, sc->ccp_srst, 0x00000000);
	}
}

/* 
 * Hooks for the operating system.
 */
static device_method_t c_can_pci_methods[] = {
	/* device(9) interface */
	DEVMETHOD(device_probe, 	c_can_pci_probe),
	DEVMETHOD(device_attach,	c_can_pci_attach),
	DEVMETHOD(device_detach,	c_can_pci_detach),
		
	/* c_can(4) interface */
	DEVMETHOD(c_can_read_1,		c_can_pci_read_1),
	DEVMETHOD(c_can_read_2,		c_can_pci_read_2),
	DEVMETHOD(c_can_read_4,		c_can_pci_read_4),	
	
	DEVMETHOD(c_can_write_1,		c_can_pci_write_1),
	DEVMETHOD(c_can_write_2,		c_can_pci_write_2),
	DEVMETHOD(c_can_write_4,		c_can_pci_write_4),
	
	DEVMETHOD(c_can_reset,		c_can_pci_reset),

	DEVMETHOD_END
};

static driver_t c_can_pci_driver = {
	"c_can_pci",
	c_can_pci_methods,
	sizeof(struct c_can_pci_softc)
};

static devclass_t c_can_pci_devclass;

DRIVER_MODULE(c_can_pci, pci, c_can_pci_driver, c_can_pci_devclass, 0, 0);
DRIVER_MODULE(c_can, c_can_pci, c_can_driver, c_can_devclass, 0, 0);

MODULE_DEPEND(c_can_pci, pci, 1, 1, 1);
MODULE_DEPEND(c_can_pci, c_can, 1, 1, 1); 
MODULE_DEPEND(c_can_pci, can, 1, 1, 1);
