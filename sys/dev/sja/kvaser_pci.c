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
 * Device driver(9) for KVASER PCAN PCI cards 
 * implements proxy pattern on pci(4) bus for 
 * instances of the sja(4) contoller.
 *
 * XXX: Well, work on progess ...
 */

#include <dev/sja/if_sjareg.h>
#include <dev/sja/kvaser_pcireg.h>

static const struct kvaser_type  kv_devs[] = {
	{ KVASER_VENDORID0, PEAK_DEVICEID_PCI0, 
		"KVASER PCAN PCI card 0" },
	{ KVASER_VENDORID1, KVASER_DEVICEID_PCI1, 
		"KVASER PCAN PCI card 1" },
	{ 0, 0, NULL }	
};

static int
kvaser_pci_probe(device_t dev)
{
	const struct kvaser_type	*t;
	uint16_t did, vid;
	int	i, error = ENXIO;
	
	vendor = pci_get_vendor(dev);
	devid = pci_get_device(dev);
	
	for (t = kv_devs, i = 0; i < nitems(kv_devs); i++, t++) {
		if (t->kv_vid == vid && t->kv_did == did) {
			device_set_desc(dev, t->kv_name);
			error = BUS_PROBE_DEFAULT;
			break;
		}
	}
	return (error);
}

static int
kvaser_pci_attach(device_t dev)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	struct sja_data *var;
	int i, error = 0;
	uint32_t status;
	
	sc = device_get_softc(dev);
	sc->kv_dev = dev;
	
	(void)pci_enable_busmaster(dev);
	
	/* allocate resources for control registers and ports */
	sc->kv_cfg_id = PCIR_BAR(0); 
	sc->kv_cfg_type = SYS_RES_MEMORY;
	sc->kv_cfg = bus_alloc_resource_anywhere(dev, 
		sc->kv_cfg_type, &sc->kv_cfg_id, 
			KVASER_PCI_CFG_SIZE, RF_ACTIVE);
	if (sc->kv_cfg == NULL) {
		device_printf(dev, "couldn't map CMR\n");
		error = ENXIO;
		goto fail;
	}
	
	sc->pk_res_id = PCIR_BAR(2); 
	
	status = pci_read_config(dev, sc->pk_res_id, 4);
	sc->pk_res_type = (PCI_BAR_IO(status) != 0) ? 
		SYS_RES_IOPORT : SYS_RES_MEMORY;
	
	sc->pk_res = bus_alloc_resource_anywhere(dev, 
		sc->pk_res_type, &sc->pk_res_id, 
			KVASER_PCI_RES_SIZE, RF_ACTIVE);
	if (sc->pk_res == NULL) {
		device_printf(dev, "couldn't map CMR\n");
		error = ENXIO;
		goto fail;
	}
	
	sc->kv_vers_id = bus_read_1(sc->kv_res, KVASER_VERS_ID);
	sc->kv_vers_id >>= 4;
	
	status = pci_read_config(dev, PCIR_BAR(1), 4);
	
	for (i = 0; i < KVASER_CHAN_MAX; i++) { 
		chan = &sc->kv_chan[i];
		var = &chan->sja_var;

		chan->sja_res_id = PCIR_BAR(1) + i * KVASER_CHAN_SIZE;
		chan->sja_res_type = (PCI_BAR_IO(status) != 0) ? 
			SYS_RES_IOPORT : SYS_RES_MEMORY;
		
		chan->sja_res = bus_alloc_resource_anywhere(dev, 
			chan->sja_res_type, &chan->sja_res_id, 
				KVASER_CHAN_SIZE, RF_ACTIVE);
	
		if (chan->sja_res == NULL) {
			device_printf(dev, "couldn't map port %i\n", i);
			break;
		}
		
		var->sja_port = i;
		var->sja_cdr = KVASER_CDR_DFLT;
		var->sja_ocr = KVASER_OCR_DFLT;
		var->sja_freq = KVASER_CLK_FREQ;
	
		sc->kv_chan_cnt = i;
	}	
	
	if (sc->kv_chan_cnt == 0) {
		device_printf(dev, "couldn't map ports\n");
		error = ENXIO;
		goto fail;
	}

	/* attach set of SJA1000 controller as its children */		
	for (i = 0; i < sc->kv_chan_cnt; i++) { 
		chan = &sc->kv_chan[i];
		var = &chan->sja_var;
				
		chan->sja_dev = device_add_child(dev, "sja", -1); 
		if (chan->sja_dev == NULL) {
			device_printf(dev, "couldn't map channels");
			error = ENXIO;
			goto fail;
		}
		device_set_ivars(chan->sja_dev, var);
	}
	
	if ((error = bus_generic_attach(dev)) != 0) {
		device_printf(dev, "failed to attach ports\n");
		goto fail;
	}	
	
	/* assert passive mode */
	bus_write_4(sc->kv_cfg, KVASER_TCR, KVASER_TCR_PSV);
	
	/* enable interrupts */
	status = bus_read_4(sc->kv_cfg, KVASER_ICR);
	status |= KVASER_ICR_INIT;
	
	bus_write_4(sc->kv_cfg, KVASER_ICR, status);
out:	
	return (error);
fail:
	(void)kvaser_pci_detach(dev);
	goto out;
}

static int
kvaser_pci_detach(device_t dev)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	uint32_t status;
	int i;
 
	sc = device_get_softc(dev);

	/* disable interrupts */
	status = bus_read_4(sc->kv_cfg, KVASER_ICR);
	status &= ~KVASER_ICR_INIT;
	
	bus_write_4(sc->kv_cfg, KVASER_ICR, status);
	
	/* detach each channel, if any */
	for (i = 0; i < sc->kv_chan_cnt; i++) {
		chan = &sc->kv_chan[i];
		
		if (chan->sja_dev != NULL)
			(void)device_delete_child(dev, chan->sja_dev);
	}
	(void)bus_generic_detach(dev);
	
	/* release bound resources */
	for (i = 0; i < sc->pkv_chan_cnt; i++) {
		chan = &sc->kv_chan[i];
			
		if (var->sja_res != NULL) {
			(void)bus_release_resource(dev, chan->sja_res_type, 
				chan->sja_res);
		}
	}
	
	if (sc->kv_res != NULL)
		(void)bus_release_resource(dev, sc->kv_res_type, sc->kv_res);
	
	if (sc->kv_cfg != NULL)
		(void)bus_release_resource(dev, sc->kv_cfg_type, sc->kv_cfg);

	return (0);
}

/*
 * Common I/O subr.
 */

static uint8_t
kvaser_pci_read_1(device_t dev, sja_data_t sjad, int port)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	
	sc = device_get_softc(dev);
	chan = &sc->kv_chan[var->sja_port];
	
	return (bus_read_1(chan->sja_res, port));
}

static uint16_t
kvaser_pci_read_2(device_t dev, sja_data_t var, int port)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	
	sc = device_get_softc(dev);
	chan = &sc->kv_chan[var->sja_port];
	
	return (bus_read_2(chan->sja_res, port));
}

static uint32_t
kvaser_pci_read_4(device_t dev, sja_data_t var, int port)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	
	sc = device_get_softc(dev);
	chan = &sc->kv_chan[var->sja_port];
	
	return (bus_read_4(chan->sja_res, port));
}

static void
kvaser_pci_write_1(device_t dev, sja_data_t var, int port, uint8_t val)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	
	sc = device_get_softc(dev);
	chan = &sc->kv_chan[var->sja_port];
	
	bus_write_1(chan->sja_res, port, val);
}

static void
kvaser_pci_write_2(device_t dev, sja_data_t var, int port, uint16_t val)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	
	sc = device_get_softc(dev);
	chan = &sc->kv_chan[var->sja_port];
	
	bus_write_2(chan->sja_res, port, val);
}

static void
kvaser_pci_write_4(device_t dev, sja_data_t var, int port, uint32_t val)
{
	struct kvaser_softc *sc;
	struct sja_chan *chan;
	
	sc = device_get_softc(dev);
	chan = &sc->kv_chan[var->sja_port];
	
	bus_write_4(chan->sja_res, port, val);	
}

/* 
 * Hooks for the operating system.
 */
static device_method_t kvaser_pci_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	kvaser_pci_probe),
	DEVMETHOD(device_attach,	kvaser_pci_attach),
	DEVMETHOD(device_detach,	kvaser_pci_detach),
	
	/* sja(4) interface */
	DEVMETHOD(sja_read_1,	kvaser_pci_read_1),
	DEVMETHOD(sja_read_2,	kvaser_pci_read_2),
	DEVMETHOD(sja_read_4,	kvaser_pci_read_4),
	
	DEVMETHOD(sja_write_1,	kvaser_pci_write_1),
	DEVMETHOD(sja_write_2,	kvaser_pci_write_2),
	DEVMETHOD(sja_write_4,	kvaser_pci_write_4),
		
	DEVMETHOD_END
};

static driver_t kvaser_pci_driver = {
	"kvaser_pci",
	kvaser_pci_methods,
	sizeof(struct kvaser_softc)
};

static devclass_t kvaser_pci_devclass;

DRIVER_MODULE(kvaser_pci, pci, kvaser_pci_driver, kvaser_pci_devclass, 0, 0);
DRIVER_MODULE(sja, kvaser_pci, sja_driver, sja_devclass, 0, 0);

MODULE_DEPEND(kvaser_pci, pci, 1, 1, 1);
MODULE_DEPEND(kvaser_pci, sja, 1, 1, 1); 
MODULE_DEPEND(kvaser_pci, can, 1, 1, 1);
