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
 * ...
 */

MODULE_DEPEND(peak, pci, 1, 1, 1);

/* 
 * XXX: Well, I'm not really sure if sja(4) 
 * XXX: should implement an so called proxy-/ 
 * XXX: adapter-pattern??? 
 */
MODULE_DEPEND(peak, sja, 1, 1, 1); 
MODULE_DEPEND(peak, can, 1, 1, 1);

/*
 * Device-table.
 */
static const struct peak_type = {
	{ /* ... */, 	/* ... */ 	},
	{ /* ... */, 	/* ... */ 	},	
 /*
  * ...
  */
};

/*
 * kobj(9) metod-table
 */
static device_method_t peak_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe, 	peak_probe),
	DEVMETHOD(device_attach,	peak_attach),
	DEVMETHOD(device_detach,	peak_detach),
	DEVMETHOD(device_suspend,	peak_suspend),
	DEVMETHOD(device_resume,	peak_resume),
	DEVMETHOD(device_shutdown,	peak_shutdown),

	/* sja1000(4) interface */
	DEVMETHOD(sja_readreg,	peak_sja_readreg),
	DEVMETHOD(sja_writereg,	peak_sja_writereg),
	DEVMETHOD(sja_statchg,	peak_sja_statchg),

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

