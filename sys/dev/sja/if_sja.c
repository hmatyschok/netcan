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

#include <dev/sja/sja.h>

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
	
	if ((status & (SJA_IR_OFF|SJA_IR_ALL)) != 0)  
		error = FILTER_STRAY;
	else {	
		CSR_WRITE_1(sja, SJA_IR, SJA_IR_OFF);
		taskqueue_enqueue(taskqueue_fast, &sja->sja_intr_task);
		error = FILTER_HANDLED;
	}
	return (error);
}

static void
sja_intr_task(void *arg)
{
	struct sja_softc *sja;
	struct ifnet *ifp = sja->sja_ifp;
	uint8_t status;
	int	count;

	sja = arg;
	ifp = sja->sja_ifp;

	SJA_LOCK(sja);

	if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
		goto done_locked;

/*
 * ...
 */
		
done_locked:
	SJA_UNLOCK(sja);
}
