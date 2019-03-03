/*-
 * Copyright (c) 1998, 2001 Nicolas Souchu
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
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */
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
 
#include <sys/cdefs.h>
__FBSDID("$FreeBSD: releng/11.1/sys/dev/iicbus/if_ic.c 315221 2017-03-14 02:06:03Z pfg $");

/*
 * I2C bus can(4) driver.
 */

#include <sys/param.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/filio.h>
#include <sys/sockio.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/bus.h>
#include <sys/time.h>
#include <sys/malloc.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>

#include <net/if_can.h>

#include <net/bpf.h>

#include <dev/iicbus/iiconf.h>
#include <dev/iicbus/iicbus.h>

#include "iicbus_if.h"

#define PCF_MASTER_ADDRESS	0xaa

struct icc_softc {
	struct ifnet	*icc_ifp;
	device_t	icc_dev;

	uint8_t	icc_addr;			/* peer I2C address */
	
	int	icc_err_cnt;
	
	struct mbuf	*icc_inb;
	char	*icc_outb;
	struct mtx	icc_lock;
};

static int	icc_encap(struct icc_softc *, struct mbuf **);
static void	icc_init(void *);
static void	icc_init_locked(struct iic_softc *);
static int	icc_ioctl(struct ifnet *, u_long, caddr_t);
static void	icc_start(struct ifnet *);

static int
icc_probe(device_t dev)
{
	
	return (BUS_PROBE_NOWILDCARD);
}

static int
icc_attach(device_t dev)
{
	struct icc_softc *icc;
	struct ifnet *ifp;

	icc = (struct icc_softc *)device_get_softc(dev);
	
	ifp = icc->icc_ifp = if_alloc(IFT_CAN);
	if (ifp == NULL)
		return (ENOSPC);

	mtx_init(&icc->icc_lock, device_get_nameunit(dev), 
		MTX_NETWORK_LOCK, MTX_DEF);
		
	icc->icc_outb = malloc(MHLEN, M_DEVBUF, M_WAITOK);	
		
	icc->icc_addr = PCF_MASTER_ADDRESS;	/* XXX: only PCF masters */
	icc->icc_dev = dev;

	ifp->if_softc = icc;
	
	if_initname(ifp, device_get_name(dev), device_get_unit(dev));
	
	ifp->if_flags = IFF_SIMPLEX | IFF_POINTOPOINT | IFF_MULTICAST;
	ifp->if_init = icc_init;
	ifp->if_start = icc_start;
	ifp->if_ioctl = icc_ioctl;

	can_ifattach(ifp, 0);
	
	ifp->if_drv_flags |= IFF_DRV_RUNNING;
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
	
	return (0);
}

/*
 * icc_intr()
 */
static int
icc_intr(device_t dev, int status, char *c)
{
	struct icc_softc *icc;
	struct ifnet *ifp;
	struct mbuf *m;

	icc = (struct icc_softc *)device_get_softc(dev);
	ifp = icc->icc_ifp;

	mtx_lock(&icc->icc_lock);

	switch (status) {
	case INTR_GENERAL:
	case INTR_START:
		/* allocate mbuf(9), if any and initialize */
		if ((m = icc->icc_inb) == NULL) {
			if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL) {
				icc->icc_err_cnt++;
				icc->icc_inb = NULL;
				break;
			}
		} else
			m->m_data = m->m_pktdat;
			
		m->m_len = m->m_pkthdr.len = 0;
		m->m_pkthdr.rcvif = ifp;
		
		icc->icc_inb = m;
		break;
	case INTR_STOP:

		/* if any error occurred during transfert,
		 * drop the packet */
		if ((m = icc->icc_inb) == NULL)
			icc->icc_err_cnt++;
		
		if (icc->icc_err_cnt != 0) {
			if_printf(ifp, "errors (%d)!\n", icc->icc_err_cnt);
			icc->icc_ierrors = 0;			/* reset error count */
			if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
			break;
		}
			
		if (m->m_pkthdr.len == 0)
			break;					/* ignore */
		
		m->m_data = m->m_pktdat;
		icc->icc_inb = NULL;
		
		mtx_unlock(&icc->icc_lock);
		(*ifp->if_input)(ifp, m);
		mtx_lock(&icc->icc_lock);
		
		break;
	case INTR_RECEIVE:

		if ((m = icc->icc_inb) == NULL) {
			icc->icc_err_cnt++;
			break;
		}

		if (m->m_pkthdr.len >= ifp->if_mtu) {
			icc->icc_err_cnt++;
			break;	
		}

		if (m->m_pkthdr.len >= MHLEN) {
			icc->icc_err_cnt++;
			break;	
		}

		*mtod(m, u_char *) = *c;
	
		m->m_data++;
		m->m_len++;
		m->m_pkthdr.len++;
		
		break;
	case INTR_NOACK:			/* xfer terminated by master */
		break;
	case INTR_TRANSMIT:
		*c = 0xff;					/* XXX */
	  	break;
	case INTR_ERROR:
		icc->icc_err_cnt++;
		break;
	default:
		panic("%s: unknown event (%d)!", __func__, status);
	}

	mtx_unlock(&icc->icc_lock);
	return (0);
}

/*
 * XXX: incomplete..
 */

static void
icc_init(void *xsc)
{
	struct icc_softc *icc;
	
	icc = (struct icc_softc *)xsc;
	
	mtx_lock(&icc->icc_lock);
	icc_init_locked(icc);
	mtx_unlock(&icc->icc_lock);
}

static void
icc_init_locked(struct iic_softc *icc)
{
	struct ifnet *ifp;
	device_t dev;
	ifp = icc->icc_ifp;
	dev = device_get_parent(icc->icc_dev);
	
	mtx_unlock(&icc->icc_lock);
	
	if (iicbus_request_bus(dev, icdev, IIC_WAIT | IIC_INTR) == 0) { 
		mtx_lock(&icc->icc_lock);	
		iicbus_reset(dev, IIC_FASTEST, 0, NULL);
		ifp->if_drv_flags |= IFF_DRV_RUNNING;
		ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;		
	} else 
		mtx_lock(&icc->icc_lock);
}

static int
icc_encap(struct icc_softc *icc, struct mbuf **mp)
{
	device_t dev;
	u_char *bp;
	int len, sent;
	struct mbuf *m;
	int error;
	
	mtx_assert(&icc->icc_mtx, MA_OWNED);
	dev = device_get_parent(icc->icc_dev);
	bp = icc->iic_outb;
	len = 0;

	if ((*m)->m_pkthdr.len > icc->icc_ifp->if_mtu) {
		error = EMSGSIZE;
		goto out;
	}

	for (m = *mp; m != NULL; m = m->m_next) {
		bcopy(*mtod(m, u_char *), bp, m->m_len);	
		
		len += m->m_len;
		bp += m->m_len;
	}
	
	error = icbus_block_write(dev, icc->icc_addr, 
		icc->iic_outb, len, &sent);
out:
	return (error);
	
}

/*
 * Transmit can(4) frame.
 */

static void
icc_start(struct ifnet *ifp)
{
	struct icc_softc *icc;
	struct mbuf *m;
	
	if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
	    IFF_DRV_RUNNING)
		return;
	
	icc = ifp->if_softc;
	mtx_lock(&icc->icc_lock);
				
	ifp->if_drv_flags |= IFF_DRV_OACTIVE;
	for (;;) {
		IFQ_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL) 
			break;

		can_bpf_mtap(ifp, m);

		if (icc_encap(icc, &m) != 0)
			if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
		else {
			if_inc_counter(ifp, IFCOUNTER_OBYTES, m->m_pkthdr.len);
			if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);	
		}
		m_freem(m);
	}								
	ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

	mtx_unlock(&icc->icc_lock);
}

static int
icc_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
	struct icc_softc *icc;
	device_t dev, parent;
	struct ifreq *ifr;
	int error;

	icc = ifp->if_softc;
	dev = icc->icc_dev;
	parent = device_get_parent(dev);
	
	ifr = (struct ifreq *)data;
	error = 0;

	switch (cmd) {
	case SIOCGDRVSPEC:
	case SIOCSDRVSPEC:
		break;
	case SIOCSIFFLAGS:
		mtx_lock(&sc->ic_lock);
		
		if ((ifp->if_flags & IFF_UP) != 0) {
		    if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
				mtx_unlock(&sc->ic_lock);
				iic_init(iic);
				mtx_lock(&sc->ic_lock);
			}
		} else if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0) {
			/* XXX: disable PCF, try to release the bus anyway */
			ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
			
			mtx_unlock(&sc->ic_lock);
			iicbus_release_bus(parent, icdev);
			mtx_lock(&sc->ic_lock);
		}
		mtx_unlock(&sc->ic_lock);
		break;
	default:
		error = can_ioctl(ifp, cmd, data);
		break;
	}

	return (error);
}

/*
 * Hooks for the operating system.
 */
 
static device_method_t icc_methods[] = {
	/* device(9) interface */
	DEVMETHOD(device_probe,		icc_probe),
	DEVMETHOD(device_attach,	icc_attach),

	/* iicbus(4) interface */
	DEVMETHOD(iicbus_intr,		icc_intr),

	{ 0, 0 }
};

static driver_t icc_driver = {
	"icc",
	icc_methods,
	sizeof(struct icc_softc),
};

static devclass_t icc_devclass;

DRIVER_MODULE(icc, iicbus, icc_driver, icc_devclass, 0, 0);
MODULE_DEPEND(icc, iicbus, IICBUS_MINVER, IICBUS_PREFVER, IICBUS_MAXVER);
MODULE_VERSION(icc, 1);
