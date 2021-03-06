/*
 * Copyright (c) 2018, 2019, 2021 Henning Matyschok, DARPA/AFRL
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
#include <sys/sockio.h>

#include <machine/bus.h>
#include <sys/rman.h>
#include <machine/resource.h>

#include <net/if.h>
#include <net/if_can.h>
#include <net/if_var.h>
#include <net/if_types.h>

/*
 * XXX
 *  Work in progres.. 
 * 
 *  Conformity with AN97076 is not accompished, see
 *
 *      linux/drivers/net/can/sja1000/sja1000.c
 *
 *  for further details.
 */

#include <dev/sja/if_sjareg.h>

#include "sja_if.h"

static int  sja_probe(device_t);
static int  sja_attach(device_t);
static int  sja_detach(device_t);

static void sja_rxeof(struct sja_softc *);
static void sja_txeof(struct sja_softc *);
static int  sja_error(struct sja_softc *, uint8_t);
static int  sja_intr(void *);
static void sja_start(struct ifnet *);
static void sja_start_locked(struct ifnet *);
static int  sja_encap(struct sja_softc *, struct mbuf **);
static int  sja_ioctl(struct ifnet *, u_long, caddr_t);
static void sja_init(void *);
static void sja_init_locked(struct sja_softc *);
static int  sja_reset(struct sja_softc *);
static void sja_rxfilter(struct sja_softc *);
static int  sja_normal_mode(struct sja_softc *);
static int  sja_set_link_timings(struct sja_softc *);
static void sja_stop(struct sja_softc *);

/*
 * can(4) link timing capabilities
 */
static const struct can_link_timecaps sja_timecaps = {
    .cltc_ps1_min =     1,
    .cltc_ps1_max =     16,
    .cltc_ps2_min =     1,
    .cltc_ps2_max =     8,
    .cltc_sjw_max =     4,
    .cltc_brp_min =     1,
    .cltc_brp_max =     64,
    .cltc_brp_inc =     1,
    .cltc_linkmode_caps =       SJA_LINKMODE_CAPS,
};

static int
sja_probe(device_t dev)
{

    device_set_desc(dev, "SJA1000 network interface");

    return (BUS_PROBE_SPECIFIC);
}

static int
sja_attach(device_t dev)
{
    struct sja_softc *sc
    struct ifnet *ifp;
    int rid, error;
    uint8_t addr;

    sc = device_get_softc(dev);
    sc->sja_dev = dev;

    /* map params inherited from parent */
    sc->sja_var = device_get_ivars(dev);

    mtx_init(&sc->sja_mtx, device_get_nameunit(dev),
        MTX_NETWORK_LOCK, MTX_DEF);

    /* reserve interrupt resources */
    rid = 0;
    sc->sja_irq = bus_alloc_resource_any(dev, SYS_RES_IRQ,
        &rid, RF_SHAREABLE | RF_ACTIVE);

    if (sc->sja_irq == NULL) {
        device_printf(dev, "couldn't reserve interrupt resources\n");
        error = ENXIO;
        goto fail;
    }

    /* allocate and initialize ifnet(9) structure */
    if ((ifp = sc->sja_ifp = if_alloc(IFT_CAN)) == NULL) {
        device_printf(dev, "couldn't if_alloc(9)\n");
        error = ENOSPC;
        goto fail;
    }
    ifp->if_softc = sc;

    if_initname(ifp, device_get_name(dev), device_get_unit(dev));

    /* XXX: IFF_PROMISC maps to CAN_LINKMODE_LISTENONLY */
    ifp->if_flags = (IFF_BROADCAST | IFF_SIMPLEX | IFF_MULTICAST);

    ifp->if_init = sja_init;
    ifp->if_start = sja_start;
    ifp->if_ioctl = sja_ioctl;

    /* initialize queue for transmission */
    IFQ_SET_MAXLEN(&ifp->if_snd, ifqmaxlen);
    IFQ_SET_READY(&ifp->if_snd);

    can_ifattach(ifp, &sja_timecaps, sc->sja_var->sja_freq);

    /* force into reset mode */
    if ((error = sja_reset(sc)) != 0) {
        device_printf(dev, "couldn't reset\n");
        goto fail1;
    }

    /* set clock divider */
    sc->sja_var->sja_cdr |= SJA_CDR_PELICAN;
    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_CDR,
        sc->sja_var->sja_cdr);

    /* set acceptance filter (accept all) */
    for (addr = SJA_ACR0; addr < SJA_AMR0; addr++)
        SJA_WRITE_1(sc->sja_dev, sc->sja_var, addr, 0x00);

    for (addr = SJA_AMR0; addr <= SJA_AMR3; addr++)
        SJA_WRITE_1(sc->sja_dev, sc->sja_var, addr, 0xff);

    /* set output control register */
    sc->sja_var->sja_ocr |= SJA_OCR_MOD_NORM;
    SJA_WRITE_1(sc->sja_dev, sc->sja_var,
        SJA_OCR, sc->sja_var->sja_ocr);

    /* set normal mode */
    if ((error = sja_normal_mode(sc)) != 0) {
        device_printf(dev, "couldn't set normal mode\n");
        goto fail1;
    }

    /* hook interrupts */
    error = bus_setup_intr(dev, sc->sja_irq,
        INTR_TYPE_NET | INTR_MPSAFE, sja_intr,
            NULL, sc, &sc->sja_intr);

    if (error != 0) {
        device_printf(dev, "couldn't set up irq\n");
        goto fail1;
    }
out:
    return (error);
fail1:
    can_ifdetach(ifp);
fail:
    (void)sja_detach(dev);
    goto out;
}

static int
sja_detach(device_t dev)
{
    struct sja_softc *sc
    struct ifnet *ifp;

    sc = device_get_softc(dev);
    ifp = sc->sja_ifp;

    if (device_is_attached(dev) != 0) {
        SJA_LOCK(sc);
        sja_stop(sc);
        SJA_UNLOCK(sc);
        can_ifdetach(ifp);
    }

    if (sc->sja_intr != NULL) {
        (void)bus_teardown_intr(dev, sc->sja_irq, sc->sja_intr);
        sc->sja_intr = NULL;
    }

    if (sc->sja_irq != NULL)
        (void)bus_release_resource(dev, SYS_RES_IRQ, 0, sc->sja_irq);

    if (ifp != NULL)
        if_free(ifp);

    mtx_destroy(&sc->sja_mtx);

    return (0);
}

static int
sja_intr(void *arg)
{
    struct sja_softc *sc
    struct ifnet *ifp;
    uint8_t status;
    int n, error;

    sc = (struct sja_softc *)arg;

    SJA_LOCK(sc);

    ifp = sc->sja_ifp;

    if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0)
        goto bad;

    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_IR);

    for (n = 0; status != SJA_IR_OFF && n < 6; n++) { /* XXX */

        if ((status & SJA_IR_RI) != 0)
            sja_rxeof(sc);

        if ((status & SJA_IR_TI) != 0)
            sja_txeof(sc);

        if ((status & SJA_IR_ERR) != 0) {
            if (sja_error(sc, status) != 0)
                break;
        }
        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_IR);
    }
    error = FILTER_HANDLED;
done:
    SJA_CLEAR_INTR(sc->sja_dev, sc->sja_var);
    SJA_UNLOCK(sc);
    return (error);
bad:
    error = FILTER_STRAY;
    goto done;
}

/*
 * Copy can(4) frame from RX buffer into mbuf(9).
 */
static void
sja_rxeof(struct sja_softc *sc)
{
    struct ifnet *ifp;
    struct mbuf *m;
    struct can_frame *cf;
    uint8_t status, addr;
    int i;

    SJA_LOCK_ASSERT(sc);
    ifp = sc->sja_ifp;

again:
    if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL) {
        if_inc_counter(ifp, IFCOUNTER_IQDROPS, 1);
        goto done;
    }

    (void)memset(mtod(m, caddr_t), 0, MHLEN);
    cf = mtod(m, struct can_frame *);

    /* fetch frame information */
    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_FI);

    /* map id */
    if ((status & SJA_FI_FF) != 0) {
        cf->can_id = (SJA_READ_4(sc->sja_dev,
            sc->sja_var, SJA_ID + 4) >> 3);
        cf->can_id |= CAN_EFF_FLAG;
        addr = SJA_DATA_EFF;
    } else {
        cf->can_id = (SJA_READ_2(sc->sja_dev,
            sc->sja_var, SJA_ID + 2) >> 5);
        addr = SJA_DATA_SFF;
    }

    /* map dlc and data region */
    if ((status & SJA_FI_RTR) != 0) {
        cf->can_id |= CAN_RTR_FLAG;
        addr = cf->can_dlc = 0;
    } else
        cf->can_dlc = (status & SJA_FI_DLC);

    for (i = 0; i < cf->can_dlc; addr++, i++)
        cf->can_data[i] = SJA_READ_1(sc->sja_dev, sc->sja_var, addr);

    /* pass can(4) frame to layer above */
    m->m_len = m->m_pkthdr.len = sizeof(struct can_frame);
    m->m_pkthdr.rcvif = ifp;

    SJA_UNLOCK(sc);
    (*ifp->if_input)(ifp, m);
    SJA_LOCK(sc);

done:   /* SJA1000, 6.4.4, note 4 */
    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_CMR, SJA_CMR_RRB);
    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_SR);

    if ((status & SJA_SR_RBS) != 0)
        goto again;
}

static int
sja_error(struct sja_softc *sc, uint8_t intr)
{
    struct ifnet *ifp;
    struct can_ifsoftc *csc;
    struct mbuf *m;
    struct can_frame *cf;
    uint8_t status, flags;

    SJA_LOCK_ASSERT(sc);
    ifp = sc->sja_ifp;
    csc = ifp->if_l2com;

    if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL)
        return (ENOBUFS);

    (void)memset(mtod(m, caddr_t), 0, MHLEN);
    cf = mtod(m, struct can_frame *);
    cf->can_id |= CAN_ERR_FLAG;

    /* fetch status information */
    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_SR);

    /*  error passive / warning condition */
    if ((intr & (SJA_IR_EPI | SJA_IR_EI)) != 0) {

        if ((status & SJA_SR_BS) != 0)
            flags = CAN_STATE_BUS_OFF;
        else if ((status & SJA_SR_ES) != 0)
            flags = CAN_STATE_ERR_WARN;
        else
            flags = CAN_STATE_ERR_ACTIVE;

        if (flags != csc->csc_flags) {
            csc->csc_flags = flags;

            if (csc->csc_flags == CAN_STATE_BUS_OFF) {
                ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
                sja_init_locked(sc);
            }

            cf->can_id |= CAN_ERR_DEV;
        }
    }

    /* data overrun condition */
    if ((intr & SJA_IR_DOI) != 0) {
        if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);

        cf->can_id |= CAN_ERR_DEV;
        cf->can_data[CAN_ERR_DF_DEV] |= CAN_ERR_DEV_RX_OVF;

        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_CMR, SJA_CMR_CDO);
        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_SR);
    }

    /* bus error condition */
    if ((intr & SJA_IR_BEI) != 0) {
        if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);

        flags = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_ECC);

        cf->can_id |= (CAN_ERR_PROTO | CAN_ERR_BE);

        /* map error condition, if any */
        if ((flags & SJA_ECC_ERR_MASK) == SJA_ECC_BE)
            cf->can_data[CAN_ERR_DF_PROTO] |= CAN_ERR_PROTO_BIT;
        else if ((flags & SJA_ECC_ERR_MASK) == SJA_ECC_FMT)
            cf->can_data[CAN_ERR_DF_PROTO] |= CAN_ERR_PROTO_FORM;
        else if ((flags & SJA_ECC_ERR_MASK) == SJA_ECC_CRC_SEQ)
            cf->can_data[CAN_ERR_DF_PROTO] |= CAN_ERR_PROTO_STUFF;

        /* map tx error condition, if any */
        if ((flags & SJA_ECC_DIR) == 0)
            cf->can_data[CAN_ERR_DF_PROTO] |= CAN_ERR_PROTO_TX;

        /* map error location */
        cf->can_data[CAN_ERR_DF_PROTO_LOC] |= flags & SJA_ECC_SEG;
    }

    /* arbitriation lost condition */
    if ((intr & SJA_IR_ALI) != 0) {
        if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);

        flags = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_ALC);

        cf->can_id |= CAN_ERR_AL;
        cf->can_data[CAN_ERR_DF_AL] |= flags & SJA_ALC_MASK;
    }

    /* map error count */
    cf->can_data[CAN_ERR_DF_RX] = SJA_READ_1(sc->sja_dev,
        sc->sja_var, SJA_RXERR);
    cf->can_data[CAN_ERR_DF_TX] = SJA_READ_1(sc->sja_dev,
        sc->sja_var, SJA_TXERR);

    /* pass can(4) frame to upper layer */
    m->m_len = m->m_pkthdr.len = sizeof(struct can_frame);
    m->m_pkthdr.rcvif = ifp;

    SJA_UNLOCK(sc);
    (*ifp->if_input)(ifp, m);
    SJA_LOCK(sc);

    return (0);
}

/*
 * Tx can(4) frame.
 */

static void
sja_start(struct ifnet *ifp)
{
    struct sja_softc *sc

    sc = ifp->if_softc;
    SJA_LOCK(sc);
    sja_start_locked(ifp);
    SJA_UNLOCK(sc);
}

static void
sja_start_locked(struct ifnet *ifp)
{
    struct sja_softc *sc
    struct can_ifsoftc *csc;
    struct mbuf *m;
    uint8_t status;

    sc = ifp->if_softc;
    SJA_LOCK_ASSERT(sc);
    csc = ifp->if_l2com;

    if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
        IFF_DRV_RUNNING)
        return;

    ifp->if_drv_flags |= IFF_DRV_OACTIVE;

    for (;;) {
        IFQ_DEQUEUE(&ifp->if_snd, m);
        if (m == NULL)
            break;

        /* IAP on bpf(4) */
        can_bpf_mtap(ifp, m);

        if (sja_encap(sc, &m) != 0)
            if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
        else {
            /* notify controller for transmission */
            if ((csc->csc_linkmodes & CAN_LINKMODE_ONE_SHOT) != 0)
                status = SJA_CMR_AT;
            else
                status = 0x00;

            if ((csc->csc_linkmodes & CAN_LINKMODE_LOOPBACK) != 0)
                status |= SJA_CMR_SRR;
            else
                status |= SJA_CMR_TR;

            SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_CMR, status);
            status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_SR);
        }
    }

    ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

static int
sja_encap(struct sja_softc *sc, struct mbuf **mp)
{
    struct mbuf *m;
    struct can_frame *cf;
    uint8_t status, addr;
    int i, len, error;

    SJA_LOCK_ASSERT(sc);

    /* get a writable copy, if any */
    if (M_WRITABLE(*mp) == 0) {
        if ((m = m_dup(*mp, M_NOWAIT)) == NULL) {
            error = ENOBUFS;
            goto out;
        }
        m_freem(*mp);
        *mp = m;
    } else
        m = *mp;

    cf = mtod(m, struct can_frame *);

    /* determine can(4) frame type and map dlc */
    if ((cf->can_id & CAN_EFF_FLAG) != 0) {
        status = SJA_FI_FF;

        if ((cf->can_id & CAN_RTR_FLAG) != 0)
            status |= SJA_FI_RTR;
    } else
        status = 0x00;

    status |= (cf->can_dlc & SJA_FI_DLC);

    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_FI, status);

    /* map can(4) ID */
    if ((cf->can_id & CAN_EFF_FLAG) != 0) {
        cf->can_id &= CAN_EFF_MASK;
        cf->can_id <<= 3;

        SJA_WRITE_4(sc->sja_dev, sc->sja_var, SJA_ID + 4, cf->can_id);

        addr = SJA_DATA_EFF;
    } else {
        cf->can_id &= CAN_SFF_MASK;
        cf->can_id <<= 5;

        SJA_WRITE_2(sc->sja_dev, sc->sja_var, SJA_ID + 2, cf->can_id);

        addr = SJA_DATA_SFF;
    }

    /* copy can(4) SDU into TX buffer */
    for (error = i = 0, len = cf->can_dlc; i < len; addr++, i++)
        SJA_WRITE_1(sc->sja_dev, sc->sja_var, addr, cf->can_data[i]);

out:
    m_freem(*mp);
    *mp = NULL;

    return (error);
}

static void
sja_txeof(struct sja_softc *sc)
{
    struct ifnet *ifp;
    struct can_ifsoftc *csc;
    uint8_t status;

    SJA_LOCK_ASSERT(sc);

    ifp = sc->sja_ifp;
    csc = ifp->if_l2com;

    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_SR);

    /* TX buffer released or TX complete */
    if ((csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK) != 0
        && (status & SJA_SR_TCS) == 0)
        if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);
    else {
        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_FI);
        status &= SJA_FI_DLC;

        if_inc_counter(ifp, IFCOUNTER_OBYTES, status);
        if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
    }
}

/*
 * Initialize sja(4) controller.
 */

static void
sja_init(void *xsc)
{
    struct sja_softc *sc

    sc = (struct sja_softc *)xsc;

    SJA_LOCK(sc);
    sja_init_locked(sc);
    SJA_UNLOCK(sc);
}

static void
sja_init_locked(struct sja_softc *sc)
{
    struct ifnet *ifp;
    struct can_ifsoftc *csc;
    struct timeval tv0, tv;
    uint8_t status, addr;

    SJA_LOCK_ASSERT(sc);
    ifp = sc->sja_ifp;
    csc = ifp->if_l2com;

    if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
        return;

    /* disable interrupts and abort pending transmission, if any */
    sja_stop(sc);

    getmicrotime(&tv0);
    getmicrotime(&tv);

    /* force controller into reset mode */
    for (; sja_timercmp(&tv0, &tv, 100);) {
        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_MOD, SJA_MOD_RM);
        DELAY(10);

        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_MOD);
        getmicrotime(&tv);
    }

    if ((status & SJA_MOD_RM) != 0) {
        csc->csc_flags = CAN_STATE_SUSPENDED;

        /* set clock divider */
        sc->sja_var->sja_cdr |= SJA_CDR_PELICAN;
        SJA_WRITE_1(sc->sja_dev, sc->sja_var,
            SJA_CDR, sc->sja_var->sja_cdr);

        /* set acceptance filter (accept all) */
        for (addr = SJA_ACR0; addr < SJA_AMR0; addr++)
            SJA_WRITE_1(sc->sja_dev, sc->sja_var, addr, 0x00);

        for (addr = SJA_AMR0; addr <= SJA_AMR3; addr++)
            SJA_WRITE_1(sc->sja_dev, sc->sja_var, addr, 0xff);

        /* set output control register */
        sc->sja_var->sja_ocr |= SJA_OCR_MOD_NORM;
        SJA_WRITE_1(sc->sja_dev, sc->sja_var,
            SJA_OCR, sc->sja_var->sja_ocr);

        /* flush error counters and error code capture */
        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_TXERR, 0x00);
        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_RXERR, 0x00);

        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_ECC);

        /* clear interrupt flags */
        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_IR);

        getmicrotime(&tv0);
        getmicrotime(&tv);

        /* leave reset mode and force controller into normal mode */
        for (; sja_timercmp(&tv0, &tv, 100);) {
            status = SJA_MOD_RM;

            if ((csc->csc_linkmodes & CAN_LINKMODE_LISTENONLY) != 0)
                status |= SJA_MOD_LOM;

            if ((csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK) != 0)
                status |= SJA_MOD_STM;

            SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_MOD, status);
            DELAY(10);

            status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_MOD);
            getmicrotime(&tv);
        }
    }

    /* enable interrupts, if any */
    if ((status & SJA_MOD_RM) == 0) {
        status = SJA_IER_ALL;

        if ((csc->csc_linkmodes & CAN_LINKMODE_BUS_ERR_REP) == 0)
            status &= ~SJA_IER_BEIE;

        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_IER, status);

        csc->csc_flags = CAN_STATE_ERR_ACTIVE;

        ifp->if_drv_flags |= IFF_DRV_RUNNING;
        ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
    }
}

/* ARGSUSED */
static int
sja_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
    struct sja_softc *sc
    struct ifreq *ifr;
    struct ifdrv *ifd;
    int error;

    sc = ifp->if_softc;
    ifr = (struct ifreq *)data;
    ifd = (struct ifdrv *)data;
    error = 0;

    switch (cmd) {
    case SIOCGDRVSPEC:
    case SIOCSDRVSPEC:
        switch (ifd->ifd_cmd) {
        case CANSLINKTIMINGS:
            SJA_LOCK(sc);
            error = sja_set_link_timings(sc);
            SJA_UNLOCK(sc);
            break;
        default:
            break;
        }
        break;
    case SIOCSIFFLAGS:
        SJA_LOCK(sc);
        if ((ifp->if_flags & IFF_UP) != 0) {
            if (ifp->if_drv_flags & IFF_DRV_RUNNING) {
                if ((ifp->if_flags ^ sc->sja_if_flags) &
                    (IFF_PROMISC | IFF_ALLMULTI))
                    sja_rxfilter(sc);

                sja_init_locked(sc);
            }
        } else {
            if ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)
                sja_stop(sc);
        }
        sc->sja_if_flags = ifp->if_flags;
        SJA_UNLOCK(sc);
        break;
    default:
        error = can_ioctl(ifp, cmd, data);
        break;
    }

    return (error);
}

/*
 * Common subr.
 */

static int
sja_normal_mode(struct sja_softc *sc)
{
    struct can_ifsoftc *csc;
    struct timeval tv0, tv;
    uint8_t status;
    int error;

    csc = sc->sja_ifp->if_l2com;

    /* flush error counters and error code capture */
    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_TXERR, 0x00);
    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_RXERR, 0x00);

    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_ECC);

    /* clear interrupt flags */
    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_IR);

    /* fetch contents of status register */
    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_MOD);

    getmicrotime(&tv0);
    getmicrotime(&tv);

    /* set normal mode and enable interrupts, if any */
    for (error = EIO; sja_timercmp(&tv0, &tv, 100); ) {

        if ((status & SJA_MOD_RM) == 0) {
            status = SJA_IER_ALL;
            status &= ~SJA_IER_BEIE;

            if ((csc->csc_linkmodes & CAN_LINKMODE_BUS_ERR_REP) == 0)
                status &= ~SJA_IER_BEIE;

            SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_IER, status);

            csc->csc_flags = CAN_STATE_ERR_ACTIVE;
            error = 0;
            break;
        }

        status = SJA_MOD_RM;

        if ((csc->csc_linkmodes & CAN_LINKMODE_LISTENONLY) != 0)
            status |= SJA_MOD_LOM;

        if ((csc->csc_linkmodes & CAN_LINKMODE_PRESUME_ACK) != 0)
            status |= SJA_MOD_STM;

        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_MOD, status);
        DELAY(10);

        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_MOD);
        getmicrotime(&tv);
    }

    return (error);
}

static int
sja_reset(struct sja_softc *sc)
{
    struct can_ifsoftc *csc;
    struct timeval tv0, tv;
    uint8_t status;
    int error;

    csc = sc->sja_ifp->if_l2com;

    /* disable interrupts, if any */
    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_IER, SJA_IER_OFF);

    /* fetch contents of status register */
    status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_MOD);

    getmicrotime(&tv0);
    getmicrotime(&tv);

    /* set reset mode, until break-condition takes place */
    for (error = EIO; sja_timercmp(&tv0, &tv, 100); ) {

        if ((status & SJA_MOD_RM) != 0) {
            csc->csc_flags = CAN_STATE_SUSPENDED;
            error = 0;
            break;
        }

        SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_MOD, SJA_MOD_RM);
        DELAY(10);

        status = SJA_READ_1(sc->sja_dev, sc->sja_var, SJA_MOD);
        getmicrotime(&tv);
    }

    return (error);
}

static void
sja_rxfilter(struct sja_softc *sc)
{

}

static int
sja_set_link_timings(struct sja_softc *sc)
{
    struct can_ifsoftc *csc;
    struct can_link_timings *clt;
    uint8_t status;

    csc = sc->sja_ifp->if_l2com;
    clt = &csc->csc_timings;

    /* baud rate prescalar and synchroniziation jump */
    status = ((clt->clt_brp - 1) & SJA_BTR0_BRP_MASK);
    status |= (((clt->clt_sjw - 1) & SJA_BTR0_SJW_MASK) << 6);

    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_BTR0, status);

    /* time segments and sampling, if any */
    status = ((clt->clt_prop + clt->clt_ps1 - 1) & 0x0f);
    status |= (((clt->clt_ps2 - 1) & 0x07) << 4);

    if ((csc->csc_linkmodes & CAN_LINKMODE_3SAMPLES) != 0)
        status |= SJA_BTR1_SAM;

    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_BTR1, status);

    return (0);
}

static void
sja_stop(struct sja_softc *sc)
{
    struct ifnet *ifp;

    SJA_LOCK_ASSERT(sc);
    ifp = sc->sja_ifp;

    ifp->if_drv_flags |= ~(IFF_DRV_RUNNING | IFF_DRV_OACTIVE);

    /* disable interrupts and abort pending transmission, if any. */

    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_IER, SJA_IER_OFF);
    SJA_WRITE_1(sc->sja_dev, sc->sja_var, SJA_CMR, SJA_CMR_AT);
}

/*
 * Common I/O subr.
 */

static uint8_t
sja_read_1(device_t dev, struct sja_data *var, int port)
{

    return (SJA_READ_1(device_get_parent(dev), var, port));
}

static uint16_t
sja_read_2(device_t dev, struct sja_data *var, int port)
{

    return (SJA_READ_2(device_get_parent(dev), var, port));
}

static uint32_t
sja_read_4(device_t dev, struct sja_data *var, int port)
{

    return (SJA_READ_4(device_get_parent(dev), var, port));
}

static void
sja_write_1(device_t dev, struct sja_data *var, int port, uint8_t val)
{

    SJA_WRITE_1(device_get_parent(dev), var, port, val);
}

static void
sja_write_2(device_t dev, struct sja_data *var, int port, uint16_t val)
{

    SJA_WRITE_2(device_get_parent(dev), var, port, val);
}

static void
sja_write_4(device_t dev, struct sja_data *var, int port, uint32_t val)
{

    SJA_WRITE_4(device_get_parent(dev), var, port, val);
}

static void
sja_clear_intr(device_t dev, struct sja_data *var)
{

    SJA_CLEAR_INTR(device_get_parent(dev), var);
}

/*
 * Hooks for the operating system.
 */

static device_method_t sja_methods[] = {
    /* device(9) interface */
    DEVMETHOD(device_probe,     sja_probe),
    DEVMETHOD(device_attach,    sja_attach),
    DEVMETHOD(device_detach,    sja_detach),

    /* sja(4) interface */
    DEVMETHOD(sja_read_1,   sja_read_1),
    DEVMETHOD(sja_read_2,   sja_read_2),
    DEVMETHOD(sja_read_4,   sja_read_4),

    DEVMETHOD(sja_write_1,  sja_write_1),
    DEVMETHOD(sja_write_2,  sja_write_2),
    DEVMETHOD(sja_write_4,  sja_write_4),

    DEVMETHOD(sja_clear_intr,   sja_clear_intr),

    DEVMETHOD_END
};

driver_t sja_driver = {
    "sja",
    sja_methods,
    sizeof(struct sja_softc)
};

devclass_t sja_devclass;

DRIVER_MODULE(sja, mskc, msk_driver, msk_devclass, NULL, NULL);
MODULE_DEPEND(sja, can, 1, 1, 1);
