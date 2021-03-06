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

#include <sys/cdefs.h>

#include "opt_can.h"

#include <sys/param.h>
#include <sys/filio.h>
#include <sys/systm.h>
#include <sys/mbuf.h>
#include <sys/time.h>
#include <sys/types.h>
#include <sys/conf.h>
#include <sys/malloc.h>
#include <sys/kernel.h>
#include <sys/lock.h>
#include <sys/module.h>
#include <sys/mutex.h>
#include <sys/proc.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/tty.h>

#include <net/if.h>
#include <net/if_clone.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_slcvar.h>

/*
 * Serial line can(4) interface implemented by tty(4) hook.
 *
 * Experimental test-case by utilizing uart(4) for Media Access Control (MAC):
 *
 *  (a) intsantiate proc. by utilizing open(2) through uart(4) device(9)
 *
 *  (b) tc[gs]etattr(3) [CS8, ...] and fork(2)
 *
 *  (c) ifconfig slcN create
 *
 *  (d) ifconfig slcN stty cuauN
 *
 * XXX
 *  This should be understood as RAD prototype for the purpose for
 *  implementing a tty(4) device-driver class, see the implementation of
 *
 *      linux/drivers/net/can/slcan.c
 *
 *  for further details for understanding the implementation of this
 *  software and/or showcase?
 */

/* common subr. */
static void slc_destroy(struct slc_softc *);
static struct tty * slc_encap(struct slc_softc *, struct mbuf **);
static int  slc_rxeof(struct slc_softc *);
static int  slc_gtty(struct slc_softc *, void *);
static int  slc_stty(struct slc_softc *, void *, struct thread *);
static int  slc_dtty(struct slc_softc *);

/* interface cloner */
static void slc_ifclone_destroy(struct ifnet *);
static int  slc_ifclone_create(struct if_clone *, int, caddr_t);

/* interface-level routines. */
static void slc_ifinit(void *);
static int  slc_ifioctl(struct ifnet *, u_long, caddr_t);
static void slc_ifstart(struct ifnet *);

/* bottom-level routines */
static th_getc_inject_t slc_txeof;
static th_getc_poll_t   slc_txeof_poll;
static th_rint_t        slc_rint;
static th_rint_poll_t   slc_rint_poll;

/* device(9)-level routines */
static d_open_t     slc_open;
static d_close_t    slc_close;
static d_ioctl_t    slc_ioctl;

static struct if_clone *sc_cloner;
static const char slc_name[] = "slc";

static struct mtx slc_list_mtx;
static TAILQ_HEAD(slc_head, slc_softc) slc_list =
    TAILQ_HEAD_INITIALIZER(slc_list);

static MALLOC_DEFINE(M_SLC, "slc", "Serial line can(4) Interface");

/* tty(4) hook */
static struct ttyhook slc_hook = {
    .th_getc_inject =   slc_txeof,
    .th_getc_poll =     slc_txeof_poll,
    .th_rint =      slc_rint,
    .th_rint_poll = slc_rint_poll,
};

/* device(9) methods */
static struct cdevsw slc_cdevsw = {
    .d_version =    D_VERSION,
    .d_open =   slc_open,
    .d_close =  slc_close,
    .d_ioctl =  slc_ioctl,
    .d_name =   slc_name,
};

/*
 * Subr. on ifclone(4).
 */

static int
slc_ifclone_create(struct if_clone *ifc, int unit, caddr_t data)
{
    struct slc_softc *sc;
    struct ifnet *ifp;
    struct cdev *dev;

    sc = malloc(sizeof(struct slc_softc), M_SLC, M_WAITOK | M_ZERO);
    if ((ifp = sc->slc_ifp = if_alloc(IFT_CAN)) == NULL) {
        free(sc, M_SLC);
        return (ENOSPC);
    }

    /* initialize char-device */
    dev = make_dev(&slc_cdevsw, unit,
            UID_ROOT, GID_WHEEL, 0600, "%s%d", slc_name, unit);
    dev->si_drv1 = sc;
    sc->slc_dev = dev;

    /* initialize its protective lock */
    mtx_init(&sc->slc_mtx, "slc_mtx", NULL, MTX_DEF);

    /* initialize queue for transmission */
    mtx_init(&sc->slc_outq.ifq_mtx, "slc_outq_mtx", NULL, MTX_DEF);
    IFQ_SET_MAXLEN(&ifp->if_snd, ifqmaxlen);
    IFQ_SET_READY(&ifp->if_snd);

    /* attach */
    mtx_lock(&slc_list_mtx);
    TAILQ_INSERT_TAIL(&slc_list, sc, slc_next);
    mtx_unlock(&slc_list_mtx);

    ifp->if_softc = sc;

    if_initname(ifp, slc_name, unit);

    ifp->if_flags = (IFF_POINTOPOINT | IFF_MULTICAST);
    ifp->if_init = slc_ifinit;
    ifp->if_start = slc_ifstart;
    ifp->if_ioctl = slc_ifioctl;

    can_ifattach(ifp, NULL, 0);

    ifp->if_mtu = SLC_MTU;  /* XXX */

    ifp->if_drv_flags |= IFF_DRV_RUNNING;
    ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;

    return (0);
}


static void
slc_ifclone_destroy(struct ifnet *ifp)
{
    struct slc_softc *sc;

    sc = ifp->if_softc;

    mtx_lock(&slc_list_mtx);
    TAILQ_REMOVE(&slc_list, sc, slc_next);
    mtx_unlock(&slc_list_mtx);
    slc_destroy(sc);
}

static void
slc_destroy(struct slc_softc *sc)
{
    struct ifnet *ifp;
    struct cdev *dev;

    /* destroy its ifnet(9) mapping */
    ifp = sc->slc_ifp;
    ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
    ifp->if_flags &= ~IFF_UP;

    can_ifdetach(ifp);
    if_free(ifp);

    sc->slc_ifp = NULL;

    /* detach hook, if any and flush queue */
    (void)slc_dtty(sc);

    mtx_destroy(&sc->slc_outq.ifq_mtx);
    mtx_destroy(&sc->slc_mtx);

    /* destroy its device(9) mapping */
    dev = sc->slc_dev;
    destroy_dev(dev);
    free(sc, M_SLC);
}

/*
 * Subr. maps to ifnet(9) structure.
 */

static void
slc_ifinit(void *xsc)
{
    struct slc_softc *sc;
    struct ifnet *ifp;

    sc = (struct slc_softc *)xsc;
    ifp = sc->slc_ifp;

    if (sc->slc_tp != NULL)
        ifp->if_flags |= IFF_UP;
    else
        ifp->if_flags &= ~IFF_UP;

    if ((ifp->if_flags & IFF_UP) != 0)
        ifp->if_drv_flags |= IFF_DRV_RUNNING;
    else
        ifp->if_drv_flags &= ~IFF_DRV_RUNNING;
}

static int
slc_ifioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
    struct slc_softc *sc;
    int error;

    if ((sc = ifp->if_softc) == NULL) {
        error = EINVAL;
        goto out;
    }
    error = 0;

    switch (cmd) {
    case SIOCGDRVSPEC:
    case SIOCSDRVSPEC:
        break;
    case SIOCSIFFLAGS:  /* XXX */
        slc_ifinit(sc);
        break;
    default:
        error = can_ioctl(ifp, cmd, data);
        break;
    }
out:
    return (error);
}

/*
 * Subr. maps to cdevsw{}.
 */

static int
slc_open(struct cdev *dev, int flag, int mode, struct thread *td)
{

    return (0);
}

static int
slc_close(struct cdev *dev, int flag, int mode, struct thread *td)
{

    return (0);
}

static int
slc_ioctl(struct cdev *dev, u_long cmd, caddr_t data, int flags,
    struct thread *td)
{
    struct slc_softc *sc;
    int error;

    sc = dev->si_drv1;

    switch (cmd) {
    case TIOCSETD:
        error = slc_stty(sc, data, td);
        break;
    case TIOCGETD:
        error = slc_gtty(sc, data);
        break;
    case TIOCNOTTY:
        error = slc_dtty(sc);
        break;
    default:
        error = ENOTTY;
        break;
    }
    return (error);
}

static int
slc_dtty(struct slc_softc *sc)
{
    struct tty *tp;
    int error;

    if ((tp = sc->slc_tp) != NULL) {
        tty_lock(tp);
        ttyhook_unregister(tp);
        mtx_lock(&sc->slc_mtx);
        sc->slc_tp = NULL;

        if (sc->slc_ifbuf != NULL) {
            m_freem(sc->slc_ifbuf);
            sc->slc_ifbuf = NULL;
        }
        IF_DRAIN(&sc->slc_outq);
        mtx_unlock(&sc->slc_mtx);
        error = 0;
    } else
        error = ESRCH;

    if (sc->slc_ifp != NULL)
        slc_ifinit(sc);

    return (error);
}

static int
slc_gtty(struct slc_softc *sc, void *data)
{
    dev_t *d = (dev_t *)data;

    if (sc->slc_tp != NULL)
        *d = tty_udev(sc->slc_tp);
    else
        *d = NODEV;

    return (0);
}

static int
slc_stty(struct slc_softc *sc, void *data, struct thread *td)
{
    struct proc *p;
    int fd;
    int error;

    if (td == NULL) {
        error = ESRCH;
        goto out;
    }

    if ((p = td->td_proc) == NULL) {
        error = ESRCH;
        goto out;
    }

    if (p->p_flag & P_WEXIT) {
        error = ESRCH;
        goto out;
    }
    fd = *(int *)data;

    /* attach tty(4) hook on selected line */
    mtx_lock(&sc->slc_mtx);
    error = ttyhook_register(&sc->slc_tp, p, fd, &slc_hook, sc);
    mtx_unlock(&sc->slc_mtx);
out:
    slc_ifinit(sc);
    return (error);
}

/*
 * Rx path of can(4) frame:
 *
 *  (a) Characters are receivend during runtime of ttydisc_rint(9) by
 *      callback of slc_rint(9) maps to th_rint(9) on tty(4) hook
 *      structure, when  e. g. uart_intr(9) on SER_INT_RXREADY event
 *      takes place.
 *
 *  (b) During runtime of slc_intr(9), the rx'd characters are stored
 *      on the data field of the allocated mbuf(9) maps to slc_ifbuf
 *      on slc_softc{} until break condition takes place.
 *
 *       #1 When
 *
 *            UART_STAT_{FRAMERR,OVERRUN,PARERR}
 *
 *          error condition was indicated by
 *
 *            TRE_{FRAMING,OVERRUN,PARITY}
 *
 *          flags.
 *
 *       #2 The amount of rx'd characters from the input
 *          stream exceeds the capacity of the mbuf(9).
 *
 *      On both cases the mbuf(9) is released by m_freem(9).
 *
 *       #3 If CR or BEL characters are parsed on input stream.
 *
 *  (c) The can(4) frame gets decapsulated and passed to
 *      protocol layer by call of can_ifinput(9) during
 *      runtime of slx_rxeof(9).
 */

static int
slc_rint(struct tty *tp, char c, int flags)
{
    struct slc_softc *sc;
    struct mbuf *m;
    int error;

    tty_lock_assert(tp, MA_OWNED);

    if ((sc = ttyhook_softc(tp)) == NULL) {
        error = ENETDOWN;
        goto out;
    }

    mtx_lock(&sc->slc_mtx);

    /* allocate mbuf(9) and initialize */
    if ((m = sc->slc_ifbuf) == NULL) {
        if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL) {
            sc->slc_flags |= SLC_ERROR;
            error = ENOBUFS;
            goto out1;
        }
        m->m_len = m->m_pkthdr.len = 0;
        sc->slc_ifbuf = m;
    }

    if (flags != 0) {
        sc->slc_flags |= SLC_ERROR;
        error = ECONNABORTED;
        goto bad;
    }

    if (m->m_len < SLC_MTU) {
        if ((c == SLC_HC_BEL) || (c == SLC_HC_CR)) {
            if (sc->slc_flags & SLC_ERROR) != 0) {
                sc->slc_flags &= ~SLC_ERROR;
                error = ECONNABORTED;
                goto bad;
            }

            m->m_data = m->m_pktdat;
            error = slc_rxeof(sc);
        } else {
            *mtod(m, u_char *) = c;

            m->m_data++;
            m->m_len++;
            m->m_pkthdr.len++;

            error = 0;
        }
    } else {
        error = EFBIG;
        goto bad;
    }
out1:
    mtx_unlock(&sc->slc_mtx);
out:
    return (error);
bad:
    m->m_data = m->m_pktdat;
    m_freem(m);

    sc->slc_ifbuf = NULL;
    goto out1;
}

static size_t
slc_rint_poll(struct tty *tp)
{

    return (1);
}

static int
slc_rxeof(struct slc_softc *sc)
{
    struct ifnet *ifp;
    struct mbuf *m;
    u_char buf[MHLEN], *bp;
    struct can_frame *cf;
    int len, error = 0;

    mtx_assert(&sc->slc_mtx, MA_OWNED);
    ifp = sc->slc_ifp;

    if ((m = sc->slc_ifbuf) == NULL) {
        error = ENOBUFS;
        goto out;
    }
    sc->slc_ifbuf = NULL;

    (void)memset((bp = buf), 0, MHLEN);
    cf = (struct can_frame *)bp;

    /* determine can(4) frame type */
    switch (*mtod(m, u_char *)) {
    case SLC_HC_SFF_RTR:
        cf->can_id |= CAN_RTR_FLAG;
                        /* FALLTHROUGH */
    case SLC_HC_SFF_DATA:
        break;
    case SLC_HC_EFF_RTR:
        cf->can_id |= CAN_RTR_FLAG;
                        /* FALLTHROUGH */
    case SLC_HC_EFF_DATA:
        cf->can_id |= CAN_EFF_FLAG;
        break;
    default:
        error = EINVAL;
        goto bad;
    }
    m_adj(m, SLC_CMD_LEN);

    /* fetch id */
    if ((len = can_hex2id(cf, mtod(m, u_char *))) < 0) {
        error = EINVAL;
        goto bad;
    }
    m_adj(m, len);

    /* fetch dlc */
    cf->can_dlc = *mtod(m, u_char *);
    m_adj(m, SLC_DLC_MAX);

    if ((cf->can_dlc < SLC_HC_DLC_INF) ||
        (cf->can_dlc > SLC_HC_DLC_SUP)) {
        error = EMSGSIZE;
        goto bad;
    }
    cf->can_dlc -= SLC_HC_DLC_INF;

    /* fetch data, if any */
    if ((cf->can_id & CAN_RTR_FLAG) == 0) {
        if (can_hex2bin(cf, mtod(m, u_char *)) < 0) {
            error = EINVAL;
            goto bad;
        }
    }

    len = sizeof(struct can_frame);

    /* reinitialize mbuf(9) and copy back */
    m->m_len = m->m_pkthdr.len = len;
    m->m_data = m->m_pktdat;

    bcopy(buf, mtod(m, u_char *), len);

    /* pass can(4) frame to layer above */
    m->m_pkthdr.rcvif = ifp;
    mtx_unlock(&sc->slc_mtx);
    (*ifp->if_input)(ifp, m);
    mtx_lock(&sc->slc_mtx);
out:
    return (error);
bad:
    if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
    m_freem(m);
    goto out;
}

/*
 * Tx path of can(4) frame:
 *
 *  (a) The can(4) frame is enqueued by if_transmit(9) on if_snd
 *      queue maps to the instance of generic ifnet(9) structure,
 *      when can_ifoutput(9) was called.
 *
 *  (b) The slc_ifstart(9) subr. gets invoked and dequeues.
 *
 *      During runtime of slc_encap(9), each can(4) frame is
 *      encoded as ASCII byte string and cached on slc_outq
 *      maps to slc_softc{}.
 *
 *      On success, the ttydevsw_outwakeup(9) invokes the
 *      tsw_outwakeup(9) callback function maps to the e. g.
 *      uart_tty_class(4).
 *
 *  (c) During runtime of ttydisc_getc(9), the cached ASCII
 *      byte string is dequeued and injected by slc_txeof(9).
 */

static void
slc_ifstart(struct ifnet *ifp)
{
    struct slc_softc *sc;
    struct mbuf *m;
    struct tty *tp;

    sc = ifp->if_softc;

    if ((ifp->if_drv_flags & (IFF_DRV_RUNNING | IFF_DRV_OACTIVE)) !=
        IFF_DRV_RUNNING)
        return;

    ifp->if_drv_flags |= IFF_DRV_OACTIVE;
    for (;;) {
        IFQ_DEQUEUE(&ifp->if_snd, m);
        if (m == NULL)
            break;

        can_bpf_mtap(ifp, m);

        /*
         * Encode can(4) frame in its ASCII
         * representation, see net/if_can.h
         * for further details.
         */
        if ((tp = slc_encap(sc, &m)) != NULL) {
            /* notify the tty(4) */
            tty_lock(tp);

            if (tty_gone(tp) == 0)
                ttydevsw_outwakeup(tp);

            tty_unlock(tp);
        }
    }
    ifp->if_drv_flags &= ~IFF_DRV_OACTIVE;
}

static struct tty *
slc_encap(struct slc_softc *sc, struct mbuf **mp)
{
    struct ifnet *ifp;
    struct tty *tp;
    struct mbuf *m;
    struct can_frame *cf;
    u_char buf[MHLEN], *bp;
    int len;

    mtx_lock(&sc->slc_mtx);
    ifp = sc->slc_ifp;

    if ((tp = sc->slc_tp) == NULL)
        goto bad;

    /* get a writable copy, if any */
    if (M_WRITABLE(*mp) == 0) {
        if ((m = m_dup(*mp, M_NOWAIT)) == NULL)
            goto bad1;

        m_freem(*mp);
        *mp = m;
    } else
        m = *mp;

    cf = mtod(m, struct can_frame *);

    (void)memset((bp = buf), 0, MHLEN);

    /* determine can(4) frame type */
    if ((cf->can_id & CAN_RTR_FLAG) != 0)
        *bp = SLC_HC_SFF_RTR;
    else
        *bp = SLC_HC_SFF_DATA;

    if ((cf->can_id & CAN_EFF_FLAG) != 0)
        *bp = toupper(*bp);

    bp += SLC_CMD_LEN;

    /* map ID */
    if ((len = can_id2hex(cf, bp)) < 0)
        goto bad1;

    bp += len;

    /* map DLC */
    *bp = cf->can_dlc + '0';
    bp += SLC_DLC_MAX;

    /* apply data, if any */
    if ((cf->can_id & CAN_RTR_FLAG) == 0) {
        if ((len = can_bin2hex(cf, bp)) < 0)
            goto bad1;

        bp += len;
    }

    /* finalize */
    *bp = SLC_HC_CR;
    bp += SLC_HC_LEN;

    /* re-initialize mbuf(9) and copy back */
    len = bp - buf;

    m->m_len = m->m_pkthdr.len = len;
    m->m_data = m->m_pktdat;

    bcopy(buf, mtod(m, u_char *), len);

    /* enqueue, if any */
    IF_LOCK(&sc->slc_outq);

    if (_IF_QFULL(&sc->slc_outq)) {
        IF_UNLOCK(&sc->slc_outq);
        goto bad1;
    }

    _IF_ENQUEUE(&sc->slc_outq, m);
    sc->slc_outqlen += len;

    IF_UNLOCK(&sc->slc_outq);

    /* do some statistics */
    if_inc_counter(ifp, IFCOUNTER_OBYTES, len);
    if_inc_counter(ifp, IFCOUNTER_OPACKETS, 1);
out:
    mtx_unlock(&sc->slc_mtx);
    return (tp);
bad1:
    tp = NULL;
bad:
    if_inc_counter(ifp, IFCOUNTER_OERRORS, 1);

    m_freem(*mp);
    *mp = NULL;
    goto out;
}

static size_t
slc_txeof(struct tty *tp, void *buf, size_t len)
{
    struct slc_softc *sc;
    struct mbuf *m;
    size_t off = 0;
    size_t m_len;

    if ((sc = ttyhook_softc(tp)) == NULL)
        goto out;   /* XXX */

    mtx_lock(&sc->slc_mtx);

    while (len > 0) {
        IF_DEQUEUE(&sc->slc_outq, m);
        if (m == NULL)
            break;

        while (m != NULL) {
            m_len = min(m->m_len, len);
            bcopy(mtod(m, caddr_t), (caddr_t)buf + off, m_len);

            m->m_data += m_len;
            m->m_len -= m_len;
            off += m_len;
            len -= m_len;

            if (m->m_len > 0)
                break;

            m = m_free(m);
        }

        if (m != NULL) {
            IF_PREPEND(&sc->slc_outq, m);
            break;
        }
    }
    IF_LOCK(&sc->slc_outq);
    sc->slc_outqlen -= off;
    IF_UNLOCK(&sc->slc_outq);
    mtx_unlock(&sc->slc_mtx);
    MPASS(sc->slc_outqlen >= 0);
out:
    return (off);
}

static size_t
slc_txeof_poll(struct tty *tp)
{
    struct slc_softc *sc;
    size_t outqlen;

    if ((sc = ttyhook_softc(tp)) != NULL)
        outqlen = sc->slc_outqlen;
    else
        outqlen = 0;

    return (outqlen);
}

/*
 * Modevent handler.
 */

static int
slc_modevent(module_t mod, int type, void *data)
{

    switch (type) {
    case MOD_LOAD:
        mtx_init(&slc_list_mtx, "slc_list_mtx", NULL, MTX_DEF);
        slc_cloner = if_clone_simple(slc_name,
            slc_ifclone_create, slc_ifclone_destroy, 0);
        break;
    case MOD_UNLOAD:
        if_clone_detach(slc_cloner);
        mtx_destroy(&slc_list_mtx);
        break;
    default:
        return (EOPNOTSUPP);
    }
    return (0);
}

static moduledata_t slc_mod = {
    "if_slc",
    slc_modevent,
    0
};

DECLARE_MODULE(if_slc, slc_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);
MODULE_DEPEND(if_slc, can, 1, 1, 1);
