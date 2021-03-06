/*  $NetBSD: can.c,v 1.2.2.1 2018/04/09 13:34:11 bouyer Exp $   */

/*-
 * Copyright (c) 2003, 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Robert Swindells and Manuel Bouyer
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
 * THIS SOFTWARE IS PROVIDED BY THE NETBSD FOUNDATION, INC. AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED
 * TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE FOUNDATION OR CONTRIBUTORS
 * BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */
/*
 * Copyright (c) 2018, 2019, 2021 Henning Matyschok, DARPA/AFRL
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
#include <sys/systm.h>
#include <sys/time.h>
#include <sys/kernel.h>
#include <sys/module.h>
#include <sys/mbuf.h>
#include <sys/socket.h>
#include <sys/sockio.h>

#include <net/if.h>
#include <net/if_var.h>
#include <net/if_types.h>
#include <net/if_can.h>
#include <net/netisr.h>
#include <net/bpf.h>

#ifdef MAC
#include <security/mac/mac_framework.h>
#endif /* MAC */

static MALLOC_DEFINE(M_IFCAN, "IFCAN", "CAN interface internals");

static int  can_restart(struct ifnet *);

static void     can_ifinput(struct ifnet *, struct mbuf *);
static int  can_ifoutput(struct ifnet *, struct mbuf *,
    const struct sockaddr *, struct route *);

/*
 * Process a received can(4) frame by netisr(9).
 */
static void
can_ifinput(struct ifnet *ifp, struct mbuf *m)
{
#ifdef DIAGNOSTIC
    struct can_hdr *ch;
#endif  /* DIAGNOSTIC */
    M_ASSERTPKTHDR(m);
    KASSERT(m->m_pkthdr.rcvif != NULL,
        ("%s: NULL interface pointer", __func__));

    if ((ifp->if_flags & IFF_UP) == 0) {
        if_printf(ifp, "discard can(4) frame at !IFF_UP\n\0");
        goto out;
    }

#ifdef DIAGNOSTIC
    if ((ifp->if_drv_flags & IFF_DRV_RUNNING) == 0) {
        if_printf(ifp, "discard can(4) frame at !IFF_DRV_RUNNING\n\0");
        goto out;
    }
#endif  /* DIAGNOSTIC */

    if (m->m_pkthdr.len < sizeof(struct can_frame)) {
        if_printf(ifp, "discard can(4) frame"
            " (len %u pkt len %u)\n\0",
            m->m_len, m->m_pkthdr.len);
        goto out1;
    }

    if (m->m_len < sizeof(struct can_frame)) {
        if ((m = m_pullup(m, sizeof(struct can_frame))) == NULL) {
            if_printf(ifp, "m_pullup(9) failed, discard can(4) frame.");
            goto out1;
        }
    }
#ifdef DIAGNOSTIC
    ch = mtod(m, struct can_hdr *);

    switch (ch->ch_id & CAN_FLAG_MASK) {
    case CAN_STD_FRM:
    case CAN_EXT_FRM:
    case CAN_RTR_FRM:
    case CAN_ERR_FRM:
        if_printf(ifp, "%s: ", __func__);
        m_print(m, m->m_pkthdr.len);
        break;
    default:
        goto out1;
    }
#endif /* DIAGNOSTIC */

#ifdef MAC
    mac_ifnet_create_mbuf(ifp, m);
#endif /* MAC */

    /* IAP for rx'd frames */
    can_bpf_mtap(ifp, m);

    /* remove annotated meta-data */
    if ((ifp->if_flags & IFF_LOOPBACK) != 0)
        can_mbuf_tag_clean(m);

    if_inc_counter(ifp, IFCOUNTER_IPACKETS, 1);
    if_inc_counter(ifp, IFCOUNTER_IBYTES, m->m_pkthdr.len);

    /* clear mbuf(9), if monitor mode enabled */
    if ((ifp->if_flags & IFF_MONITOR) != 0)
        goto out;

    M_SETFIB(m, ifp->if_fib);
    netisr_dispatch(NETISR_CAN, m);
    return;
out1:
    if_inc_counter(ifp, IFCOUNTER_IERRORS, 1);
out:
    m_freem(m);
}

/*
 * Enqueue can(4) frame and transmit.
 */
static int
can_ifoutput(struct ifnet *ifp, struct mbuf *m,
    const struct sockaddr *dst, struct route *ro)
{
    int error;

    if (dst == NULL) {
        error = EINVAL;
        goto bad;
    }

    if (dst->sa_family != AF_CAN) {
        error = EINVAL;
        goto bad;
    }

    if (ifp->if_l2com == NULL) {
        error = ENXIO;
        goto bad;
    }

#ifdef MAC
    if ((error = mac_ifnet_check_transmit(ifp, m)) != 0)
        goto bad;
#endif  /* MAC */

    if ((ifp->if_flags & IFF_MONITOR) != 0) {
        error = ENETDOWN;
        goto bad;
    }

    if (((ifp->if_flags & IFF_UP) == 0) &&
        ((ifp->if_drv_flags & IFF_DRV_RUNNING) != 0)) {
        error = ENETDOWN;
        goto bad;
    }

    if ((ifp->if_flags & IFF_PROMISC) != 0) {
        error = ENETDOWN;
        goto bad;
    }

#ifdef DIAGNOSTIC
    if_printf(ifp, "%s: ", __func__);
    m_print(m, m->m_pkthdr.len);
#endif /* DIAGNOSTiC */

    error = (*ifp->if_transmit)(ifp, m);
out:
    return (error);
bad:
    m_freem(m);
    goto out;
}

void
can_ifattach(struct ifnet *ifp, const struct can_link_timecaps *cltc,
    uint32_t freq)
{
    struct can_ifsoftc *csc;

    if_attach(ifp);

    ifp->if_mtu = CAN_MTU;  /* XXX */
    ifp->if_input = can_ifinput;
    ifp->if_output = can_ifoutput;

    bpfattach(ifp, DLT_CAN_SOCKETCAN, 0);

    KASSERT((ifp->if_l2com != NULL),
        ("%s: ifp->if_l2com == NULL", __func__));
    csc = ifp->if_l2com;

    if (cltc != NULL) {
        bcopy(cltc, &csc->csc_timecaps,
            sizeof(struct can_link_timecaps));
        csc->csc_timecaps.cltc_clock_freq = freq;
    }
    mtx_init(&csc->csc_mtx, "csc_mtx", NULL, MTX_DEF);

    if_printf(ifp, "Index: %d\n", ifp->if_index);
}

void
can_ifdetach(struct ifnet *ifp)
{
    struct can_ifsoftc *csc;

    KASSERT((ifp->if_l2com != NULL),
        ("%s: ifp->if_l2com == NULL", __func__));
    csc = ifp->if_l2com;
    mtx_destroy(&csc->csc_mtx);

    bpfdetach(ifp);
    if_detach(ifp);
}

void
can_ifinit_timings(struct can_ifsoftc *csc)
{
    CSC_LOCK(csc);

    /* uninitialized parameters is all-one */
    (void)memset(&csc->csc_timings, 0xff,
        sizeof(struct can_link_timings));
    CSC_UNLOCK(csc);
}

/*
 * Restart for bus-off recovery.
 */
int
can_restart(struct ifnet *ifp)
{
    struct mbuf *m;
    struct can_frame *cf;
    int error = 0;

    if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL) {
        error = ENOBUFS;
        goto done;
    }

    (void)memset(mtod(m, caddr_t), 0, MHLEN);

    cf = mtod(m, struct can_frame *);
    cf->can_id |= (CAN_ERR_FLAG | CAN_ERR_RESTARTED);

    /* pass can(4) frame to layer above */
    m->m_len = m->m_pkthdr.len = sizeof(struct can_frame);
    m->m_pkthdr.rcvif = ifp;

    (*ifp->if_input)(ifp, m);

done:
    if_printf(ifp, "restarted\n");

    (*ifp->if_init)(ifp->if_softc);

    return (error);
}

/* ARGSUSED */
int
can_ioctl(struct ifnet *ifp, u_long cmd, caddr_t data)
{
    struct can_ifsoftc *csc;
    struct ifreq *ifr;
    struct ifdrv *ifd;
    uint32_t mode;
    int error;

    ifr = (struct ifreq *)data;
    ifd = (struct ifdrv *)data;
    error = 0;

    if ((ifp->if_flags & IFF_UP) == 0) {
        error = ENETDOWN;
        goto out;
    }

    if (ifp->if_type == IFT_CAN) {
        if ((csc = ifp->if_l2com) == NULL)
            error = EOPNOTSUPP;
        else
            error = 0;
    } else
        error = EOPNOTSUPP;

    if (error != 0)
        goto out;

    switch (cmd) {
    case SIOCSIFMTU:

        if (ifr->ifr_mtu == CAN_MTU)
            ifp->if_mtu = ifr->ifr_mtu;
        else
            error = EINVAL;

        break;
    case SIOCGDRVSPEC:
    case SIOCSDRVSPEC:      /* FALLTHROUGH */

        switch (ifd->ifd_cmd) {
        case CANGLINKTIMECAP:

            if (ifd->ifd_len != sizeof(struct can_link_timecaps))
                error = EINVAL;
            else
                error = copyout(&csc->csc_timecaps,
                    ifd->ifd_data, ifd->ifd_len);
            break;
        case CANGLINKTIMINGS:

            if (ifd->ifd_len != sizeof(struct can_link_timings))
                error = EINVAL;
            else
                error = copyout(&csc->csc_timings,
                    ifd->ifd_data, ifd->ifd_len);
            break;
        case CANGLINKMODE:

            if (ifd->ifd_len != sizeof(uint32_t))
                error = EINVAL;
            else
                error = copyout(&csc->csc_linkmodes,
                    ifd->ifd_data, ifd->ifd_len);
            break;
        case CANSLINKTIMINGS:

            if (ifd->ifd_len != sizeof(struct can_link_timings))
                error = EINVAL;
            else
                error = copyin(ifd->ifd_data,
                    &csc->csc_timings, ifd->ifd_len);

            if (error != 0)
                break;

            error = (*ifp->if_ioctl)(ifp, cmd, (caddr_t)ifd);
            break;
        case CANSLINKMODE:
        case CANCLINKMODE:  /* FALLTHROUGH */

            if (ifd->ifd_len != sizeof(uint32_t))
                error = EINVAL;
            else
                error = copyin(ifd->ifd_data, &mode, ifd->ifd_len);

            if (error != 0)
                break;

            if ((mode & csc->csc_timecaps.cltc_linkmode_caps) != mode) {
                error = EINVAL;
                break;
            }

            /* XXX: locking */
            if (ifd->ifd_cmd == CANSLINKMODE)
                csc->csc_linkmodes |= mode;
            else
                csc->csc_linkmodes &= ~mode;

        case CANSRESTART:       /* FALLTHROUGH */

            error = can_restart(ifp);
            break;
        default:
            error = EOPNOTSUPP;
            break;
        }
        break;
    default:
        break;
    }
out:
    return (error);
}

/*
 * Capture a can(4) frame.
 */
void
can_bpf_mtap(struct ifnet *ifp, struct mbuf *m)
{
    struct can_frame *cf;
    canid_t oid;

    /* bpf(4) wants the can(4) id in network byte order */

    cf = mtod(m, struct can_frame *);
    oid = cf->can_id;
    cf->can_id = htonl(oid);
    bpf_mtap(ifp->if_bpf, m);
    cf->can_id = oid;
}

/*
 * Cleanup mbuf(9) tag, keeping the PACKET_TAG_ND_OUTGOING tag.
 */
void
can_mbuf_tag_clean(struct mbuf *m)
{
    struct m_tag *sotag;

    if ((sotag = m_tag_find(m, PACKET_TAG_ND_OUTGOING, NULL)) != NULL)
        m_tag_unlink(m, sotag);

    m_tag_delete_nonpersistent(m);
    if (sotag != NULL)
        m_tag_prepend(m, sotag);
}

/*
 * Subr. for encoding / decoding can(4) ID and SDU.
 */

static const char can_hex_tbl[] = "0123456789ABCDEF\0";

int
can_bin2hex(struct can_frame *cf, u_char *buf)
{
    int len, i;
    u_char *bp, *dp;
    u_char c;

    if ((bp = buf) == NULL || cf == NULL)
        return (-1);

    if ((len = cf->can_dlc) > CAN_DLC_MAX)
        return (-1);

    for (dp = bp, i = 0; i < len; i++) {
        c = cf->can_data[i];

        *dp = ((c & 0xf0) >> 4);

        if (isalpha(*dp) && islower(*dp))
            *dp = toupper(*dp);

        *dp = can_hex_tbl[*dp];

        dp += 1;

        *dp = (c & 0x0f);

        if (isalpha(*dp) && islower(*dp))
            *dp = toupper(*dp);

        *dp = can_hex_tbl[*dp];

        dp += 1;
    }

    len = dp - bp;

    return (len);
}

int
can_hex2bin(struct can_frame *cf, u_char *buf)
{
    int len, i;
    u_char *bp;
    u_char c1, c0;

    if ((bp = buf) == NULL || cf == NULL)
        return (-1);

    if ((len = cf->can_dlc) > CAN_DLC_MAX)
        return (-1);

    for (i = 0; i < len; i++) {
        c1 = *bp;
        bp += 1;

        if (isdigit(c1))
            c1 -= '0';
        else if (isalpha(c1))
            c1 -= (isupper(c1)) ? 'A' - 10 : 'a' - 10;
        else
            return (-1);

        c0 = *bp;
        bp += 1;

        if (isdigit(c0))
            c0 -= '0';
        else if (isalpha(c0))
            c0 -= (isupper(c0)) ? 'A' - 10 : 'a' - 10;
        else
            return (-1);

        cf->can_data[i] = ((c1 << 4) | c0);
    }
    return (0);
}

int
can_id2hex(struct can_frame *cf, u_char *buf)
{
    canid_t id;
    int len;
    u_char *bp, *ep;
    u_char c;

    if ((bp = buf) == NULL || cf == NULL)
        return (-1);

    if ((cf->can_id & CAN_EFF_FLAG) != 0) {
        id = (cf->can_id & CAN_EFF_MASK);
        len = SLC_EFF_ID_LEN;
    } else {
        id = (cf->can_id & CAN_SFF_MASK);
        len = SLC_SFF_ID_LEN;
    }

    for (ep = bp + len - 1; ep >= bp; ep--, id >>= 4) {
        c = (id & 0x0f);

        if (isalpha(c) && islower(c))
            c = toupper(c);

        *ep = can_hex_tbl[c];
    }
    return (len);
}

int
can_hex2id(struct can_frame *cf, u_char *buf)
{
    int len;
    canid_t u, v;
    u_char *bp, *ep;
    u_char c;

    if ((bp = buf) == NULL || cf == NULL)
        return (-1);

    if ((cf->can_id & CAN_EFF_FLAG) != 0)
        len = SLC_EFF_ID_LEN;
    else
        len = SLC_SFF_ID_LEN;

    for (u = v = 0, ep = bp + len - 1; bp <= ep; v <<= 4) {
        c = *bp;
        bp += 1;

        if (isdigit(c))
            c -= '0';
        else if (isalpha(c))
            c -= (isupper(c)) ? 'A' - 10 : 'a' - 10;
        else
            return (-1);

        v |= (c & 0x0f);
        u = v;
    }
    cf->can_id |= u;

    return (len);
}

/*
 * Allocate or free common structure of can(4) interface.
 */

static void *
can_alloc(u_char type, struct ifnet *ifp)
{
    struct can_ifsoftc *csc;

    csc = malloc(sizeof(struct can_ifsoftc),
        M_IFCAN, M_WAITOK | M_ZERO);
    csc->csc_ifp = ifp;

    return (csc);
}

static void
can_free(void *com, u_char type)
{

    free(com, M_IFCAN);
}

/*
 * Module description.
 */

static int
can_modevent(module_t mod, int type, void *data)
{

    switch (type) {
    case MOD_LOAD:
        if_register_com_alloc(IFT_CAN, can_alloc, can_free);
        break;
    case MOD_UNLOAD:
        if_deregister_com_alloc(IFT_CAN);
        break;
    default:
        return (EOPNOTSUPP);
    }
    return (0);
}

static moduledata_t can_mod = {
    "can",
    can_modevent,
    0
};

DECLARE_MODULE(can, can_mod, SI_SUB_PSEUDO, SI_ORDER_ANY);
MODULE_VERSION(can, 1);
