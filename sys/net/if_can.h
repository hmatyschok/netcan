/*  $NetBSD: can_link.h,v 1.2 2017/05/27 21:02:56 bouyer Exp $  */

/*-
 * Copyright (c) 2017 The NetBSD Foundation, Inc.
 * All rights reserved.
 *
 * This code is derived from software contributed to The NetBSD Foundation
 * by Manuel Bouyer
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

#ifndef _NET_IF_CAN_H
#define _NET_IF_CAN_H

/*
 * XXX
 *  Work in progres..
 */
#define IFT_CAN     IFT_PVC

/*
 * can(4) ID structure
 *
 *  bits 0-28   : can(4) identifier (11/29 bits, see bit 31)
 *  bit2 29-31  : see below
 */

typedef enum canid {

    CAN_STD_FRM =                       0x00000000,
    CAN_EXT_FRM =                       0x80000000,
    CAN_RTR_FRM =                       0x40000000,
    CAN_ERR_FRM =                       0x20000000,
    CAN_FLAG_MASK =                     0xe0000000,

    CAN_ERR_TX_TIMO =                   0x00000001,
    CAN_ERR_AL =                        0x00000002,
    CAN_ERR_DEV =                       0x00000004,
    CAN_ERR_PROTO =                     0x00000008,
    CAN_ERR_TRX =                       0x00000010,
    CAN_ERR_ACK =                       0x00000020,
    CAN_ERR_BO =                        0x00000040,
    CAN_ERR_BE =                        0x00000080,
    CAN_ERR_RESTARTED =                 0x00000100,

    CAN_ERR_AL_UNSPEC =                 0x00000000,

    CAN_ERR_DEV_UNSPEC =                0x00000000,
    CAN_ERR_DEV_RX_OVF =                0x00000001,
    CAN_ERR_DEV_TX_OVF =                0x00000002,
    CAN_ERR_DEV_RX_WARN =               0x00000004,
    CAN_ERR_DEV_TX_WARN =               0x00000008,
    CAN_ERR_DEV_RX_PSV =                0x00000010,
    CAN_ERR_DEV_TX_PSV =                0x00000020,
    CAN_ERR_DEV_ACTIVE =                0x00000040,

    CAN_ERR_PROTO_UNSPEC =              0x00000000,
    CAN_ERR_PROTO_BIT =                 0x00000001,
    CAN_ERR_PROTO_FORM =                0x00000002,
    CAN_ERR_PROTO_STUFF =               0x00000004,
    CAN_ERR_PROTO_BIT0 =                0x00000008,
    CAN_ERR_PROTO_BIT1 =                0x00000010,
    CAN_ERR_PROTO_OVERLOAD =            0x00000020,
    CAN_ERR_PROTO_ACTIVE =              0x00000040,
    CAN_ERR_PROTO_TX =                  0x00000080,

    CAN_ERR_PROTO_LOC_UNSPEC =          0x00000000,
    CAN_ERR_PROTO_LOC_SOF =             0x00000003,
    CAN_ERR_PROTO_LOC_ID28_21 =         0x00000002,
    CAN_ERR_PROTO_LOC_ID20_18 =         0x00000006,
    CAN_ERR_PROTO_LOC_SRTR =            0x00000004,
    CAN_ERR_PROTO_LOC_IDE =             0x00000005,
    CAN_ERR_PROTO_LOC_ID17_13 =         0x00000007,
    CAN_ERR_PROTO_LOC_ID12_05 =         0x0000000f,
    CAN_ERR_PROTO_LOC_ID04_00 =         0x0000000e,
    CAN_ERR_PROTO_LOC_RTR =             0x0000000c,
    CAN_ERR_PROTO_LOC_RES1 =            0x0000000d,
    CAN_ERR_PROTO_LOC_RES0 =            0x00000009,
    CAN_ERR_PROTO_LOC_DLC =             0x0000000b,
    CAN_ERR_PROTO_LOC_DATA =            0x0000000a,
    CAN_ERR_PROTO_LOC_CRC_SEQ =         0x00000008,
    CAN_ERR_PROTO_LOC_CRC_DEL =         0x00000018,
    CAN_ERR_PROTO_LOC_ACK =             0x00000019,
    CAN_ERR_PROTO_LOC_ACK_DEL =         0x0000001b,
    CAN_ERR_PROTO_LOC_EOF =             0x0000001a,
    CAN_ERR_PROTO_LOC_INTERM =          0x00000012,

    CAN_ERR_TRX_UNSPEC =                0x00000000,
    CAN_ERR_TRX_CANH_NO_WIRE =          0x00000004,
    CAN_ERR_TRX_CANH_SHORT_TO_BAT =     0x00000005,
    CAN_ERR_TRX_CANH_SHORT_TO_VCC =     0x00000006,
    CAN_ERR_TRX_CANH_SHORT_TO_GND =     0x00000007,
    CAN_ERR_TRX_CANL_NO_WIRE =          0x00000040,
    CAN_ERR_TRX_CANL_SHORT_TO_BAT =     0x00000050,
    CAN_ERR_TRX_CANL_SHORT_TO_VCC =     0x00000060,
    CAN_ERR_TRX_CANL_SHORT_TO_GND =     0x00000070,
    CAN_ERR_TRX_CANL_SHORT_TO_CANH =    0x00000080,

    CAN_EFF_FLAG =                      0x80000000,
    CAN_RTR_FLAG =                      0x40000000,
    CAN_ERR_FLAG =                      0x20000000,
    CAN_FLAG_MASK =                     0xe0000000,

    CAN_SFF_MASK =                      0x000007ff,
    CAN_EFF_MASK =                      0x1fffffff,
    CAN_ERR_MASK =                      0x1fffffff,
} canid_t, can_err_mask_t;

typedef enum can_dlc {
    CAN_DLC_MAX =                       8,
} can_dlc_t;

/* can(4) header */
struct can_hdr {
    canid_t ch_id;      /* ID + EFF/RTR/ERR flags */
    uint8_t ch_dlc;     /* SDU length in byte (0 .. CAN_DLC_MAX) */
    uint8_t __pad;
    uint8_t __res0;
    uint8_t __res1;
};

/* can(4) frame */
struct can_frame {
    canid_t can_id;     /* ID + EFF/RTR/ERR flags */
    uint8_t can_dlc;    /* SDU length in byte (0 .. CAN_DLC_MAX) */
    uint8_t __pad;
    uint8_t __res0;
    uint8_t __res1;
    uint8_t can_data[CAN_MAX_DLEN] __aligned(8);
};

#define CAN_MTU     (sizeof(struct can_frame))

/* index for data field on error class */
typedef enum can_err_df {
    CAN_ERR_DF_AL,
    CAN_ERR_DF_DEV,
    CAN_ERR_DF_PROTO,
    CAN_ERR_DF_PROTO_LOC,
    CAN_ERR_DF_PROTO_TRX,
    CAN_ERR_DF_RX,
    CAN_ERR_DF_TX,
} can_err_df_t;

/*
 * XXX
 *  Namespace???
 */

typedef enum can_fd_dlc {
    CAN_FD_DLC_MAX =        64,
} can_fd_dlc_t;

struct can_fd_frame {
    canid_t can_id;
    uint8_t can_dlc;
    uint8_t __pad;
    uint8_t __res0;
    uint8_t __res1;
    uint8_t can_data[CAN_FD_DLC_MAX] __aligned(8);
};

#define CAN_FD_MTU         (sizeof(struct canfd_frame))

/*
 * ASCII representation of can(4) frame:
 *
 *  <frm> ::= <type> <id> <dlc> <data> <hc>;
 *
 * encodes the
 *
 *  <type> ::= "R" | "T" | "r" | "t";
 *
 * by
 *
 *   "R" denotes remote frame, extended frame format
 *
 *   "T" denotes data frame, extendet frame format
 *
 *   "r" denotes remote frame, base frame format
 *
 *   "t" denotes data frame, base frame format
 *
 * with
 *
 *  <id> ::= <sff> | <eff>;
 *
 * where encodes the 11 identifier bits
 *
 *  <sff> ::= 3 * <byte>;
 *
 * or the 29 identifier bits
 *
 *  <eff> ::= 8 * <byte>;
 *
 * by
 *
 *  <byte> ::= <xdigit> <xdigit>;
 *
 * with
 *
 *  <xdigit> ::= <digit> | "A" | "B" | "C" | "D" | "E" | "F";
 *
 * and the
 *
 *  <dlc> ::= <digit> - "9";
 *
 * by
 *
 *  <digit> ::= "0" | "1" | "2" | "3" | "4" |
 *              "5" | "6" | "7" | "8" | "9";
 *
 * and
 *
 *  <data> ::= <dlc> * <byte>;
 *
 * with
 *
 *  <hc> ::= "CR" | "BEL";
 *
 * finally.
 */

#define SLC_CMD_LEN     (sizeof(u_char))
#define SLC_SFF_ID_LEN  (sizeof(u_char) * 3)
#define SLC_EFF_ID_LEN  (sizeof(u_char) * 8)
#define SLC_DLC_MAX     (sizeof(u_char))

#define SLC_HC_SFF_DATA     't'
#define SLC_HC_SFF_RTR  'r'
#define SLC_HC_EFF_DATA     'T'
#define SLC_HC_EFF_RTR  'R'

#define SLC_HC_CR       '\r'
#define SLC_HC_BEL      '\a'

#define SLC_HC_DLC_INF  '0'
#define SLC_HC_DLC_SUP  '9'

/*
 * The can(4) ID based filter checks received
 *
 *   can_id & can_filter.cf_mask
 *
 * against
 *
 *   can_filter.cf_id & can_filter.cf_mask
 *
 * Valid flags for
 *
 *   can_id maps to CAN_INV_FILTER
 *
 * denotes invert filter and valid flags for
 *
 *   can_mask maps to CAN_ERR_FLAG
 *
 * denotes filter for error message frames.
 */
struct can_filter {
    canid_t cf_id;
    canid_t cf_mask;
};

#define CAN_INV_FILTER  0x20000000U

/* transport protocol class address information (e.g. ISOTP) */
struct can_tp {
    canid_t     ct_rx_id;
    canid_t     ct_tx_id;
};

/*
 * can(4) bus link-layer related commands, from the SIOCSDRVSPEC
 */

/* get timing capabilities from HW */
struct can_link_timecaps {
    uint32_t    cltc_prop_min; /* prop seg, in tq */
    uint32_t    cltc_prop_max;
    uint32_t    cltc_ps1_min; /* phase1 seg, in tq */
    uint32_t    cltc_ps1_max;
    uint32_t    cltc_ps2_min; /* phase 2 seg, in tq */
    uint32_t    cltc_ps2_max;
    uint32_t    cltc_sjw_max;   /* Synchronisation Jump Width */
    uint32_t    cltc_brp_min;   /* bit-rate prescaler */
    uint32_t    cltc_brp_max;
    uint32_t    cltc_brp_inc;
    uint32_t    cltc_clock_freq; /* prescaler input clock, in hz */
    uint32_t    cltc_linkmode_caps; /* link mode, see below */
};
#define CANGLINKTIMECAP 0 /* get struct can_link_timecaps */

/* [gs]et timing parameters */
struct can_link_timings {
    uint32_t    clt_brp;    /* prescaler value */
    uint32_t    clt_prop;   /* Propagation segment in tq */
    uint32_t    clt_ps1;    /* Phase segment 1 in tq */
    uint32_t    clt_ps2;    /* Phase segment 2 in tq */
    uint32_t    clt_sjw;    /* Synchronisation jump width in tq */
};
#define CANGLINKTIMINGS 1 /* get struct can_link_timings */
#define CANSLINKTIMINGS 2 /* set struct can_link_timings */

/* link-level modes */
#define CAN_LINKMODE_LOOPBACK       0x01    /* Loopback mode */
#define CAN_LINKMODE_LISTENONLY     0x02    /* Listen-only mode */
#define CAN_LINKMODE_3SAMPLES       0x04    /* Triple sampling mode */
#define CAN_LINKMODE_ONE_SHOT   0x08    /* One-shot mode */
#define CAN_LINKMODE_BUS_ERR_REP    0x10    /* Bus-error reporting */
#define CAN_LINKMODE_FD     0x20    /* can(4) FD mode */
#define CAN_LINKMODE_PRESUME_ACK    0x40    /* Ignore missing can(4) ACKs */
#define CAN_LINKMODE_FD_NON_ISO     0x80    /* can(4) FD, non-ISO mode */

#if 0
#define CAN_IFFBITS \ /* XXX: incomplete, ... work in progres. */
    "\020\1LOOPBACK\2LISTENONLY\3TRIPLESAMPLE\4PRESUMEACK"
#endif

#define CANGLINKMODE    3 /* (uint32_t) get bits */
#define CANSLINKMODE    4 /* (uint32_t) set bits */
#define CANCLINKMODE    5 /* (uint32_t) clear bits */

/* restart device(9) */
#define CANSRESTART     6

/* link-level states */
#define CAN_STATE_ERR_ACTIVE    0x00    /* RX/TX error count < 96 */
#define CAN_STATE_ERR_WARN      0x01    /* RX/TX error count < 128 */
#define CAN_STATE_ERR_PSV       0x02    /* RX/TX error count < 256 */
#define CAN_STATE_BUS_OFF       0x04    /* RX/TX error count >= 256 */
#define CAN_STATE_DETACHED      0x08    /* device(9) is stopped */
#define CAN_STATE_SUSPENDED     0x10    /* device(9) is sleeping */

#ifdef _KERNEL
#include <sys/ctype.h>
#include <sys/callout.h>
#include <sys/queue.h>
#include <sys/time.h>

/*
 * Common structure for can(4) interface drivers maps to if_l2com.
 *
 * XXX IFT_OTHER schould replaced by IFT_CAN maps to  IFT_PVC, see
 * the implementation of if_[dr]egister_com_alloc in net/if.c for
 * futher details.
 */
struct can_ifsoftc {
    struct ifnet    *csc_ifp;   /* our ifnet(9) interface */
    struct can_link_timecaps    csc_timecaps; /* timing capabilities */
    struct can_link_timings csc_timings; /* operating timing values */
    uint32_t    csc_linkmodes;
    uint32_t    csc_flags;
    struct callout  csc_timo;   /* callout for error control */
    struct mtx  csc_mtx;
};
#define CSC_LOCK(csc)       mtx_lock(&(csc)->csc_mtx)
#define CSC_UNLOCK(csc)     mtx_unlock(&(csc)->csc_mtx)
#define CSC_LOCK_ASSERT(csc)    mtx_assert(&(csc)->csc_mtx, MA_OWNED)

/* common subr. */
int     can_bin2hex(struct can_frame *, u_char *);
int     can_hex2bin(struct can_frame *, u_char *);
int     can_id2hex(struct can_frame *, u_char *);
int     can_hex2id(struct can_frame *, u_char *);
void    can_mbuf_tag_clean(struct mbuf *);

/* interface-layer */
void    can_ifattach(struct ifnet *, const struct can_link_timecaps *,
    uint32_t);
void    can_ifdetach(struct ifnet *);
void    can_bpf_mtap(struct ifnet *, struct mbuf *);
void    can_ifinit_timings(struct can_ifsoftc *);
int     can_ioctl(struct ifnet *, u_long, caddr_t);
#endif /* _KERNEL */
#endif /* _NET_IF_CAN_H */
