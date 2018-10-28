/*	$NetBSD: can_link.h,v 1.2 2017/05/27 21:02:56 bouyer Exp $	*/

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

#ifndef _NET_IF_CAN_H
#define _NET_IF_CAN_H

/*
 * CAN frame delimeter
 */
#define CAN_SOF_FLAG 	0x01
#define CAN_EOF_FLAG 	0x80

/*
 * CAN id structure
 * bits 0-28	: CAN identifier (11/29 bits, see bit 31)
 * bit2 29-31	: see below
 */

typedef uint32_t canid_t;
typedef uint32_t can_err_mask_t;

/* canid_t bits 29-31 descriptions */
#define CAN_EFF_FLAG 	0x80000000U	/* extended frame format */
#define CAN_RTR_FLAG 	0x40000000U	/* remote transmission request */
#define CAN_ERR_FLAG 	0x20000000U	/* error message frame */
#define CAN_FLAG_MASK 	0xe0000000U

/* valid bits in CAN ID for frame formats */
#define CAN_SFF_MASK 	0x000007ffU /* standard frame format (SFF) */
#define CAN_EFF_MASK 	0x1fffffffU /* extended frame format (EFF) */
#define CAN_ERR_MASK 	0x1fffffffU /* error frame format */

/* CAN SDU length and DLC definitions according to ISO 11898-1 */
#define CAN_MAX_DLC 	8
#define CAN_MAX_DLEN 	8

/* CAN header */
struct can_hdr {
	canid_t	ch_id; /* ID + EFF/RTR/ERR flags */
	uint8_t	ch_dlc; /* frame SDU length in byte (0 .. CAN_MAX_DLEN) */
	uint8_t	__pad;
	uint8_t	__res0;
	uint8_t __res1;
};
#define CAN_STD_FRM 	0x00000000U
#define CAN_
#define CAN_EXT_FRM 	CAN_EFF_FLAG
#define CAN_RTR_FRM 	CAN_RTR_FLAG
#define CAN_ERR_FRM 	CAN_ERR_FLAG

/* CAN frame */
struct can_frame {
	canid_t	can_id; /* ID + EFF/RTR/ERR flags */
	uint8_t	can_dlc; /* frame SDU length in byte (0 .. CAN_MAX_DLEN) */
	uint8_t	__pad;
	uint8_t	__res0;
	uint8_t __res1;
	uint8_t	can_data[CAN_MAX_DLEN] __aligned(8);
};

#define CAN_MTU         (sizeof(struct can_frame))

/* CAN-FD frame */
#define CANFD_MAX_DLEN 	64

struct canfd_frame {
	canid_t	can_id; /* ID + EFF/RTR/ERR flags */
	uint8_t	can_dlc; /* frame SDU length in byte (0 .. CAN_MAX_DLEN) */
	uint8_t	__pad;
	uint8_t	__res0;
	uint8_t __res1;
	uint8_t	can_data[CANFD_MAX_DLEN] __aligned(8);
};

#define CANFD_MTU         (sizeof(struct canfd_frame))

/* Serial line CAN */
#define SLC_CMD_LEN 	(sizeof(u_char))
#define SLC_SFF_ID_LEN 	(sizeof(u_char) * 3)
#define SLC_EFF_ID_LEN 	(sizeof(u_char) * 8)
#define SLC_DLC_LEN 	(sizeof(u_char)) 

#define SLC_HC_SFF_DATA 	't'
#define SLC_HC_SFF_RTR 	'r'
#define SLC_HC_EFF_DATA 	'T'
#define SLC_HC_EFF_RTR 	'R'

#define SLC_HC_CR 		'\r'
#define SLC_HC_BEL 		'\a'

#define SLC_HC_DLC_INF 	'0'
#define SLC_HC_DLC_SUP 	'9'

/*
 * CAN ID based filter
 * checks received can_id & can_filter.cf_mask against
 *   can_filter.cf_id & can_filter.cf_mask
 * valid flags for can_id:
 *     CAN_INV_FILTER: invert filter
 * valid flags for can_mask:
 *     CAN_ERR_FLAG: filter for error message frames
 */
struct can_filter {
	canid_t cf_id;
	canid_t cf_mask;
};

#define CAN_INV_FILTER 	0x20000000U

/* transport protocol class address information (e.g. ISOTP) */
struct can_tp { 
	canid_t ct_rx_id; 
	canid_t ct_tx_id; 
};

/*
 * CAN bus link-layer related commands, from the SIOCSDRVSPEC
 */

/* get timing capabilities from HW */
struct can_link_timecaps {
	uint32_t cltc_prop_min; /* prop seg, in tq */
	uint32_t cltc_prop_max;
	uint32_t cltc_ps1_min; /* phase1 seg, in tq */
	uint32_t cltc_ps1_max;
	uint32_t cltc_ps2_min; /* phase 2 seg, in tq */
	uint32_t cltc_ps2_max;
	uint32_t cltc_sjw_max;	/* Synchronisation Jump Width */
	uint32_t cltc_brp_min;	/* bit-rate prescaler */
	uint32_t cltc_brp_max;
	uint32_t cltc_brp_inc;
	uint32_t cltc_clock_freq; /* prescaler input clock, in hz */
	uint32_t cltc_linkmode_caps; /* link mode, see below */
};
#define CANGLINKTIMECAP	0 /* get struct can_link_timecaps */

/* get/set timing parameters */
struct can_link_timings {
	uint32_t clt_brp;	/* prescaler value */
	uint32_t clt_prop;	/* Propagation segment in tq */
	uint32_t clt_ps1;	/* Phase segment 1 in tq */
	uint32_t clt_ps2;	/* Phase segment 2 in tq */
	uint32_t clt_sjw;	/* Synchronisation jump width in tq */
};
#define CANGLINKTIMINGS	1 /* get struct can_link_timings */
#define CANSLINKTIMINGS	2 /* set struct can_link_timings */

/* link-level modes */
#define CAN_LINKMODE_LOOPBACK		0x01    /* Loopback mode */
#define CAN_LINKMODE_LISTENONLY		0x02    /* Listen-only mode */
#define CAN_LINKMODE_3SAMPLES		0x04    /* Triple sampling mode */
#define CAN_LINKMODE_PRESUME_ACK	0x08    /* Ignore missing CAN ACKs */
#define CAN_IFFBITS \
    "\020\1LOOPBACK\2LISTENONLY\3TRIPLESAMPLE\4PRESUMEACK"

#define CANGLINKMODE	3 /* (uint32_t) get bits */
#define CANSLINKMODE	4 /* (uint32_t) set bits */
#define CANCLINKMODE	5 /* (uint32_t) clear bits */

#ifdef _KERNEL
#include <sys/ctype.h>
#include <sys/callout.h>
#include <sys/queue.h>

/*
 * Common structure for CAN interface drivers maps to if_l2com.
 * 
 * XXX: IFT_OTHER schould replaced by 
 * XXX:
 * XXX:  IFT_CAN maps to IFT_PVC		= 0xf1
 * XXX:
 * XXX: from net/if_types.h see  
 * XXX: 
 * XXX:  if_[dr]egister_com_alloc(9)
 * XXX:
 * XXX: in net/if.c for futher details.
 */
struct can_ifsoftc {
	struct ifnet 	*csc_ifp; 	/* our ifnet(9) interface */
	struct can_link_timecaps 	csc_timecaps; /* timing capabilities */
	struct can_link_timings 	csc_timings; /* operating timing values */
	uint32_t 	csc_linkmodes;
	struct callout 	csc_timo; 	/* callout for error control */
	struct mtx 	csc_mtx;
};

#define IFT_CAN 	IFT_PVC

/* common subr. */
int 	can_bin2hex(struct can_frame *, u_char *);
int 	can_hex2bin(u_char *, struct can_frame *);
int 	can_id2hex(struct can_frame *, u_char *);
int 	can_hex2id(u_char *, struct can_frame *);
void 	can_mbuf_tag_clean(struct mbuf *);

/* interface-layer */
void 	can_ifattach(struct ifnet *);
void 	can_ifdetach(struct ifnet *);
void 	can_bpf_mtap(struct ifnet *, struct mbuf *);
void 	can_ifinit_timings(struct can_ifsoftc *);
#endif /* _KERNEL */
#endif /* _NET_IF_CAN_H */
