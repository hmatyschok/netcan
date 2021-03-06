/*  $NetBSD: can.h,v 1.3 2017/05/30 13:30:51 bouyer Exp $   */

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
 * Copyright (c) 2018, 2021 Henning Matyschok, DARPA/AFRL
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
#ifndef _NETCAN_CAN_H
#define _NETCAN_CAN_H

/* protocols */
typedef enum canproto {
    CANPROTO_CAN,
    CANPROTO_RAW,       /* RAW socket(9)s */
    CANPROTO_BCM,       /* BCM socket(9)s */
    CANPROTO_NPROTO,    /* sentinel */
} canproto_t;

/*
 * Socket address, CAN style
 */
struct sockaddr_can {
    u_int8_t    scan_len;
    sa_family_t scan_family;
    int         scan_ifindex;
    union {
        /* CAN Id filter */
        struct can_filter cf;
        /* transport protocol class address information (e.g. ISOTP) */
        struct can_tp tp;
        /* reserved for future CAN protocols address information */
    } scan_addr;
};

/*
 * Options for use with [gs]etsockopt(2) for raw sockets.
 *
 * First word of comment is data type; bool is stored in int.
 */
#define SOL_CAN_RAW CANPROTO_RAW

/*
 * Subset over CANPROTO_RAW familiy of options.
 *
 * XXX
 *  Work in progres..
 */
typedef enum canproto_raw_opt {
    CANPROTO_RAW_OPT_FILTER =           1,  /* struct can_filter: set filter */
    CANPROTO_RAW_OPT_LOOPBACK =         4,  /* loopback, default setup */
    CANPROTO_RAW_RECV_OWN_MSGS,             /* XXX */
    CANPROTO_RAW_OPT_MAX,
} canproto_raw_opt_t;

#ifdef _KERNEL

#define satoscan(sa)    ((struct sockaddr_can *)(sa))
#define scantosa(scan)  ((struct sockaddr *)(scan))

#endif /* _KERNEL */
#endif /* _NETCAN_CAN_H */
