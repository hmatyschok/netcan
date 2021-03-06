/*
 * Copyright (c) 2018, 2021 Henning Matyschok, DARPA/AFRL
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

#ifndef _NET_IF_SLCVAR_H_
#define _NET_IF_SLCVAR_H_

#ifndef _KERNEL
#include <sys/types.h>
#endif
#include <sys/ioccom.h>
#include <net/if_can.h>

/*
 * Definitions for serial line can(4) interface data structures.
 */

#define SLC_MTU     33 /* XXX ??? */

#ifdef _KERNEL
struct slc_softc {
    struct ifnet    *slc_ifp;
    struct cdev *slc_dev;
    struct tty  *slc_tp;        /* pointer against tty{} structure */
    struct mbuf *slc_ifbuf;
    struct ifqueue  slc_outq;       /* tx-queue */
    size_t      slc_outqlen;
    uint32_t    slc_flags;
    TAILQ_ENTRY(slc_softc) slc_next;
    struct mtx  slc_mtx;
};
#define SLC_ERROR       0x0001

#endif /* _KERNEL */
#endif /* _NET_IF_SLCVAR_H_ */
