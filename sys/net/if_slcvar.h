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

#ifndef _NET_IF_SLCVAR_H_
#define _NET_IF_SLCVAR_H_

#include <net/if_can.h>

/* 
 * Commands for SIOC[GS]DRVSPEC ioctl(2) requests.
 * 
 * Example:
 *
 *  (void)memset(&ifd, 0, sizoef(ifd));
 *  (void)strlcpy(ifd.ifd_name, "slc0", strlen("slc0"));
 * 
 *  if ((slc_fd = socket(AF_LOCAL, SOCK_DGRAM, 0) < 0) 
 *      fatal(EX_DATAERR, "Can't open %s", ifd.ifd_name);
 * 
 *  (void)strlcpy(tty_name, "/dev/ttyU0", strlen("/dev/ttyU0"));
 * 
 *  if ((tty_fd = open(tty_name, O_RDONLY | O_NONBLOCK)) < 0)
 *      fatal(EX_DATAERR, "Can't open %s device", tty_name); 
 *
 *  ifd.ifd_cmd = SLCSTTY;
 *  ifd.ifd_len = sizeof(int);
 *  ifd.ifd_data = &tty_fd;
 * 
 *  if (ioctl(slc_fd, SIOCSDRVSPEC, &ifd) != 0)
 *      fatal(EX_UNAVAILABLE, "Can't attach %s @ %s", 
 *          tty_name, ifd.ifd_name);
 *
 *  (void)close(tty_fd);
 *  (void)close(slc_fd);   
 */
#define SLCSTTY 	0
#define SLCGTTY 	1

/*
 * Definitions for serial line CAN interface data structures.
 */

#define SLC_MTU 	33 /* includes ext. CAN frame size */
 
#define SLC_CMD_LEN 	1
#define SLC_SFF_ID_LEN 	3
#define SLC_EFF_ID_LEN 	8
#define SLC_DLC_LEN 	1 
 
#define SLC_HC_SFF_DATA 	't'
#define SLC_HC_SFF_RTR 	'r'
#define SLC_HC_EFF_DATA 	'T'
#define SLC_HC_EFF_RTR 	'R'

#define SLC_HC_CR 		'\r'
#define SLC_HC_BEL 		'\a'

#define SLC_HC_DLC_INF 	'0'
#define SLC_HC_DLC_SUP 	'9'

#ifdef _KERNEL
struct slc_softc {
	struct ifnet 	*slc_ifp;
	struct tty 	*slc_tp;		/* pointer to tty structure */
	struct mbuf 	*slc_inb;
	struct ifqueue	slc_outq;		/* queue of outgoing data */
	size_t		slc_outqlen;	/* number of bytes in outq */
	u_int 	slc_flags;
	TAILQ_ENTRY(slc_softc) slc_next;
	struct mtx 	slc_mtx;
};

#define	SLC2IFP(slc)	((slc)->slc_csc.csc_ifp)
#define	SLC2DEV(slc)	((slc)->slc_csc.csc_dev)

/* internal flags */ 	/* XXX */
#define	SLC_DETACHED	0x00000000U
#define	SLC_ATTACHED	0x00000001U
#define	SLC_CONNECTED	0x00000002U
#define	SLC_ERROR 	0x00000004U
#define	SLC_DEBUG 	0x00000008U
#endif /* _KERNEL */
#endif /* _NET_IF_SLCVAR_H_ */
