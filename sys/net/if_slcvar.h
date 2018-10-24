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

#include <sys/ioccom.h>
#include <net/if_can.h>

/* 
 * Commands for ioctl(2) requests.
 * 
 * Example:
 *
 *  (void)memset(&ifd, 0, sizeof(ifd));
 *  (void)strlcpy(ifd.ifd_name, "slc0", strlen("slc0"));
 * 
 *  if ((slc_fd = socket(AF_LOCAL, SOCK_DGRAM, 0) < 0) 
 *      fatal(EX_DATAERR, "Can't open %s", ifd.ifd_name);
 * 
 *  ifd.ifd_cmd = SLCGTTY;
 *  ifd.ifd_len = sizeof(dev_t);
 *  ifd.ifd_data = &tty_dev;
 * 
 *  if (ioctl(slc_fd, SIOCGDRVSPEC, &ifd) != 0)
 *      fatal(EX_UNAVAILABLE, "Can't attach %s @ %s", 
 *          tty_name, ifd.ifd_name);
 *
 *  (void)printf("%s attached @ %s\n", ifd.ifd_name, 
 *      devname(tty_dev, S_IFCHR));
 * 
 *  (void)close(slc_fd); 
 * 
 * ...  
 */
#define SLCSTTY 	_IOW('T', 0, int)
#define SLCGTTY 	_IOW('T', 1, dev_t)
#define SLCDTTY 	_IO('T', 2)

/*
 * Definitions for serial line CAN interface data structures.
 */

#define SLC_MTU 	33 /* includes ext. CAN frame size */

#ifdef _KERNEL
struct slc_softc {
	struct ifnet 	*slc_ifp;
	struct cdev 	*slc_dev;
	struct tty 	*slc_tp;		/* pointer to tty structure */
	struct mbuf 	*slc_inb;
	struct ifqueue	slc_outq;		/* queue of outgoing data */
	size_t		slc_outqlen;	/* number of bytes in outq */
	u_int 	slc_flags;
	TAILQ_ENTRY(slc_softc) slc_next;
	struct mtx 	slc_mtx;
};

/* internal flags */ 	/* XXX */
#define	SLC_DETACHED	0x00000000U
#define	SLC_ATTACHED	0x00000001U
#define	SLC_CONNECTED	0x00000002U
#define	SLC_ERROR 	0x00000004U
#define	SLC_DEBUG 	0x00000008U
#endif /* _KERNEL */
#endif /* _NET_IF_SLCVAR_H_ */
