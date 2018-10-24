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

#include <sys/param.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/sockio.h>
#include <sys/stat.h>

#include <err.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <net/if_slcvar.h>

#include "ifconfig.h"

static void
slc_status(int s)
{
	struct ifdrv ifd;
	dev_t tty_dev;	
	
	tty_dev = NODEV;
	
	(void)memset(&ifd, 0, sizeof(ifd));
	(void)strlcpy(ifd.ifd_name, ifr.ifr_name, sizeof(ifd.ifd_name));
	
	ifd.ifd_cmd = IFSLCGTTY;
	ifd.ifd_len = sizeof(tty_dev);
	ifd.ifd_data = &tty_dev;

	if (ioctl(s, SIOCGDRVSPEC, &ifd) < 0)
		return;
	
	if (tty_dev == NODEV)
		return;
		
	(void)printf("\attached @ %s\n", devname(tty_dev, S_IFCHR));
} 

static struct afswtch af_slc = {
	.af_name	= "af_slc",
	.af_af		= AF_UNSPEC,
	.af_other_status = slc_status,
};

static __constructor void
slc_ctor(void)
{
	
	af_register(&af_slc);
}
