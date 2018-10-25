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
	
	ifd.ifd_cmd = TIOCGETD;
	ifd.ifd_len = sizeof(tty_dev);
	ifd.ifd_data = &tty_dev;

	if (ioctl(s, SIOCGDRVSPEC, &ifd) < 0)
		return;
	
	if (tty_dev == NODEV)
		return;
		
	(void)printf("\tattached: %s\n", devname(tty_dev, S_IFCHR));
} 

static void
slc_stty(const char *val, int d, int s, const struct afswtch *afp)
{
	char dev_name[MAXPATHLEN];
	int slc_fd, tty_fd;

	(void)memset(dev_name, 0, sizeof(dev_name));
	(void)snprintf(dev_name, sizeof(dev_name), "%s%s", 
		_PATH_DEV, ifr.ifr_name);
	
	if ((slc_fd = open(dev_name, O_RDONLY)) < 0)
		err(1, "cannot open(2) %s", dev_name);	

	(void)memset(dev_name, 0, sizeof(dev_name));
	(void)snprintf(dev_name, sizeof(dev_name), "%s%s", 
		_PATH_DEV, val);
	
	if ((tty_fd = open(dev_name, O_RDONLY | O_NONBLOCK)) < 0)
		err(1, "TIOCSETD Can't open %s device", dev_name); 	
	
	if (isatty(tty_fd) == 0)
		err(1, "TIOCSETD %s not a terminal type device", dev_name);
	
	if (ioctl(slc_dev, TIOCSETD, &tty_fd) < 0)
		err(1, "TIOCSETD Can't attach %s to %s", 
			dev_name, ifr.ifr_name);
			
	(void)close(tty_fd);
	(void)close(slc_fd);
}

static void
slc_dtty(const char *val, int d, int s, const struct afswtch *afp)
{
	char dev_name[MAXPATHLEN];
	int slc_fd;

	(void)memset(dev_name, 0, sizeof(dev_name));
	(void)snprintf(dev_name, sizeof(dev_name), "%s%s", 
		_PATH_DEV, ifr.ifr_name);
	
	if ((slc_fd = open(dev_name, O_RDONLY)) < 0)
		err(1, "cannot open(2) %s", dev_name);	

	if (ioctl(slc_dev, TIOCNOTTY, &tty_fd) < 0)
		err(1, "TIOCSETD Can't attach %s to %s", 
			dev_name, ifr.ifr_name);

	(void)close(slc_fd);
}

static struct cmd slc_cmds[] = {
	DEF_CMD_ARG("stty",		slc_stty),
	DEF_CMD("dtty", 0,		slc_dtty),
};

static struct afswtch af_slc = {
	.af_name	= "af_slc",
	.af_af		= AF_UNSPEC,
	.af_other_status = slc_status,
};

static __constructor void
slc_ctor(void)
{
	int i;

	for (i = 0; i < nitems(slc_cmds);  i++)
		cmd_register(&slc_cmds[i]);
	af_register(&af_slc);
}
