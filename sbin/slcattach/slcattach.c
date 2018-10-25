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
#include <sys/stat.h>
#include <sys/ioctl.h>

#include <net/if.h>
#include <net/if_slcvar.h>

#include <err.h>
#include <fcntl.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sysexits.h>
#include <unistd.h>

static char	dev_name[MAXPATHLEN]; /* path name of cdev{} maps to if_slc(4) */
static int dev_fd = -1;

static char	tty_name[MAXPATHLEN]; /* path name of aquired line */
static int tty_fd = -1;

/* struct termios tty; 	 tty(4) configuration / state */

int
main(int argc, char *argv[])
{
	struct stat sb;
	sigset_t nsigset;

	if (argc != 3) {
		(void)fprintf(stderr, 
		"usage: %s [devname] [ttyname]\n",
			getprogname());
		exit(1);
	}

	/* open char-device maps to if_slc(4) */
	(void)strlcpy(dev_name, argv[1], sizeof(dev_name));
	
	if ((dev_fd = open(dev_name, O_RDONLY)) < 0)
		errx(EX_OSFILE, "cannot open(2) %s", dev_name);
		
	if (fstat(dev_fd, &sb) < 0)
		errx(EX_DATAERR, "cannot fstat(2) %s", dev_name);	
		
	if ((sb.st_mode & S_IFMT) != S_IFCHR)
		errx(EX_DATAERR, "%s is not a character device", dev_name);

	(void)memset(&sb, 0, sizeof(sb));

	/* open(2) and attach tty(4) line */	
	(void)strlcpy(tty_name, argv[2], sizeof(tty_name));
	
	if ((tty_fd = open(tty_name, O_RDONLY | O_NONBLOCK)) < 0)
		errx(EX_OSFILE, "cannot open(2) %s", tty_name);
		
	if (fstat(tty_fd, &sb) < 0)
		errx(EX_DATAERR, "cannot fstat(2) %s", tty_name);	
		
	if ((sb.st_mode & S_IFMT) != S_IFCHR)
		errx(EX_DATAERR, "%s is not a character device", tty_name);

	if (ioctl(dev_fd, TIOCSETD, &tty_fd) != 0)
		errx(EX_UNAVAILABLE, "cannot attach %s to %s",
			dev_name, tty_name);
	
	(void)close(tty_fd);
		
	/* detach */
	if (daemon(0, 0) != 0)
		err(EX_OSERR, "couldn't detach");
	
	sigemptyset(&nsigset);
	for (;;)
		sigsuspend(&nsigset);
		
	/* NOTREACHED */	
}
