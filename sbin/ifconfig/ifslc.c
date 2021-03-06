/*
 * Copyright (c) 2018, 2019, 2021 Henning Matyschok, DARPA/AFRL
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
#include <sysexits.h>
#include <termios.h>
#include <unistd.h>

#include <net/if.h>
#include <net/if_slcvar.h>

#include "ifconfig.h"

#ifndef _PATH_DEV
#define _PATH_DEV   "/dev/"
#endif

static void
slc_status(int s)
{
    char dev[MAXPATHLEN];
    int slc_fd;
    dev_t tty_dev;

    (void)memset(dev, 0, sizeof(dev));
    (void)snprintf(dev, sizeof(dev), "%s%s",
        _PATH_DEV, ifr.ifr_name);

    if ((slc_fd = open(dev, O_RDONLY)) < 0)
        return;

    tty_dev = NODEV;

    if (ioctl(slc_fd, TIOCGETD, &tty_dev) < 0)
        goto out;

    if (tty_dev == NODEV)
        goto out;

    (void)printf("\tattached: %s\n", devname(tty_dev, S_IFCHR));
out:
    (void)close(slc_fd);
}

static void
slc_stty(const char *val, int d, int s, const struct afswtch *afp)
{
    char dev[MAXPATHLEN];
    int slc_fd, tty_fd;
    struct termios tty;

    (void)memset(dev, 0, sizeof(dev));
    (void)snprintf(dev, sizeof(dev), "%s%s",
        _PATH_DEV, ifr.ifr_name);

    if ((slc_fd = open(dev, O_RDONLY)) < 0)
        err(EX_SOFTWARE, "cannot open(2) %s", dev);

    (void)memset(dev, 0, sizeof(dev));
    (void)snprintf(dev, sizeof(dev), "%s%s", _PATH_DEV, val);

    if ((tty_fd = open(dev, O_RDONLY | O_NONBLOCK)) < 0)
        err(EX_SOFTWARE, "TIOCSETD Can't open %s device", dev);

    if (isatty(tty_fd) == 0)
        err(EX_SOFTWARE, "TIOCSETD %s not a tty(4) device", dev);

    if (tcgetattr(tty_fd, &tty) < 0)
        err(EX_SOFTWARE, "TIOCSETD Can't tcgetattr(3) from %s", dev);

    tty.c_cflag = CREAD | CS8;
    tty.c_iflag = 0;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(tty_fd, TCSANOW, &tty) < 0)
        err(EX_SOFTWARE, "TIOCSETD Can't tcsetattr(3) to %s", dev);

    if (ioctl(slc_fd, TIOCSETD, &tty_fd) < 0)
        err(EX_SOFTWARE, "TIOCSETD Can't attach %s to %s",
            dev, ifr.ifr_name);

    (void)close(tty_fd);
    (void)close(slc_fd);
}

static void
slc_dtty(const char *val, int d, int s, const struct afswtch *afp)
{
    char dev[MAXPATHLEN];
    int slc_fd;

    (void)memset(dev, 0, sizeof(dev));
    (void)snprintf(dev, sizeof(dev), "%s%s",
        _PATH_DEV, ifr.ifr_name);

    if ((slc_fd = open(dev, O_RDONLY)) < 0)
        err(EX_SOFTWARE, "cannot open(2) %s", dev);

    if (ioctl(slc_fd, TIOCNOTTY) < 0)
        err(EX_SOFTWARE, "TIOCNOTTY Can't detach");

    (void)close(slc_fd);
}

static struct cmd slc_cmds[] = {
    DEF_CMD_ARG("stty",     slc_stty),
    DEF_CMD("dtty", 0,      slc_dtty),
};

static struct afswtch af_slc = {
    .af_name    = "af_slc",
    .af_af      = AF_UNSPEC,
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
