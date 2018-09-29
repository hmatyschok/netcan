/*	$NetBSD: can_proto.c,v 1.2 2017/05/27 21:02:56 bouyer Exp $	*/

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
#include <sys/cdefs.h>

#include <sys/param.h>
#include <sys/socket.h>
#include <sys/protosw.h>
#include <sys/domain.h>
#include <sys/mbuf.h>

#include <net/if.h>
#include <net/route.h>

/*
 * CAN protocol family
 */

static struct pr_usrreqs nousrreqs;

#include <netcan/can.h>
#include <netcan/can_var.h>

FEATURE(can, "Controller Area Network protocol");

extern struct domain candomain;

struct protosw cansw[] = {
{
	.pr_type =		0,
	.pr_domain =		&candomain,
	.pr_protocol =		CANPROTO_CAN,
	.pr_init =		can_init,
	.pr_usrreqs =		&nousrreqs
},
{
	.pr_type = 		SOCK_RAW,
	.pr_domain = 		&candomain,
	.pr_protocol =		CANPROTO_RAW,
	.pr_init = 		rcan_init,
	.pr_flags = 		PR_ATOMIC|PR_ADDR,
	.pr_usrreqs = 		&rcan_usrreqs,
	.pr_ctloutput = 	&rcan_ctloutput,
}
};

struct domain candomain = {
	.dom_family = 		AF_CAN,
	.dom_name = 		"can",
#if 0
	.dom_init = 		can_dominit,
#endif
	.dom_protosw = 		cansw,
	.dom_protoswNPROTOSW = &cansw[nitems(cansw)],
#if 0
	.dom_ifattach =		can_domifattach,
	.dom_ifdetach =		can_domifdetach
#endif
};

DOMAIN_SET(can);

/*
 * Declarations for OIDs for sysctl(9) MIB on PF_CAN.
 */
SYSCTL_NODE(_net,      PF_CAN,		can,	CTLFLAG_RW, 0,
	"Control Area Network Family");
	
SYSCTL_NODE(_net_can, CANPROTO_RAW,	raw,	CTLFLAG_RW, 0,	"RAW");
