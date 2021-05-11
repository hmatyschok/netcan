/*	$NetBSD: if_sl.c,v 1.128 2017/04/13 00:47:33 maya Exp $	*/

/*
 * Copyright (c) 1987, 1989, 1992, 1993
 *	The Regents of the University of California.  All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the University nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE REGENTS AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE REGENTS OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 *	@(#)if_sl.c	8.9 (Berkeley) 1/9/95
 */
 /*
 * Copyright (c) 2019 Henning Matyschok
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
/*
 * Serial Line CAN interface. The implementation is derived from the
 * implementation from Serial Line Interface by:
 *
 * Rick Adams
 * Center for Seismic Studies
 * 1300 N 17th Street, Suite 1450
 * Arlington, Virginia 22209
 * (703)276-7900
 * rick@seismo.ARPA
 * seismo!rick
 *
 * Pounded on heavily by Chris Torek (chris@mimsy.umd.edu, umcp-cs!chris).
 * N.B.: this belongs in netinet, not net, the way it stands now.
 * Should have a link-layer type designation, but wouldn't be
 * backwards-compatible.
 *
 * Converted to 4.3BSD Beta by Chris Torek.
 * Other changes made at Berkeley, based in part on code by Kirk Smith.
 * W. Jolitz added slip abort.
 *
 * Hacked almost beyond recognition by Van Jacobson (van@helios.ee.lbl.gov).
 * Added priority queuing for "interactive" traffic; hooks for TCP
 * header compression; ICMP filtering (at 2400 baud, some cretin
 * pinging you can use up all your bandwidth).  Made low clist behavior
 * more robust and slightly less likely to hang serial line.
 * Sped up a bunch of things.
 */

#include <sys/cdefs.h>
__KERNEL_RCSID(0, "$NetBSD: if_sl.c,v 1.128 2017/04/13 00:47:33 maya Exp $");

#ifdef _KERNEL_OPT
#include "opt_can.h"
#endif

#include <sys/param.h>
#include <sys/proc.h>
#include <sys/malloc.h>
#include <sys/mbuf.h>
#include <sys/buf.h>
#include <sys/dkstat.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/file.h>
#include <sys/conf.h>
#include <sys/tty.h>
#include <sys/kernel.h>
#include <sys/socketvar.h>

#include <sys/systm.h>
#include <sys/kauth.h>

#include <sys/cpu.h>
#include <sys/intr.h>
#include <sys/device.h>
#include <sys/module.h>

#include <net/if.h>
#include <net/if_types.h>
#include <net/netisr.h>
#include <net/route.h>

#include <netcan/can.h>

#include <net/if_slcvar.h>

#include <sys/time.h>
#include <net/bpf.h>

#include "ioconf.h"

#define	SLC_MTU		33

#define	SLCAN_HIWAT	roundup(50, TTROUND)
#ifndef __NetBSD__					/* XXX - cgd */
#define	CLISTRESERVE	1024	/* Can't let clists get too low */
#endif	/* !__NetBSD__ */

/*
 * XXX Work in progress..
 */
static int		slc_clone_create(struct if_clone *, int);
static int		slc_clone_destroy(struct ifnet *);

static LIST_HEAD(, slc_softc) slc_softc_list;

struct if_clone slc_cloner =
    IF_CLONE_INITIALIZER("slc", slc_clone_create, slc_clone_destroy);

static int	slc_bintohex(struct can_frame *, u_char *);
static int	slc_hextobin(struct can_frame *, u_char *);
static int	slc_idtohex(struct can_frame *, u_char *);
static int	slc_hextoid(struct can_frame *, u_char *);

static struct mbuf *	slc_btom(struct slc_softc *);
static struct tty *	slc_encap(struct slc_softc *, struct mbuf **);
static void	slc_rxeof(struct slc_softc *);
static void	slc_txeof(struct slc_softc *);

static void	slc_intr(void *);

static int	slc_create(struct slc_softc *);

static int	slc_close(struct tty *, int);
static int	slc_input(int, struct tty *);
static int	slc_ifioctl(struct ifnet *, u_long, void *);
static int	slc_open(dev_t, struct tty *);
static void	slc_ifstart(struct ifnet *);
static int	slc_start(struct tty *);
static int	slc_ioctl(struct tty *, u_long, void *, int, struct lwp *);

static struct linesw slc_disc = {
	.l_name = "slc",
	.l_open = slc_open,
	.l_close = slc_close,
	.l_read = ttyerrio,
	.l_write = ttyerrio,
	.l_ioctl = slc_ioctl,
	.l_rint = slc_input,
	.l_start = slc_start,
	.l_modem = nullmodem,
	.l_poll = ttyerrpoll
};

void
slc_attach(int n __unused)
{

	/*
	 * Nothing to do here, initialization is handled by the
	 * module initialization code in slc_init() below.
	 */
}

static void
slc_init(void)
{

	if (ttyldisc_attach(&slc_disc) != 0)
		panic("%s", __func__);

	LIST_INIT(&slc_softc_list);
	if_clone_attach(&slc_cloner);
}

static int
slc_detach(void)
{
	int error;

	if (!LIST_EMPTY(&slc_softc_list))
		error = EBUSY;
	else
		error = 0;

	if (error == 0)
		error = ttyldisc_detach(&slc_disc);

	if (error == 0)
		if_clone_detach(&slc_cloner);

	return (error);
}

static int
slc_clone_create(struct if_clone *ifc, int unit)
{
	struct slc_softc *sc;
	struct ifnet *ifp;

	sc = malloc(sizeof(struct slc_softc), M_DEVBUF, M_WAIT|M_ZERO);
	sc->slc_unit = unit;
	
	ifp = sc->slc_if = if_alloc(IFT_OTHER);
	
	if_initname(ifp, ifc->ifc_name, unit);
	
	ifp->if_softc = sc;
	ifp->if_flags = (IFF_POINTOPOINT | IFF_MULTICAST);
	
	ifp->if_ioctl = slc_ioctl;
	ifp->if_start = slc_ifstart;
	
	can_ifattach(ifp);
	
	ifp->if_mtu = SLC_MTU;
	
	LIST_INSERT_HEAD(&slc_softc_list, sc, slc_iflist);
	
	return (0);
}

static int
slc_clone_destroy(struct ifnet *ifp)
{
	struct slc_softc *sc;
	int error;

	sc = (struct slc_softc *)ifp->if_softc;

	if (sc->slc_tp == NULL) {
		LIST_REMOVE(sc, slc_iflist);

		can_ifdetach(ifp);
		if_free(ifp);

		free(sc, M_DEVBUF);
		error = 0;
	} else
		error = EBUSY;	/* Not removing it */

	return (error);
}

/*
 * Line specific open routine.
 *
 * Attach the given tty(4) to the first available slc(4) unit.
 */
/* ARGSUSED */
static int
slc_open(dev_t dev, struct tty *tp)
{
	struct slc_softc *sc;
	int error;

	if (tp->t_linesw == &slc_disc) {
		error = 0;
		goto out;
	}
	error = ENXIO;
	
	LIST_FOREACH(sc, &slc_softc_list, slc_iflist) {
		if (sc->slc_tp == NULL) {
			sc->slc_intr = softint_establish(SOFTINT_NET,
			    slc_intr, sc);
			if (sc->slc_intr == NULL) {
				error = ENOMEM;
				break;
			}

			tp->t_sc = (void *)sc;
			sc->slc_tp = tp;
			sc->slc_if->if_baudrate = tp->t_ospeed;
			mutex_spin_enter(&tty_lock);
			tp->t_state |= (TS_ISOPEN | TS_XCLUDE);
			ttyflush(tp, FREAD | FWRITE);
			/*
			 * make sure tty output queue is large enough
			 * to hold a full-sized packet (including frame
			 * end, and a possible timestamp)
			 */
			if (tp->t_outq.c_cn < MHLEN) {
				sc->slc_oldbufsize = tp->t_outq.c_cn;
				sc->slc_oldbufquot = tp->t_outq.c_cq != 0;

				clfree(&tp->t_outq);
				mutex_spin_exit(&tty_lock);
				error = clalloc(&tp->t_outq, MHLEN, 0);
				if (error != 0) {
					softint_disestablish(sc->slc_intr);
					/*
					 * clalloc() might return -1 which
					 * is no good, so we need to return
					 * something else.
					 */
					error = ENOMEM; /* XXX ?! */
					break;
				}
			} else {
				sc->slc_oldbufsize = sc->slc_oldbufquot = 0;
				mutex_spin_exit(&tty_lock);
			}
			error = 0;
			break;
		}
	}
out:		
	return (error);
}

/*
 * Line specific close routine.
 *
 * Detach the tty(4) from the slc(4) unit.
 */
static int
slc_close(struct tty *tp, int flag)
{
	struct slc_softc *sc;
	int s;

	ttywflush(tp);
	sc = (struct slc_softc *)tp->t_sc;

	if (sc != NULL) {
		softint_disestablish(sc->slc_intr);
		s = splnet();
		if_down(sc->slc_if);
		IF_PURGE(&sc->slc_outq);
		splx(s);

		s = spltty();
		ttyldisc_release(tp->t_linesw);
		tp->t_linesw = ttyldisc_default();
		tp->t_state = 0;

		sc->slc_tp = NULL;
		tp->t_sc = NULL;

		if (sc->slc_ifbuf != NULL) {
			m_freem(sc->slc_ifbuf);
			sc->slc_ifbuf = NULL;
		}
		IF_PURGE(&sc->slc_inq);

		/*
		 * If necessary, install a new outq buffer of the
		 * appropriate size.
		 */
		if (sc->slc_oldbufsize != 0) {
			clfree(&tp->t_outq);
			clalloc(&tp->t_outq, sc->slc_oldbufsize,
			    sc->slc_oldbufquot);
		}
		splx(s);
	}

	return (0);
}

/*
 * Line specific (tty) ioctl routine.
 * Provide a way to get the sl unit number.
 */
/* ARGSUSED */
static int
slc_ioctl(struct tty *tp, u_long cmd, void *data, int flag,
    struct lwp *l)
{
	struct slc_softc *sc;
	int error;

	sc = (struct slc_softc *)tp->t_sc;
	error = 0;

	switch (cmd) {
	case SLIOCGUNIT:
		*(int *)data = sc->slc_unit;	/* XXX */
		break;
	default:
		error = EPASSTHROUGH;
		break;
	}
	return (error);
}
/*
 * Start output on interface.  Get another datagram
 * to send from the interface queue and map it to
 * the interface before starting output.
 */
 
static void
slc_ifstart(struct ifnet *ifp)
{
	struct slc_softc *sc;
	struct mbuf *m;
	struct tty *tp

	sc = ifp->if_softc;

	for (;;) {
		IF_DEQUEUE(&ifp->if_snd, m);
		if (m == NULL)
			break;

		if ((m->m_flags & M_PKTHDR) == 0)
			panic("%s: no header mbuf(9)", __func__);
		
		can_bpf_mtap(ifp, m, 0);

		/*
		 * Encode can(4) frame in its ASCII representation,
		 * see net/if_slcvar.h for further details.
		 */
		if ((tp = slc_encap(slc, &m)) != NULL) {
			s = spltty();
			if ((sc->slc_outqlen = sc->slc_tp->t_outq.c_cc) == 0)
				slc_start(sc->slc_tp);
			splx(s);
		} else
			ifp->if_oerrors++;
	}
}

static struct tty *
slc_encap(struct slc_softc *sc, struct mbuf **mp)
{
	struct tty *tp;
	struct mbuf *m;
	struct can_frame *cf;
	u_char buf[MHLEN], *bp;
	struct bintime bt;
	int s, len;

	if ((tp = sc->slc_tp) == NULL)
		goto bad;
		
	if (((tp->t_state & TS_CARR_ON) == 0) &&
	    ((sc->slc_tp->t_cflag & CLOCAL) == 0)) {
		(void)printf("%s: no carrier and not local\n", 
			sc->slc_if->if_xname);
		goto bad1;
	}	
	
	/* get a writable copy, if any */
	if (m_makewritable(mp, 0, M_COPYALL, M_DONTWAIT) != 0)
		goto bad1;

	m = *mp;

	cf = mtod(m, struct can_frame *);

	(void)memset((bp = buf), 0, MHLEN);

	/* determine can(4) frame type */
	if ((cf->can_id & CAN_RTR_FLAG) != 0)
		*bp = SLC_HC_SFF_RTR;
	else
		*bp = SLC_HC_SFF_DATA;

	if ((cf->can_id & CAN_EFF_FLAG) != 0)
		*bp = toupper(*bp);
		
	bp += SLC_CMD_LEN;	

	/* encode ID */
	if ((len = can_idtohex(cf, bp)) < 0)
		goto bad1;
		
	bp += len;

	/* encode DLC */
	*bp = cf->can_dlc + '0';
	bp += SLC_DLC_LEN;

	/* encode data, if any */
	if ((cf->can_id & CAN_RTR_FLAG) == 0) {
		if ((len = slc_bintohex(cf, bp)) < 0)
			goto bad1;

		bp += len;
	}

	/* finalize */
	*bp = SLC_HC_CR;
	bp += SLC_HC_LEN;

	/* re-initialize mbuf(9) and copy back */
	len = bp - buf; 

	m->m_len = m->m_pkthdr.len = len;
	m->m_data = m->m_pktdat;

	(void)memcpy(mtod(m, u_char *), buf, len);

	/* enqueue, if any */
	s = spltty();
	IF_ENQUEUE(&sc->slc_outq, m);
	sc->slc_outqlen += len;
	splx(s);
out:
	return (tp);
bad1:
	tp = NULL;
bad:
	m_freem(*mp);
	*mp = NULL;
	goto out;
}
		
static int
slc_start(struct tty *tp)
{
	struct slc_softc *sc;

	/*
	 * If there is more in the output queue, just send it now.
	 * We are being called in lieu of ttstart and must do what
	 * it would.
	 */
	if (tp->t_outq.c_cc != 0) {
		(*tp->t_oproc)(tp);
		if (tp->t_outq.c_cc > SLCAN_HIWAT)
			goto out;
	}

	if ((sc = (struct slc_softc *)tp->t_sc) != NULL)
		softint_schedule(sc->slc_intr);
out:
	return (0);
}

/*
 * tty interface receiver interrupt.
 */
static int
slc_input(int c, struct tty *tp)
{
	struct slc_softc *sc;
	struct mbuf *m;
	int error;

	tk_nin++;

	if ((sc = (struct slc_softc *)tp->t_sc) == NULL) {
		error = 0;
		goto out;
	}
	
	if (((c & TTY_ERRORMASK) != 0) ||
		(((tp->t_state & TS_CARR_ON) == 0) &&
	    ((tp->t_cflag & CLOCAL) == 0))) {
		sc->slc_flags |= SC_ERROR;
		error = 0;
		goto out;
	}
	c &= TTY_CHARMASK;

	sc->slc_if->if_ibytes++;

	/* allocate mbuf(9) and initialize */
	if ((m = sc->slc_ifbuf) == NULL) {
		if ((m = m_gethdr(M_NOWAIT, MT_DATA)) == NULL) {
			error = ENOBUFS;
			goto out;
		}
		m->m_len = m->m_pkthdr.len = 0;
		sc->slc_ifbuf = m;
	}

	if (m->m_len < SLC_MTU) {
		if ((c == SLC_HC_BEL) || (c == SLC_HC_CR)) {
			if ((sc->slc_flags & SLC_ERROR) != 0) {
				sc->slc_flags &= ~SLC_ERROR;
				error = ECONNABORTED;
				goto bad;
			}
			m->m_data = m->m_pktdat;
			
			if ((m = slc_btom(sc)) != NULL) {
				IF_ENQUEUE(&sc->slc_inq, m);
				softint_schedule(sc->slc_intr);
			}
		} else {
			*mtod(m, u_char *) = c;

			m->m_data++;
			m->m_len++;
			m->m_pkthdr.len++;
		}
		error = 0;
	} else {
		error = EFBIG;
		goto bad;
	}
out:
	return (error);
bad:
	m->m_data = m->m_pktdat;
	m_freem(m);
	
	sc->slc_ifbuf = NULL;
	goto out;
}

static struct mbuf *
slc_btom(struct slc_softc *sc)
{
	struct mbuf *m;
	u_char buf[MHLEN], *bp;
	struct can_frame *cf;
	int len;

	if ((m = sc->slc_ifbuf) == NULL)
		goto out;
		
	sc->slc_ifbuf = NULL;

	(void)memset((bp = buf), 0, MHLEN);
	cf = (struct can_frame *)bp;

	/* determine can(4) frame type */
	switch (*mtod(m, u_char *)) {
	case SLC_HC_SFF_RTR:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */
	case SLC_HC_SFF_DATA:
		break;
	case SLC_HC_EFF_RTR:
		cf->can_id |= CAN_RTR_FLAG;
					 	/* FALLTHROUGH */
	case SLC_HC_EFF_DATA:
		cf->can_id |= CAN_EFF_FLAG;
		break;
	default:
		goto bad;
	}
	m_adj(m, SLC_CMD_LEN);

	/* fetch id */
	if ((len = can_hextoid(cf, mtod(m, u_char *))) < 0)
		goto bad;

	m_adj(m, len);

	/* fetch dlc */
	cf->can_dlc = *mtod(m, u_char *);
	m_adj(m, SLC_DLC_LEN);

	if ((cf->can_dlc < SLC_HC_DLC_INF) ||
		(cf->can_dlc > SLC_HC_DLC_SUP))
		goto bad;

	cf->can_dlc -= SLC_HC_DLC_INF;

	/* fetch data, if any */
	if ((cf->can_id & CAN_RTR_FLAG) == 0) {
		if (can_hextobin(cf, mtod(m, u_char *)) < 0)
			goto bad;
	}

	len = sizeof(struct can_frame);

	/* reinitialize mbuf(9) and copy back */
	m->m_len = m->m_pkthdr.len = len;
	m->m_data = m->m_pktdat;

	(void)memcpy(mtod(m, u_char *), buf, len);

	m_set_rcvif(m, sc->slc_if);
out:
	return (m);
bad:
	m_freem(m);
	m = NULL;
	goto out;
}


static void
slc_intr(void *arg)
{
	struct slc_softc *sc;

	sc = (struct slc_softc *)arg;
	KASSERT(sc->slc_tp != NULL);

	mutex_enter(softnet_lock);
	
	slc_txeof(sc);
	
	slc_rxeof(sc);
	
	mutex_exit(softnet_lock);
}

/*
 * Input processing loop.
 */
static void
slc_rxeof(struct slc_softc *sc)
{
	struct mbuf *m;
	int s;

	for (;;) {
		s = spltty();
		IF_DEQUEUE(&sc->slc_inq, m);
		splx(s);
		if (m == NULL)
			break;

		can_input(sc->slc_if, m);
	}
}

/*
 * Output processing loop.
 */
static void
slc_txeof(struct slc_softc *sc) 
{
	struct tty *tp;
	struct mbuf *m;
	int s;

	tp = sc->slc_tp;
	
	for (;;) {

		/*
		 * Do not remove the packet from the queue if it
		 * doesn't look like it will fit into the current
		 * serial output queue.  With a packet full of
		 * escapes, this could be as bad as MTU*2.
		 */
		s = spltty();
		if (tp->t_outq.c_cn - tp->t_outq.c_cc <
		    2 * sc->slc_if->if_mtu) {
			splx(s);
			break;
		}

		/*
		 * Get a packet and send it to the interface.
		 */
		IF_DEQUEUE(&sc->slc_outq, m);
		splx(s);

		if (m == NULL)
			break;

		s = spltty();

		while (m != NULL) {	
			/*
			 * Put N characters at once
		     * into the tty output queue.
			 */
			if (b_to_q(mtod(m, u_char *), m->m_len, &tp->t_outq))
				break;
				
			sc->slc_if->if_obytes += m->m_len;

			m = m_free(m);
		}

		if (m != NULL) {
			ndflush(&tp->t_outq, MHLEN);
			m_freem(m);
		} else {
			sc->slc_if->if_opackets++;

			/*
			* We now have characters in the output queue,
			* kick the serial port.
			*/
			(*tp->t_oproc)(tp);
		}

		splx(s);
	}
}

/*
 * Process an ioctl request.
 */
static int
slc_ifioctl(struct ifnet *ifp, u_long cmd, void *data)
{
	struct ifreq *ifr;
	int s, error;
	struct slc_softc *sc;

	ifr = (struct ifreq *)data;
	s = splnet(); 
	error = 0;
	sc = ifp->if_softc;

	switch (cmd) {
	case SIOCINITIFADDR:
		error = EAFNOSUPPORT;
		break;
	case SIOCSIFMTU:
		if ((unsigned)ifr->ifr_mtu != SLC_MTU)
			error = EINVAL;
		break;
	case SIOCADDMULTI:
	case SIOCDELMULTI:
		error = EAFNOSUPPORT;
		break;
	default:
		error = ifioctl_common(ifp, cmd, data);
		break;
	}
	splx(s);
	return error;
}

/*
 * Subr. for encoding / decoding can(4) ID and SDU.
 */
 
static const char slc_hex_tbl[] = "0123456789ABCDEF";

static int
slc_bintohex(struct can_frame *cf, u_char *buf)
{
	int len, i;
	u_char *bp, *dp;
	u_char c;

	if ((bp = buf) == NULL || cf == NULL)
		return (-1);

	if ((len = cf->can_dlc) >= CAN_MAX_DLC)
		return (-1);

	for (dp = bp, i = 0; i < len; i++) {
		c = cf->data[i];

		*dp = ((c & 0xf0) >> 4);

		if (isalpha(*dp) && islower(*dp))
			*dp = toupper(*dp);

		*dp = slc_hex_tbl[*dp];

		dp += 1;

		*dp = (c & 0x0f); 

		if (isalpha(*dp) && islower(*dp))
			*dp = toupper(*dp);

		*dp = slc_hex_tbl[*dp];

		dp += 1;
	}

	len = dp - bp;

	return (len);
}

static int
slc_hextobin(struct can_frame *cf, u_char *buf)
{
	int len, i;
	u_char *bp;
	u_char c1, c0;

	if ((bp = buf) == NULL || cf == NULL)
		return (-1);

	if ((len = cf->can_dlc) >= CAN_MAX_DLC)
		return (-1);

	for (i = 0; i < len; i++) {
		c1 = *bp;
		bp += 1;

		if (isdigit(c1))
			c1 -= '0';
		else if (isalpha(c1))
			c1 -= (isupper(c1)) ? 'A' - 10 : 'a' - 10;
		else
			return (-1);

		c0 = *bp;
		bp += 1;

		if (isdigit(c0))
			c0 -= '0';
		else if (isalpha(c0))
			c0 -= (isupper(c0)) ? 'A' - 10 : 'a' - 10;
		else
			return (-1);

		cf->data[i] = ((c1 << 4) | c0);
	}
	return (0);
}

static int
slc_idtohex(struct can_frame *cf, u_char *buf)
{
	canid_t id;
	int len;
	u_char *bp, *ep;
	u_char c;

	if ((bp = buf) == NULL || cf == NULL)
		return (-1);

	if ((cf->can_id & CAN_EFF_FLAG) != 0) {
		id = (cf->can_id & CAN_EFF_MASK);
		len = SLC_EFF_ID_LEN;
	} else {
		id = (cf->can_id & CAN_SFF_MASK);
		len = SLC_SFF_ID_LEN;
	}

	for (ep = bp + len - 1; ep >= bp; ep--, id >>= 4) {
		c = (id & 0x0f);

		if (isalpha(c) && islower(c))
			c = toupper(c);

		*ep = slc_hex_tbl[c];
	}
	return (len);
}

static int
slc_hextoid(struct can_frame *cf, u_char *buf)
{
	int len;
	canid_t u, v;
	u_char *bp, *ep;
	u_char c;

	if ((bp = buf) == NULL || cf == NULL)
		return (-1);

	if ((cf->can_id & CAN_EFF_FLAG) != 0)
		len = SLC_EFF_ID_LEN;
	else 
		len = SLC_SFF_ID_LEN;

	for (u = v = 0, ep = bp + len - 1; bp <= ep; v <<= 4) {
		c = *bp;
		bp += 1;

		if (isdigit(c))
			c -= '0';
		else if (isalpha(c))
			c -= (isupper(c)) ? 'A' - 10 : 'a' - 10;
		else
			return (-1);

		v |= (c & 0x0f);
		u = v;
	}
	cf->can_id |= u;

	return (len);
}

/*
 * Module infrastructure
 */

#include "if_module.h"

IF_MODULE(MODULE_CLASS_DRIVER, slc, "slc");
