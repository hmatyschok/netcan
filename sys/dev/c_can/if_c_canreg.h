/*
 * Copyright (c) 2018, 2019 Henning Matyschok
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
 * C_CAN, 3. Programmer's modell, Fig. 5: C_CAN Register Summary
 */
 
#define C_CAN_CR		0x00		/* control register */
#define C_CAN_SR		0x02		/* status register */
#define C_CAN_ECR       0x04		/* error counter */
#define C_CAN_BTR		0x06		/* bit timing register */
#define C_CAN_IR		0x08		/* intrrupt register */
#define C_CAN_TR		0x0a		/* test register */
#define C_CAN_BRPER		0x0c		/* BRP extension register */	
	/* 0x0e	reserved */

/* interface regiser set 1 */
#define C_CAN_IF1_CMR		0x10	/* IF1 command request */
#define C_CAN_IF1_CMR_MASK		0x12	/* IF1 command mask */
#define C_CAN_IF1_MASK0		0x14		/* IF1 mask 1 */
#define C_CAN_IF1_MASK1		0x16		/* IF1 mask 2 */
#define C_CAN_IF1_AR0		0x18		/* IF1 arbitriation 1 */
#define C_CAN_IF1_AR1		0x1a		/* IF1 arbitriation 2 */
#define C_CAN_IF1_MCR		0x1c		/* IF1 message control */
#define C_CAN_IF1_DATA0		0x1e		/* IF1 data register 1 */
#define C_CAN_IF1_DATA1		0x20		/* IF1 data register 2 */
#define C_CAN_IF1_DATA2		0x22		/* IF1 data register 3 */
#define C_CAN_IF1_DATA3		0x24		/* IF1 data register 4 */
	/* 0x28 - 0xe3	reserved */

/* interface regiser set 2 */
#define C_CAN_IF2_CMR		0x40		/* see above */
#define C_CAN_IF2_CMR_MASK		0x42
#define C_CAN_IF2_MASK0		0x44
#define C_CAN_IF2_MASK1		0x46
#define C_CAN_IF2_AR0		0x48
#define C_CAN_IF2_AR1		0x4a
#define C_CAN_IF2_MCR		0x4c
#define C_CAN_IF2_DATA0		0x4e
#define C_CAN_IF2_DATA1		0x50
#define C_CAN_IF2_DATA2		0x52
#define C_CAN_IF2_DATA3		0x54
	/* 0x56 - 0x7e	reserved */
	
#define C_CAN_TX1_REQ		0x80		/* TX request 1 */
#define C_CAN_TX2_REQ       0x82		/* TX request 2 */
	/* 0x84 - 0x8e	reserved */

#define C_CAN_NEW_DAT0		0x90		/* new data 1 */
#define C_CAN_NEW_DAT1		0x92		/* new data 2 */
	/* 0x94 - 0x9e	reserved */
	
#define C_CAN_ISR_PEND0		0xa0		/* interrupt pending 1 */		
#define C_CAN_ISR_PEND1		0xa2		/* interrupt pending 1 */
	/* 0xa4 - 0xae	reserved */

#define C_CAN_MSG_VAL0		0xb0		/* message valid 1 */
#define C_CAN_MSG_VAL1		0xb2		/* message valid 2 */
	/* 0xb4 - 0xbe	reserved */

/*
 * ...
 */

struct c_can_softc {
	struct ifnet	*c_can_ifp;		/* generic ifnet(9) glue */
	device_t	c_can_dev;
	int		c_can_port;
	struct resource 	*c_can_irq;
	struct resource		*c_can_res;
	uint8_t		c_can_cdr;
	uint8_t		c_can_ocr;
	struct mtx	c_can_mtx;
	void	*c_can_intr;
};
#define	C_CAN_LOCK(c_can)		mtx_lock(&(c_can)->c_can_mtx)
#define	C_CAN_UNLOCK(c_can)		mtx_unlock(&(c_can)->c_can_mtx)
#define	C_CAN_LOCK_ASSERT(c_can)	mtx_assert(&(c_can)->c_can_mtx, MA_OWNED)

/* utility-macros */
#define	c_can_timercmp(tvp, uvp, val)	\
	(((uvp)->tv_sec - (tvp)->tv_sec) < (val))
