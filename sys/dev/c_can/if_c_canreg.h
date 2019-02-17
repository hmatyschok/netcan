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
#define C_CAN_IF1_CMMR		0x12	/* IF1 command mask */
#define C_CAN_IF1_MASK0		0x14		/* IF1 mask 1 */
#define C_CAN_IF1_MASK1		0x16		/* IF1 mask 2 */
#define C_CAN_IF1_ARB0		0x18		/* IF1 arbitriation 1 */
#define C_CAN_IF1_ARB1		0x1a		/* IF1 arbitriation 2 */
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
#define C_CAN_IF2_ID0		0x48
#define C_CAN_IF2_ID1		0x4a
#define C_CAN_IF2_MCR		0x4c
#define C_CAN_IF2_DATA0		0x4e
#define C_CAN_IF2_DATA1		0x50
#define C_CAN_IF2_DATA2		0x52
#define C_CAN_IF2_DATA3		0x54
	/* 0x56 - 0x7e	reserved */
	
#define C_CAN_TX1_RQST		0x80		/* TX request 1 */
#define C_CAN_TX2_RQST       0x82		/* TX request 2 */
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
 * C_CAN, 3.2.1 can(4) Control Register [CR] 
 */
#define C_CAN_CR_INIT	0x0001	/* initializiation (rw) */
#define C_CAN_CR_IE		0x0002		/* interrupt enable (rw) */
#define C_CAN_CR_SIE	0x0004	/* status change interrupt enable (rw) */
#define C_CAN_CR_EIE	0x0008		/* error interrupt enable (rw) */
#define C_CAN_CR_DAR	0x0020	/* disable automatic retransmission (rw) */	
#define C_CAN_CR_CCE	0x0040	/* configuration change enable (rw) */
#define C_CAN_CR_TEST	0x0080		/* test mode (rw) */
#define C_CAN_CR_INTR_MASK \
	(C_CAN_CR_IE|C_CAN_CR_SIE|C_CAN_CR_EIE)

/* 
 * C_CAN, 3.2.2 Status Register [SR] 
 */ 
#define C_CAN_SR_LEC_MASK	0x0007	/* lost error code mask (rw) */

#define C_CAN_SR_NO_ERR		0x00
#define C_CAN_SR_STUFF_ERR      0x01
#define C_CAN_SR_FORM_ERR       0x02
#define C_CAN_SR_ACK_ERR        0x03
#define C_CAN_SR_BIT1_ERR       0x04
#define C_CAN_SR_BIT0_ERR       0x05
#define C_CAN_SR_CRC_ERR        0x06
#define C_CAN_SR_UNUSED_ERR     0x07

#define C_CAN_SR_LEC(reg) \
	((reg) & C_CAN_SR_LEC_MASK)

#define IS_C_CAN_SR_NO_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_NO_ERR) 	
#define IS_C_CAN_SR_STUFF_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_STUFF_ERR)
#define IS_C_CAN_SR_FORM_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_FORM_ERR)
#define IS_C_CAN_SR_ACK_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_ACK_ERR)
#define IS_C_CAN_SR_BIT1_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_BIT1_ERR)
#define IS_C_CAN_SR_BIT0_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_BIT0_ERR)
#define IS_C_CAN_SR_CRC_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_CRC_ERR)
#define IS_C_CAN_SR_UNUSED_ERR(reg) \
	(C_CAN_SR_LEC(reg) == C_CAN_SR_UNUSED_ERR)

#define C_CAN_SR_TX_OK		0x0008	/* tx'd message successfully (rw) */
#define C_CAN_SR_RX_OK		0x0010	/* rx'd message successfully (rw) */
#define C_CAN_SR_PE		0x0020	/* error passive status (r) */
#define C_CAN_SR_EW		0x0040	/* error warning status (r) */
#define C_CAN_SR_BO		0x0080	/* bus-off status (r) */

/* 
 * C_CAN, 3.2.3 Error Counter [ECR] 
 */ 
#define C_CAN_ECR_TEC_MASK		0x00ff		/* tx error counter (r) */
#define C_CAN_ECR_REC_MASK		0xef00		/* rx error counter (r) */
#define C_CAN_ECR_RR		0x8000		/* rx error passive (r) */

/* 
 * C_CAN, 3.2.4 Bit Timing Register [BTR] 
 */ 
#define C_CAN_BTR_BRP_MASK		0x003f
#define C_CAN_BTR_SJW_MASK		0x00c0
#define C_CAN_BTR_TSEG1_MASK		0x0f00
#define C_CAN_BTR_TSEG2_MASK		0x7000

#define C_CAN_BTR_BRP(reg) \	/* baud rate prescalar (rw) */
	((reg) & C_CAN_BTR_BRP_MASK)
#define C_CAN_BTR_SJW(reg) \	/* sync jump width (rw) */
	(((reg) & C_CAN_BTR_SJW_MASK) >> 6)
#define C_CAN_BTR_TSEG1(reg) \	/* phase segment 1 (rw) */
	(((reg) & C_CAN_BTR_TSEG1_MASK) >> 8)
#define C_CAN_BTR_TSEG2(reg) \	/* phase segment 2 (rw) */
	(((reg) & C_CAN_BTR_TSEG2_MASK) >> 12)

/* 
 * C_CAN, 3.2.6 BRP Extension Register [BRPER] 
 */ 
#define C_CAN_BRPER_BRPE_MASK		0x000f

/*
 * ...
 */

#define C_CAN_IFX_RX		0 
#define C_CAN_IFX_TX		1 

#define C_CAN_IFX_CMR_MESG_NUM_MASK		0x002f	 

#define C_CAN_IFX_CMR_MESG_NUM(reg) \		/* index for message (r) */
	((reg) & C_CAN_IFX_CMR_MESG_NUM_MASK)

#define C_CAN_IFX_CMR_BUSY		0x8000		/* busy flag (r) */

/*
 * ...
 */
#define C_CAN_IFX_CMMR_DATA_B		0x01
#define C_CAN_IFX_CMMR_DATA_A		0x02 	
#define C_CAN_IFX_CMMR_TX_RQST		0x04
#define C_CAN_IFX_CMMR_NEW_DAT		C_CAN_IFX_CMMR_TX_RQST
#define C_CAN_IFX_CMMR_CLR_INT_PND		0x08	
#define C_CAN_IFX_CMMR_CONTROL		0x10
#define C_CAN_IFX_CMMR_ARB		0x20
#define C_CAN_IFX_CMMR_MASK		0x40
#define C_CAN_IFX_CMMR_WR_RD		0x80	/* write = 1 / read = 0 (rw) */

/* initialize tx buffer */
#define C_CAN_IFX_CMMR_TX_INIT \
	(C_CAN_IFX_CMMR_DATA_B|C_CAN_IFX_CMMR_DATA_A| \
	C_CAN_IFX_CMMR_TX_RQST|C_CAN_IFX_CMMR_CONTROL| \
	C_CAN_IFX_CMMR_ARB)

/* invalidate message object */
#define C_CAN_IFX_CMMR_MO_INVAL	\
	(C_CAN_IFX_CMMR_CONTROL|C_CAN_IFX_CMMR_ARB)

/* initialize rx buffer */
#define C_CAN_IFX_CMMR_RX_INIT \
	(C_CAN_IFX_CMMRL_MO_INVAL|C_CAN_IFX_CMMR_MASK)

/* clear interrupt, but keep new data */
#define C_CAN_IFX_CMMR_RX_LOW \
	(C_CAN_IFX_CMMR_DATA_B|C_CAN_IFX_CMMR_DATA_A| \
	C_CAN_IFX_CMMR_CLR_INT_PND|C_CAN_IFX_CMMR_RX_INIT)

/* clear interrupt bit and new data */
#define C_CAN_IFX_CMMR_RX_HIGH \
	(C_CAN_IFX_CMMR_RCV_LOW|C_CAN_IFX_CMMR_CLR_NEWDAT)

/*
 * ...
 */

#define C_CAN_IFX_MCR_DLC_MASK		0x000f	/* data length code (rw) */
#define C_CAN_IFX_MCR_EOB		0x0008		/* end of buffer (rw) */
#define C_CAN_IFX_MCR_TX_RQST		0x0010		/* tx request (rw) */
#define C_CAN_IFX_MCR_RMT_EN		0x0020		/* remote enable (rw) */
#define C_CAN_IFX_MCR_RX_IE		0x0040		/* rx intr enable (rw) */
#define C_CAN_IFX_MCR_TX_IE		0x0080		/* tx intr enable (rw) */
#define C_CAN_IFX_MCR_UMASK		0x0100		/* use acceptance mask (rw) */
#define C_CAN_IFX_MCR_INT_PND		0x0200	/* interrupt pending (rw) */
#define C_CAN_IFX_MCR_MSG_LST		0x0400	/* message lost (rw) */
#define C_CAN_IFX_MCR_NEW_DAT		0x0800	/* new data (rw) */


#define C_CAN_IFX_MCR_RX \
	(C_CAN_IFX_MCR_RX_IE|C_CAN_IFX_MCR_UMASK)
#define C_CAN_IFX_MCR_RX_EOB \
	(C_CAN_IFX_MCR_RX|C_CAN_IFX_MCR_EOB)

#define C_CAN_IFX_MCR_TX \
	(C_CAN_IFX_MCR_TX_IE|C_CAN_IFX_MCR_EOB)

/*
 * ...
 */
#define C_CAN_IFX_ARBX_TX		0x20000000
#define C_CAN_IFX_ARBX_MSG_XTD		0x40000000
#define C_CAN_IFX_ARBX_MSG_VAL		0x80000000
		
/*
 * Work in progress..
 */
 
struct c_can_softc {
	struct ifnet	*cc_ifp;		/* generic ifnet(9) glue */
	device_t	cc_dev;
	struct resource 	*cc_irq;
	struct task		cc_intr_task;
	void	*cc_intr;
	uint32_t	cc_freq;
	uint16_t	cc_status;
	struct mtx	cc_mtx;
};
#define	C_CAN_LOCK(cc)		mtx_lock(&(cc)->cc_mtx)
#define	C_CAN_UNLOCK(cc)		mtx_unlock(&(cc)->cc_mtx)
#define	C_CAN_LOCK_ASSERT(cc)	mtx_assert(&(cc)->cc_mtx, MA_OWNED)

/* utility-macros */
#define	c_can_timercmp(tvp, uvp, val)	\
	(((uvp)->tv_sec - (tvp)->tv_sec) < (val))
