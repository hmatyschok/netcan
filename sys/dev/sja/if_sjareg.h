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

/*
 * SJA1000, 6.4.1 PeliCAN Address Layout
 */

#define SJA_MOD         0x00        /* mode */
#define SJA_CMR         0x01        /* control */
#define SJA_SR          0x02        /* status */
#define SJA_IR      0x03        /* interrupt status */
#define SJA_IER     0x04        /* interrupt enable */
    /* 0x05 reserved */
#define SJA_BTR0        0x06        /* bus timing 0 */
#define SJA_BTR1        0x07        /* bus timing 1 */
#define SJA_OCR     0x08        /* output control */
#define SJA_TR      0x09        /* test */
    /* 0x0a reserved */
#define SJA_ALC     0x0b        /* arbitration lost capature */
#define SJA_ECC     0x0c        /* error code capature */
#define SJA_EWLR        0x0d        /* error warning limit */
#define SJA_RXERR       0x0e        /* rx error counter */
#define SJA_TXERR       0x0f        /* tx error counter */
#define SJA_ACR0        0x10        /* acceptance code 0 */
#define SJA_ACR1        0x11        /* acceptance code 1 */
#define SJA_ACR2        0x12        /* acceptance code 2 */
#define SJA_ACR3        0x13        /* acceptance code 3 */
#define SJA_AMR0        0x14        /* acceptance mask 0 */
#define SJA_AMR1        0x15        /* acceptance mask 1 */
#define SJA_AMR2        0x16        /* acceptance mask 2 */
#define SJA_AMR3        0x17        /* acceptance mask 3 */
    /* 0x18 - 0x1c reserved */
#define SJA_RMC     0x1d        /* rx message counter */
#define SJA_RBSA        0x1e        /* rx ring start-addr */
#define SJA_CDR     0x1f        /* clock divider */
    /* 0x20 - 0x5f internal RAM [FIFO] */
    /* 0x60 - 0x6c internal RAM [TX buf] */
    /* 0x6d - 0x6d internal RAM [free] */
    /* 0x70 - 0xff reserved */

/*
 * SJA1000, 6.4.3 Mode Registers [MOD]
 */
#define SJA_MOD_RM      0x01    /* reset mode */
#define SJA_MOD_LOM     0x02    /* listen only mode */
#define SJA_MOD_STM     0x04    /* self test mode */
#define SJA_MOD_AFM     0x08    /* acceptance filter mode */
#define SJA_MOD_SM      0x10    /* sleep mode */

/*
 * SJA1000, 6.4.4 Command Registers [CMR]
 */
#define SJA_CMR_TR      0x01    /* transmission request */
#define SJA_CMR_AT      0x02    /* abort transmission */
#define SJA_CMR_RRB     0x04    /* release receive buffer */
#define SJA_CMR_CDO     0x08    /* clear data overrun */
#define SJA_CMR_SRR     0x10    /* self reception test */

/*
 * SJA1000, 6.4.5 Status Register [SR]
 */
#define SJA_SR_RBS      0x01    /* receive buffer status */
#define SJA_SR_DOS      0x02    /* data overrun status */
#define SJA_SR_TBS      0x04    /* transmit buffer status */
#define SJA_SR_TCS      0x08    /* transmission complete status */
#define SJA_SR_RS       0x10    /* receive status */
#define SJA_SR_TS       0x20    /* transmit status */
#define SJA_SR_ES       0x40    /* error status */
#define SJA_SR_BS       0x80    /* bus status */

/*
 * SJA1000, 6.4.6 Interrupt Register [IR]
 */

#define SJA_IR_RI       0x01    /* receive interrupt */
#define SJA_IR_TI       0x02    /* transmit interrupt */
#define SJA_IR_EI       0x04    /* error warning interrupt */
#define SJA_IR_DOI      0x08    /* data overrun interrupt */
#define SJA_IR_WUI      0x10    /* wake-up interrupt */
#define SJA_IR_EPI      0x20    /* error passive interrupt */
#define SJA_IR_ALI      0x40    /* arbitration lost interrupt */
#define SJA_IR_BEI      0x80    /* bus error interrupt */

#define SJA_IR_ERR \
    (SJA_IR_EI|SJA_IR_DOI|SJA_IR_EPI|SJA_IR_ALI|SJA_IR_BEI)

#define SJA_IR_OFF      0x00
#define SJA_IR_ALL      0xff

/*
 * SJA1000, 6.4.7 Interrupt Enable Register [IER]
 */
#define SJA_IER_RIE     0x01    /* receive interrupt */
#define SJA_IER_TIE     0x02    /* transmit interrupt */
#define SJA_IER_EIE     0x04    /* error warning interrupt */
#define SJA_IER_DOIE        0x08    /* data overrun interrupt */
#define SJA_IER_WUIE        0x10    /* wake-up interrupt */
#define SJA_IER_EPIE        0x20    /* error passive interrupt */
#define SJA_IER_ALIE        0x40    /* arbitration lost interrupt */
#define SJA_IER_BEIE        0x80    /* bus error interrupt */

#define SJA_IER_OFF     0x00
#define SJA_IER_ALL     0xff

/*
 * SJA1000, 6.4.8 Arbitration Lost Capature Register [ALC]
 */
#define SJA_ALC_BIT0        0x01    /* ALC in 2^0 + 1 bit of id */
#define SJA_ALC_BIT1        0x02    /* ALC in 2^1 + 1 bit of id */
#define SJA_ALC_BIT2        0x04    /* ALC in 2^2 + 1 bit of id */
#define SJA_ALC_BIT3        0x08    /* ALC in 2^3 + 1 bit of id */
#define SJA_ALC_BIT4        0x10    /* ALC in 2^4 + 1 bit of id */
#define SJA_ALC_MASK        0x1f

#if 0
#define SJA_ALC_VAL(reg)        ((reg) & SJA_ALC_MASK)
#endif

/*
 * SJA1000, 6.4.9 Error Code Capature Register [ECC]
 */
#define SJA_ECC_SEG     0x1f
#define SJA_ECC_DIR     0x20    /* error occured during reception */

#define SJA_ECC_ERR_MASK    0xc0

#define SJA_ECC_BE      0x00        /* bit error */
#define SJA_ECC_SOF     0x03        /* start of frame */
#define SJA_ECC_FMT     0x04        /* format error */
#define SJA_ECC_ID28TO21        0x02        /* id bits 28 ... 21 */
#define SJA_ECC_ID20TO18        0x06        /* id bits 28 ... 21 */
#define SJA_ECC_SRTR        0x04        /* bit SRTR */
#define SJA_ECC_IDE     0x05        /* bit IDE */
#define SJA_ECC_ID17TO13        0x07        /* id bits 17 ... 13 */
#define SJA_ECC_ID12TO05        0x0f        /* id bits 12 ... 5 */
#define SJA_ECC_ID04TO00        0x0e        /* id bits 4 ... 0 */
#define SJA_ECC_RTR     0x0c        /* bit RTR */
#define SJA_ECC_RSVD1       0x0d        /* reserved bit 1 */
#define SJA_ECC_RSVD0       0x09        /* reserved bit 0 */
#define SJA_ECC_DLC     0x0b        /* data length code */
#define SJA_ECC_DF      0x0a        /* data field */
#define SJA_ECC_CRC_SEQ     0x08        /* CRC sequence */
#define SJA_ECC_CRC_DEL     0x18        /* CRC delimeter */
#define SJA_ECC_ACK_SLOT        0x19        /* ACK slot */
#define SJA_ECC_EOF     0x1a        /* end of frame */
#define SJA_ECC_INTERM      0x12        /* intermission */
#define SJA_ECC_AEF     0x11        /* active error flag */
#define SJA_ECC_PEF     0x16        /* passive error flag */
#define SJA_ECC_TDB     0x13        /* tolerate dominant bits */
#define SJA_ECC_ED      0x17        /* tolerate dominant bits */
#define SJA_ECC_OF      0x1c        /* overload flag */
#define SJA_ECC_SEG     0x1f        /* segment flag */

/*
 * SJA1000, 6.4.10 Error Warning Limit Register [EWLR]
 */
#define SJA_EWL_DFLT        0x96

/*
 * SJA1000, 6.4.11 RX Error Counter Register [RXERR]
 */
#define SJA_RXERR_DFLT      0x00

/*
 * SJA1000, 6.4.12 TX Error Counter Register [TXERR]
 */
#define SJA_TXERR_DFLT      0x00

/*
 * SJA1000, 6.4.1{3,4} {T,R}X Buffer
 */
#define SJA_FI      0x16        /* maps to CAN frame information */
#define SJA_ID      0x17        /* maps to CAN id */
#define SJA_DATA_SFF        0x19        /* maps to data region, SFF */
#define SJA_DATA_EFF        0x21        /* maps to data region, EFF */

#define SJA_FI_DLC      0x0f    /* data length code bits */
#define SJA_FI_DC       0x30    /* don't care bits, loopback */
#define SJA_FI_RTR      0x40    /* remote transmission request */
#define SJA_FI_FF       0x80    /* frame format */

/*
 * SJA1000, 6.4.16 RX Message Counter [RMC]
 */
#define SJA_RMC_MASK    0x1f

/*
 * SJA1000, 6.4.17 RX Buffer Start Address [RBSA]
 */
#define SJA_RBSA_MASK   0x3f

/*
 * SJA1000, 6.5.1 Bus Timing Register 0 [BTR0]
 */
#define SJA_BTR0_BRP_MASK       0x3f    /* baud rate prescaler */
#define SJA_BTR0_SJW_MASK       0xc0    /* synchroniziation junp width */

/*
 * SJA1000, 6.5.2 Bus Timing Register 1 [BTR1]
 */
#define SJA_BTR1_SAM        0x80    /* sampling */

#define SJA_BTR0_BRP(reg)   ((reg) & SJA_BTR0_BRP_MASK)
#define SJA_BTR0_SJW(reg)   (((reg) & SJA_BTR0_BRP_MASK) >> 6)

/*
 * SJA1000, 6.5.3 Output Control Register [OCR]
 */
#define SJA_OCR_MOD_BPH     0x00        /* bi-phase output mode */
#define SJA_OCR_MOD_TST     0x01        /* test output mode */
#define SJA_OCR_MOD_NORM        0x02        /* normal output mode */
#define SJA_OCR_MOD_CLK     0x03        /* clock output mode */

#define SJA_OCR_MOD_MASK        0x07
#define SJA_OCR_MOD(reg)    ((reg) & SJA_OCR_MOD_MASK)

#define SJA_OCR_TX0_INV     0x04        /* invert */
#define SJA_OCR_TX0_PDN     0x08        /* pulldown */
#define SJA_OCR_TX0_PUP     0x10        /* pullup */
#define SJA_OCR_TX0_PSHP        0x18        /* pushpull */

#define SJA_OCR_TX1_INV     0x20
#define SJA_OCR_TX1_PDN     0x40
#define SJA_OCR_TX1_PUP     0x80
#define SJA_OCR_TX1_PSHP        0xc0

#define SJA_OCR_TX_MASK     0xfc
#define SJA_OCR_TX_SHIFT        2

/*
 * SJA1000, 6.5.4 Clock Divider Register [CDR]
 */
#define SJA_CDR_CLK_OUT     0x07
#define SJA_CDR_CLK_OFF     0x08        /* clock off on clock out */
#define SJA_CDR_RXINPEN     0x20        /* TX1 is RX irq output */
#define SJA_CDR_CBP     0x040       /* can(4) input comperator bypass */
#define SJA_CDR_PELICAN     0x80        /* PeliCAN mode */

/* clock divider */
#define SJA_CDR_CD(reg)     ((reg) & SJA_CDR_CLK_OUT)

struct sja_data {
    int         sja_port;   /* index of used channel */

    /* default parameter */
    uint8_t     sja_cdr;
    uint8_t     sja_ocr;
    uint32_t        sja_freq;
};

struct sja_chan {
    device_t    sja_dev;

    /* allocated resources */
    struct resource     *sja_res;
    int     sja_res_id;
    int     sja_res_type;

    uint32_t    sja_aln;
    uint32_t    sja_flags;

    /* instance variables */
    struct sja_data     sja_var;
};

struct sja_softc {
    struct ifnet    *sja_ifp;       /* generic ifnet(9) glue */
    device_t    sja_dev;
    struct sja_data     *sja_var;   /* params got from its parent */
    struct resource     *sja_irq;
    void    *sja_intr;
    int     sja_if_flags;
    struct mtx  sja_mtx;
};
#define SJA_LOCK(sja)       mtx_lock(&(sja)->sja_mtx)
#define SJA_UNLOCK(sja)     mtx_unlock(&(sja)->sja_mtx)
#define SJA_LOCK_ASSERT(sja)    mtx_assert(&(sja)->sja_mtx, MA_OWNED)

/* utility-macros */
#define sja_timercmp(tvp, uvp, val) \
    (((uvp)->tv_sec - (tvp)->tv_sec) < (val))

/* linkmode capabilities */
#define SJA_LINKMODE_CAPS \
    (CAN_LINKMODE_LOOPBACK|CAN_LINKMODE_LISTENONLY| \
    CAN_LINKMODE_3SAMPLES|CAN_LINKMODE_ONE_SHOT| \
    CAN_LINKMODE_BUS_ERR_REP|CAN_LINKMODE_PRESUME_ACK)

extern driver_t sja_driver;
extern devclass_t sja_devclass;
