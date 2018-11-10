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

/*
 * SJA1000, 6.4.1 PeliCAN Address Layout
 */
 
#define SJA_MOD 			0x00 		/* mode */ 
#define SJA_CMR 			0x01 		/* control */
#define SJA_SR 			0x02 		/* status */
#define SJA_IR 		0x03 		/* interrupt status */
#define SJA_IER 		0x04 		/* interrupt enable */
	/* 0x05 reserved */
#define SJA_BTR0 		0x06 		/* bus timing 0 */
#define SJA_BTR1 		0x07 		/* bus timing 1 */
#define SJA_OCR 		0x08 		/* output control */
#define SJA_TEST 		0x09 		/* test */
	/* 0x0a reserved */
#define SJA_ALC 		0x0b 		/* arbitration lost capature */
#define SJA_ECC 		0x0c 		/* error code capature */
#define SJA_EWL 		0x0d 		/* error warning limit */
#define SJA_RX_EC 		0x0e 		/* rx error counter */ 
#define SJA_TX_EC		0x0f 		/* tx error counter */
#define SJA_AC0 		0x10 		/* acceptance code 0 */
#define SJA_AC1 		0x11 		/* acceptance code 1 */
#define SJA_AC2 		0x12 		/* acceptance code 2 */
#define SJA_AC3 		0x13 		/* acceptance code 3 */
#define SJA_AM0 		0x14 		/* acceptance mask 0 */
#define SJA_AM1 		0x15 		/* acceptance mask 1 */
#define SJA_AM2 		0x16 		/* acceptance mask 2 */
#define SJA_AM3 		0x17 		/* acceptance mask 3 */
	/* 0x18 - 0x1c reserved */
#define SJA_RC_MC 		0x1d 		/* rx message counter */
#define SJA_RXADDR 		0x1e 		/* rx ring start-addr */
#define SJA_CLKDIV 		0x1f 		/* clock divider */
#define SJA_RAM0        0x20    /* internal RAM addr 0 [FIFO] */
#define SJA_RAM1        0x21    /* internal RAM addr 1 [FIFO] */
#define SJA_RAM2        0x22    /* internal RAM addr 2 [FIFO] */
#define SJA_RAM3        0x23    /* internal RAM addr 3 [FIFO] */
#define SJA_RAM4        0x24    /* internal RAM addr 4 [FIFO] */
#define SJA_RAM5        0x25    /* internal RAM addr 5 [FIFO] */
#define SJA_RAM6        0x26    /* internal RAM addr 6 [FIFO] */
#define SJA_RAM7        0x27    /* internal RAM addr 7 [FIFO] */
#define SJA_RAM8        0x28    /* internal RAM addr 8 [FIFO] */
#define SJA_RAM9        0x29    /* internal RAM addr 9 [FIFO] */
#define SJA_RAM10       0x2a    /* internal RAM addr 10 [FIFO] */
#define SJA_RAM11       0x2b    /* internal RAM addr 11 [FIFO] */
#define SJA_RAM12       0x2c    /* internal RAM addr 12 [FIFO] */
#define SJA_RAM13       0x2d    /* internal RAM addr 13 [FIFO] */
#define SJA_RAM14       0x2e    /* internal RAM addr 14 [FIFO] */
#define SJA_RAM15       0x2f    /* internal RAM addr 15 [FIFO] */
#define SJA_RAM16       0x30    /* internal RAM addr 16 [FIFO] */
#define SJA_RAM17       0x31    /* internal RAM addr 17 [FIFO] */
#define SJA_RAM18       0x32    /* internal RAM addr 18 [FIFO] */
#define SJA_RAM19       0x33    /* internal RAM addr 19 [FIFO] */
#define SJA_RAM20       0x34    /* internal RAM addr 20 [FIFO] */
#define SJA_RAM21       0x35    /* internal RAM addr 21 [FIFO] */
#define SJA_RAM22       0x36    /* internal RAM addr 22 [FIFO] */
#define SJA_RAM23       0x37    /* internal RAM addr 23 [FIFO] */
#define SJA_RAM24       0x38    /* internal RAM addr 24 [FIFO] */
#define SJA_RAM25       0x39    /* internal RAM addr 25 [FIFO] */
#define SJA_RAM26       0x3a    /* internal RAM addr 26 [FIFO] */
#define SJA_RAM27       0x3b    /* internal RAM addr 27 [FIFO] */
#define SJA_RAM28       0x3c    /* internal RAM addr 28 [FIFO] */
#define SJA_RAM29       0x3d    /* internal RAM addr 29 [FIFO] */
#define SJA_RAM30       0x3e    /* internal RAM addr 30 [FIFO] */
#define SJA_RAM31       0x3f    /* internal RAM addr 31 [FIFO] */
#define SJA_RAM32       0x40    /* internal RAM addr 32 [FIFO] */
#define SJA_RAM33       0x41    /* internal RAM addr 33 [FIFO] */
#define SJA_RAM34       0x42    /* internal RAM addr 34 [FIFO] */
#define SJA_RAM35       0x43    /* internal RAM addr 35 [FIFO] */
#define SJA_RAM36       0x44    /* internal RAM addr 36 [FIFO] */
#define SJA_RAM37       0x45    /* internal RAM addr 37 [FIFO] */
#define SJA_RAM38       0x46    /* internal RAM addr 38 [FIFO] */
#define SJA_RAM39       0x47    /* internal RAM addr 39 [FIFO] */
#define SJA_RAM40       0x48    /* internal RAM addr 40 [FIFO] */
#define SJA_RAM41       0x49    /* internal RAM addr 41 [FIFO] */
#define SJA_RAM42       0x4a    /* internal RAM addr 42 [FIFO] */
#define SJA_RAM43       0x4b    /* internal RAM addr 43 [FIFO] */
#define SJA_RAM44       0x4c    /* internal RAM addr 44 [FIFO] */
#define SJA_RAM45       0x4d    /* internal RAM addr 45 [FIFO] */
#define SJA_RAM46       0x4e    /* internal RAM addr 46 [FIFO] */
#define SJA_RAM47       0x4f    /* internal RAM addr 47 [FIFO] */
#define SJA_RAM48       0x50    /* internal RAM addr 48 [FIFO] */
#define SJA_RAM49       0x51    /* internal RAM addr 49 [FIFO] */
#define SJA_RAM50       0x52    /* internal RAM addr 50 [FIFO] */
#define SJA_RAM51       0x53    /* internal RAM addr 51 [FIFO] */
#define SJA_RAM52       0x54    /* internal RAM addr 52 [FIFO] */
#define SJA_RAM53       0x55    /* internal RAM addr 53 [FIFO] */
#define SJA_RAM54       0x56    /* internal RAM addr 54 [FIFO] */
#define SJA_RAM55       0x57    /* internal RAM addr 55 [FIFO] */
#define SJA_RAM56       0x58    /* internal RAM addr 56 [FIFO] */
#define SJA_RAM57       0x59    /* internal RAM addr 57 [FIFO] */
#define SJA_RAM58       0x5a    /* internal RAM addr 58 [FIFO] */
#define SJA_RAM59       0x5b    /* internal RAM addr 59 [FIFO] */
#define SJA_RAM60       0x5c    /* internal RAM addr 60 [FIFO] */
#define SJA_RAM61       0x5d    /* internal RAM addr 61 [FIFO] */
#define SJA_RAM62       0x5e    /* internal RAM addr 62 [FIFO] */
#define SJA_RAM63       0x5f    /* internal RAM addr 63 [FIFO] */
#define SJA_RAM64       0x60    /* internal RAM addr 64 [TX buffer] */
#define SJA_RAM65       0x61    /* internal RAM addr 65 [TX buffer] */
#define SJA_RAM66       0x62    /* internal RAM addr 66 [TX buffer] */
#define SJA_RAM67       0x63    /* internal RAM addr 67 [TX buffer] */
#define SJA_RAM68       0x64    /* internal RAM addr 68 [TX buffer] */
#define SJA_RAM69       0x65    /* internal RAM addr 69 [TX buffer] */
#define SJA_RAM70       0x66    /* internal RAM addr 70 [TX buffer] */
#define SJA_RAM71       0x67    /* internal RAM addr 71 [TX buffer] */
#define SJA_RAM72       0x68    /* internal RAM addr 72 [TX buffer] */
#define SJA_RAM73       0x69    /* internal RAM addr 73 [TX buffer] */
#define SJA_RAM74       0x6a    /* internal RAM addr 74 [TX buffer] */
#define SJA_RAM75       0x6b    /* internal RAM addr 75 [TX buffer] */
#define SJA_RAM76       0x6c    /* internal RAM addr 76 [TX buffer] */
#define SJA_RAM77       0x6d    /* internal RAM addr 77 [free] */
#define SJA_RAM78       0x6e    /* internal RAM addr 78 [free] */
#define SJA_RAM79       0x6f    /* internal RAM addr 79 [free] */

/* 
 * SJA1000, 6.4.3 Mode Registers [MOD] 
 */
#define SJA_MOD_RM 		0x00 	/* reset mode */
#define SJA_MOD_LOM 	0x01 	/* listen only mode */
#define SJA_MOD_STM 	0x02 	/* self test mode */
#define SJA_MOD_AFM 	0x04 	/* acceptance filter mode */
#define SJA_MOD_SM 		0x08  	/* sleep mode */
#define SJA_MOD_RSVD 	0xe0 	/* reserved */

/* 
 * SJA1000, 6.4.4 Command Registers [CMR] 
 */
#define SJA_CMR_TR 		0x01 	/* transmission request */
#define SJA_CMR_AT 		0x02 	/* abort transmission */
#define SJA_CMR_RRB 	0x04 	/* release receive buffer */
#define SJA_CMR_CDO 	0x08 	/* clear data overrun */
#define SJA_CMR_SRR 	0x10 	/* self reception test */
#define SJA_CMR_RSVD 	0xe0 	/* reserved */

/* 
 * SJA1000, 6.4.5 Status Register [SR] 
 */
#define SJA_SR_RBS 		0x01 	/* receive buffer status */
#define SJA_SR_DOS 		0x02 	/* data overrun status */
#define SJA_SR_TBS 		0x04 	/* transmit buffer status */
#define SJA_SR_TCS 		0x08 	/* transmission complete status */
#define SJA_SR_RS 		0x10 	/* receive status */
#define SJA_SR_TS 		0x20 	/* transmit status */
#define SJA_SR_ES 		0x40 	/* error status */
#define SJA_SR_BS 		0x80 	/* bus status */

/*
 * XXX: work in progress..
 */