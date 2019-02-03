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
 * General constants.
 * 
 * PEAK Systems vendor ID.
 */
#define PEAK_VENDORID		0x001C	/* the PCI device and vendor IDs */

/*
 * PEAK Systems PCAN device IDs.
 */
#define PEAK_DEVICEID_PCI		0x0001
#define PEAK_DEVICEID_PCIEC		0x0002
#define PEAK_DEVICEID_PCIE		0x0003
#define PEAK_DEVICEID_CPCI		0x0004
#define PEAK_DEVICEID_MPCI		0x0005
#define PEAK_DEVICEID_PC_104P		0x0006
#define PEAK_DEVICEID_PCI_104E		0x0007
#define PEAK_DEVICEID_MPCIE		0x0008
#define PEAK_DEVICEID_PCIE		0x0009
#define PEAK_DEVICEID_PCIEC34		0x000A

struct peak_type {
	uint16_t 	pk_vid;
	uint16_t 	pk_did;
	const char 	*pk_name;
};

#define PEAK_SUBDEVID_DUAL_CHAN		0x0004
#define PEAK_SUBDEVID_TRIPLE_CHAN		0x0010
#define PEAK_SUBDEVID_QUAD_CHAN		0x0012

#define PEAK_UNI_CHAN		1
#define PEAK_DUAL_CHAN		2
#define PEAK_TRIPLE_CHAN	3
#define PEAK_QUAD_CHAN		4

/*
 * Important register.
 */ 
#define PEAK_ICR		0x00		/* interrupt control register */
#define PEAK_ICR_INT_GP		0x02		/*   GPIO internal register */
#define PEAK_ICR_INT_GP0		0x0002		/* channel 1 */
#define PEAK_ICR_INT_GP1		0x0001		/* channel 2 */
#define PEAK_ICR_INT_GP2		0x0040		/* channel 3 */
#define PEAK_ICR_INT_GP3		0x0080		/* channel 4 */

#define PEAK_GPIO_ICR		0x18	/* GPIO interface control register */
#define PEAK_GPIO_ICR_ENB		0x00
#define PEAK_GPIO_ICR_IO		0x1a	/*   clear command register */
#define PEAK_GPIO_ICR_IO_ENB		0x0005


#define PEAK_MISC		0x1c		/* miscellaneous register */
#define PEAK_MISC_CR		0x1f	/*   command register */

#define PEAK_MISC_CR_TOG_RST		0x05
#define PEAK_MISC_CR_PP_EPP		0x04

#define PEAK_CSID		0x2e

/*
 * Default values.
 */
#define PEAK_OCR_DFLT		(SJA_OCR_TX0_PSHP)
#define PEAK_CDR_DFLT		(SJA_CDR_CBP | SJA_CDR_CLK_OUT)
#define PEAK_CLK_FREQ		(16000000 / 2)

#define PEAK_CFG_SIZE		0x1000	/* Size of the config PCI bar */
#define PEAK_CHAN_SIZE		0x0400	/* Size used by the channel */

struct peak_chan {
	struct sja_chan	pkc_chan;
	uint16_t	pkc_flags;
};

struct peak_softc {
	device_t 	pk_dev;
	struct resource		*pk_res;
	int			pk_res_id;
	int			pk_res_type;
	uint32_t	pk_chan_cnt;
	struct peak_chan	pk_chan[PEAK_QUAD_CHAN];
};
