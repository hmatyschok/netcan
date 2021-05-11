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
 * General constants.
 * 
 * EMS CPC-PCI/PCIe/104P CAN card vendor ID.
 */
#define EMS_VENDORID_SIEMENS			0x110a
#define EMS_VENDORID_PLX			0x10b5

/*
 * PLX9030 PCI-bridge card device IDs.
 */
#define EMS_DEVICEID_SIEMENS			0x2104
#define EMS_DEVICEID_PLX_9030			0x9030

#define EMS_SUBDEVID_CPC_PCI2			0x4000
#define EMS_SUBDEVID_CPC_104P2			0x4002
#define EMS_SUBDEVID_ANY			0xffff

#define EMS_SUBVENDID_PLX			0x10b5
#define EMS_SUBVENDID_ANY			0xffff

struct ems_desc {
	int		ems_bar;
	int		ems_off;
	rman_res_t		ems_cnt;
};

struct ems_data {
	struct ems_desc		ems_cfg;
	struct ems_desc		ems_res;
	struct ems_desc		ems_chan;
	
	int		ems_chan_cnt;
	
	uint32_t		ems_aln;
	
	uint32_t 		ems_icr_addr;
	uint32_t 		ems_icr_rst;
};

struct ems_type {
	uint16_t 	ems_vid;
	uint16_t 	ems_did;
	uint16_t	ems_sub_vid;
	uint16_t	ems_sub_did;
	struct ems_data		*ems_id;
	const char 	*ems_name;
};

/*
 * Important register.
 * 
 * EMS CPC-PCI v1 cards.
 */
#define EMS_PITA_ICR		0x00	/* interrupt control register */
#define EMS_PITA_ICR_INT0		0x00000002		/* interrupt status */
#define EMS_PITA_ICR_INT0_ENB		0x00020000	/* interrupt enable */
#define EMS_PITA_ICR_INT_RST \		/* clear interrupt */
	(EMS_PITA_ICR_INT0|EMS_PITA_ICR_INT0_ENB)

#define EMS_PITA_MISC		0x1c	/* misc. register */
#define EMS_PITA_MISC_PIM		0x04000000	/* parallel interface mode */


/* EMS CPC-{PCI,104P} v2 cards. */
#define EMS_PLX_ICR		0x4c   /* Interrupt Control/Status register */
#define EMS_PLX_ICR_INT0_ENB		0x0001 /* LINTi1 Enable */
#define EMS_PLX_ICR_PINT_ENB		0x0040 /* PCI Interrupt Enable */
#define EMS_PLX_ICR_INT0_CLR		0x0400 /* Local Edge Triggerable Interrupt Clear */
#define EMS_PLX_ICR_INT_RST \
	(EMS_PLX_ICR_INT0_ENB|EMS_PLX_ICR_PINT_ENB|EMS_PLX_ICR_INT0_CLR)

/*
 * Default values.
 */
#define EMS_OCR_DFLT		(SJA_OCR_TX0_PSHP | SJA_OCR_TX1_PSHP)
#define EMS_CDR_DFLT		(SJA_CDR_CBP | SJA_CDR_CLK_OUT)
#define EMS_CLK_FREQ		(16000000 / 2)

#define EMS_DUAL_CHAN		2
#define EMS_CHAN_MAX		4

struct ems_softc {
	device_t 	ems_dev;

	struct ems_data	*ems_id;

	/* ICR / TCR */
	struct resource		*ems_cfg;
	int			ems_cfg_id;
	int			ems_cfg_type;
	
	struct resource		*ems_res;
	int			ems_res_id;
	int			ems_res_type;

	/* set of sja(4) controller */
	struct sja_chan		ems_chan[EMS_CHAN_MAX]; 
};
