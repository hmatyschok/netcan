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
 * PLX90xx PCI-bridge card vendor ID.
 */
#define PLX_VENDORID_ADLINK			0x144A
#define PLX_VENDORID_CAN200PCI			0x10b5
#define PLX_VENDORID_IXXAT			0x10b5
#define PLX_VENDORID_TEWS			0x1498
#define PLX_VENDORID_CTI			0x12c4
#define PLX_VENDORID_MOXA			0x1393
#define PLX_VENDORID_PLX			0x10b5

/*
 * PLX90xx PCI-bridge card device IDs.
 */
#define PLX_DEVICEID_ADLINK			0x7841
#define PLX_DEVICEID_CAN200PCI			0x9030
#define PLX_DEVICEID_IXXAT			0x9050
#define PLX_DEVICEID_MARATHON_PCI			0x2715
#define PLX_DEVICEID_MARATHON_PCIE			0x3432
#define PLX_DEVICEID_TEWS_TMPC810			0x032A
#define PLX_DEVICEID_CTI_CRG001			0x0900
#define PLX_DEVICEID_MOXA			0x0100
#define PLX_DEVICEID_PLX_9030			0x9030
#define PLX_DEVICEID_PLX_9050			0x9050
#define PLX_DEVICEID_PLX_9056			0x9056

#define PLX_SUBDEVID_ESD_PCI200			0x0004
#define PLX_SUBDEVID_ESD_PCI266			0x0009
#define PLX_SUBDEVID_ESD_PMC266			0x000e
#define PLX_SUBDEVID_ESD_CPCI200			0x010b
#define PLX_SUBDEVID_ESD_PCIE2000			0x0200
#define PLX_SUBDEVID_ESD_PCI104200			0x0501
#define PLX_SUBDEVID_CAN200PCI			0x0301
#define PLX_SUBDEVID_IXXAT			0x2540
#define PLX_SUBDEVID_ANY			0xffff

#define PLX_SUBVENDID_CAN200PCI			0xe1c5
#define PLX_SUBVENDID_ESD			0x12fe
#define PLX_SUBVENDID_ANY			0xffff

/*
 * Default values.
 */
#define PLX_OCR_DFLT		(SJA_OCR_TX0_PSHP | SJA_OCR_TX1_PSHP)
#define PLX_CDR_DFLT		(SJA_CDR_CBP | SJA_CDR_CLK_OUT)
#define PLX_CLK_FREQ		(16000000 / 2)

/*
 * Important register.
 * 
 * PLX9030/9050/9052
 */
#define PLX_ICR		0x4c		/* interrupt control / status */
#define PLX_TCR		0x50		/* control / software reset */

#define PLX_ICR_INT0_ENB		0x00000001	/* local interrupt 0 */
#define PLX_ICR_INT1_ENB		0x00000008	/* local interrupt 1 */
#define PLX_ICR_PINT_ENB		0x00000040	/* PCI interrupt */

#define PLX_TCR_RST		0x40000000	/* pci(4) adapter software reset */

/* PLX9056 */
#define PLX_9056_ICR		0x68		/* interrupt control / status */
#define PLX_9056_TCR		0x6c		/* control / software reset */

#define PLX_9056_ICR_INT0_ENB		0x00000800
#define PLX_9056_ICR_PINT_ENB		0x00000100
#define PLX_9056_TCR_RCR		0x20000000 /* read configuration */

#define PLX_CHAN_MAX		2

struct plx_desc {
	int		plx_bar;
	int		plx_off;
	rman_res_t		plx_cnt;
};

struct plx_data {
	/* parameter for resource allocation */
	struct plx_desc		plx_res;
	struct plx_desc		plx_chan[PLX_CHAN_MAX];
	
	/* parameter for initializiation */
	uint32_t		plx_icr_read;
	uint32_t		plx_icr_addr;
	uint32_t		plx_icr;
	
	/* parameter for software reset */	
	uint32_t		plx_tcr_addr;
	uint32_t		plx_tcr_rst;
	uint32_t		plx_tcr_rcr;		/* set, if PLX9056 */
};

struct plx_type {
	uint16_t 	plx_vid;
	uint16_t 	plx_did;
	uint16_t	plx_sub_vid;
	uint16_t	plx_sub_did;
	struct plx_data		*plx_id;
	const char 	*plx_name;
};

struct plx_softc {
	device_t 	plx_dev;

	struct plx_data		*plx_id;

	/* ICR / TCR */
	struct resource		*plx_res;
	int			plx_res_id;
	int			plx_res_type;

	/* set of sja(4) controller */
	struct sja_chan		plx_chan[PLX_CHAN_MAX];
};
