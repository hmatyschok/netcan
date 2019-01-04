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
 * KVASER PCAN PCI card vendor ID.
 */
#define KVASER_VENDORID0		0x10e8	/* the PCI device and vendor IDs */
#define KVASER_VENDORID1		0x1a07

/*
 * KVASER PCAN PCI card device IDs.
 */
#define KVASER_DEVICEID_PCI0		0x8406
#define KVASER_DEVICEID_PCI1		0x0008

/*
 * Default values.
 */
#define KVASER_OCR_DFLT		(SJA_OCR_TX0_PSHP | SJA_OCR_TX1_PSHP)
#define KVASER_CDR_DFLT		(SJA_CDR_CBP | SJA_CDR_CLK_OUT)
#define KVASER_CLK_FREQ		(16000000 / 2)

/*
 * Important register.
 */

#define KVASER_ICR		0x38		/* interrupt control register */
#define KVASER_TCR		0x60		/* transfer control register */

/*
 * ...
 */

#define KVASER_PCI_CFG_SIZE		0x80
#define KVASER_PCI_RES_SIZE		0x08 
#define KVASER_PCI_BASE_SIZE		0x80
 
#define KVASER_CHAN_SIZE     0x20

#define KVASER_CHAN_MAX		4

/*
 * ...
 */

#define KVASER_ICR_INIT		0x00002000UL
#define KVASER_TCR_PSV		0x80808080UL

#define KVASER_VERS_ID		7
#define KVASER_VERS_ID_PRESUMED   14

struct kvaser_softc {
	device_t 	kv_dev;

	/* S5920 */
	struct resource		*kv_cfg;
	int			kv_cfg_id;
	int			kv_cfg_type;

	/* XILINX board */
	struct resource		*kv_res;
	int			kv_res_id;
	int			kv_res_type;
	uint8_t		kv_vers_id;

	/* PCI port */
	struct resource		*kv_base;
	int			kv_base_id;
	int			kv_base_type;

	/* set of sja(4) controller */
	uint32_t	kv_chan_cnt;
	struct sja_chan		kv_chan[KVASER_CHAN_MAX]; 
};
