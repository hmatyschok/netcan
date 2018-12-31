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

#define PEAK_SUBDEVID_DUAL_CHAN		0x0004
#define PEAK_SUBDEVID_TRIPLE_CHAN		0x0010
#define PEAK_SUBDEVID_QUAD_CHAN		0x0012

#define PEAK_UNI_CHAN		1
#define PEAK_DUAL_CHAN		2
#define PEAK_TRIPLE_CHAN	3
#define PEAK_QUAD_CHAN		4

/*
 * XXX: work on progress ...
 */ 

#define PEAK_ICR		0x00		/* interrupt control register */
#define PEAK_GPIO_ICR		0x18	/* GPIO interface control register */
#define PEAK_MISC		0x1c		/* miscellaneous register */
#define PEAK_CSID		0x2e	

/*
 * ...
 */

#define PEAK_CFG_SIZE		0x1000	/* Size of the config PCI bar */
#define PEAK_CHAN_SIZE		0x0400	/* Size used by the channel */

/*
 * ...
 */

#define PEAK_GPIO_ICR_INIT		0x0005
#define PEAK_GPIO_ICR_START		0x00

#define PEAK_MISC_TOG_RST		0x05

#define PEAK_MISC_PMUX_STOP		0x04

/*
 * Parent device(9) accessing e. g. PCI-BUS, etc.
 */
struct peak_softc {
	device_t 	pk_dev;
	struct resource		*pk_res;
	int			pk_res_id;
	int			pk_res_type;
	uint32_t	pk_chan_cnt;
	struct sja_chan	*pk_chan; 
};
