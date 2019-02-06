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
 * ... card vendor ID.
 */
#define C_CAN_VENDORID_STMICRO			0x104a
#define C_CAN_VENDORID_INTEL			0x8086

/*
 * ... card device IDs.
 */
#define C_CAN_DEVICEID_STMICRO_CAN			0xcc11
#define C_CAN_DEVICEID_PCH_CAN			0x8818

struct c_can_pci_type {
	uint16_t 	ccp_vid;
	uint16_t 	ccp_did;
	const char 	*ccp_name;
	int		ccp_bar;		/* pci(4) BAR */
	int		ccp_aln;		/* alignement */ 
	uint32_t		ccp_rst;	/* port for reset */
	uint32_t		ccp_freq;		/* CLK frequency */
};

/*
 * Default values.
 */
#define C_CAN_STA2X11_CLK_FREQ		52000000	/* 52 MHz */
#define C_CAN_PCH_CLK_FREQ		50000000		/* 50 MHz */

/*
 * Important register.
 * 
 * ...
 */

struct c_can_pci_softc {
	device_t 	ccp_dev;
	struct resource		*ccp_res;
	int			ccp_res_type;		
	int		ccp_shift;
	uint32_t		ccp_rst;
	uint32_t		ccp_freq;
	device_t		ccp_can;
};
