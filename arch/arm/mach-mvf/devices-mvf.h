/*
 * based on arch/arm/mach-mx6/devices-imx6q.h
 *
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 */

#include <mach/mvf.h>
#include <mach/devices-common.h>

extern const struct mvf_dcuv4_data vf600_dcuv4_data[] __initconst;
#define vf600_add_dcuv4(id, pdata)	mvf_add_dcuv4(id, &vf600_dcuv4_data[id], pdata)

extern const struct imx_imx_uart_1irq_data mvf_imx_uart_data[] __initconst;
#define mvf_add_imx_uart(id, pdata)	\
	imx_add_imx_uart_1irq(&mvf_imx_uart_data[id], pdata)

extern const struct imx_snvs_rtc_data vf6xx_imx_snvs_rtc_data __initconst;
#define vf6xx_add_imx_snvs_rtc()	\
	imx_add_snvs_rtc(&vf6xx_imx_snvs_rtc_data)

#define mvf_add_lcdif(pdata)	\
	platform_device_register_resndata(NULL, "mvf_lcdif",\
			0, NULL, 0, pdata, sizeof(*pdata));

#define mvf_add_v4l2_output(id)	\
	platform_device_register_resndata(NULL, "mvf_v4l2_output",\
			id, NULL, 0, NULL, 0);

#if 0
extern const struct imx_dma_data vf6xx_dma_data __initconst;
#define vf6xx_add_dma()	imx_add_dma(&vf6xx_dma_data);
#endif

extern const struct imx_fec_data vf6xx_fec_data __initconst;
#define vf6xx_add_fec(pdata)	\
	imx_add_fec(&vf6xx_fec_data, pdata)
