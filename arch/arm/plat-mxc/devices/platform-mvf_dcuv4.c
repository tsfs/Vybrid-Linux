/*
 * Copyright (C) 2011-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
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

#include <mach/hardware.h>
#include <mach/devices-common.h>
#include <linux/clk.h>

#define mvf_dcu4_data_entry_single(soc, id, size, dcu_pg, dcu_blank)	\
	{								\
		.iobase = soc ## _DCU ## id ## _BASE_ADDR,			\
		.irq = soc ## _INT_DCU ## id,				\
		.iosize = size,						\
		.pg = dcu_pg,						\
		.blank = dcu_blank,					\
	}

#ifdef CONFIG_SOC_VF6XX
#include <mach/iomux-vmvf.h>
#include <mach/iomux-vf6xx.h>
void vf600_dcuv4_pg(int id, int enable)
{
	/*TODO; get rid of this and use clock enable?*/
	//if(enable)
		//val = readl(MX51_IO_ADDRESS(MX51_SRC_BASE_ADDR));
		//writel(MXC_PGCR_PCR, MX51_PGC_IPU_PGCR);

}

void vf600_dcuv4_blank(int id, int enable)
{
	/*TODO: Add check for which DCU*/
	/*TODO: Get rid of the backlight enable and use PWM module */
	if(enable)
	{
		if(id == 0)
		{
			//disable pclk
			mxc_iomux_vmvf_setup_pad(VF6XX_PAD_PAD_110__RGPIOC_GPIO110);
			//disable backlight
			mxc_iomux_vmvf_setup_pad((_VF6XX_PAD_PAD_30__RGPIOC_GPIO30 | MUX_CTRL_PAD(VF6XX_PAD_CTRL_OBE)));
		}

	}
	else
	{
		if(id == 0)
		{
			//enable pclk
			mxc_iomux_vmvf_setup_pad(VF6XX_PAD_PAD_110__TCON0_DATA_OUT18);
			//enable backlight
			mxc_iomux_vmvf_setup_pad((_VF6XX_PAD_PAD_30__RGPIOC_GPIO30 | MUX_CTRL_PAD(NO_PAD_CTRL)));
		}
	}
}

const struct mvf_dcuv4_data vf600_dcuv4_data[] __initconst = {
		mvf_dcu4_data_entry_single(MVF, 0, SZ_4M,
			vf600_dcuv4_pg, vf600_dcuv4_blank),
		mvf_dcu4_data_entry_single(MVF, 1, SZ_4M,
			vf600_dcuv4_pg, vf600_dcuv4_blank),
};
#endif

struct platform_device *__init mvf_add_dcuv4(
		const int id,
		const struct mvf_dcuv4_data *data,
		struct mvf_dcuv4_platform_data *pdata)
{
	struct resource res[] = {
		{
			.start = data->iobase,
			.end = data->iobase + data->iosize - 1,
			.flags = IORESOURCE_MEM,
		}, {
			.start = data->irq,
			.end = data->irq,
			.flags = IORESOURCE_IRQ,
		},
	};

	pdata->pg = data->pg;
	pdata->blank = data->blank;

	return imx_add_platform_device_dmamask("mvf-dcuv4", id,
			res, ARRAY_SIZE(res), pdata, sizeof(*pdata),
			DMA_BIT_MASK(32));
}

struct platform_device *__init mvf_add_dcuv4_fb(
		const int id,
		const struct dcuv4_fb_platform_data *pdata)
{
	if (pdata->res_size[0] > 0) {
		struct resource res[] = {
			{
				.start = pdata->res_base[0],
				.end = pdata->res_base[0] + pdata->res_size[0] - 1,
				.flags = IORESOURCE_MEM,
			}, {
				.start = 0,
				.end = 0,
				.flags = IORESOURCE_MEM,
			},
		};

		if (pdata->res_size[1] > 0) {
			res[1].start = pdata->res_base[1];
			res[1].end = pdata->res_base[1] +
					pdata->res_size[1] - 1;
		}

		return imx_add_platform_device_dmamask("mxc_sdc_fb",
				id, res, ARRAY_SIZE(res), pdata,
				sizeof(*pdata), DMA_BIT_MASK(32));
	} else
		return imx_add_platform_device_dmamask("mxc_sdc_fb", id,
				NULL, 0, pdata, sizeof(*pdata),
				DMA_BIT_MASK(32));
}
