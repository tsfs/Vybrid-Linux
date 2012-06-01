/*
 * based on arch/arm/plat-mxc/iomux-v3.c
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 */
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/string.h>
#include <linux/gpio.h>

#include <mach/hardware.h>
#include <asm/mach/map.h>
#include <mach/iomux-vmvf.h>

static void __iomem *base;

/*
 * configures a single pad in the iomuxer
 */
int mxc_iomux_vmvf_setup_pad(iomux_vmvf_cfg_t pad)
{
	u32 mux_ctrl_pad_ofs = (pad & MUX_CTRL_PAD_OFS_MASK) >> MUX_CTRL_PAD_OFS_SHIFT;
	u32 sel_input_ofs = (pad & MUX_SEL_INPUT_OFS_MASK) >> MUX_SEL_INPUT_OFS_SHIFT;
	u32 sel_input = (pad & MUX_SEL_INPUT_MASK) >> MUX_SEL_INPUT_SHIFT;
	u32 mux_ctrl_pad = (pad & MUX_CTRL_PAD_MASK) >> MUX_CTRL_PAD_SHIFT;

	if (mux_ctrl_pad_ofs < NON_MUX_CTRL_PAD_I)
		__raw_writel(mux_ctrl_pad, base + mux_ctrl_pad_ofs);

	if (sel_input_ofs)
		__raw_writel(sel_input, base + sel_input_ofs);

	return 0;
}
EXPORT_SYMBOL(mxc_iomux_vmvf_setup_pad);

int mxc_iomux_vmvf_setup_multiple_pads(iomux_vmvf_cfg_t *pad_list, unsigned count)
{
	iomux_vmvf_cfg_t *p = pad_list;
	int i;
	int ret;

	for (i = 0; i < count; i++) {
		ret = mxc_iomux_vmvf_setup_pad(*p);
		if (ret)
			return ret;
		p++;
	}
	return 0;
}
EXPORT_SYMBOL(mxc_iomux_vmvf_setup_multiple_pads);

void mxc_iomux_set_gpr_register(int group, int start_bit, int num_bits, int value)
{
	int i = 0;
	u32 reg;
	reg = __raw_readl(base + group * 4);
	while (num_bits) {
		reg &= ~(1<<(start_bit + i));
		i++;
		num_bits--;
	}
	reg |= (value << start_bit);
	__raw_writel(reg, base + group * 4);
}
EXPORT_SYMBOL(mxc_iomux_set_gpr_register);

void mxc_iomux_vmvf_init(void __iomem *iomux_vmvf_base)
{
	base = iomux_vmvf_base;
}
