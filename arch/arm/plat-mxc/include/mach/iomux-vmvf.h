/*
 * based on arch/arm/plat-mxc/include/mach/iomux-v3.h
 *
 * Copyright (C) 2009 by Jan Weitzel Phytec Messtechnik GmbH,
 *			<armlinux@phytec.de>
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

#ifndef __MACH_IOMUX_VMVF_H__
#define __MACH_IOMUX_VMVF_H__

/*
 *	build IOMUX_PAD structure
 *
 * This iomux scheme is based around pads, which are the physical balls
 * on the processor.
 *
 * - Each pad has a MUX pad control register (IOMUXC_SW_MUX_CTRL_PAD_x)
 *   which controls an output routing and things like driving strength and
 *   pullup/pulldown.
 * - Each pad can have but not necessarily does have an input routing
 *   register (IOMUXC_x_SELECT_INPUT)
 *
 * The two register sets do not have a fixed offset to each other,
 * hence we order this table by pad control registers (which all pads
 * have) and put the optional input routing registers into additional
 * fields.
 *
 * The naming convention for the pad modes is MX35_PAD_<padname>__<padmode>
 * If <padname> or <padmode> refers to a GPIO, it is named
 * GPIO_<unit>_<num>
 *
 * IOMUX/PAD Bit field definitions
 *
 * MUX_CTRL_PAD_OFS: 		    0..11 (12)
 * SEL_INPUT_OFS:		   12..23 (12)
 * MUX_CTRL_PAD + NO_PAD_CTRL:	   24..47 (24)
 * SEL_INP:			   48..51  (4)
 * reserved:			   52..63 (12)
*/

typedef u64 iomux_vmvf_cfg_t;

#define MUX_CTRL_PAD_OFS_SHIFT	0
#define MUX_CTRL_PAD_OFS_MASK	((iomux_vmvf_cfg_t)0xfff << MUX_CTRL_PAD_OFS_SHIFT)
#define MUX_SEL_INPUT_OFS_SHIFT	12
#define MUX_SEL_INPUT_OFS_MASK	((iomux_vmvf_cfg_t)0xfff << MUX_SEL_INPUT_OFS_SHIFT)

#define MUX_CTRL_PAD_SHIFT	24
#define MUX_CTRL_PAD_MASK	((iomux_vmvf_cfg_t)0xffffff << MUX_CTRL_PAD_SHIFT)
#define MUX_SEL_INPUT_SHIFT	48
#define MUX_SEL_INPUT_MASK	((iomux_vmvf_cfg_t)0xf << MUX_SEL_INPUT_SHIFT)

#define MUX_CTRL_PAD(x)		((iomux_vmvf_cfg_t)(x) << MUX_CTRL_PAD_SHIFT)

#define IOMUX_PAD(_mux_ctrl_pad_ofs, _mux_ctrl_pad, _sel_input_ofs, _sel_input)	\
	(((iomux_vmvf_cfg_t)(_mux_ctrl_pad_ofs) << MUX_CTRL_PAD_OFS_SHIFT) | \
	 ((iomux_vmvf_cfg_t)(_mux_ctrl_pad) << MUX_CTRL_PAD_SHIFT) | \
	 ((iomux_vmvf_cfg_t)(_sel_input_ofs) << MUX_SEL_INPUT_OFS_SHIFT) | \
	 ((iomux_vmvf_cfg_t)(_sel_input) << MUX_SEL_INPUT_SHIFT))

#define NEW_MUX_CTRL_PAD(cfg, mux_ctrl_pad)	\
	(((cfg) & ~MUX_CTRL_PAD_MASK) | MUX_CTRL_PAD(mux_ctrl_pad))
/*
 * Use to set PAD control
 */
#define NO_PAD_CTRL			(0 << 23)

#define MUX_CTL_PAD_MUX_MODE_ALT0	(0 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT1	(1 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT2	(2 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT3	(3 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT4	(4 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT5	(5 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT6	(6 << 20)
#define MUX_CTL_PAD_MUX_MODE_ALT7	(7 << 20)

#define MUX_CTL_PAD_DDR_INPUT_CMOS	(0 << 16)
#define MUX_CTL_PAD_DDR_INPUT_DIFF	(1 << 16)

#define MUX_CTL_PAD_DDR_TRIM_MIN	(0 << 14)
#define MUX_CTL_PAD_DDR_TRIM_50PS	(1 << 14)
#define MUX_CTL_PAD_DDR_TRIM_100PS	(2 << 14)
#define MUX_CTL_PAD_DDR_TRIM_150PS	(3 << 14)

#define MUX_CTL_PAD_SPEED_LOW		(0 << 12)
#define MUX_CTL_PAD_SPEED_MED01		(1 << 12)
#define MUX_CTL_PAD_SPEED_MED10		(2 << 12)
#define MUX_CTL_PAD_SPEED_HIGH		(3 << 12)

#define MUX_CTL_PAD_SRE_SLOW		(0 << 11)
#define MUX_CTL_PAD_SRE_FAST		(1 << 11)

#define MUX_CTL_PAD_ODE			(1 << 10)

#define MUX_CTL_PAD_HYS			(1 << 9)

#define MUX_CTL_PAD_DSE_DISABLE		(0 << 6)
#define MUX_CTL_PAD_DSE_150ohm		(1 << 6) /* 240 Ohm if pad is DDR */
#define MUX_CTL_PAD_DSE_75ohm		(2 << 6) /* 120 Ohm if pad is DDR */
#define MUX_CTL_PAD_DSE_50ohm		(3 << 6) /* 80 Ohm if pad is DDR */
#define MUX_CTL_PAD_DSE_37ohm		(4 << 6) /* 60 Ohm if pad is DDR */
#define MUX_CTL_PAD_DSE_30ohm		(5 << 6) /* 48 Ohm if pad is DDR */
#define MUX_CTL_PAD_DSE_25ohm		(6 << 6)
#define MUX_CTL_PAD_DSE_20ohm		(7 << 6) /* 34 Ohm if pad is DDR */

#define MUX_CTL_PAD_PUS_100K_DOWN	(0 << 4 | MUX_CTL_PAD_PUE)
#define MUX_CTL_PAD_PUS_47K_UP		(1 << 4 | MUX_CTL_PAD_PUE)
#define MUX_CTL_PAD_PUS_100K_UP		(2 << 4 | MUX_CTL_PAD_PUE)
#define MUX_CTL_PAD_PUS_22K_UP		(3 << 4 | MUX_CTL_PAD_PUE)

#define MUX_CTL_PAD_PKE			(1 << 3)
#define MUX_CTL_PAD_PUE			(1 << 2 | MUX_CTL_PAD_PKE)

#define MUX_CTL_PAD_OBE			(1 << 1)
#define MUX_CTL_PAD_IBE			(1 << 0)

#define NON_MUX_CTRL_PAD_I		0x2EC

#if 0
#define MX51_NUM_GPIO_PORT	4

#define GPIO_PIN_MASK 0x1f

#define GPIO_PORT_SHIFT 5
#define GPIO_PORT_MASK (0x7 << GPIO_PORT_SHIFT)

#define GPIO_PORTA	(0 << GPIO_PORT_SHIFT)
#define GPIO_PORTB	(1 << GPIO_PORT_SHIFT)
#define GPIO_PORTC	(2 << GPIO_PORT_SHIFT)
#define GPIO_PORTD	(3 << GPIO_PORT_SHIFT)
#define GPIO_PORTE	(4 << GPIO_PORT_SHIFT)
#define GPIO_PORTF	(5 << GPIO_PORT_SHIFT)
#endif

/*
 * setups a single pad in the iomuxer
 */
int mxc_iomux_vmvf_setup_pad(iomux_vmvf_cfg_t pad);

/*
 * setups mutliple pads
 * convenient way to call the above function with tables
 */
int mxc_iomux_vmvf_setup_multiple_pads(iomux_vmvf_cfg_t *pad_list, unsigned count);

/*
 * Initialise the iomux controller
 */
void mxc_iomux_vmvf_init(void __iomem *iomux_vmvf_base);
#endif /* __MACH_IOMUX_VMVF_H__*/

