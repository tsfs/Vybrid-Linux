/*
 * Freescale eSDHC i.MX controller driver for the platform bus.
 *
 * derived from the OF-version.
 *
 * Copyright (c) 2010 Pengutronix e.K.
 *   Author: Wolfram Sang <w.sang@pengutronix.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License.
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/gpio.h>
#include <linux/slab.h>
#include <linux/mmc/host.h>
#include <linux/mmc/sdhci-pltfm.h>
#include <linux/mmc/mmc.h>
#include <linux/mmc/sdio.h>
#include <linux/mmc/sd.h>
#include <mach/hardware.h>
#include <mach/esdhc.h>
#include "sdhci.h"
#include "sdhci-pltfm.h"
#include "sdhci-esdhc.h"

/* VENDOR SPEC register */
#define SDHCI_VENDOR_SPEC		0xC0
#define  SDHCI_VENDOR_SPEC_SDIO_QUIRK	0x00000002

#define SDHCI_MIX_CTRL_EXE_TUNE		(1 << 22)
#define SDHCI_MIX_CTRL_SMPCLK_SEL	(1 << 23)
#define SDHCI_MIX_CTRL_AUTO_TUNE	(1 << 24)
#define SDHCI_MIX_CTRL_FBCLK_SEL	(1 << 25)

#define SDHCI_SYS_CTRL			0x2C
#define SDHCI_SYS_CTRL_RSTA_LSH 24

#define SDHCI_DLL_CTRL			0x60
#define SDHCI_DLL_OVERRIDE_OFFSET	0x9
#define SDHCI_DLL_OVERRIDE_EN_OFFSET	0x8

#define SDHCI_TUNE_CTRL_STATUS		0x68
#define SDHCI_TUNE_CTRL_STEP		0x1
#define SDHCI_TUNE_CTRL_MIN		0x0
#define SDHCI_TUNE_CTRL_MAX		((1 << 7) - 1)

#define  SDHCI_VENDOR_SPEC_VSELECT	(1 << 1)
#define  SDHCI_VENDOR_SPEC_FRC_SDCLK_ON	(1 << 8)

#define  SDHCI_PRESENT_STATE_CLSL	(1 << 23)
#define  SDHCI_PRESENT_STATE_DLSL_L4	(0xF << 24)
#define  SDHCI_PRESENT_STATE_DLSL_H4	(0xF << 28)

#define ESDHC_FLAG_GPIO_FOR_CD_WP	(1 << 0)

#define	SDHCI_CTRL_D3CD			0x08

#define SDHCI_PROT_CTRL_DMASEL_MASK		(3 << 8)
#define SDHCI_PROT_CTRL_DTW		(3 << 1)
#define SDHCI_PROT_CTRL_8BIT		(2 << 1)
#define SDHCI_PROT_CTRL_4BIT		(1 << 1)
#define SDHCI_PROT_CTRL_1BIT		(0 << 1)
#define SDHCI_PROT_CTRL_LCTL		(1 << 0)

/*
 * The CMDTYPE of the CMD register (offset 0xE) should be set to
 * "11" when the STOP CMD12 is issued on imx53 to abort one
 * open ended multi-blk IO. Otherwise the TC INT wouldn't
 * be generated.
 * In exact block transfer, the controller doesn't complete the
 * operations automatically as required at the end of the
 * transfer and remains on hold if the abort command is not sent.
 * As a result, the TC flag is not asserted and SW  received timeout
 * exeception. Bit1 of Vendor Spec registor is used to fix it.
 */
#define ESDHC_FLAG_MULTIBLK_NO_INT	(1 << 1)

struct pltfm_imx_data {
	int flags;
	u32 scratchpad;
	/* uhs mode for sdhc host control2 */
	unsigned char uhs_mode;
};

static inline void esdhc_clrset_le(struct sdhci_host *host, u32 mask, u32 val, int reg)
{
	void __iomem *base = host->ioaddr + (reg & ~0x3);
	u32 shift = (reg & 0x3) * 8;

	writel(((readl(base) & ~(mask << shift)) | (val << shift)), base);
}

static u32 esdhc_readl_le(struct sdhci_host *host, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;

	/* fake CARD_PRESENT flag on mx25/35 */
	u32 val = readl(host->ioaddr + reg);

	/*
	 * mx6q: SDHCI_PRESENT_STATE bit 16, CINST is not functional on SD3.
	 * So move the section up, and check GPIO for card presence again in
	 * the following block.
	 */
	if (reg == SDHCI_PRESENT_STATE && cpu_is_mx6()) {
		u32 fsl_prss = readl(host->ioaddr + SDHCI_PRESENT_STATE);
		/* save the least 20 bits */
		val |= fsl_prss & 0x000FFFFF;
		/* move dat[0-3] line signal bits */
		val |= (fsl_prss & 0x0F000000) >> 4;
		/* move cmd line signal bits */
		val |= (fsl_prss & 0x00800000) << 1;
	}

	if (unlikely((reg == SDHCI_PRESENT_STATE)
			&& (imx_data->flags & ESDHC_FLAG_GPIO_FOR_CD_WP))) {
		struct esdhc_platform_data *boarddata =
				host->mmc->parent->platform_data;

		if (boarddata && gpio_is_valid(boarddata->cd_gpio)
				&& gpio_get_value(boarddata->cd_gpio))
			/* no card, if a valid gpio says so... */
			val &= ~SDHCI_CARD_PRESENT;
		else
			/* ... in all other cases assume card is present */
			val |= SDHCI_CARD_PRESENT;
	}

	if (reg == SDHCI_INT_STATUS && cpu_is_mx6()
		&& mx6q_revision() == IMX_CHIP_REVISION_1_0) {
		/*
		 * on mx6q TO1.0, there is low possibility that
		 * DATA END interrupt comes ealier than DMA
		 * END interrupt which is conflict with standard
		 * host controller spec. In this case, read the
		 * status register again will workaround this issue.
		 */
		if ((val & SDHCI_INT_DATA_END) && \
			!(val & SDHCI_INT_DMA_END))
			val = readl(host->ioaddr + reg);
	} else if (reg == SDHCI_CAPABILITIES_1 && cpu_is_mx6()) {
		/*
		 * on mx6q, no cap_1 available, fake one.
		 */
		val = SDHCI_SUPPORT_DDR50 | SDHCI_SUPPORT_SDR104 | \
			  SDHCI_SUPPORT_SDR50;
	} else if (reg == SDHCI_MAX_CURRENT && cpu_is_mx6()) {
		/*
		 * on mx6q, no max current available, fake one.
		 */
		val = 0;
		val |= 0xFF << SDHCI_MAX_CURRENT_330_SHIFT;
		val |= 0xFF << SDHCI_MAX_CURRENT_300_SHIFT;
		val |= 0xFF << SDHCI_MAX_CURRENT_180_SHIFT;
	}

	return val;
}

static void esdhc_writel_le(struct sdhci_host *host, u32 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 data;

	if (unlikely((reg == SDHCI_INT_ENABLE || reg == SDHCI_SIGNAL_ENABLE))) {
		if (imx_data->flags & ESDHC_FLAG_GPIO_FOR_CD_WP)
			/*
			 * these interrupts won't work with a custom
			 * card_detect gpio (only applied to mx25/35)
			 */
			val &= ~(SDHCI_INT_CARD_REMOVE | \
				SDHCI_INT_CARD_INSERT);

		if (!(val & SDHCI_INT_CARD_INT) && cpu_is_mx6()
			&& mx6q_revision() == IMX_CHIP_REVISION_1_0)
			/*
			 * write 1 to clear card interrupt status bit
			 * (only applied to mx6q TO1.0)
			 * uSDHC used for mx6q has such problem which is
			 * not consistant with standard host controller
			 * definition.
			 * eSDHC used for mx25/35/50/51/53 does not have
			 * such problem.
			 */
			writel(SDHCI_INT_CARD_INT, \
				host->ioaddr + SDHCI_INT_STATUS);

		if (val & SDHCI_INT_CARD_INT && !cpu_is_mx6()) {
			/*
			 * clear D3CD bit and set D3CD bit to avoid
			 * losing card interrupt
			 * (applied to all processors except mx6q)
			 * eSDHC controller used for mx25/35/50/51/53
			 * has such issue, so that we need to do following
			 * operation to avoid losing card interrupt.
			 * uSDCH controller used for mx6q and after won't
			 * have such problem.
			 */
			data = readl(host->ioaddr + SDHCI_HOST_CONTROL);
			data &= ~SDHCI_CTRL_D3CD;
			writel(data, host->ioaddr + SDHCI_HOST_CONTROL);
			data |= SDHCI_CTRL_D3CD;
			writel(data, host->ioaddr + SDHCI_HOST_CONTROL);
		}
	}

	if (unlikely((imx_data->flags & ESDHC_FLAG_MULTIBLK_NO_INT)
				&& (reg == SDHCI_INT_STATUS)
				&& (val & SDHCI_INT_DATA_END))) {
			u32 v;
			v = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
			v &= ~SDHCI_VENDOR_SPEC_SDIO_QUIRK;
			writel(v, host->ioaddr + SDHCI_VENDOR_SPEC);
	}

	writel(val, host->ioaddr + reg);
}

static u16 esdhc_readw_le(struct sdhci_host *host, int reg)
{
	u16 ret;
	u32 val;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;

	if (unlikely(reg == SDHCI_HOST_VERSION))
		reg ^= 2;

	switch (reg) {
	case SDHCI_HOST_CONTROL2:
		ret = 0;
		/* collect bit info from several regs */
		val = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
		ret |= (val & SDHCI_VENDOR_SPEC_VSELECT)
			? SDHCI_CTRL_VDD_180 : 0;

		val = readl(host->ioaddr + SDHCI_MIX_CTRL);
		ret |= (val & SDHCI_MIX_CTRL_EXE_TUNE)
			? SDHCI_CTRL_EXEC_TUNING : 0;
		ret |= (val & SDHCI_MIX_CTRL_SMPCLK_SEL)
			? 0 : SDHCI_CTRL_TUNED_CLK ;
		ret |= SDHCI_CTRL_UHS_MASK & imx_data->uhs_mode;
		/* no preset enable available  */
		ret &= ~SDHCI_CTRL_PRESET_VAL_ENABLE;

		return ret;
	}

	return readw(host->ioaddr + reg);
}

void esdhc_post_tuning(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 reg;

	imx_data->scratchpad &= ~SDHCI_MIX_CTRL_EXE_TUNE;
	reg = readl(host->ioaddr + SDHCI_MIX_CTRL);
	reg &= ~SDHCI_MIX_CTRL_EXE_TUNE;
	writel(reg, host->ioaddr + SDHCI_MIX_CTRL);
}

static void esdhc_reset(struct sdhci_host *host)
{
	unsigned long timeout;
	u32 reg;

	reg = readl(host->ioaddr + SDHCI_SYS_CTRL);
	reg |= 1 << SDHCI_SYS_CTRL_RSTA_LSH;
	writel(reg, host->ioaddr + SDHCI_SYS_CTRL);

	/* Wait max 100ms */
	timeout = 100;

	/* hw clears the bit when it's done */
	while (readl(host->ioaddr + SDHCI_SYS_CTRL)
			& (1 << SDHCI_SYS_CTRL_RSTA_LSH)) {
		if (timeout == 0) {
			printk(KERN_ERR "%s: Reset never completed.\n",
					mmc_hostname(host->mmc));
			return;
		}
		timeout--;
		mdelay(1);
	}
}

void esdhc_prepare_tuning(struct sdhci_host *host, u32 val)
{
	u32 reg;

	esdhc_reset(host);
	mdelay(1);

	reg = readl(host->ioaddr + SDHCI_MIX_CTRL);
	reg |= SDHCI_MIX_CTRL_EXE_TUNE | \
		SDHCI_MIX_CTRL_SMPCLK_SEL | \
		SDHCI_MIX_CTRL_FBCLK_SEL;
	writel(reg, host->ioaddr + SDHCI_MIX_CTRL);
	writel((val << 8), host->ioaddr + SDHCI_TUNE_CTRL_STATUS);
}


static void esdhc_writew_le(struct sdhci_host *host, u16 val, int reg)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct esdhc_platform_data *boarddata =
				host->mmc->parent->platform_data;
	struct pltfm_imx_data *imx_data = pltfm_host->priv;
	u32 orig_reg;

	switch (reg) {
	case SDHCI_CLOCK_CONTROL:
		orig_reg = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
		if (val & SDHCI_CLOCK_CARD_EN) {
			writel(orig_reg | SDHCI_VENDOR_SPEC_FRC_SDCLK_ON, \
				host->ioaddr + SDHCI_VENDOR_SPEC);
		} else {
			writel(orig_reg & ~SDHCI_VENDOR_SPEC_FRC_SDCLK_ON, \
				host->ioaddr + SDHCI_VENDOR_SPEC);
		}

		return;
	case SDHCI_HOST_CONTROL2:
		orig_reg = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
		if (val & SDHCI_CTRL_VDD_180) {
			orig_reg |= SDHCI_VENDOR_SPEC_VSELECT;
			writel(orig_reg, host->ioaddr + SDHCI_VENDOR_SPEC);
		} else {
			orig_reg &= ~SDHCI_VENDOR_SPEC_VSELECT;
			writel(orig_reg, host->ioaddr + SDHCI_VENDOR_SPEC);
		}

		/*
		 * FSL sdhc controls bus and signal voltage via one bit
		 * VSELECT in VENDOR_SPEC, which has been set in
		 * SDHCI_POWER_CONTROL. So we skip the SDHCI_CTRL_VDD_180
		 * here.
		 *
		 * ignore exec_tuning flag written to SDHCI_HOST_CONTROL2,
		 * tuning will be handled differently for FSL SDHC ip.
		 */
		orig_reg = readl(host->ioaddr + SDHCI_MIX_CTRL);
		orig_reg &= ~SDHCI_MIX_CTRL_SMPCLK_SEL;

		orig_reg |= (val & SDHCI_CTRL_TUNED_CLK)
			? 0 : SDHCI_MIX_CTRL_SMPCLK_SEL;

		if (val & SDHCI_CTRL_UHS_DDR50) {
			orig_reg |= SDHCI_MIX_CTRL_DDREN;
			imx_data->scratchpad |= SDHCI_MIX_CTRL_DDREN;
		} else {
			orig_reg &= ~SDHCI_MIX_CTRL_DDREN;
			imx_data->scratchpad &= ~SDHCI_MIX_CTRL_DDREN;
		}
		writel(orig_reg, host->ioaddr + SDHCI_MIX_CTRL);

		/* set clock frequency again */
		esdhc_set_clock(host, host->clock);
		imx_data->uhs_mode = val & SDHCI_CTRL_UHS_MASK;

		/* delay line setting */
		if (!boarddata->delay_line)
			return;

		if (val & SDHCI_CTRL_UHS_DDR50)
			writel((boarddata->delay_line \
					<< SDHCI_DLL_OVERRIDE_OFFSET) \
					| (1 << SDHCI_DLL_OVERRIDE_EN_OFFSET),
					host->ioaddr + SDHCI_DLL_CTRL);
		else
			writel(0, host->ioaddr + SDHCI_DLL_CTRL);

		return;
	case SDHCI_TRANSFER_MODE:
		/*
		 * Postpone this write, we must do it together with a
		 * command write that is down below.
		 */
		if ((imx_data->flags & ESDHC_FLAG_MULTIBLK_NO_INT)
				&& (host->cmd->opcode == SD_IO_RW_EXTENDED)
				&& (host->cmd->data->blocks > 1)
				&& (host->cmd->data->flags & MMC_DATA_READ)) {
			u32 v;
			v = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
			v |= SDHCI_VENDOR_SPEC_SDIO_QUIRK;
			writel(v, host->ioaddr + SDHCI_VENDOR_SPEC);
		}
		imx_data->scratchpad = val;
		return;
	case SDHCI_COMMAND:
		if ((host->cmd->opcode == MMC_STOP_TRANSMISSION)
			&& (imx_data->flags & ESDHC_FLAG_MULTIBLK_NO_INT))
			val |= SDHCI_CMD_ABORTCMD;

		writel(0x08800880, host->ioaddr + SDHCI_CAPABILITIES_1);
		if (cpu_is_mx6()) {
			imx_data->scratchpad |= \
			(readl(host->ioaddr + SDHCI_MIX_CTRL) & \
				(SDHCI_MIX_CTRL_EXE_TUNE | \
				SDHCI_MIX_CTRL_SMPCLK_SEL | \
				SDHCI_MIX_CTRL_AUTO_TUNE | \
				SDHCI_MIX_CTRL_FBCLK_SEL | \
				SDHCI_MIX_CTRL_DDREN));

			writel(imx_data->scratchpad,
				host->ioaddr + SDHCI_MIX_CTRL);

			writel(val << 16,
				host->ioaddr + SDHCI_TRANSFER_MODE);
		} else {
			writel(val << 16 | imx_data->scratchpad,
				host->ioaddr + SDHCI_TRANSFER_MODE);
		}
		return;
	case SDHCI_BLOCK_SIZE:
		val &= ~SDHCI_MAKE_BLKSZ(0x7, 0);
		break;
	}
	esdhc_clrset_le(host, 0xffff, val, reg);
}

static u8 esdhc_readb_le(struct sdhci_host *host, int reg)
{
	u8 ret;
	u32 reg_val;

	ret = 0;
	switch (reg) {
	case SDHCI_POWER_CONTROL:
		reg_val = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
		ret |= reg_val & SDHCI_VENDOR_SPEC_VSELECT
			? SDHCI_POWER_180 : SDHCI_POWER_330;
		/* could not power off */
		ret |= SDHCI_POWER_ON;
		return ret;
	case SDHCI_HOST_CONTROL:
		reg_val = readl(host->ioaddr + SDHCI_HOST_CONTROL);
		ret |= reg_val & SDHCI_PROT_CTRL_LCTL
			? SDHCI_CTRL_LED : ~SDHCI_CTRL_LED;
		ret |= (reg_val & SDHCI_PROT_CTRL_DMASEL_MASK) >> 5;
		if (SDHCI_PROT_CTRL_8BIT == (reg_val & SDHCI_PROT_CTRL_DTW)) {
			ret &= ~SDHCI_CTRL_4BITBUS;
			ret |= SDHCI_CTRL_8BITBUS;
		} else if (SDHCI_PROT_CTRL_4BIT
				== (reg_val & SDHCI_PROT_CTRL_DTW)) {
			ret &= ~SDHCI_CTRL_8BITBUS;
			ret |= SDHCI_CTRL_4BITBUS;
		} else if (SDHCI_PROT_CTRL_1BIT
				== (reg_val & SDHCI_PROT_CTRL_DTW))
			ret &= ~(SDHCI_CTRL_8BITBUS | SDHCI_CTRL_4BITBUS);
		return ret;
	case SDHCI_SOFTWARE_RESET:
		reg_val = readl(host->ioaddr + SDHCI_CLOCK_CONTROL);
		ret = reg_val >> 24;
		return ret;
	case SDHCI_RESPONSE + 3:
		reg_val = readl(host->ioaddr + SDHCI_RESPONSE);
		ret = reg_val >> 24;
		return ret;
	case SDHCI_RESPONSE + 7:
		reg_val = readl(host->ioaddr + SDHCI_RESPONSE + 4);
		ret = reg_val >> 24;
		return ret;
	case SDHCI_RESPONSE + 11:
		reg_val = readl(host->ioaddr + SDHCI_RESPONSE + 8);
		ret = reg_val >> 24;
		return ret;
	default:
		ret = 0;
		break;
	}
	return ret;
}

static void esdhc_writeb_le(struct sdhci_host *host, u8 val, int reg)
{
	u32 new_val;

	switch (reg) {
	case SDHCI_POWER_CONTROL:
		/*
		 * FSL put some DMA bits here
		 * If your board has a regulator, code should be here
		 */
		if (val == (SDHCI_POWER_ON | SDHCI_POWER_180)) {
			u32 reg;

			/* switch to 1.8V */
			reg = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
			reg |= SDHCI_VENDOR_SPEC_VSELECT;
			writel(reg, host->ioaddr + SDHCI_VENDOR_SPEC);

			/* stop sd clock */
			writel(reg & ~SDHCI_VENDOR_SPEC_FRC_SDCLK_ON, \
				host->ioaddr + SDHCI_VENDOR_SPEC);

			/* sleep at least 5ms */
			mdelay(5);

			/* restore sd clock status */
			writel(reg, host->ioaddr + SDHCI_VENDOR_SPEC);
		} else {
			u32 reg;

			reg = readl(host->ioaddr + SDHCI_VENDOR_SPEC);
			reg &= ~SDHCI_VENDOR_SPEC_VSELECT;
			writel(reg, host->ioaddr + SDHCI_VENDOR_SPEC);
		}
		return;
	case SDHCI_HOST_CONTROL:
		/* FSL messed up here, so we can just keep those three */
		new_val = val & (SDHCI_CTRL_LED);
		if (val & SDHCI_CTRL_8BITBUS) {
			new_val |= SDHCI_PROT_CTRL_8BIT;
			new_val &= ~SDHCI_PROT_CTRL_4BIT;
		} else if (val & SDHCI_CTRL_4BITBUS) {
			new_val &= ~SDHCI_PROT_CTRL_8BIT;
			new_val |= SDHCI_PROT_CTRL_4BIT;
		}
		/* ensure the endianess */
		new_val |= ESDHC_HOST_CONTROL_LE;
		/* DMA mode bits are shifted */
		new_val |= (val & SDHCI_CTRL_DMA_MASK) << 5;

		esdhc_clrset_le(host, 0xffff, new_val, reg);
		return;
	}
	esdhc_clrset_le(host, 0xff, val, reg);
}

static unsigned int esdhc_pltfm_get_max_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return clk_get_rate(pltfm_host->clk);
}

static unsigned int esdhc_pltfm_get_min_clock(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	return clk_get_rate(pltfm_host->clk) / 256 / 16;
}

static unsigned int esdhc_pltfm_get_ro(struct sdhci_host *host)
{
	struct esdhc_platform_data *boarddata = host->mmc->parent->platform_data;

	if (boarddata && gpio_is_valid(boarddata->wp_gpio))
		return gpio_get_value(boarddata->wp_gpio);
	else
		return -ENOSYS;
}

static int plt_8bit_width(struct sdhci_host *host, int width)
{
	u32 reg = readl(host->ioaddr + SDHCI_HOST_CONTROL);

	reg &= ~SDHCI_PROT_CTRL_DTW;

	if (width == MMC_BUS_WIDTH_8)
		reg |= SDHCI_PROT_CTRL_8BIT;
	else if (width == MMC_BUS_WIDTH_4)
		reg |= SDHCI_PROT_CTRL_4BIT;

	writel(reg, host->ioaddr + SDHCI_HOST_CONTROL);
	return 0;
}

static void plt_clk_ctrl(struct sdhci_host *host, bool enable)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);

	if (enable) {
		clk_enable(pltfm_host->clk);
		host->clk_status = true;
	} else {
		clk_disable(pltfm_host->clk);
		host->clk_status = false;
	}
}

static struct sdhci_ops sdhci_esdhc_ops = {
	.read_l = esdhc_readl_le,
	.read_w = esdhc_readw_le,
	.read_b = esdhc_readb_le,
	.write_l = esdhc_writel_le,
	.write_w = esdhc_writew_le,
	.write_b = esdhc_writeb_le,
	.set_clock = esdhc_set_clock,
	.get_max_clock = esdhc_pltfm_get_max_clock,
	.get_min_clock = esdhc_pltfm_get_min_clock,
	.pre_tuning = esdhc_prepare_tuning,
	.post_tuning = esdhc_post_tuning,
	.platform_8bit_width = plt_8bit_width,
	.platform_clk_ctrl = plt_clk_ctrl,
};

static irqreturn_t cd_irq(int irq, void *data)
{
	struct sdhci_host *sdhost = (struct sdhci_host *)data;
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(sdhost);
	struct pltfm_imx_data *imx_data = pltfm_host->priv;

	writel(0, sdhost->ioaddr + SDHCI_MIX_CTRL);
	writel(0, sdhost->ioaddr + SDHCI_TUNE_CTRL_STATUS);

	if (cpu_is_mx6()) {
		imx_data->scratchpad &= ~SDHCI_MIX_CTRL_DDREN;
		imx_data->scratchpad &= ~SDHCI_MIX_CTRL_FBCLK_SEL;
		imx_data->scratchpad &= ~SDHCI_MIX_CTRL_SMPCLK_SEL;
	}

	esdhc_reset(sdhost);
	mdelay(1);

	tasklet_schedule(&sdhost->card_tasklet);
	return IRQ_HANDLED;
};

static int esdhc_pltfm_init(struct sdhci_host *host, struct sdhci_pltfm_data *pdata)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct esdhc_platform_data *boarddata = host->mmc->parent->platform_data;
	struct clk *clk;
	int err;
	struct pltfm_imx_data *imx_data;
	u32 reg;

	clk = clk_get(mmc_dev(host->mmc), NULL);
	if (IS_ERR(clk)) {
		dev_err(mmc_dev(host->mmc), "clk err\n");
		return PTR_ERR(clk);
	}
	clk_enable(clk);
	pltfm_host->clk = clk;

	imx_data = kzalloc(sizeof(struct pltfm_imx_data), GFP_KERNEL);
	if (!imx_data) {
		clk_disable(pltfm_host->clk);
		clk_put(pltfm_host->clk);
		return -ENOMEM;
	}
	pltfm_host->priv = imx_data;

	if (!cpu_is_mx25())
		host->quirks |= SDHCI_QUIRK_BROKEN_TIMEOUT_VAL;

	if (cpu_is_mx25() || cpu_is_mx35())
		/* Fix errata ENGcm07207 present on i.MX25 and i.MX35 */
		host->quirks |= SDHCI_QUIRK_NO_MULTIBLOCK;

	/* write_protect can't be routed to controller, use gpio */
	sdhci_esdhc_ops.get_ro = esdhc_pltfm_get_ro;

	if (!(cpu_is_mx25() || cpu_is_mx35() || cpu_is_mx51() || cpu_is_mx6()))
		imx_data->flags |= ESDHC_FLAG_MULTIBLK_NO_INT;

	host->ocr_avail_sd = MMC_VDD_29_30 | MMC_VDD_30_31 | \
			MMC_VDD_32_33 | MMC_VDD_33_34;
	host->ocr_avail_mmc = MMC_VDD_29_30 | MMC_VDD_30_31 | \
			MMC_VDD_32_33 | MMC_VDD_33_34;

	if (boarddata->support_18v)
		host->ocr_avail_sd |= MMC_VDD_165_195;
	if (boarddata->support_8bit)
		host->mmc->caps |= MMC_CAP_8_BIT_DATA;
	if (boarddata->keep_power_at_suspend)
		host->mmc->pm_caps |= MMC_PM_KEEP_POWER;
	if (cpu_is_mx6()) {
		host->mmc->caps |= MMC_CAP_1_8V_DDR;
		host->tuning_min = SDHCI_TUNE_CTRL_MIN;
		host->tuning_max = SDHCI_TUNE_CTRL_MAX;
		host->tuning_step = SDHCI_TUNE_CTRL_STEP;
		host->clk_mgr_en = true;
	}

	reg = readl(host->ioaddr + SDHCI_MIX_CTRL);
	reg &= ~SDHCI_MIX_CTRL_DDREN;
	writel(reg, host->ioaddr + SDHCI_MIX_CTRL);
	/* disable card interrupt enable bit, and clear status bit
	 * the default value of this enable bit is 1, but it should
	 * be 0 regarding to standard host controller spec 2.1.3.
	 * if this bit is 1, it may cause some problems.
	 * there's dat1 glitch when some cards inserting into the slot,
	 * thus wrongly generate a card interrupt that will cause
	 * system panic because it lacks of sdio handler
	 * following code will solve the problem.
	 */
	reg = sdhci_readl(host, SDHCI_INT_ENABLE);
	reg &= ~SDHCI_INT_CARD_INT;
	sdhci_writel(host, reg, SDHCI_INT_ENABLE);
	sdhci_writel(host, SDHCI_INT_CARD_INT, SDHCI_INT_STATUS);

	if (boarddata) {
		/* Device is always present, e.x, populated emmc device */
		if (boarddata->always_present) {
			imx_data->flags |= ESDHC_FLAG_GPIO_FOR_CD_WP;
			host->quirks &= ~SDHCI_QUIRK_BROKEN_CARD_DETECTION;
			if (host->clk_mgr_en)
				clk_disable(pltfm_host->clk);
			return 0;
		}

		err = gpio_request_one(boarddata->wp_gpio, GPIOF_IN, "ESDHC_WP");
		if (err) {
			dev_warn(mmc_dev(host->mmc),
				"no write-protect pin available!\n");
			boarddata->wp_gpio = err;
		}

		err = gpio_request_one(boarddata->cd_gpio, GPIOF_IN, "ESDHC_CD");
		if (err) {
			dev_warn(mmc_dev(host->mmc),
				"no card-detect pin available!\n");
			goto no_card_detect_pin;
		}

		err = request_irq(gpio_to_irq(boarddata->cd_gpio), cd_irq,
				 IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
				 mmc_hostname(host->mmc), host);
		if (err) {
			dev_warn(mmc_dev(host->mmc), "request irq error\n");
			goto no_card_detect_irq;
		}

		imx_data->flags |= ESDHC_FLAG_GPIO_FOR_CD_WP;
		/* Now we have a working card_detect again */
		host->quirks &= ~SDHCI_QUIRK_BROKEN_CARD_DETECTION;
	}

	if (host->clk_mgr_en)
		clk_disable(pltfm_host->clk);
	return 0;

 no_card_detect_irq:
	gpio_free(boarddata->cd_gpio);
 no_card_detect_pin:
	boarddata->cd_gpio = err;
	kfree(imx_data);
	if (host->clk_mgr_en)
		clk_disable(pltfm_host->clk);
	return 0;
}

static void esdhc_pltfm_exit(struct sdhci_host *host)
{
	struct sdhci_pltfm_host *pltfm_host = sdhci_priv(host);
	struct esdhc_platform_data *boarddata = host->mmc->parent->platform_data;
	struct pltfm_imx_data *imx_data = pltfm_host->priv;

	if (boarddata && gpio_is_valid(boarddata->wp_gpio))
		gpio_free(boarddata->wp_gpio);

	if (boarddata && gpio_is_valid(boarddata->cd_gpio)) {
		gpio_free(boarddata->cd_gpio);

		if (!(host->quirks & SDHCI_QUIRK_BROKEN_CARD_DETECTION))
			free_irq(gpio_to_irq(boarddata->cd_gpio), host);
	}

	if (!host->clk_mgr_en)
		clk_disable(pltfm_host->clk);
	clk_put(pltfm_host->clk);
	kfree(imx_data);
}

struct sdhci_pltfm_data sdhci_esdhc_imx_pdata = {
	.quirks = ESDHC_DEFAULT_QUIRKS | SDHCI_QUIRK_BROKEN_ADMA
			| SDHCI_QUIRK_BROKEN_CARD_DETECTION
			| SDHCI_QUIRK_NO_HISPD_BIT,
	/* ADMA has issues. Might be fixable */
	.ops = &sdhci_esdhc_ops,
	.init = esdhc_pltfm_init,
	.exit = esdhc_pltfm_exit,
};
