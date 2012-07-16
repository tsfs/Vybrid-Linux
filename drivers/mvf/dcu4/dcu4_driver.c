/*
 * Copyright 2005-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file dcu4_driver.c
 *
 * @brief This file contains the DCU driver common API functions.
 *
 * @ingroup DCU
 */
#include <linux/types.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/err.h>
#include <linux/spinlock.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/irqdesc.h>
#include <linux/clk.h>
#include <mach/clock.h>
#include <mach/hardware.h>
#include <mach/dcu-v4.h>
#include <mach/devices-common.h>
#include <asm/cacheflush.h>
#include <linux/delay.h>

#include "dcu.h"
#include "dcu4_regs.h"

/*
 * DCU Driver name
 */
#define MVF_DCU "mvf-dcuv4"

/*
 * DCU V4 limitations
 */
#define MAX_DISP_WIDTH		2048
#define MAX_DISP_HEIGHT		2048
#define MAX_DCU_LAYERS		64
#define DCU_UNBLANK			0
#define DCU_BLANK			1

static struct dcu_soc dcu_array[MVF_DCU_MAX_NUM];
static int g_dcu_hw_rev;
static int major;
static struct class *dcu_class;
static uint32_t *tcon0_ctrl1;

/* Static functions */
static irqreturn_t dcu_irq_handler(int irq, void *desc);

static dcu_bpp_format format_to_layerencoding(uint32_t fmt)
{
	switch (fmt){
	case V4L2_PIX_FMT_RGB565:
		return BPP16_RGB565;
	case V4L2_PIX_FMT_RGB24:
		return BPP24_RGB888;
	case V4L2_PIX_FMT_RGB32:
		return BPP32_ARGB8888;
	default:
		return BPP_INVALID;
	}
}

static uint8_t layerencoding_to_min_width(dcu_bpp_format layerencoding)
{
	switch (layerencoding){
	case BPP1_CLUT:
	case BPP2_CLUT:
		return 16;
	case BPP4_CLUT:
	case BPP4_LUMINANCE_OFFSET_MODE:
	case BPP4_TRANSPARENCY_MODE:
		return 8;
	case BPP8_CLUT:
	case BPP8_TRANSPARENCY_MODE:
	case BPP8_LUMINANCE_OFFSET_MODE:
	case YCbCr422:
	case BPP24_RGB888:
		return 4;
	case BPP16_RGB565:
	case BPP16_ARGB1555:
	case BPP16_ARGB4444:
	case BPP16_APAL8:
		return 2;
	case BPP32_ARGB8888:
		return 1;
	default:
		return 	BPP_INVALID;
	}
}

static void _dcu_lock(struct dcu_soc *dcu)
{
	/*TODO:Do we need the irq check?? See the flow of operations
	 * from V4L2 and FB*/
	if (!in_irq() && !in_softirq())
	{
		mutex_lock(&dcu->mutex_lock);

		if(dcu_read(dcu, DCU_UPDATE_MODE_OFFSET) & DCU_UPDATE_MODE_READREG_MASK)
		{
			int retval;
			retval = wait_for_completion_interruptible_timeout(
						&dcu->layer_transfer_complete, 1 * HZ);
			if (retval == 0) {
				dev_err(dcu->platform_dev, "MVF DCU lock: timeout\n");
			}
		}
	}
}

static void _dcu_unlock(struct dcu_soc *dcu)
{
	uint32_t reg;
	/* TODO: Make this more efficient? */
	reg = dcu_read(dcu, DCU_UPDATE_MODE_OFFSET);
	if(reg & DCU_UPDATE_MODE_MODE_MASK)
		dev_err(dcu->platform_dev, "MVF DCU configured for unsupported automatic DCU update mode\n");
	else
		reg |= DCU_UPDATE_MODE_READREG_MASK;
	dcu_write(dcu, reg, DCU_UPDATE_MODE_OFFSET);

	init_completion(&dcu->layer_transfer_complete);
	dcu_clear_irq(dcu, dcu->dcu_layer_transfer_complete_irq);
	dcu_enable_irq(dcu, dcu->dcu_layer_transfer_complete_irq);

	/*TODO:Do we need the irq check?? See the flow of operations
	 * from V4L2 and FB*/
	if (!in_irq() && !in_softirq())
		mutex_unlock(&dcu->mutex_lock);
}

static void _dcu_get(struct dcu_soc *dcu)
{
	if (atomic_inc_return(&dcu->dcu_use_count) == 1)
		clk_enable(dcu->dcu_clk);
}

static void _dcu_put(struct dcu_soc *dcu)
{
	if (atomic_dec_return(&dcu->dcu_use_count) == 0)
		clk_disable(dcu->dcu_clk);
}
#if 0
static void dcu_reset(struct dcu_soc *dcu)
{
	u32 reg;
	reg = dcu_read(dcu, DCU_DCU_MODE_OFFSET);
	dcu_write(dcu, reg | DCU_DCU_MODE_DCU_SW_RESET_MASK, DCU_DCU_MODE_OFFSET);
}
#endif

static inline struct dcu_soc *pixelclk2dcu(struct clk *clk)
{
	struct dcu_soc *dcu;
	struct clk *base = clk - clk->id;

	dcu = container_of(base, struct dcu_soc, pixel_clk);

	return dcu;
}

static unsigned long _dcu_pixel_clk_get_rate(struct clk *clk)
{
	struct dcu_soc *dcu = pixelclk2dcu(clk);
	u32 div;
	u64 final_rate = clk_get_rate(clk->parent);

	_dcu_get(dcu);
	div = dcu_read(dcu, DCU_DIV_RATIO_OFFSET);
	_dcu_put(dcu);
	/* Actual value in register is div-1, so add 1 to get the real divider */
	div++;
	do_div(final_rate, div);
	return (unsigned long)final_rate;
}

static unsigned long _dcu_pixel_clk_round_rate(struct clk *clk, unsigned long rate)
{
	u64 div, final_rate;
	u32 remainder;
	u64 parent_rate = (unsigned long long)clk_get_rate(clk->parent);

	div = parent_rate;
	remainder = do_div(div, rate);
	/* Round the divider value */
	if (remainder > (rate/2))
		div++;
	if (div == 0)            /* Min DI disp clock divider is 1 */
		div = 1;
	else if (div > (DCU_DIV_RATIO_DIV_RATIO_MASK+1))
		div = DCU_DIV_RATIO_DIV_RATIO_MASK+1;

	final_rate = parent_rate;
	do_div(final_rate, div);
	return final_rate;
}

static int _dcu_pixel_clk_set_rate(struct clk *clk, unsigned long rate)
{
	u64 div, parent_rate;
	u32 remainder;
	struct dcu_soc *dcu = pixelclk2dcu(clk);

	parent_rate = (unsigned long long)clk_get_rate(clk->parent);
	div = parent_rate;
	remainder = do_div(div, rate);
	/* Round the divider value */
	if (remainder > (rate/2))
		div++;
	if (div == 0)            /* Min DI disp clock divider is 1 */
		div = 1;
	if (div > (DCU_DIV_RATIO_DIV_RATIO_MASK+1))
		return -EINVAL;

	/* While writing to the DIV_RATIO we need to subtract 1 */
	div--;
	dcu_write(dcu, (u32)div, DCU_DIV_RATIO_OFFSET);

	return 0;
}

static int _dcu_pixel_clk_enable(struct clk *clk)
{
	/* We do not have an option to enable or disable this clock */
	/* TODO: Do we need to look at the IOMUX settings to enable the clock
	 * to the pin/
	 */
	return 0;
}

static void _dcu_pixel_clk_disable(struct clk *clk)
{
	/* We do not have an option to enable or disable this clock */
	/* TODO: Do we need to look at the IOMUX settings to enable the clock
	 * to the pin/
	 */
}

static int _dcu_pixel_clk_set_parent(struct clk *clk, struct clk *parent)
{
	/* The parent for the pixel clock is always DCU clock */
	/* However even though we cannot change this we do not have a good way
	 * of tell which DCU is the parent of this clock. So for now we will go
	 * ahead and implement this stub just for registering the parent
	 */
	/* TODO: Get rid of this function and setup the parent of this clock in
	 * a more elegant fashion
	 */
	return 0;
}

#ifdef CONFIG_CLK_DEBUG
#define __INIT_CLK_DEBUG(n)    .name = #n,
#else
#define __INIT_CLK_DEBUG(n)
#endif
struct clk dcu_pixel_clk = {
	__INIT_CLK_DEBUG(pixel_clk)
	.id = 0,
	.get_rate = _dcu_pixel_clk_get_rate,
	.set_rate = _dcu_pixel_clk_set_rate,
	.round_rate = _dcu_pixel_clk_round_rate,
	.set_parent = _dcu_pixel_clk_set_parent,
	.enable = _dcu_pixel_clk_enable,
	.disable = _dcu_pixel_clk_disable,
};

struct clk_lookup dcu_lookups[MVF_DCU_MAX_NUM] = {
	{
		.con_id = "pixel_clk",
	},
#if(MVF_DCU_MAX_NUM > 1)
	{
		.con_id = "pixel_clk",
	},
#endif
};


static int __devinit dcu_clk_setup_enable(struct dcu_soc *dcu,
		struct platform_device *pdev)
{
	char dcu_clk[] = "dcu0_clk_root";

	dcu_clk[3] += pdev->id;

	dcu->dcu_clk = clk_get(dcu->platform_dev, dcu_clk);
	if (IS_ERR(dcu->dcu_clk)) {
		dev_err(dcu->platform_dev, "clk_get failed");
		return PTR_ERR(dcu->dcu_clk);
	}
	clk_set_rate(dcu->dcu_clk,120000000);
	dev_dbg(dcu->platform_dev, "dcu_clk = %lu\n", clk_get_rate(dcu->dcu_clk));

	dcu->pixel_clk = dcu_pixel_clk;

	dcu_lookups[pdev->id].clk = &dcu->pixel_clk;
	dcu_lookups[pdev->id].dev_id = dev_name(dcu->platform_dev);

	clkdev_add(&dcu_lookups[pdev->id]);
	clk_debug_register(&dcu->pixel_clk);
	clk_enable(dcu->dcu_clk);
	clk_set_parent(&dcu->pixel_clk, dcu->dcu_clk);

	return 0;
}


/*!
 * This function is called to initialize a LCD panel. It is called by the FB driver
 * when any display related FB parameters changes or on init.
 * Currently applies only to FB0 or FB1 and not for overlay FB's
 *
 * @param		dcu				dcu handler
 *
 * @param       pixel_clk       Desired pixel clock frequency in Hz.
 *
 * @param       width           The width of panel in pixels.
 *
 * @param       height          The height of panel in pixels.
 *
 * @param       hStartWidth     The number of pixel clocks between the HSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       hSyncWidth      The width of the HSYNC signal in units of pixel
 *                              clocks.
 *
 * @param       hEndWidth       The number of pixel clocks between the end of
 *                              valid data and the HSYNC signal for next line.
 *
 * @param       vStartWidth     The number of lines between the VSYNC
 *                              signal pulse and the start of valid data.
 *
 * @param       vSyncWidth      The width of the VSYNC signal in units of lines
 *
 * @param       vEndWidth       The number of lines between the end of valid
 *                              data and the VSYNC signal for next frame.
 *
 * @param       sig             Bitfield of signal polarities for LCD interface.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t dcu_init_panel(struct dcu_soc *dcu, uint32_t pixel_clk,
			    uint16_t width, uint16_t height,
			    uint16_t h_start_width, uint16_t h_sync_width,
			    uint16_t h_end_width, uint16_t v_start_width,
			    uint16_t v_sync_width, uint16_t v_end_width,
			    uint32_t v_to_h_sync, dcu_di_signal_cfg_t sig)
{
	uint32_t reg;
	uint32_t rounded_pixel_clk;

	/* Setup panel resolution and timings*/
	dev_dbg(dcu->platform_dev, "panel size = %d x %d\n", width, height);

	if ((h_sync_width == 0) || (v_sync_width == 0) ||
		(h_start_width == 0) || (v_start_width == 0) ||
		(h_end_width == 0) || (v_end_width == 0))
	{
		dev_dbg(dcu->platform_dev, "HSYNC, VSYNC width and front/back porch width should be a minimum of 1\n");
		return -EINVAL;
	}

	if((width % 16) != 0)
	{
		dev_dbg(dcu->platform_dev, "Display width needs to be a multiple of 16\n");
		return -EINVAL;
	}

	if((width > MAX_DISP_WIDTH) || (height > MAX_DISP_HEIGHT))
	{
		dev_dbg(dcu->platform_dev, "Max width supported is %d, max height supported is %d\n",
				MAX_DISP_WIDTH, MAX_DISP_HEIGHT);
		return -EINVAL;
	}

	if (sig.interlaced) {
		dev_dbg(dcu->platform_dev, "DCU does not support interlaced format\n");
		_dcu_unlock(dcu);
		return -EINVAL;
	}

	_dcu_lock(dcu);
	reg = (uint32_t)((width/16) << DCU_DISP_SIZE_DELTA_X_SHIFT) |
				(uint32_t)(height << DCU_DISP_SIZE_DELTA_Y_SHIFT);

	dcu_write(dcu, reg, DCU_DISP_SIZE_OFFSET);

	reg = (uint32_t)(h_sync_width << DCU_HSYN_PARA_PW_H_SHIFT) |
				(uint32_t)(h_start_width << DCU_HSYN_PARA_BP_H_SHIFT) |
				(uint32_t)(h_end_width << DCU_HSYN_PARA_FP_H_SHIFT);

	dcu_write(dcu, reg, DCU_HSYN_PARA_OFFSET);

	reg = (uint32_t)(v_sync_width << DCU_VSYN_PARA_PW_V_SHIFT) |
					(uint32_t)(v_start_width << DCU_VSYN_PARA_BP_V_SHIFT) |
					(uint32_t)(v_end_width << DCU_VSYN_PARA_FP_V_SHIFT);

	dcu_write(dcu, reg, DCU_VSYN_PARA_OFFSET);

	/* Setup signal polarity data*/
	/* Read the signal polarity register first since its tied in with the PDI*/
	reg = dcu_read(dcu, DCU_SYNPOL_OFFSET);
	reg &= ~(DCU_SYNPOL_INV_HS_MASK | DCU_SYNPOL_INV_VS_MASK |
			DCU_SYNPOL_NEG_MASK | DCU_SYNPOL_INV_PXCK_MASK);

	if (!sig.Hsync_pol)
		reg |= DCU_SYNPOL_INV_HS_MASK;
	if (!sig.Vsync_pol)
		reg |= DCU_SYNPOL_INV_VS_MASK;
	if (sig.data_pol)
		reg |= DCU_SYNPOL_NEG_MASK;
	if (sig.clk_pol)
		reg |= DCU_SYNPOL_INV_PXCK_MASK;

	dcu_write(dcu, reg, DCU_SYNPOL_OFFSET);
	_dcu_unlock(dcu);

	/* Init clocking */
	dev_dbg(dcu->platform_dev, "pixel clk = %d\n", pixel_clk);

	clk_set_parent(&dcu->pixel_clk, dcu->dcu_clk);
	rounded_pixel_clk = clk_round_rate(&dcu->pixel_clk, pixel_clk);
	clk_set_rate(&dcu->pixel_clk, rounded_pixel_clk);
	dcu->display_configured = true;

	return 0;
}
EXPORT_SYMBOL(dcu_init_panel);

static void dcu_blank(struct dcu_soc *dcu, int blank)
{
	struct mvf_dcuv4_platform_data *plat_data = dcu->platform_dev->platform_data;
	struct platform_device *pdev = to_platform_device(dcu->platform_dev);
	if (plat_data->blank)
			plat_data->blank(pdev->id, blank);
}

void dcu_enable(struct dcu_soc *dcu)
{
	u32 reg;
	dcu_blank(dcu, DCU_UNBLANK);
	/* TODO: Use count and clock */
	_dcu_lock(dcu);
	/* Enable the VSYNC and HSYNC */
	reg = dcu_read(dcu, DCU_DCU_MODE_OFFSET);
	reg |= DCU_DCU_MODE_RASTER_EN_MASK;
	reg &= ~DCU_DCU_MODE_DCU_MODE_MASK;
	reg |= (DCU_NORMAL_MODE << DCU_DCU_MODE_DCU_MODE_SHIFT);
	dcu_write(dcu, reg, DCU_DCU_MODE_OFFSET);
	_dcu_unlock(dcu);
}
EXPORT_SYMBOL(dcu_enable);

void dcu_disable(struct dcu_soc *dcu)
{
	u32 reg;
	/* TODO: Use count and clock */
	_dcu_lock(dcu);
	/* Disable the VSYNC and HSYNC */
	reg = dcu_read(dcu, DCU_DCU_MODE_OFFSET);
	reg &= ~DCU_DCU_MODE_RASTER_EN_MASK;
	reg &= ~DCU_DCU_MODE_DCU_MODE_MASK;
	dcu_write(dcu, reg, DCU_DCU_MODE_OFFSET);
	_dcu_unlock(dcu);
	dcu_blank(dcu, DCU_BLANK);
}
EXPORT_SYMBOL(dcu_disable);

void dcu_uninit_panel(struct dcu_soc *dcu)
{
	_dcu_lock(dcu);

	/* TODO: Support blanking the display??? */

	_dcu_unlock(dcu);

}
EXPORT_SYMBOL(dcu_uninit_panel);

struct dcu_soc *dcu_get_soc(int id)
{
	if (id >= MVF_DCU_MAX_NUM)
		return ERR_PTR(-ENODEV);
	else if (!dcu_array[id].online)
		return ERR_PTR(-ENODEV);
	else
		return &(dcu_array[id]);
}
EXPORT_SYMBOL_GPL(dcu_get_soc);

static irqreturn_t dcu_layer_transfer_complete_irq_handler(int irq, void *dev_id)
{
	struct dcu_soc *dcu = dev_id;
	complete(&dcu->layer_transfer_complete);
	dcu_disable_irq(dcu, irq);
	return IRQ_HANDLED;
}

/*!
 * This function is called by the driver framework to initialize the DCU
 * hardware. This function performs the following operations:
 * 	1. Initialize software DCU module
 * 		- Setup use count,
* 	2. Register the IRQ handler
* 	3. Remap the IO memory
* 	4. Initialize DCU clock ??
* 	5. Reset the DCU internal state and critical registers
* 	6. Registering the DCU device driver
 *
 * @param	pdev	The device structure for the DCU passed in by the
 *			driver framework.
 *
 * @return      Returns 0 on success or negative error code on error
 */
static int __devinit dcu_probe(struct platform_device *pdev)
{
	struct mvf_dcuv4_platform_data *plat_data = pdev->dev.platform_data;
	struct dcu_soc *dcu;
	struct resource *res;
	unsigned long dcu_base;
	unsigned int layer_count;
	int ret = 0;

	if (pdev->id >= MVF_DCU_MAX_NUM)
		return -ENODEV;

	dcu = &dcu_array[pdev->id];
	memset(dcu, 0, sizeof(struct dcu_soc));

	spin_lock_init(&dcu->spin_lock);
	mutex_init(&dcu->mutex_lock);
	atomic_set(&dcu->dcu_use_count, 1);
	atomic_set(&dcu->layer_use_count, 1);

	/* Only one rev of DCU4, do not have any variants yet
	 * so store and forget */
	g_dcu_hw_rev = plat_data->rev;

	dcu->platform_dev = &pdev->dev;

	dcu->irq_generic = platform_get_irq(pdev, 0);
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);

	if (!res || dcu->irq_generic < 0) {
		ret = -ENODEV;
		goto failed_get_res;
	}

	if (request_irq(dcu->irq_generic, dcu_irq_handler, 0, pdev->name, dcu) != 0) {
		dev_err(dcu->platform_dev, "request DCU generic interrupt failed\n");
		ret = -EBUSY;
		goto failed_req_irq_generic;
	}

	dcu_base = res->start;

	dcu->dcu_base_reg = ioremap(dcu_base, SZ_8K);
	dcu->clut_tile_mem_base = ioremap(dcu_base + DCU_CLUT_OFFSET, SZ_8K);
	dcu->gamma_r_mem_base = ioremap(dcu_base + DCU_GAMMARED_OFFSET, PAGE_SIZE);
	dcu->gamma_g_mem_base = ioremap(dcu_base + DCU_GAMMAGREEN_OFFSET, PAGE_SIZE);
	dcu->gamma_b_mem_base = ioremap(dcu_base + DCU_GAMMABLUE_OFFSET, PAGE_SIZE);
	dcu->cursor_mem_base = ioremap(dcu_base + DCU_CURSOR_OFFSET, PAGE_SIZE);

	if (!dcu->dcu_base_reg || !dcu->clut_tile_mem_base || !dcu->gamma_r_mem_base ||
		!dcu->gamma_g_mem_base || !dcu->gamma_b_mem_base || !dcu->cursor_mem_base) {
		ret = -ENOMEM;
		goto failed_ioremap;
	}

	dev_dbg(dcu->platform_dev, "DCU Base Regs = %p\n", dcu->dcu_base_reg);
	dev_dbg(dcu->platform_dev, "DCU CLUT mem = %p\n", dcu->clut_tile_mem_base);
	dev_dbg(dcu->platform_dev, "DCU Gamma R mem = %p\n", dcu->gamma_r_mem_base);
	dev_dbg(dcu->platform_dev, "DCU Gamma G mem = %p\n", dcu->gamma_g_mem_base);
	dev_dbg(dcu->platform_dev, "DCU Gamma B mem = %p\n", dcu->gamma_b_mem_base);
	dev_dbg(dcu->platform_dev, "DCU Cursor mem = %p\n", dcu->cursor_mem_base);

	ret = dcu_clk_setup_enable(dcu, pdev);
	if (ret < 0) {
		dev_err(dcu->platform_dev, "dcu clk setup failed\n");
		goto failed_clk_setup;
	}

	platform_set_drvdata(pdev, dcu);

	/* dcu_reset(dcu); */

	/* Layer config registers are RAM with unknown values.
	 * Initialize layers to be off, this is a must since reset value is unknown */
	for(layer_count=0;layer_count<MAX_DCU_LAYERS;layer_count++)
	{
		dcu_write(dcu, 0, DCU_CTRLDESCLx_y_OFFSET(layer_count,DCU_CTRLDESCL_OFFSET4));
	}
	register_dcu_device(dcu, pdev->id);	
	/* TODO: Cleanup and move to TCON header file/driver */
	tcon0_ctrl1 = ioremap(TCON0_BASE + TCON_CTRL1_OFFSET, PAGE_SIZE);
	writel(TCON_CTRL1_TCON_BYPASS_MASK, tcon0_ctrl1);

	/*Setup the layer transfer IRQ */
	dcu->dcu_layer_transfer_complete_irq = DCU_IRQ_LYR_TRANS_FINISH;
	if (dcu_request_irq(dcu, dcu->dcu_layer_transfer_complete_irq, dcu_layer_transfer_complete_irq_handler, 0,
			MVF_DCU, dcu) != 0) {
		dev_err(dcu->platform_dev, "Error registering DCU irq %d\n",
			dcu->dcu_layer_transfer_complete_irq);
		goto failed_clk_setup;
	}
	dcu_disable_irq(dcu, dcu->dcu_layer_transfer_complete_irq);


	/* TODO: Why is the clock being disabled? */
	//clk_disable(dcu->dcu_clk);
	dcu->online = true;
	dcu->display_configured = false;

	return ret;

failed_clk_setup:
	iounmap(dcu->dcu_base_reg);
	iounmap(dcu->clut_tile_mem_base);
	iounmap(dcu->gamma_r_mem_base);
	iounmap(dcu->gamma_g_mem_base);
	iounmap(dcu->gamma_b_mem_base);
	iounmap(dcu->cursor_mem_base);
failed_ioremap:
	if (dcu->irq_generic)
		free_irq(dcu->irq_generic, dcu);
failed_req_irq_generic:
failed_get_res:
	return ret;
}

int __devexit dcu_remove(struct platform_device *pdev)
{
	struct dcu_soc *dcu = platform_get_drvdata(pdev);

	unregister_dcu_device(dcu, pdev->id);

	if (dcu->irq_generic)
		free_irq(dcu->irq_generic, dcu);

	clk_put(dcu->dcu_clk);

	iounmap(dcu->dcu_base_reg);
	iounmap(dcu->clut_tile_mem_base);
	iounmap(dcu->gamma_r_mem_base);
	iounmap(dcu->gamma_g_mem_base);
	iounmap(dcu->gamma_b_mem_base);
	iounmap(dcu->cursor_mem_base);
	return 0;
}

void dcu_dump_registers(struct dcu_soc *dcu)
{
	/* Display related */
	dev_dbg(dcu->platform_dev, "DCU_DISP_SIZE = \t0x%08X\n", dcu_read(dcu, DCU_DISP_SIZE_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_HSYN_PARA = \t0x%08X\n", dcu_read(dcu, DCU_HSYN_PARA_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_VSYN_PARA = \t0x%08X\n", dcu_read(dcu, DCU_VSYN_PARA_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_SYNPOL = \t0x%08X\n", dcu_read(dcu, DCU_SYNPOL_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_DIV_RATIO = \t0x%08X\n", dcu_read(dcu, DCU_DIV_RATIO_OFFSET));

	/* Global DCU related */
	dev_dbg(dcu->platform_dev, "DCU_DCU_MODE = \t0x%08X\n", dcu_read(dcu, DCU_DCU_MODE_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_BGND = \t0x%08X\n", dcu_read(dcu, DCU_BGND_OFFSET));

	/* Interrupt status */
	dev_dbg(dcu->platform_dev, "DCU_PARR_ERR_STATUS1 = \t0x%08X\n", dcu_read(dcu, DCU_PARR_ERR_STATUS1_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_PARR_ERR_STATUS2 = \t0x%08X\n", dcu_read(dcu, DCU_PARR_ERR_STATUS2_OFFSET));
	dev_dbg(dcu->platform_dev, "DCU_PARR_ERR_STATUS3 = \t0x%08X\n", dcu_read(dcu, DCU_PARR_ERR_STATUS3_OFFSET));

	/* Layer 0 info*/
	dev_dbg(dcu->platform_dev, "DCU_CTRLDESCL0_OFFSET1 = \t0x%08X\n",
			dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(0,DCU_CTRLDESCL_OFFSET1)));
	dev_dbg(dcu->platform_dev, "DCU_CTRLDESCL0_OFFSET2 = \t0x%08X\n",
				dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(0,DCU_CTRLDESCL_OFFSET2)));
	dev_dbg(dcu->platform_dev, "DCU_CTRLDESCL0_OFFSET3 = \t0x%08X\n",
				dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(0,DCU_CTRLDESCL_OFFSET3)));
	dev_dbg(dcu->platform_dev, "DCU_CTRLDESCL0_OFFSET4 = \t0x%08X\n",
				dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(0,DCU_CTRLDESCL_OFFSET4)));

}


/*!
 * This function is called to initialize buffer(s) for logical DCU layer.
 *
 * @param		dcu			dcu handler
 *
 * @param       layer       Input parameter for the logical layer ID.
 *
 * @param       pixel_fmt   Input parameter for pixel format of buffer.
 *                          Pixel format is a FOURCC ASCII code.
 *
 * @param       width       Input parameter for width of buffer in pixels.
 *
 * @param       height      Input parameter for height of buffer in pixels.
 *
 * @param       phyaddr     Input parameter buffer physical address.
  *
 * @param       u			private u offset for additional cropping,
 *							zero if not used.
 *
 * @param       v		private v offset for additional cropping,
 *						zero if not used.
 *
 * @return      Returns 0 on success or negative error code on fail
 */
int32_t dcu_init_layer(struct dcu_soc *dcu, uint8_t layer,
							uint32_t pixel_fmt,
							uint16_t width, uint16_t height,
							dma_addr_t phyaddr_0,
							int16_t x_pos, int16_t y_pos)
{
	int ret = 0;
	uint32_t reg;
	uint8_t layer_encoding_format;
	uint8_t min_width;

	dev_dbg(dcu->platform_dev, "init layer = %d\n", layer);

	/* Check for valid layer number and resolution as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		ret = -EINVAL;
		goto err;
	}

	if((width > MAX_DISP_WIDTH) || (height > MAX_DISP_HEIGHT))
	{
		dev_err(dcu->platform_dev, "Max width supported is %d, max height supported is %d\n",
				MAX_DISP_WIDTH, MAX_DISP_HEIGHT);
		ret = -EINVAL;
		goto err;
	}

	if((x_pos > (MAX_DISP_WIDTH-1) ) || (x_pos < -MAX_DISP_WIDTH) ||
			(y_pos > (MAX_DISP_HEIGHT-1)) || (y_pos < -MAX_DISP_HEIGHT))
	{
		dev_err(dcu->platform_dev, "Offset is out of range. Range for x offset is %d to %d\n \
				y offset is %d to %d\n", (MAX_DISP_WIDTH-1), -MAX_DISP_WIDTH,
				(MAX_DISP_HEIGHT-1), -MAX_DISP_HEIGHT);
		ret = -EINVAL;
		goto err;
	}

	/* Check if the DCU supports the pixel format provided by the user */
	layer_encoding_format = format_to_layerencoding(pixel_fmt);

	if(layer_encoding_format == BPP_INVALID)
	{
		dev_err(dcu->platform_dev, "Image format %d is not supported by hardware\n", pixel_fmt);
		ret = -EINVAL;
		goto err;
	}

	min_width = layerencoding_to_min_width(layer_encoding_format);
	if(width % min_width)
	{
		dev_err(dcu->platform_dev, "Width must be multiple integer multiple of the  number of\
			pixels that are represented by a 32-bit word: %d\n", min_width);
		ret = -EINVAL;
		goto err;
	}

	_dcu_get(dcu);
	_dcu_lock(dcu);

	/* Check if layer is already being used */
	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	if (reg & DCU_CTRLDESCLn_4_EN_MASK) {
		dev_warn(dcu->platform_dev, "Warning: layer already initialized %d\n", layer);
	}

	/* Write all the parameters passed in */
	reg = (uint32_t)(layer_encoding_format << DCU_CTRLDESCLn_4_BPP_SHIFT);
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));

	reg = (uint32_t)(width << DCU_CTRLDESCLn_1_WIDTH_SHIFT) |
					(uint32_t)(height << DCU_CTRLDESCLn_1_HEIGHT_SHIFT);
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET1));

	dcu_write(dcu, phyaddr_0, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET3));

	/* Default offset from display is 0,0. User will can change this using IOCTL */
	/* TODO: Check if 2's complement format will cause any issues */
	reg = (uint32_t)(y_pos << DCU_CTRLDESCLn_2_POSY_SHIFT) |
			(uint32_t)(x_pos << DCU_CTRLDESCLn_2_POSX_SHIFT);

	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET2));

	_dcu_unlock(dcu);

err:
	return ret;
}
EXPORT_SYMBOL(dcu_init_layer);

/*!
 * This function is called to uninitialize a DCU layer.
 *
 * @param	dcu		dcu handler
 * @param   layer	Input parameter for the logical layer ID to uninit.
 */
void dcu_uninit_layer(struct dcu_soc *dcu, uint8_t layer)
{
	uint32_t reg;
	uint32_t reg_offset_count;

	/* Check for valid layer numbe as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return;
	}

	_dcu_lock(dcu);
	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	if (reg & DCU_CTRLDESCLn_4_EN_MASK) {
		dev_err(dcu->platform_dev, "Disable layer first %d\n", layer);
		_dcu_unlock(dcu);
		return;
	}

	/* Clear out all layer data */
	for(reg_offset_count=0; reg_offset_count <= DCU_CTRLDESCL_OFFSET_MAX; reg_offset_count++)
		dcu_write(dcu, 0, DCU_CTRLDESCLx_y_OFFSET(layer, reg_offset_count));

	/* TODO: Do we need to decrement the usecount for each layer
	 * after we uninit it?
	 */

	/* TODO: Do we need to disable any interrupts here? */

	/* TODO: Do we need to disable the DCU/clocks once all the layers are disabled */

	/* TODO: Do we need to clear any status registers associated with this layer? */

	/* Sample code for waiting from IPU

	if(wait_for_stop)
	{
		int timeout = 50;

		dcu_write(dcu, IPUIRQ_2_MASK(IPU_IRQ_BG_SYNC_EOF),
				IPUIRQ_2_STATREG(IPU_IRQ_BG_SYNC_EOF));
		while ((dcu_read(dcu, IPUIRQ_2_STATREG(IPU_IRQ_BG_SYNC_EOF)) &
					IPUIRQ_2_MASK(IPU_IRQ_BG_SYNC_EOF)) == 0) {
			msleep(10);
			timeout -= 10;
			if (timeout <= 0) {
				dev_err(dcu->platform_dev, "warning: wait for bg sync eof timeout\n");
				break;
			}
		}
	}
	*/
	_dcu_unlock(dcu);
	_dcu_put(dcu);

	//WARN_ON(dcu->dcu_use_count < 0);
}
EXPORT_SYMBOL(dcu_uninit_layer);

/*!
 * This function is called to update the physical address of a buffer for
 * a logical DCU layer.
 *
 * @param	dcu		dcu handler
 * @param	layer	Input parameter for the logical layer ID.
 * @param   phyaddr	Input parameter buffer physical address
 * @return  Returns 0 on success or negative error code on fail
 */
int32_t dcu_update_layer_buffer(struct dcu_soc *dcu, uint8_t layer,
				dma_addr_t phyaddr)
{
	/* Check for valid layer numbe as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	_dcu_lock(dcu);

	dcu_write(dcu, phyaddr, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET3));

	_dcu_unlock(dcu);

	return 0;
}
EXPORT_SYMBOL(dcu_update_layer_buffer);


/*!
 * This function is called to initialize a buffer for logical DCU layer.
 *
 * @param	dcu		dcu handler
 * @param   layer   Input parameter for the logical layer ID.
 * @param	x_pos	x offset from display start, can be positive or negative
 * @param	y_pos	y offset from display start, can be positive or negative
 * @return  Returns 0 on success or negative error code on fail
 */
int32_t dcu_set_layer_position(struct dcu_soc *dcu, uint8_t layer, int16_t x_pos,
		int16_t y_pos)
{
	uint32_t reg;
	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	if((x_pos > (MAX_DISP_WIDTH-1) ) || (x_pos < -MAX_DISP_WIDTH) ||
			(y_pos > (MAX_DISP_HEIGHT-1)) || (y_pos < -MAX_DISP_HEIGHT))
	{
		dev_err(dcu->platform_dev, "Offset is out of range. Range for x offset is %d to %d\n \
				y offset is %d to %d\n", (MAX_DISP_WIDTH-1), -MAX_DISP_WIDTH,
				(MAX_DISP_HEIGHT-1), -MAX_DISP_HEIGHT);
		return -EINVAL;
	}

	_dcu_lock(dcu);

	/* TODO: Do we need to check 2's complement format for negative position */
	reg = (uint32_t)(y_pos << DCU_CTRLDESCLn_2_POSY_SHIFT) |
			(uint32_t)(x_pos << DCU_CTRLDESCLn_2_POSX_SHIFT);

	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET2));

	_dcu_unlock(dcu);
	return 0;
}
EXPORT_SYMBOL(dcu_set_layer_position);


int32_t dcu_get_layer_position(struct dcu_soc *dcu, uint8_t layer, int16_t *x_pos,
		int16_t *y_pos)
{
	uint32_t reg;
	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	_dcu_lock(dcu);
	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET2));
	_dcu_unlock(dcu);

	*y_pos = (reg & DCU_CTRLDESCLn_2_POSY_MASK) >> DCU_CTRLDESCLn_2_POSY_SHIFT;
	*x_pos = (reg & DCU_CTRLDESCLn_2_POSX_MASK) >> DCU_CTRLDESCLn_2_POSX_SHIFT;

	return 0;
}
EXPORT_SYMBOL(dcu_get_layer_position);

/*!
 * This function check whether a logical layer was enabled.
 *
 * @param	dcu		dcu handler
 * @param   layer   Input parameter for the logical layer ID.
 *
 * @return  This function returns 1 while request layer is enabled or
 *          0 for not enabled.
 */
int32_t dcu_is_layer_enabled(struct dcu_soc *dcu, uint8_t layer)
{
	uint32_t reg;
	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	if (reg & DCU_CTRLDESCLn_4_EN_MASK)
		return 1;

	return 0;
}
EXPORT_SYMBOL(dcu_is_layer_enabled);

/*!
 * This function enables a logical layer.
 *
 * @param	dcu		dcu handler
 * @param   layer   Input parameter for the logical layer ID.
 *
 * @return  This function returns 0 on success or negative error code on
 *          fail.
 */
int32_t dcu_enable_layer(struct dcu_soc *dcu, uint8_t layer)
{
	u32 reg;
	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	_dcu_lock(dcu);
	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	if (reg & DCU_CTRLDESCLn_4_EN_MASK)
	{
		dev_err(dcu->platform_dev, "Warning: layer already enabled %d\n", layer);
		_dcu_unlock(dcu);
		return -EACCES;
	}

	/* TODO: Do we need to check the usecount for each layer
	 * to ensure it is initialized before it is enabled?
	 */
	reg |= DCU_CTRLDESCLn_4_EN_MASK;
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));

	_dcu_unlock(dcu);
	return 0;
}
EXPORT_SYMBOL(dcu_enable_layer);

/*!
 * This function disables a logical layer.
 *
 * @param	dcu		dcu handler
 * @param	layer	Input parameter for the logical layer ID.
 *
 * @param   wait_for_stop	Flag to set whether to wait for layer end
 *                          of frame or return immediately.
 *
 * @return	This function returns 0 on success or negative error code on
 *              fail.
 */
int32_t dcu_disable_layer(struct dcu_soc *dcu, uint8_t layer, bool wait_for_stop)
{
	uint32_t reg;

	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	_dcu_lock(dcu);

	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	if (!(reg & DCU_CTRLDESCLn_4_EN_MASK))
	{
		dev_err(dcu->platform_dev, "Warning: layer already disabled %d\n", layer);
		_dcu_unlock(dcu);
		return -EACCES;
	}

	reg &= ~DCU_CTRLDESCLn_4_EN_MASK;
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));

	_dcu_unlock(dcu);

	return 0;
}
EXPORT_SYMBOL(dcu_disable_layer);

/*!
 * This function sets the DCU Background color.
 *
 * @param	dcu		dcu handler
 * @param   color   desired rgb color for background
 *
 */
void dcu_set_bgnd_color(struct dcu_soc *dcu, dcu_color_t color)
{
	uint32_t reg;

	_dcu_lock(dcu);

	reg = (uint32_t)(color.color_r << DCU_BGND_BGND_R_SHIFT) |
			(uint32_t)(color.color_g << DCU_BGND_BGND_G_SHIFT) |
			(uint32_t)(color.color_b << DCU_BGND_BGND_B_SHIFT);
	dcu_write(dcu, reg, DCU_BGND_OFFSET);

	_dcu_unlock(dcu);
}
EXPORT_SYMBOL(dcu_set_bgnd_color);

/*!
 * This function configures alpha .
 *
 * @param	dcu		dcu handler
 * @param 	layer	layer id
 * @param   alpha_value   desired layer transparency value
 * @param 	aa		mode of alpha blending
 *
 */
int32_t dcu_config_layer_alpha(struct dcu_soc *dcu, uint8_t layer,
		uint8_t alpha_value, alpha_aa_config aa)
{
	uint32_t reg;
	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	_dcu_lock(dcu);
	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	reg |= (uint32_t)(alpha_value << DCU_CTRLDESCLn_4_TRANS_SHIFT);
	reg |= (uint32_t)(aa << DCU_CTRLDESCLn_4_AB_SHIFT);
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	_dcu_unlock(dcu);

	return 0;
}
EXPORT_SYMBOL(dcu_config_layer_alpha);

/*!
 * This function sets chroma keying RGB color values
 *
 * @param	dcu		dcu handler
 * @param 	layer	layer id
 * @param 	color_max	max color value
 * @param 	color_min	min color value
 * @param 	enable		enable or disable chroma keying
 */
int32_t dcu_set_chroma_keying(struct dcu_soc *dcu, uint8_t layer,
		dcu_color_t color_max, dcu_color_t color_min, bool enable)
{
	uint32_t reg;
	/* Check for valid layer number as supported by DCU */
	if(layer >= MAX_DCU_LAYERS)
	{
		dev_err(dcu->platform_dev, "Invalid layer number %d\n", layer);
		return -EINVAL;
	}

	_dcu_lock(dcu);
	reg = (uint32_t)(color_max.color_r << DCU_BGND_BGND_R_SHIFT) |
				(uint32_t)(color_max.color_g << DCU_BGND_BGND_G_SHIFT) |
				(uint32_t)(color_max.color_b << DCU_BGND_BGND_B_SHIFT);
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET5));

	reg = (uint32_t)(color_min.color_r << DCU_BGND_BGND_R_SHIFT) |
					(uint32_t)(color_min.color_g << DCU_BGND_BGND_G_SHIFT) |
					(uint32_t)(color_min.color_b << DCU_BGND_BGND_B_SHIFT);
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET6));

	reg = dcu_read(dcu, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	if(enable)
		reg |= DCU_CTRLDESCLn_4_BB_MASK;
	else
		reg &= ~DCU_CTRLDESCLn_4_BB_MASK;
	dcu_write(dcu, reg, DCU_CTRLDESCLx_y_OFFSET(layer,DCU_CTRLDESCL_OFFSET4));
	_dcu_unlock(dcu);

	return 0;
}
EXPORT_SYMBOL(dcu_set_chroma_keying);

static irqreturn_t dcu_irq_handler(int irq, void *desc)
{
	struct dcu_soc *dcu = desc;
	uint32_t line;
	irqreturn_t result = IRQ_NONE;
	uint32_t int_stat;
	unsigned long lock_flags = 0;
	
	//Param 1 interrupts
	spin_lock_irqsave(&dcu->spin_lock, lock_flags);
	int_stat = dcu_read(dcu, DCU_PARR_ERR_STATUS1_OFFSET);
	int_stat &= ~(dcu_read(dcu, DCU_MASK_PARR_ERR_STATUS1_OFFSET));

	if (int_stat) {
		dcu_write(dcu, int_stat, DCU_PARR_ERR_STATUS1_OFFSET);
		dev_err(dcu->platform_dev,
				"DCU Error - DCU_PARR_ERR_STATUS1 = 0x%08X\n", int_stat);
		/* Disable interrupts so we only get error once */
		int_stat |=
			dcu_read(dcu, DCU_PARR_ERR_STATUS1_OFFSET);
		dcu_write(dcu, int_stat, DCU_MASK_PARR_ERR_STATUS1_OFFSET);
	}
	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	//Param 2 interrupts
	spin_lock_irqsave(&dcu->spin_lock, lock_flags);
	int_stat = dcu_read(dcu, DCU_PARR_ERR_STATUS2_OFFSET);
	int_stat &= ~(dcu_read(dcu, DCU_MASK_PARR_ERR_STATUS2_OFFSET));

	if (int_stat) {
		dcu_write(dcu, int_stat, DCU_PARR_ERR_STATUS2_OFFSET);
		dev_err(dcu->platform_dev,
				"DCU Error - DCU_PARR_ERR_STATUS2 = 0x%08X\n", int_stat);
		/* Disable interrupts so we only get error once */
		int_stat |=
			dcu_read(dcu, DCU_PARR_ERR_STATUS2_OFFSET);
		dcu_write(dcu, int_stat, DCU_MASK_PARR_ERR_STATUS2_OFFSET);
	}
	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	//Param 3 interrupts
	spin_lock_irqsave(&dcu->spin_lock, lock_flags);
	int_stat = dcu_read(dcu, DCU_PARR_ERR_STATUS3_OFFSET);
	int_stat &= ~(dcu_read(dcu, DCU_MASK_PARR_ERR_STATUS3_OFFSET));

	if (int_stat) {
		dcu_write(dcu, int_stat, DCU_PARR_ERR_STATUS3_OFFSET);
		dev_err(dcu->platform_dev,
				"DCU Error - DCU_PARR_ERR_STATUS3 = 0x%08X\n", int_stat);
		/* Disable interrupts so we only get error once */
		int_stat |=
			dcu_read(dcu, DCU_PARR_ERR_STATUS3_OFFSET);
		dcu_write(dcu, int_stat, DCU_MASK_PARR_ERR_STATUS3_OFFSET);
	}
	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	spin_lock_irqsave(&dcu->spin_lock, lock_flags);
	int_stat = dcu_read(dcu, DCU_INT_STATUS_OFFSET);
	int_stat &= ~(dcu_read(dcu, DCU_INT_MASK_OFFSET));
	dcu_write(dcu, int_stat, DCU_INT_STATUS_OFFSET);
	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);
	while ((line = ffs(int_stat)) != 0) {
		line--;
		int_stat &= ~(1UL << line);
		result |=
			dcu->irq_list[line].handler(line,
						   dcu->irq_list[line].
						   dev_id);
	}

	return result;
}

/*!
 * This function enables the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b dcu_irq_line enum.
 *
 * @param	dcu		dcu handler
 * @param   irq		Interrupt line to enable interrupt for.
 *
 */
void dcu_enable_irq(struct dcu_soc *dcu, uint32_t irq)
{
	uint32_t reg;
	unsigned long lock_flags;

	_dcu_get(dcu);

	spin_lock_irqsave(&dcu->spin_lock, lock_flags);

	/* TODO: Make this code more flexible to handle PDI and Error IRQ */
	reg = dcu_read(dcu, DCU_INT_MASK_OFFSET);
	reg &= ~DCU_IRQ_MASK(irq);
	dcu_write(dcu, reg, DCU_INT_MASK_OFFSET);

	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	_dcu_put(dcu);
}
EXPORT_SYMBOL(dcu_enable_irq);

/*!
 * This function disables the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b dcu_irq_line enum.
 *
 * @param	dcu		dcu handler
 * @param   irq     Interrupt line to disable interrupt for.
 *
 */
void dcu_disable_irq(struct dcu_soc *dcu, uint32_t irq)
{
	uint32_t reg;
	unsigned long lock_flags;

	_dcu_get(dcu);

	spin_lock_irqsave(&dcu->spin_lock, lock_flags);

	/* TODO: Make this code more flexible to handle PDI and Error IRQ */
	reg = dcu_read(dcu, DCU_INT_MASK_OFFSET);
	reg |= DCU_IRQ_MASK(irq);
	dcu_write(dcu, reg, DCU_INT_MASK_OFFSET);

	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	_dcu_put(dcu);
}
EXPORT_SYMBOL(dcu_disable_irq);

/*!
 * This function clears the interrupt for the specified interrupt line.
 * The interrupt lines are defined in \b dcu_irq_line enum.
 *
 * @param	dcu		dcu handler
 * @param       irq             Interrupt line to clear interrupt for.
 *
 */
void dcu_clear_irq(struct dcu_soc *dcu, uint32_t irq)
{
	unsigned long lock_flags;

	_dcu_get(dcu);

	spin_lock_irqsave(&dcu->spin_lock, lock_flags);

	/* TODO: Make this code more flexible to handle PDI and Error IRQ */
	dcu_write(dcu, DCU_IRQ_MASK(irq), DCU_INT_STATUS_OFFSET);

	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	_dcu_put(dcu);
}
EXPORT_SYMBOL(dcu_clear_irq);

/*!
 * This function returns the current interrupt status for the specified
 * interrupt line. The interrupt lines are defined in \b dcu_irq_line enum.
 *
 * @param	dcu		dcu handler
 * @param       irq             Interrupt line to get status for.
 *
 * @return      Returns true if the interrupt is pending/asserted or false if
 *              the interrupt is not pending.
 */
bool dcu_get_irq_status(struct dcu_soc *dcu, uint32_t irq)
{
	uint32_t reg;

	_dcu_get(dcu);

	/* TODO: Make this code more flexible to handle PDI and Error IRQ */
	/* TODO: Why does this code not have irq save? */
	reg = dcu_read(dcu, DCU_INT_STATUS_OFFSET);

	_dcu_put(dcu);

	if (reg & DCU_IRQ_MASK(irq))
		return true;
	else
		return false;
}
EXPORT_SYMBOL(dcu_get_irq_status);

/*!
 * This function registers an interrupt handler function for the specified
 * interrupt line. The interrupt lines are defined in \b dcu_irq_line enum.
 *
 * @param	dcu		dcu handler
 * @param       irq             Interrupt line to get status for.
 *
 * @param       handler         Input parameter for address of the handler
 *                              function.
 *
 * @param       irq_flags       Flags for interrupt mode. Currently not used.
 *
 * @param       devname         Input parameter for string name of driver
 *                              registering the handler.
 *
 * @param       dev_id          Input parameter for pointer of data to be
 *                              passed to the handler.
 *
 * @return      This function returns 0 on success or negative error code on
 *              fail.
 */
int dcu_request_irq(struct dcu_soc *dcu, uint32_t irq,
		    irqreturn_t(*handler) (int, void *),
		    uint32_t irq_flags, const char *devname, void *dev_id)
{
	unsigned long lock_flags;

	BUG_ON(irq >= DCU_IRQ_COUNT);

	_dcu_get(dcu);

	spin_lock_irqsave(&dcu->spin_lock, lock_flags);

	if (dcu->irq_list[irq].handler != NULL) {
		dev_err(dcu->platform_dev,
			"handler already installed on irq %d\n", irq);
		spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);
		return -EINVAL;
	}

	dcu->irq_list[irq].handler = handler;
	dcu->irq_list[irq].flags = irq_flags;
	dcu->irq_list[irq].dev_id = dev_id;
	dcu->irq_list[irq].name = devname;

	/* TODO: Make this code more flexible to handle PDI and Error IRQ */
	/* clear irq stat for previous use */
	dcu_write(dcu, DCU_IRQ_MASK(irq), DCU_INT_STATUS_OFFSET);

	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);

	_dcu_put(dcu);

	dcu_enable_irq(dcu, irq);	/* enable the interrupt */

	return 0;
}
EXPORT_SYMBOL(dcu_request_irq);

/*!
 * This function unregisters an interrupt handler for the specified interrupt
 * line. The interrupt lines are defined in \b dcu_irq_line enum.
 *
 * @param	dcu		dcu handler
 * @param       irq             Interrupt line to get status for.
 *
 * @param       dev_id          Input parameter for pointer of data to be passed
 *                              to the handler. This must match value passed to
 *                              dcu_request_irq().
 *
 */
void dcu_free_irq(struct dcu_soc *dcu, uint32_t irq, void *dev_id)
{
	unsigned long lock_flags;

	dcu_disable_irq(dcu, irq);	/* disable the interrupt */

	spin_lock_irqsave(&dcu->spin_lock, lock_flags);
	if (dcu->irq_list[irq].dev_id == dev_id)
		dcu->irq_list[irq].handler = NULL;
	spin_unlock_irqrestore(&dcu->spin_lock, lock_flags);
}
EXPORT_SYMBOL(dcu_free_irq);

static int dcu_suspend_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mvf_dcuv4_platform_data *plat_data = pdev->dev.platform_data;
	struct dcu_soc *dcu = platform_get_drvdata(pdev);

	if (atomic_read(&dcu->dcu_use_count)) {
		/* save and disable enabled layers*/

		/* save sub-modules status and disable all */


		clk_disable(dcu->dcu_clk);
	}

	if (plat_data->pg)
		plat_data->pg(pdev->id, 1);

	return 0;
}

static int dcu_resume_noirq(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct mvf_dcuv4_platform_data *plat_data = pdev->dev.platform_data;
	struct dcu_soc *dcu = platform_get_drvdata(pdev);

	if (plat_data->pg)
		plat_data->pg(pdev->id, 0);

	if (atomic_read(&dcu->dcu_use_count)) {
		clk_enable(dcu->dcu_clk);

		/* restore buf ready regs */

		/* re-enable sub-modules*/

		/* restore idamc sub addr regs */

		/* restart idma layer*/
	} else {
		_dcu_get(dcu);
//		_dcu_dmfc_init(dcu, dmfc_type_setup, 1);
//		_dcu_init_dc_mappings(dcu);
		/* Set sync refresh layers as high priority */
//		dcu_idmac_write(dcu, 0x18800001L, IDMAC_CHA_PRI(0));
		_dcu_put(dcu);
	}

	return 0;
}

static int mvf_dcu_open(struct inode *inode, struct file *file)
{
	return 0;
}

static long mvf_dcu_ioctl(struct file *file,
		unsigned int cmd, unsigned long arg)
{
	//int __user *argp = (void __user *)arg;
	int ret = 0;

	switch (cmd) {

	default:
		break;
	}
	return ret;
}

static int mvf_dcu_release(struct inode *inode, struct file *file)
{
	return 0;
}

static struct file_operations mvf_dcu_fops = {
	.owner = THIS_MODULE,
	.open = mvf_dcu_open,
	.release = mvf_dcu_release,
	.unlocked_ioctl = mvf_dcu_ioctl,
};

/*!
 * This function enables the DCU colorbar mode
 *
 * @param	dcu		dcu handler
 *
 */
void dcu_enable_colorbar_mode(struct dcu_soc *dcu)
{
	uint32_t reg;
	/* TODO: ensure clock is enabled */
	/* TODO: Do we need to check usecount? */
	dcu_blank(dcu, DCU_UNBLANK);
	_dcu_lock(dcu);
	/* Enable the VSYNC and HSYNC */
	reg = dcu_read(dcu, DCU_DCU_MODE_OFFSET);
	reg |= DCU_DCU_MODE_RASTER_EN_MASK;
	reg &= ~DCU_DCU_MODE_DCU_MODE_MASK;
	reg |= (DCU_COLOR_BAR_MODE << DCU_DCU_MODE_DCU_MODE_SHIFT);
	dcu_write(dcu, reg, DCU_DCU_MODE_OFFSET);
	_dcu_unlock(dcu);
}
EXPORT_SYMBOL(dcu_enable_colorbar_mode);

/*!
 * This function disables the DCU colorbar mode
 *
 * @param	dcu		dcu handler
 *
 */
void dcu_disable_colorbar_mode(struct dcu_soc *dcu)
{
	uint32_t reg;

	_dcu_lock(dcu);
	reg = dcu_read(dcu, DCU_DCU_MODE_OFFSET);
	reg &= ~DCU_DCU_MODE_RASTER_EN_MASK;
	reg &= ~DCU_DCU_MODE_DCU_MODE_MASK;
	reg |= (DCU_OFF << DCU_DCU_MODE_DCU_MODE_SHIFT);
	/* TODO: Do we need to check usecount? */
	dcu_write(dcu, reg, DCU_DCU_MODE_OFFSET);
	_dcu_unlock(dcu);
	dcu_blank(dcu, DCU_BLANK);

}
EXPORT_SYMBOL(dcu_disable_colorbar_mode);

static ssize_t get_dcu_colorbar_state(struct device *dev,
			      struct device_attribute *attr, char *buf)
{
	uint32_t reg;
	struct dcu_soc *dcu = container_of(dev, struct dcu_soc, dcu_cdev);

	_dcu_lock(dcu);
	reg = dcu_read(dcu, DCU_DCU_MODE_OFFSET);
	reg &= DCU_DCU_MODE_DCU_MODE_MASK;
	_dcu_unlock(dcu);

	reg = reg >> DCU_DCU_MODE_DCU_MODE_SHIFT;
	if(reg == DCU_COLOR_BAR_MODE)
		return sprintf(buf, "Colorbar mode enabled\n");
	else
		return sprintf(buf, "Colorbar mode not enabled\n");
}

static ssize_t set_dcu_colorbar_state(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t count)
{
	struct dcu_soc *dcu = container_of(dev, struct dcu_soc, dcu_cdev);
	int colorbarmode, r;

	r = kstrtoint(buf, 0, &colorbarmode);
	if (r)
		return r;

	/* TODO: Do we need to ensure DCU, display, clocks. TCON etc is enabled? */
	if(colorbarmode)
		dcu_enable_colorbar_mode(dcu);
	else
		dcu_disable_colorbar_mode(dcu);

	return count;
}
DEVICE_ATTR(dcu_colorbar_mode, 0644, get_dcu_colorbar_state, set_dcu_colorbar_state);

static struct device_attribute *dcu_sysfs_attrs[] = {
	&dev_attr_dcu_colorbar_mode,
	NULL
};

/*!
 * This function performs the following operations:
 * 	1. Register DCU as char device
 * 	2. Creates a DCU class
 * 	3. Setups up the device attributes *
 */
int register_dcu_device(struct dcu_soc *dcu, int id)
{
	int ret = 0;
	int i;
	struct device_attribute *attr;

	if (!major) {
		major = register_chrdev(0, "mvf_dcu", &mvf_dcu_fops);
		if (major < 0) {
			printk(KERN_ERR "Unable to register mvf_dcu as a char device\n");
			ret = major;
			goto register_cdev_fail;
		}

		dcu_class = class_create(THIS_MODULE, "mvf_dcu");
		if (IS_ERR(dcu_class)) {
			ret = PTR_ERR(dcu_class);
			goto dcu_class_fail;
		}

		dcu->dcu_cdev = device_create(dcu_class, NULL, MKDEV(major, 0),
				NULL, "mvf_dcu");
		if (IS_ERR(dcu->dcu_cdev)) {
			ret = PTR_ERR(dcu->dcu_cdev);
			goto dev_create_fail;
		}

		/* create device sysfs files */
		i = 0;
		while ((attr = dcu_sysfs_attrs[i++]) != NULL) {
			ret = device_create_file(dcu->dcu_cdev, attr);
			if (ret)
				dev_err(dcu->dcu_cdev, "Error %d on creating file\n", ret);
		}
		/*
		ret = device_create_file(dcu->dcu_cdev, &dcu_colorbar_mode);
		if (ret){
			dev_err(dcu->dcu_cdev, "Error %d on creating file\n", ret);
		}
		*/
		dcu->dcu_cdev->dma_mask = kmalloc(sizeof(*dcu->dcu_cdev->dma_mask), GFP_KERNEL);
		*dcu->dcu_cdev->dma_mask = DMA_BIT_MASK(32);
		dcu->dcu_cdev->coherent_dma_mask = DMA_BIT_MASK(32);
	}

	return ret;

dev_create_fail:
	if (id == 0) {
		class_destroy(dcu_class);
		unregister_chrdev(major, "mvf_dcu");
	}
dcu_class_fail:
	if (id == 0)
		unregister_chrdev(major, "mvf_dcu");
register_cdev_fail:
	return ret;
}

void unregister_dcu_device(struct dcu_soc *dcu, int id)
{
	int i;
	struct device_attribute *attr;

	if (major) {
		i = 0;
		while ((attr = dcu_sysfs_attrs[i++]) != NULL) {
			device_remove_file(dcu->dcu_cdev, attr);
		}
		device_destroy(dcu_class, MKDEV(major, 0));
		class_destroy(dcu_class);
		unregister_chrdev(major, "mvf_dcu");
		major = 0;
	}
}

static const struct dev_pm_ops mvfdcu_pm_ops = {
	.suspend_noirq = dcu_suspend_noirq,
	.resume_noirq = dcu_resume_noirq,
};

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mvfdcu_driver = {
	.driver = {
		   .name = MVF_DCU,
		   .pm = &mvfdcu_pm_ops,
		   },
	.probe = dcu_probe,
	.remove = dcu_remove,
};

static int32_t __init dcu_gen_init(void)
{
	int32_t ret;

	ret = platform_driver_register(&mvfdcu_driver);
	return 0;
}

subsys_initcall(dcu_gen_init);

static void __exit dcu_gen_uninit(void)
{
	platform_driver_unregister(&mvfdcu_driver);
}

module_exit(dcu_gen_uninit);
