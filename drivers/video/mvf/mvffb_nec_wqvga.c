/*
 * Copyright (C) 2011 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/mvffb.h>
#include <linux/fsl_devices.h>
#include "mvf_dispdrv.h"

struct mvf_lcdif_data {
	struct platform_device *pdev;
	struct mvf_dispdrv_handle *disp_lcdif;
};

#define DISPDRV_LCD	"lcd"

static struct fb_videomode lcdif_modedb[] = {
	{
	 /* 480x272 @ 75 Hz pixel clock - typical - 10.87Mhz*/
	 "NEC-WQVGA", 75, 480, 272, KHZ2PICOS(10870), 2, 2, 1, 1, 41, 2,
	/*Active low HSYC/VSYNC, Sample on falling edge of pxl clk, output not negated*/
	 0,
	 FB_VMODE_NONINTERLACED,
	 0,},
};

static int lcdif_modedb_sz = ARRAY_SIZE(lcdif_modedb);

static int lcdif_init(struct mvf_dispdrv_handle *disp,
	struct mvf_dispdrv_setting *setting)
{
	int ret, i;
	struct mvf_lcdif_data *lcdif = mvf_dispdrv_getdata(disp);
	struct fsl_mvf_lcd_platform_data *plat_data
			= lcdif->pdev->dev.platform_data;
	struct fb_videomode *modedb = lcdif_modedb;
	int modedb_sz = lcdif_modedb_sz;

	printk("lcd-if: init called with dcu = %d!!\n", plat_data->dcu_id);
	printk("The pixel clock in modedb, picosecs = %ld, HZ = %ld\n", KHZ2PICOS(10870), ((PICOS2KHZ(91996)) * 1000UL));

	/* use platform defined dcu */
	setting->dev_id = plat_data->dcu_id;

	ret = fb_find_mode(&setting->fbi->var, setting->fbi, setting->dft_mode_str,
				modedb, modedb_sz, NULL, setting->default_bpp);
	if (!ret) {
		fb_videomode_to_var(&setting->fbi->var, &modedb[0]);
		setting->if_fmt = plat_data->default_ifmt;
	}

	INIT_LIST_HEAD(&setting->fbi->modelist);
	for (i = 0; i < modedb_sz; i++) {
		struct fb_videomode m;
		fb_var_to_videomode(&m, &setting->fbi->var);
		if (fb_mode_is_equal(&m, &modedb[i])) {
			fb_add_videomode(&modedb[i],
					&setting->fbi->modelist);
			break;
		}
	}

	return ret;
}

void lcdif_deinit(struct mvf_dispdrv_handle *disp)
{
	/*TODO*/
}

static struct mvf_dispdrv_driver lcdif_drv = {
	.name 	= DISPDRV_LCD,
	.init 	= lcdif_init,
	.deinit	= lcdif_deinit,
};

static int mvf_lcdif_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct mvf_lcdif_data *lcdif;
	printk("lcdif: probe called!!!\n");

	lcdif = kzalloc(sizeof(struct mvf_lcdif_data), GFP_KERNEL);
	if (!lcdif) {
		ret = -ENOMEM;
		goto alloc_failed;
	}

	lcdif->pdev = pdev;
	lcdif->disp_lcdif = mvf_dispdrv_register(&lcdif_drv);
	mvf_dispdrv_setdata(lcdif->disp_lcdif, lcdif);

	dev_set_drvdata(&pdev->dev, lcdif);

	printk("lcdif: registeration and setdata complete\n");
alloc_failed:
	return ret;
}

static int mvf_lcdif_remove(struct platform_device *pdev)
{
	struct mvf_lcdif_data *lcdif = dev_get_drvdata(&pdev->dev);

	mvf_dispdrv_puthandle(lcdif->disp_lcdif);
	mvf_dispdrv_unregister(lcdif->disp_lcdif);
	kfree(lcdif);
	return 0;
}

static struct platform_driver mvf_lcdif_driver = {
	.driver = {
		   .name = "mvf_lcdif",
		   },
	.probe = mvf_lcdif_probe,
	.remove = mvf_lcdif_remove,
};

static int __init mvf_lcdif_init(void)
{
	return platform_driver_register(&mvf_lcdif_driver);
}

static void __exit mvf_lcdif_exit(void)
{
	platform_driver_unregister(&mvf_lcdif_driver);
}

module_init(mvf_lcdif_init);
module_exit(mvf_lcdif_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MVF DCUV4 LCD driver");
MODULE_LICENSE("GPL");
