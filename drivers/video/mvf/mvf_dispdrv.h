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
#ifndef __MVF_DISPDRV_H__
#define __MVF_DISPDRV_H__
#include <linux/fb.h>

struct mvf_dispdrv_handle {
	struct mvf_dispdrv_driver *drv;
};

struct mvf_dispdrv_setting {
	/*input-feedback parameter*/
	struct fb_info *fbi;
	int if_fmt;
	int default_bpp;
	char *dft_mode_str;

	/*feedback parameter*/
	int dev_id;
};

struct mvf_dispdrv_driver {
	const char *name;
	int (*init) (struct mvf_dispdrv_handle *, struct mvf_dispdrv_setting *);
	void (*deinit) (struct mvf_dispdrv_handle *);
	/* display driver enable function for extension */
	int (*enable) (struct mvf_dispdrv_handle *);
	/* display driver disable function, called at early part of fb_blank */
	void (*disable) (struct mvf_dispdrv_handle *);
	/* display driver setup function, called at early part of fb_set_par */
	int (*setup) (struct mvf_dispdrv_handle *, struct fb_info *fbi);
};

struct mvf_dispdrv_handle *mvf_dispdrv_register(struct mvf_dispdrv_driver *drv);
int mvf_dispdrv_unregister(struct mvf_dispdrv_handle *handle);
struct mvf_dispdrv_handle *mvf_dispdrv_gethandle(char *name,
	struct mvf_dispdrv_setting *setting);
void mvf_dispdrv_puthandle(struct mvf_dispdrv_handle *handle);
int mvf_dispdrv_setdata(struct mvf_dispdrv_handle *handle, void *data);
void *mvf_dispdrv_getdata(struct mvf_dispdrv_handle *handle);
#endif
