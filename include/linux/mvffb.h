/*
 * Copyright 2004-2012 Freescale Semiconductor, Inc. All Rights Reserved.
 */

/*
 * The code contained herein is licensed under the GNU Lesser General
 * Public License.  You may obtain a copy of the GNU Lesser General
 * Public License Version 2.1 or later at the following locations:
 *
 * http://www.opensource.org/licenses/lgpl-license.html
 * http://www.gnu.org/copyleft/lgpl.html
 */

/*
 * @file arch-mvf/   mvffb.h
 *
 * @brief Global header file for the MVF Frame buffer
 *
 * @ingroup Framebuffer
 */
#ifndef __ASM_ARCH_MVFFB_H__
#define __ASM_ARCH_MVFFB_H__

#include <linux/fb.h>

#define FB_SYNC_INV_PIX_CLK		0x80000000 /* If set Display samples data on the rising edge */
#define FB_SYNC_DATA_INVERT		0x20000000 /* If set Output to display to be negated */

enum {

	DISABLE_ALPHA_BLEND = 0,
	ENABLE_ALPHA_BLEND = 2,
};

struct mvffb_layer_alpha {
	int blend_enable;
	int alpha;
};

struct mvffb_layer_pos {
	__u16 x; //position
	__u16 y;
};

/* Custom IOCTL's to support advanced FB operations */
#define MVFFB_WAIT_FOR_VSYNC	_IOW('F', 0x20, u_int32_t)
#define MVFFB_SET_LAYER_ALPHA  _IOW('F', 0x21, struct mvffb_layer_alpha)
#define MVFFB_GET_FB_BLANK     _IOR('F', 0x22, u_int32_t)
#define MVFFB_SETUP_LAYER		_IOW('F', 0x23, struct mvffb_layer_pos)
//#define MVFFB_SET_CLR_KEY       _IOW('F', 0x22, struct mvffb_color_key)
//#define MVFFB_SET_OVERLAY_POS   _IOWR('F', 0x24, struct mvffb_pos)
//#define MVFFB_SET_LOC_ALPHA     _IOWR('F', 0x26, struct mvffb_loc_alpha)
//#define MVFFB_SET_LOC_ALP_BUF    _IOW('F', 0x27, unsigned long)
//#define MVFFB_SET_GAMMA	       _IOW('F', 0x28, struct mvffb_gamma)

//#define MVFFB_ENABLE_OVERLAY_DOUBLE_BUFFER		_IOW('F',0x2d, u_int32_t)

#ifdef __KERNEL__

extern struct fb_videomode mvffb_modedb[];
extern int mvffb_modedb_sz;

#endif				/* __KERNEL__ */
#endif
