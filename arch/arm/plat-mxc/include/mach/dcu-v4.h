/*
 * Copyright (c) 2010 Sascha Hauer <s.hauer@pengutronix.de>
 * Copyright (C) 2011 Freescale Semiconductor, Inc.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2 of the License, or (at your
 * option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of MERCHANTABILITY
 * or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
 * for more details.
 */

#ifndef __MACH_DCU_V4_H_
#define __MACH_DCU_V4_H_

#include <linux/ipu.h>


typedef struct
{
	unsigned enable:1;
	unsigned tile_en:1;
	unsigned data_sel:1;
	unsigned saftey_en:1;
	unsigned trans:8;
	unsigned bpp:4;
	unsigned rle_en:1;
	unsigned luoffs:11;
	unsigned bb:1;
	unsigned ab:2;
}dcu_layer_cfg_t;

typedef struct
{
	unsigned color_r:8;
	unsigned color_g:8;
	unsigned color_b:8;
}dcu_color_t;

/*!
 * Union of initialization parameters for a logical layer.
 */
typedef union {

	struct {
		uint32_t width;
		uint32_t height;
		uint32_t pos_x;
		uint32_t pos_y;
		uint32_t addr;
		dcu_layer_cfg_t layer_cfg;
		dcu_color_t ckmax;
		dcu_color_t ckmin;
		uint32_t tile_ver_size;
		uint32_t tile_hor_size;
		uint32_t fg_fcolor;
		uint32_t fg_bcolor;
	}layer_mem;
}dcu_layer_params_t;

/*!
 * Enumeration of DCU interrupt sources.
 */
enum dcu_irq_line {
	DCU_IRQ_VSYNC = 0,
	DCU_IRQ_UNDRUN = DCU_IRQ_VSYNC + 1,
	DCU_IRQ_LS_BF_VS  = DCU_IRQ_VSYNC + 2,
	DCU_IRQ_VS_BLANK  = DCU_IRQ_VSYNC + 3,
	DCU_IRQ_CRC_READY  = DCU_IRQ_VSYNC + 4,
	DCU_IRQ_CRC_OVERFLOW  = DCU_IRQ_VSYNC + 5,
	DCU_IRQ_P1_FIFO_LO_FLAG  = DCU_IRQ_VSYNC + 6,
	DCU_IRQ_P1_FIFO_HI_FLAG = DCU_IRQ_VSYNC + 7,
	DCU_IRQ_P2_FIFO_LO_FLAG = DCU_IRQ_VSYNC + 8,
	DCU_IRQ_P2_FIFO_HI_FLAG = DCU_IRQ_VSYNC + 9,
	DCU_IRQ_PROG_END  = DCU_IRQ_VSYNC + 10,
	/* DCU_IRQ_IPM_ERROR  = DCU_IRQ_VSYNC + 11, */
	DCU_IRQ_LYR_TRANS_FINISH = DCU_IRQ_VSYNC + 12,
	DCU_IRQ_DMA_TRANS_FINISH = DCU_IRQ_VSYNC + 14,
	DCU_IRQ_P3_FIFO_LO_FLAG = DCU_IRQ_VSYNC + 16,
	DCU_IRQ_P3_FIFO_HI_FLAG = DCU_IRQ_VSYNC + 17,
	DCU_IRQ_P4_FIFO_LO_FLAG = DCU_IRQ_VSYNC + 18,
	DCU_IRQ_P4_FIFO_HI_FLAG = DCU_IRQ_VSYNC + 19,
	DCU_IRQ_P5_FIFO_LO_FLAG = DCU_IRQ_VSYNC + 20,
	DCU_IRQ_P5_FIFO_HI_FLAG = DCU_IRQ_VSYNC + 21,
	DCU_IRQ_P6_FIFO_LO_FLAG = DCU_IRQ_VSYNC + 22,
	DCU_IRQ_P6_FIFO_HI_FLAG = DCU_IRQ_VSYNC + 23,
	DCU_IRQ_P1_EMPTY = DCU_IRQ_VSYNC + 26,
	DCU_IRQ_P2_EMPTY = DCU_IRQ_VSYNC + 27,
	DCU_IRQ_P3_EMPTY = DCU_IRQ_VSYNC + 28,
	DCU_IRQ_P4_EMPTY = DCU_IRQ_VSYNC + 29,
	DCU_IRQ_P5_EMPTY = DCU_IRQ_VSYNC + 30,
	DCU_IRQ_P6_EMPTY = DCU_IRQ_VSYNC + 31,
	DCU_IRQ_COUNT
};

enum dcu_mode
{
	DCU_OFF = 0,
	DCU_NORMAL_MODE,
	DCU_TEST_MODE,
	DCU_COLOR_BAR_MODE
};

typedef enum
{
	BPP1_CLUT = 0,
	BPP2_CLUT,
	BPP4_CLUT,
	BPP8_CLUT,
	BPP16_RGB565,
	BPP24_RGB888,
	BPP32_ARGB8888,
	BPP4_TRANSPARENCY_MODE,
	BPP8_TRANSPARENCY_MODE,
	BPP4_LUMINANCE_OFFSET_MODE,
	BPP8_LUMINANCE_OFFSET_MODE,
	BPP16_ARGB1555,
	BPP16_ARGB4444,
	BPP16_APAL8,
	YCbCr422,
	BPP_INVALID
}dcu_bpp_format;

/* TODO: Give users more options */
typedef enum
{
	ALPHA_BLEND_DISABLED = 0,
	ALPHA_BLEND_ENABLED = 2
}alpha_aa_config;

/*!
 * Bitfield of Display Interface signal polarities.
 */
typedef struct {
	unsigned interlaced:1;
	unsigned data_pol:1;	/* true = inverted */
	unsigned clk_pol:1;	/* true = rising edge */
	unsigned Hsync_pol:1;	/* true = active high */
	unsigned Vsync_pol:1;
} dcu_di_signal_cfg_t;


struct dcu_soc;
struct dcu_soc *dcu_get_soc(int id);

/* DCU Layer support */
int32_t dcu_init_layer(struct dcu_soc *dcu, uint8_t layer,
							uint32_t pixel_fmt,
							uint16_t width, uint16_t height,
							dma_addr_t phyaddr_0,
							int16_t x_pos, int16_t y_pos);
void dcu_uninit_layer(struct dcu_soc *dcu, uint8_t layer);

int32_t dcu_update_layer_buffer(struct dcu_soc *dcu, uint8_t layer,
				dma_addr_t phyaddr);

int32_t dcu_set_layer_position(struct dcu_soc *dcu, uint8_t layer, int16_t x_pos,
				int16_t y_pos);

int32_t dcu_get_layer_position(struct dcu_soc *dcu, uint8_t layer, int16_t *x_pos,
				int16_t *y_pos);

int32_t dcu_is_layer_enabled(struct dcu_soc *dcu, uint8_t layer);

int32_t dcu_enable_layer(struct dcu_soc *dcu, uint8_t layer);

int32_t dcu_disable_layer(struct dcu_soc *dcu, uint8_t layer, bool wait_for_stop);

int32_t dcu_config_layer_alpha(struct dcu_soc *dcu, uint8_t layer,
		uint8_t alpha_value, alpha_aa_config aa);

int32_t dcu_set_chroma_keying(struct dcu_soc *dcu, uint8_t layer,
		dcu_color_t color_max, dcu_color_t color_min, bool enable);

/* DCU IRQ Support */
void dcu_enable_irq(struct dcu_soc *dcu, uint32_t irq);

void dcu_disable_irq(struct dcu_soc *dcu, uint32_t irq);

void dcu_clear_irq(struct dcu_soc *dcu, uint32_t irq);

bool dcu_get_irq_status(struct dcu_soc *dcu, uint32_t irq);

int dcu_request_irq(struct dcu_soc *dcu, uint32_t irq,
		    irqreturn_t(*handler) (int, void *),
		    uint32_t irq_flags, const char *devname, void *dev_id);

void dcu_free_irq(struct dcu_soc *dcu, uint32_t irq, void *dev_id);

/* DCU device driver support */
int register_dcu_device(struct dcu_soc *dcu, int id);

void unregister_dcu_device(struct dcu_soc *dcu, int id);

/* Display API */
int32_t dcu_init_panel(struct dcu_soc *dcu, uint32_t pixel_clk,
			    uint16_t width, uint16_t height,
			    uint16_t h_start_width, uint16_t h_sync_width,
			    uint16_t h_end_width, uint16_t v_start_width,
			    uint16_t v_sync_width, uint16_t v_end_width,
			    uint32_t v_to_h_sync, dcu_di_signal_cfg_t sig);

void dcu_uninit_panel(struct dcu_soc *dcu);

void dcu_enable(struct dcu_soc *dcu);

void dcu_disable(struct dcu_soc *dcu);

/* DCU global feature support */
void dcu_set_bgnd_color(struct dcu_soc *dcu, dcu_color_t color);



/* Later releases of DCU should support these
 *
int32_t dcu_disp_set_gamma_correction(struct dcu_soc *dcu, uint8_t layer, bool enable,
				int constk[], int slopek[]);
*/

struct dcuv4_fb_platform_data {
	char				disp_dev[32];
	u32				interface_pix_fmt;
	char				*mode_str;
	int				default_bpp;
	int 			dcu_id;

	/* reserved mem */
	resource_size_t 		res_base[2];
	resource_size_t 		res_size[2];
};

struct mvf_dcuv4_platform_data {
	int rev;
	void (*pg) (int, int);
	void (*blank) (int, int);
};

#endif /* __MACH_DCU_V4_H_ */
