/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @defgroup Framebuffer Driver for DCU4
 */

/*!
 * @file mvf_dcu4_fb.c
 *
 * @brief MVF Frame buffer driver for DCU4
 *
 * @ingroup Framebuffer
 */

/*!
 * Include files
 */
#include "mvf_dispdrv.h"
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/fb.h>
#include <linux/delay.h>
#include <linux/init.h>
#include <linux/ioport.h>
#include <linux/dma-mapping.h>
#include <linux/clk.h>
#include <linux/console.h>
#include <linux/io.h>
#include <linux/mvffb.h>
#include <linux/uaccess.h>
#include <linux/fsl_devices.h>
#include <asm/mach-types.h>
#include <mach/dcu-v4.h>
#include <linux/platform_device.h>
#include <linux/videodev2.h>

/*
 * Driver name
 */
#define MVFFB_NAME      "mvf_dcu4_fb"

/*!
 * Structure containing the MVF specific framebuffer information.
 */
//TODO: How are multiple overlays handled?
struct mvffb_info {
	int default_bpp;
	int cur_blank;
	int next_blank;
	int dcu_id;
	u32 dcu_pix_fmt;
	bool layer;
	uint32_t vsync_irq;
	struct fb_info *ovfbi[32];

	struct completion vsync_complete;

	void *dcu;
	struct mvf_dispdrv_handle *dispdrv;

	bool alpha_en;
	u32 pseudo_palette[16];
};

struct mvffb_alloc_list {
	struct list_head list;
	dma_addr_t phy_addr;
	void *cpu_addr;
	u32 size;
};

static bool g_dp_in_use[2];
LIST_HEAD(fb_alloc_list);

static uint32_t bpp_to_pixfmt(struct fb_info *fbi)
{
	//FIXME: For now Supporting 3 formats as given below
	uint32_t pixfmt = 0;

	if (fbi->var.nonstd)
		return fbi->var.nonstd;

	switch (fbi->var.bits_per_pixel) {
	case 24:
		pixfmt = V4L2_PIX_FMT_RGB24;
		break;
	case 32:
		pixfmt = V4L2_PIX_FMT_RGB32;
		break;
	case 16:
		pixfmt = V4L2_PIX_FMT_RGB565;
		break;
	}
	return pixfmt;
}

static bool dcu_usage[2];
static int dcu_set_usage(int dcu)
{
	if (dcu_usage[dcu])
		return -EBUSY;
	else
		dcu_usage[dcu] = true;
	return 0;
}

static void dcu_clear_usage(int dcu)
{
	dcu_usage[dcu] = false;
}

static struct fb_info *found_registered_fb(int layer_id, int dcu_id)
{
	int i;
	struct mvffb_info *mvf_fbi;
	struct fb_info *fbi = NULL;

	for (i = 0; i < num_registered_fb; i++) {
		mvf_fbi =
			((struct mvffb_info *)(registered_fb[i]->par));

		if ((mvf_fbi->dcu_id == dcu_id) && (registered_fb[i]->node == layer_id)) {
			fbi = registered_fb[i];
			break;
		}
	}
	return fbi;
}

static irqreturn_t mvffb_vsync_irq_handler(int irq, void *dev_id);
static int mvffb_blank(int blank, struct fb_info *info);
static int mvffb_map_video_memory(struct fb_info *fbi);
static int mvffb_unmap_video_memory(struct fb_info *fbi);

/*
 * Set fixed framebuffer parameters based on variable settings.
 *
 * @param       info     framebuffer information pointer
 */
static int mvffb_set_fix(struct fb_info *info)
{
	struct fb_fix_screeninfo *fix = &info->fix;
	struct fb_var_screeninfo *var = &info->var;

	fix->line_length = var->xres_virtual * var->bits_per_pixel / 8;

	fix->type = FB_TYPE_PACKED_PIXELS;
	fix->accel = FB_ACCEL_NONE;
	fix->visual = FB_VISUAL_TRUECOLOR;
	fix->xpanstep = 1;
	fix->ywrapstep = 1;
	fix->ypanstep = 1;

	return 0;
}

/*
 * Set framebuffer parameters and change the operating mode.
 *
 * @param       info     framebuffer information pointer
 */
static int mvffb_set_par(struct fb_info *fbi)
{
	int retval = 0;
	u32 mem_len;
	dcu_di_signal_cfg_t sig_cfg;
	struct mvffb_info *mvf_fbi = (struct mvffb_info *)fbi->par;

	printk("Framebuffer: Setting params for fb%d\n", fbi->node);
	dev_dbg(fbi->device, "Reconfiguring framebuffer\n");

	if (mvf_fbi->dispdrv && mvf_fbi->dispdrv->drv->setup) {
		retval = mvf_fbi->dispdrv->drv->setup(mvf_fbi->dispdrv, fbi);
		if (retval < 0) {
			dev_err(fbi->device, "setup error, dispdrv:%s.\n",
					mvf_fbi->dispdrv->drv->name);
			return -EINVAL;
		}
	}

	dcu_clear_irq(mvf_fbi->dcu, mvf_fbi->vsync_irq);
	dcu_disable_irq(mvf_fbi->dcu, mvf_fbi->vsync_irq);

	if(dcu_is_layer_enabled(mvf_fbi->dcu, fbi->node))
	{
		printk("Framebuffer: Disabling Layer%d\n", fbi->node);
		retval = dcu_disable_layer(mvf_fbi->dcu, fbi->node, true);
		if (retval < 0) {
			dev_err(fbi->device, "Error disabling layer, %d\n", fbi->node);
			return -EINVAL;
		}
		dcu_uninit_layer(mvf_fbi->dcu, fbi->node);
	}

	mvffb_set_fix(fbi);

	mem_len = fbi->var.yres_virtual * fbi->fix.line_length;
	if (!fbi->fix.smem_start || (mem_len > fbi->fix.smem_len)) {
		if (fbi->fix.smem_start)
			mvffb_unmap_video_memory(fbi);

		if (mvffb_map_video_memory(fbi) < 0)
			return -ENOMEM;
	}

	if (mvf_fbi->next_blank != FB_BLANK_UNBLANK)
	{
		printk("Framebuffer: Setting par, next_blank != unblank\n");
		return retval;
	}

	if (!mvf_fbi->layer) {
		printk("Framebuffer: Configuring background in set_par\n");

		memset(&sig_cfg, 0, sizeof(sig_cfg));
		if (fbi->var.vmode & FB_VMODE_INTERLACED)
			sig_cfg.interlaced = true;
		else
			sig_cfg.interlaced = false;
		if (fbi->var.sync & FB_SYNC_HOR_HIGH_ACT)
			sig_cfg.Hsync_pol = true;
		else
			sig_cfg.Hsync_pol = false;
		if (fbi->var.sync & FB_SYNC_VERT_HIGH_ACT)
			sig_cfg.Vsync_pol = true;
		else
			sig_cfg.Vsync_pol = false;
		if (fbi->var.sync & FB_SYNC_INV_PIX_CLK)
			sig_cfg.clk_pol = true;
		else
			sig_cfg.clk_pol = false;
		if (fbi->var.sync & FB_SYNC_DATA_INVERT)
			sig_cfg.data_pol = true;
		else
			sig_cfg.data_pol = false;

		dev_dbg(fbi->device, "pixclock = %ul Hz\n",
			(u32) (PICOS2KHZ(fbi->var.pixclock) * 1000UL));

		if (dcu_init_panel(mvf_fbi->dcu,
					(PICOS2KHZ(fbi->var.pixclock)) * 1000UL,
					fbi->var.xres, fbi->var.yres,
					fbi->var.left_margin,
					fbi->var.hsync_len,
					fbi->var.right_margin,
					fbi->var.upper_margin,
					fbi->var.vsync_len,
					fbi->var.lower_margin,
					0, sig_cfg) != 0) {
			dev_err(fbi->device,
				"mvffb: Error initializing panel.\n");
			return -EINVAL;
		}
		dcu_enable(mvf_fbi->dcu);
	}

	fbi->mode =
		(struct fb_videomode *)fb_match_mode(&fbi->var,
						 &fbi->modelist);

	dcu_init_layer(mvf_fbi->dcu, fbi->node, bpp_to_pixfmt(fbi), fbi->var.xres, fbi->var.yres, fbi->fix.smem_start, 0, 0);
	dcu_update_layer_buffer(mvf_fbi->dcu, fbi->node, fbi->fix.smem_start);
	dcu_enable_layer(mvf_fbi->dcu, fbi->node);
	dcu_set_layer_position(mvf_fbi->dcu, fbi->node, 0,0);

	printk("Framebuffer: FB Params are Resolution x = %d, y = %d, left_margin = %d, hsync_len= %d, right_margin = %d, upper_margin = %d, vsync_len = %d lower_margin = %d bpp = %d\n", \
			fbi->var.xres, fbi->var.yres, fbi->var.left_margin, fbi->var.hsync_len, fbi->var.right_margin, \
			fbi->var.upper_margin, fbi->var.vsync_len, fbi->var.lower_margin, fbi->var.bits_per_pixel);

	if (mvf_fbi->dispdrv && mvf_fbi->dispdrv->drv->enable) {
		retval = mvf_fbi->dispdrv->drv->enable(mvf_fbi->dispdrv);
		if (retval < 0) {
			dev_err(fbi->device, "enable error, dispdrv:%s.\n",
					mvf_fbi->dispdrv->drv->name);
			return -EINVAL;
		}
	}

	return retval;
}

/*
 * Check framebuffer variable parameters and adjust to valid values.
 *
 * @param       var      framebuffer variable parameters
 *
 * @param       info     framebuffer information pointer
 */
static int mvffb_check_var(struct fb_var_screeninfo *var, struct fb_info *info)
{
	u32 vtotal;
	u32 htotal;

/*  DCU does not support rotation, TODO: add software implementation.
	if (var->rotate > IPU_ROTATE_VERT_FLIP)
		var->rotate = IPU_ROTATE_NONE;
*/
	if (var->xres_virtual < var->xres)
		var->xres_virtual = var->xres;

/* TODO: Remove magic number 2 and replace with double buffer or not */
	if (var->yres_virtual <= var->yres)
		var->yres_virtual = var->yres * 2;

	if ((var->bits_per_pixel != 32) && (var->bits_per_pixel != 24) &&
	    (var->bits_per_pixel != 16) && (var->bits_per_pixel != 8))
		var->bits_per_pixel = 16;

	switch (var->bits_per_pixel) {
	case 8:
		var->red.length = 3;
		var->red.offset = 5;
		var->red.msb_right = 0;

		var->green.length = 3;
		var->green.offset = 2;
		var->green.msb_right = 0;

		var->blue.length = 2;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 16:
		var->red.length = 5;
		var->red.offset = 11;
		var->red.msb_right = 0;

		var->green.length = 6;
		var->green.offset = 5;
		var->green.msb_right = 0;

		var->blue.length = 5;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 24:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 0;
		var->transp.offset = 0;
		var->transp.msb_right = 0;
		break;
	case 32:
		var->red.length = 8;
		var->red.offset = 16;
		var->red.msb_right = 0;

		var->green.length = 8;
		var->green.offset = 8;
		var->green.msb_right = 0;

		var->blue.length = 8;
		var->blue.offset = 0;
		var->blue.msb_right = 0;

		var->transp.length = 8;
		var->transp.offset = 24;
		var->transp.msb_right = 0;
		break;
	}

	if (var->pixclock < 1000) {
		htotal = var->xres + var->right_margin + var->hsync_len +
		    var->left_margin;
		vtotal = var->yres + var->lower_margin + var->vsync_len +
		    var->upper_margin;
		var->pixclock = (vtotal * htotal * 6UL) / 100UL;
		var->pixclock = KHZ2PICOS(var->pixclock);
		dev_dbg(info->device,
			"pixclock set for 60Hz refresh = %u ps\n",
			var->pixclock);
	}

	var->height = -1;
	var->width = -1;
	var->grayscale = 0;

	return 0;
}

/*
 * Function to handle custom ioctls for MVF framebuffer.
 *
 * @param       inode   inode struct
 *
 * @param       file    file struct
 *
 * @param       cmd     Ioctl command to handle
 *
 * @param       arg     User pointer to command arguments
 *
 * @param       fbi     framebuffer information pointer
 */
static int mvffb_ioctl(struct fb_info *fbi, unsigned int cmd, unsigned long arg)
{
	int retval = 0;
	int __user *argp = (void __user *)arg;
	struct mvffb_info *mvf_fbi = (struct mvffb_info *)fbi->par;

	switch (cmd) {
	case MVFFB_SET_LAYER_ALPHA:
		{
			struct mvffb_layer_alpha ga;

			if (copy_from_user(&ga, (void *)arg, sizeof(ga))) {
				retval = -EFAULT;
				break;
			}

			if(dcu_config_layer_alpha(mvf_fbi->dcu,
					fbi->node,
					ga.alpha,
					ga.blend_enable)) {
				retval = -EINVAL;
				break;
			}

			printk("Framebuffer: Set Layer Alpha of %s to %d\n", fbi->fix.id, ga.alpha);
			dev_dbg(fbi->device, "Set Layer Alpha of %s to %d\n", fbi->fix.id, ga.alpha);

			break;
		}
	case FBIO_ALLOC:
		{
			int size;
			struct mvffb_alloc_list *mem;

			mem = kzalloc(sizeof(*mem), GFP_KERNEL);
			if (mem == NULL)
				return -ENOMEM;

			if (get_user(size, argp))
				return -EFAULT;

			mem->size = PAGE_ALIGN(size);

			mem->cpu_addr = dma_alloc_coherent(fbi->device, size,
							   &mem->phy_addr,
							   GFP_DMA);
			if (mem->cpu_addr == NULL) {
				kfree(mem);
				return -ENOMEM;
			}

			list_add(&mem->list, &fb_alloc_list);

			dev_dbg(fbi->device, "allocated %d bytes @ 0x%08X\n",
				mem->size, mem->phy_addr);

			if (put_user(mem->phy_addr, argp))
				return -EFAULT;

			break;
		}
	case FBIO_FREE:
		{
			unsigned long offset;
			struct mvffb_alloc_list *mem;

			if (get_user(offset, argp))
				return -EFAULT;

			retval = -EINVAL;
			list_for_each_entry(mem, &fb_alloc_list, list) {
				if (mem->phy_addr == offset) {
					list_del(&mem->list);
					dma_free_coherent(fbi->device,
							  mem->size,
							  mem->cpu_addr,
							  mem->phy_addr);
					kfree(mem);
					retval = 0;
					break;
				}
			}

			break;
		}
	case MVFFB_GET_FB_BLANK:
		{
			struct mvffb_info *mvf_fbi =
				(struct mvffb_info *)fbi->par;

			if (put_user(mvf_fbi->cur_blank, argp))
				return -EFAULT;
			break;
		}
	case MVFFB_SETUP_LAYER:
		{
			struct mvffb_layer_pos pos;
			struct fb_info *bg_fbi = NULL;
			struct mvffb_info *bg_mvffbi = NULL;

			if (copy_from_user(&pos, (void *)arg, sizeof(pos))) {
				retval = -EFAULT;
				break;
			}

			bg_fbi = found_registered_fb(0, mvf_fbi->dcu_id);
			if (bg_fbi)
				bg_mvffbi = ((struct mvffb_info *)(bg_fbi->par));

			if (bg_fbi == NULL) {
				printk("Framebuffer: Cannot find the background framebuffer\n");
				dev_err(fbi->device, "Cannot find the "
					"background framebuffer \n");
				retval = -ENOENT;
				break;
			}

			// if fb is unblank, check if the pos fit the display
			if (mvf_fbi->cur_blank == FB_BLANK_UNBLANK) {
				if (fbi->var.xres + pos.x > bg_fbi->var.xres) {
					if (bg_fbi->var.xres < fbi->var.xres)
						pos.x = 0;
					else
						pos.x = bg_fbi->var.xres - fbi->var.xres;
				}
				if (fbi->var.yres + pos.y > bg_fbi->var.yres) {
					if (bg_fbi->var.yres < fbi->var.yres)
						pos.y = 0;
					else
						pos.y = bg_fbi->var.yres - fbi->var.yres;
				}
			}

			retval = dcu_set_layer_position(mvf_fbi->dcu, fbi->node, pos.x, pos.y);
			if(retval != 0)
			{
				break;
			}

			if (copy_to_user((void *)arg, &pos, sizeof(pos))) {
				retval = -EFAULT;
				break;
			}

			break;
		}
	case MVFFB_WAIT_FOR_VSYNC:
	{
		init_completion(&mvf_fbi->vsync_complete);
		dcu_clear_irq(mvf_fbi->dcu, mvf_fbi->vsync_irq);
		dcu_enable_irq(mvf_fbi->dcu, mvf_fbi->vsync_irq);
		retval = wait_for_completion_interruptible_timeout(&mvf_fbi->vsync_complete, 1 * HZ);
		if (retval == 0) {
			dev_err(fbi->device,
				"MVFFB_WAIT_FOR_VSYNC: timeout %d\n",
				retval);
			retval = -ETIME;
		} else if (retval > 0) {
			retval = 0;
		}
		break;
	}
	default:
		retval = -EINVAL;
	}
	return retval;
}

/*
 * mvffb_blank():
 *      Blank the display.
 */
static int mvffb_blank(int blank, struct fb_info *info)
{
	struct mvffb_info *mvf_fbi = (struct mvffb_info *)info->par;
	int ret = 0;

	dev_dbg(info->device, "blank = %d\n", blank);

	if (mvf_fbi->cur_blank == blank)
		return 0;

	mvf_fbi->next_blank = blank;

	switch (blank) {
	case FB_BLANK_POWERDOWN:
	case FB_BLANK_VSYNC_SUSPEND:
	case FB_BLANK_HSYNC_SUSPEND:
	case FB_BLANK_NORMAL:
		if (mvf_fbi->dispdrv && mvf_fbi->dispdrv->drv->disable)
			mvf_fbi->dispdrv->drv->disable(mvf_fbi->dispdrv);
		//TODO: Remove / understand true. Not used in dcu driver.
		dcu_disable_layer(mvf_fbi->dcu, info->node, true);
		dcu_uninit_layer(mvf_fbi->dcu, info->node);
		dcu_uninit_panel(mvf_fbi->dcu);
		break;
	case FB_BLANK_UNBLANK:
		ret = mvffb_set_par(info);
		break;
	}
	if (!ret)
		mvf_fbi->cur_blank = blank;
	return ret;
}

/*
 * Pan or Wrap the Display
 *
 * This call looks only at xoffset, yoffset and the FB_VMODE_YWRAP flag
 *
 * @param               var     Variable screen buffer information
 * @param               info    Framebuffer information pointer
 */
static int
mvffb_pan_display(struct fb_var_screeninfo *var, struct fb_info *info)
{
	struct mvffb_info *mvf_fbi = (struct mvffb_info *)info->par;
	u_int y_bottom;
	unsigned int fr_xoff, fr_yoff, fr_w, fr_h;
	unsigned long base;
	int fb_stride, retval;

	if (info->var.yoffset == var->yoffset)
	{
		printk("Framebuffer: Offsets are same\n");
		return 0;	// No change, do nothing
	}

	// no pan display during fb blank
	if (mvf_fbi->cur_blank != FB_BLANK_UNBLANK)
	{
		return -EINVAL;
	}

	y_bottom = var->yoffset;

	if (y_bottom > info->var.yres_virtual)
	{
		return -EINVAL;
	}

	fb_stride = info->fix.line_length;

	base = info->fix.smem_start;
	fr_xoff = var->xoffset;
	fr_w = info->var.xres_virtual;
	if (!(var->vmode & FB_VMODE_YWRAP)) {
		printk("Framebuffer: Y wrap disabled\n");
		dev_dbg(info->device, "Y wrap disabled\n");
		fr_yoff = var->yoffset % info->var.yres;
		fr_h = info->var.yres;
		base += info->fix.line_length * info->var.yres *
			(var->yoffset / info->var.yres);
	} else {
		printk("Framebuffer: Y wrap enabled\n");
		dev_dbg(info->device, "Y wrap enabled\n");
		fr_yoff = var->yoffset;
		fr_h = info->var.yres_virtual;
	}
	base += fr_yoff * fb_stride + fr_xoff;

	init_completion(&mvf_fbi->vsync_complete);
	dcu_clear_irq(mvf_fbi->dcu, mvf_fbi->vsync_irq);
	dcu_enable_irq(mvf_fbi->dcu, mvf_fbi->vsync_irq);
	retval = wait_for_completion_interruptible_timeout(&mvf_fbi->vsync_complete, 1 * HZ);
	if (retval == 0) {
		dev_err(info->device, "Error - VSYNC timeout error %d", retval);
	}

	dev_dbg(info->device, "Updating %s buf address=0x%08lX\n",
		info->fix.id, base);

	if (dcu_update_layer_buffer(mvf_fbi->dcu, info->node, base) == 0) {
		printk("Framebuffer: Updated %s buf address=0x%08lX\n",
				info->fix.id, base);
		dev_dbg(info->device, "Updated %s buf address=0x%08lX\n",
			info->fix.id, base);
	} else {
		printk("Error updating buf to address=0x%08lX\n",
				base);
		dev_err(info->device,
			"Error updating buf to address=0x%08lX\n", base);
		return -EBUSY;
	}

	dev_dbg(info->device, "Update complete\n");

	info->var.yoffset = var->yoffset;
	return 0;
}

/*
 * Function to handle custom mmap for MVF framebuffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @param       vma     Pointer to vm_area_struct
 */
static int mvffb_mmap(struct fb_info *fbi, struct vm_area_struct *vma)
{
	bool found = false;
	u32 len;
	unsigned long offset = vma->vm_pgoff << PAGE_SHIFT;
	struct mvffb_alloc_list *mem;

	if (offset < fbi->fix.smem_len) {
		/* mapping framebuffer memory */
		len = fbi->fix.smem_len - offset;
		vma->vm_pgoff = (fbi->fix.smem_start + offset) >> PAGE_SHIFT;
	} else {
		list_for_each_entry(mem, &fb_alloc_list, list) {
			if (offset == mem->phy_addr) {
				found = true;
				len = mem->size;
				break;
			}
		}
		if (!found)
			return -EINVAL;
	}

	len = PAGE_ALIGN(len);
	if (vma->vm_end - vma->vm_start > len)
		return -EINVAL;

	/* make buffers bufferable */
	vma->vm_page_prot = pgprot_writecombine(vma->vm_page_prot);

	vma->vm_flags |= VM_IO | VM_RESERVED;

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    vma->vm_end - vma->vm_start, vma->vm_page_prot)) {
		dev_dbg(fbi->device, "mmap remap_pfn_range failed\n");
		return -ENOBUFS;
	}

	return 0;
}

/*!
 * This structure contains the pointers to the control functions that are
 * invoked by the core framebuffer driver to perform operations like
 * blitting, rectangle filling, copy regions and cursor definition.
 */
static struct fb_ops mvffb_ops = {
	.owner = THIS_MODULE,
	.fb_set_par = mvffb_set_par,
	.fb_check_var = mvffb_check_var,
	.fb_pan_display = mvffb_pan_display,
	.fb_ioctl = mvffb_ioctl,
	.fb_mmap = mvffb_mmap,
	//.fb_fillrect = cfb_fillrect,
	//.fb_copyarea = cfb_copyarea,
	//.fb_imageblit = cfb_imageblit,
	.fb_blank = mvffb_blank,
};

static irqreturn_t mvffb_vsync_irq_handler(int irq, void *dev_id)
{
	struct fb_info *fbi = dev_id;
	struct mvffb_info *mvf_fbi = fbi->par;

	complete(&mvf_fbi->vsync_complete);
	dcu_disable_irq(mvf_fbi->dcu, irq);
	return IRQ_HANDLED;
}

/*
 * Suspends the framebuffer and blanks the screen. Power management support
 */
static int mvffb_suspend(struct platform_device *pdev, pm_message_t state)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mvffb_info *mvf_fbi = (struct mvffb_info *)fbi->par;
	int saved_blank;

	console_lock();
	fb_set_suspend(fbi, 1);
	saved_blank = mvf_fbi->cur_blank;
	mvffb_blank(FB_BLANK_POWERDOWN, fbi);
	mvf_fbi->next_blank = saved_blank;
	console_unlock();

	return 0;
}

/*
 * Resumes the framebuffer and unblanks the screen. Power management support
 */
static int mvffb_resume(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mvffb_info *mvf_fbi = (struct mvffb_info *)fbi->par;

	console_lock();
	mvffb_blank(mvf_fbi->next_blank, fbi);
	fb_set_suspend(fbi, 0);
	console_unlock();

	return 0;
}

/*
 * Main framebuffer functions
 */

/*!
 * Allocates the DRAM memory for the frame buffer.      This buffer is remapped
 * into a non-cached, non-buffered, memory region to allow palette and pixel
 * writes to occur without flushing the cache.  Once this area is remapped,
 * all virtual memory access to the video memory should occur at the new region.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mvffb_map_video_memory(struct fb_info *fbi)
{
	if (fbi->fix.smem_len < fbi->var.yres_virtual * fbi->fix.line_length)
		fbi->fix.smem_len = fbi->var.yres_virtual *
				    fbi->fix.line_length;

	fbi->screen_base = dma_alloc_writecombine(fbi->device,
				fbi->fix.smem_len,
				(dma_addr_t *)&fbi->fix.smem_start,
				GFP_DMA);
	if (fbi->screen_base == 0) {
		dev_err(fbi->device, "Unable to allocate framebuffer memory\n");
		fbi->fix.smem_len = 0;
		fbi->fix.smem_start = 0;
		return -EBUSY;
	}

	printk("allocated fb %d @ paddr=0x%08X, size=%d.\n",
			fbi->node, (uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);
	dev_dbg(fbi->device, "allocated fb @ paddr=0x%08X, size=%d.\n",
		(uint32_t) fbi->fix.smem_start, fbi->fix.smem_len);

	fbi->screen_size = fbi->fix.smem_len;

	/* Clear the screen */
	memset((char *)fbi->screen_base, 0, fbi->fix.smem_len);

	return 0;
}

/*!
 * De-allocates the DRAM memory for the frame buffer.
 *
 * @param       fbi     framebuffer information pointer
 *
 * @return      Error code indicating success or failure
 */
static int mvffb_unmap_video_memory(struct fb_info *fbi)
{
	dma_free_writecombine(fbi->device, fbi->fix.smem_len,
			      fbi->screen_base, fbi->fix.smem_start);
	fbi->screen_base = 0;
	fbi->fix.smem_start = 0;
	fbi->fix.smem_len = 0;
	return 0;
}

/*!
 * Initializes the framebuffer information pointer. After allocating
 * sufficient memory for the framebuffer structure, the fields are
 * filled with custom information passed in from the configurable
 * structures.  This includes information such as bits per pixel,
 * color maps, screen width/height and RGBA offsets.
 *
 * @return      Framebuffer structure initialized with our information
 */
static struct fb_info *mvffb_init_fbinfo(struct device *dev, struct fb_ops *ops)
{
	struct fb_info *fbi;
	struct mvffb_info *mvffbi;

	/*
	 * Allocate sufficient memory for the fb structure
	 */
	fbi = framebuffer_alloc(sizeof(struct mvffb_info), dev);
	if (!fbi)
		return NULL;

	mvffbi = (struct mvffb_info *)fbi->par;

	fbi->var.activate = FB_ACTIVATE_NOW;

	fbi->fbops = ops;
	fbi->flags = FBINFO_MODULE;
	fbi->pseudo_palette = mvffbi->pseudo_palette;

	/*
	 * Allocate colormap
	 */
	fb_alloc_cmap(&fbi->cmap, 16, 0);

	return fbi;
}

static int mvffb_dispdrv_init(struct platform_device *pdev,
		struct fb_info *fbi)
{
	struct dcuv4_fb_platform_data *plat_data = pdev->dev.platform_data;
	struct mvffb_info *mvffbi = (struct mvffb_info *)fbi->par;
	struct mvf_dispdrv_setting setting;
	char disp_dev[32], *default_dev = "lcd";
	int ret = 0;

	setting.if_fmt = plat_data->interface_pix_fmt;
	setting.dft_mode_str = plat_data->mode_str;
	setting.default_bpp = plat_data->default_bpp;

	//TODO: What should be our default bpp?
	if (!setting.default_bpp)
		setting.default_bpp = 16;
	setting.fbi = fbi;
	if (!strlen(plat_data->disp_dev)) {
		memcpy(disp_dev, default_dev, strlen(default_dev));
		disp_dev[strlen(default_dev)] = '\0';
	} else {
		memcpy(disp_dev, plat_data->disp_dev,
				strlen(plat_data->disp_dev));
		disp_dev[strlen(plat_data->disp_dev)] = '\0';
	}

	dev_info(&pdev->dev, "Register mvf display driver %s\n", disp_dev);

	mvffbi->dispdrv = mvf_dispdrv_gethandle(disp_dev, &setting);
	if (IS_ERR(mvffbi->dispdrv)) {
		ret = PTR_ERR(mvffbi->dispdrv);
		dev_err(&pdev->dev, "NO mvf display driver found!\n");
		return ret;
	} else {
		// fix-up
		mvffbi->dcu_pix_fmt = setting.if_fmt;
		mvffbi->default_bpp = setting.default_bpp;

		// setting - set default setting of dev_id if plat_data dcu_id is DCU0
		if(!plat_data->dcu_id)
			mvffbi->dcu_id = setting.dev_id;
	}

	return ret;
}

/*
 * Parse user specified options (`video=trident:')
 * example:
 * 	orig - video=mvffb0:dev=lcd,800x480M-16@55,if=RGB565,bpp=16,noaccel
 * 	TODO: video=mvffb0:dev=lcd,if=BGRA8888,dcu=0,NEC-WQVGA,primary video=mvffb1:dev=lcd,if=BGRA8888,dcu=1,NEC-WQVGA
 * 	Proposed: video=mvfdcu0fb:BGRA8888,NEC-WQVGA dcu0_primary
 * 	Supported Below: video=mvffb0:dev=lcd,if=BGRA8888,dcu=0,NEC-WQVGA
 * 	TODO: re-visit the supported formats below to enhance the list
 */
static int mvffb_option_setup(struct platform_device *pdev)
{
	struct dcuv4_fb_platform_data *pdata = pdev->dev.platform_data;
	char *options, *opt, *fb_mode_str = NULL;
	char name[] = "mvffb0";

	name[5] += pdev->id;
	fb_get_options(name, &options);

	if (!options || !*options)
		return 0;

	while ((opt = strsep(&options, ",")) != NULL) {
		if (!*opt)
			continue;

		if (!strncmp(opt, "dev=", 4)) {
			memcpy(pdata->disp_dev, opt + 4, strlen(opt) - 4);
			pdata->disp_dev[strlen(opt) - 4] = '\0';
			continue;
		}
		if (!strncmp(opt, "if=", 3)) {
			if (!strncmp(opt+3, "RGB24", 5)) {
				pdata->interface_pix_fmt = V4L2_PIX_FMT_RGB24;
				continue;
			}
			if (!strncmp(opt+3, "RGB565", 6)) {
				pdata->interface_pix_fmt = V4L2_PIX_FMT_RGB565;
				continue;
			}
			if (!strncmp(opt+3, "RGB32", 5)) {
				pdata->interface_pix_fmt = V4L2_PIX_FMT_RGB32;
				continue;
			}

			//TODO:Support Other formats.
			/*if (!strncmp(opt+6, "BGR24", 5)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_BGR24;
				continue;
			}
			if (!strncmp(opt+3, "GBR24", 5)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_GBR24;
				continue;
			}
			if (!strncmp(opt+3, "RGB666", 6)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_RGB666;
				continue;
			}
			if (!strncmp(opt+3, "YUV444", 6)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_YUV444;
				continue;
			}
			if (!strncmp(opt+3, "LVDS666", 7)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_LVDS666;
				continue;
			}
			if (!strncmp(opt+3, "YUYV16", 6)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_YUYV;
				continue;
			}
			if (!strncmp(opt+3, "UYVY16", 6)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_UYVY;
				continue;
			}
			if (!strncmp(opt+3, "YVYU16", 6)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_YVYU;
				continue;
			}
			if (!strncmp(opt+3, "VYUY16", 6)) {
				pdata->interface_pix_fmt = IPU_PIX_FMT_VYUY;
				continue;
			}
			*/
		}

		if(!strncmp(opt,"dcu=", 4))
			pdata->dcu_id =
					simple_strtoul(opt + 4, NULL, 0);
		if (!strncmp(opt, "bpp=", 4))
			pdata->default_bpp =
				simple_strtoul(opt + 4, NULL, 0);
		else
			fb_mode_str = opt;
	}

	if (fb_mode_str)
		pdata->mode_str = fb_mode_str;

	return 0;
}

static int mvffb_register(struct fb_info *fbi)
{
	struct mvffb_info *mvffbi = (struct mvffb_info *)fbi->par;
	struct fb_videomode m;
	int ret = 0;

	char disp_id[] = "DCU0 BG LAYER";
	char layer_id[] = "DCU0 FG LAYER";

	printk("Framebuffer: mvffb register\n");

	if(!mvffbi->layer)
	{
		disp_id[3] += mvffbi->dcu_id;
		strcpy(fbi->fix.id, disp_id);
	}
	else
	{
		layer_id[3] += mvffbi->dcu_id;
		strcpy(fbi->fix.id, layer_id);
	}

	mvffb_check_var(&fbi->var, fbi);
	printk("Framebuffer: check var complete.\n");

	mvffb_set_fix(fbi);
	printk("Framebuffer: set fix complete.\n");

	/*added first mode to fbi modelist*/
	if (!fbi->modelist.next || !fbi->modelist.prev)
		INIT_LIST_HEAD(&fbi->modelist);
	fb_var_to_videomode(&m, &fbi->var);
	printk("Framebuffer: fb var to videomode complete.\n");
	fb_add_videomode(&m, &fbi->modelist);
	printk("Framebuffer: fb add videomode complete.\n");

	fbi->var.activate |= FB_ACTIVATE_FORCE;
	console_lock();
	fbi->flags |= FBINFO_MISC_USEREVENT;
	ret = fb_set_var(fbi, &fbi->var);
	printk("Framebuffer: fb set var.\n");
	fbi->flags &= ~FBINFO_MISC_USEREVENT;
	console_unlock();

	printk("Framebuffer: Is next blank unblank.\n");
	if (mvffbi->next_blank == FB_BLANK_UNBLANK) {
		console_lock();
		fb_blank(fbi, FB_BLANK_UNBLANK);
		console_unlock();
	}
	printk("Framebuffer: registering framebuffer.\n");

	ret = register_framebuffer(fbi);

	printk("Framebuffer: mvffb registered node %d\n", fbi->node);

	return ret;
}

static void mvffb_unregister(struct fb_info *fbi)
{
	struct mvffb_info *mvffbi = (struct mvffb_info *)fbi->par;

	if (mvffbi->vsync_irq)
		dcu_free_irq(mvffbi->dcu, mvffbi->vsync_irq, fbi);

	unregister_framebuffer(fbi);
}

//Setting up the overlay frame buffer and registering the same.
static int mvffb_setup_overlay(struct platform_device *pdev,
		struct fb_info *fbi_bg, struct resource *res)
{
	struct fb_info *ovfbi;
	struct mvffb_info *mvffbi_bg = (struct mvffb_info *)fbi_bg->par;
	struct mvffb_info *mvffbi_fg;
	int ret = 0;

	printk("Framebuffer: In setup overlay\n");

	ovfbi = mvffb_init_fbinfo(&pdev->dev, &mvffb_ops);
	if (!ovfbi) {
		ret = -ENOMEM;
		goto init_ovfbinfo_failed;
	}
	mvffbi_fg = (struct mvffb_info *)ovfbi->par;

	mvffbi_fg->dcu = dcu_get_soc(mvffbi_bg->dcu_id);
	if (IS_ERR(mvffbi_fg->dcu)) {
		ret = -ENODEV;
		goto get_dcu_failed;
	}
	mvffbi_fg->dcu_id = mvffbi_bg->dcu_id;
	mvffbi_fg->dcu_pix_fmt = mvffbi_bg->dcu_pix_fmt;
	mvffbi_fg->layer = true;
	mvffbi_fg->cur_blank = mvffbi_fg->next_blank = FB_BLANK_POWERDOWN;

	// Need dummy values until real panel is configured
	ovfbi->var.xres = 240;
	ovfbi->var.yres = 320;

	if (res && res->start && res->end) {
		ovfbi->fix.smem_len = res->end - res->start + 1;
		ovfbi->fix.smem_start = res->start;
		ovfbi->screen_base = ioremap(
					ovfbi->fix.smem_start,
					ovfbi->fix.smem_len);
	}

	printk("Framebuffer: Now registering overlay %d\n", ovfbi->node);
	ret = mvffb_register(ovfbi);
	if (ret < 0)
		goto register_ov_failed;

	mvffbi_bg->ovfbi[ovfbi->node] = ovfbi;

	return ret;

register_ov_failed:
get_dcu_failed:
	fb_dealloc_cmap(&ovfbi->cmap);
	framebuffer_release(ovfbi);
init_ovfbinfo_failed:
	return ret;
}

static void mvffb_unsetup_overlay(struct fb_info *fbi_bg, int layer_num)
{
	struct mvffb_info *mvffbi_bg = (struct mvffb_info *)fbi_bg->par;
	struct fb_info *ovfbi = mvffbi_bg->ovfbi[layer_num];

	printk("Framebuffer: Unsetup Overlay\n");

	mvffb_unregister(ovfbi);

	if (&ovfbi->cmap)
		fb_dealloc_cmap(&ovfbi->cmap);
	framebuffer_release(ovfbi);
}

/*!
 * Probe routine for the framebuffer driver. It is called during the
 * driver binding process. The following functions are performed in
 * this routine: Framebuffer initialization, Memory allocation and
 * mapping, Framebuffer registration, DCU initialization.
 *
 * @return      Appropriate error code to the kernel common code
 */
static int mvffb_probe(struct platform_device *pdev)
{
	//struct dcuv4_fb_platform_data *plat_data = pdev->dev.platform_data;
	struct fb_info *fbi;
	struct mvffb_info *mvffbi;
	struct resource *res;
	int ret = 0, i = 0;

	printk("Framebuffer: Probe Entered.\n");
	/*
	 * Initialize FB structures
	 */
	fbi = mvffb_init_fbinfo(&pdev->dev, &mvffb_ops);
	if (!fbi) {
		ret = -ENOMEM;
		goto init_fbinfo_failed;
	}
	printk("Framebuffer: init complete.\n");

	mvffb_option_setup(pdev);

	printk("Framebuffer: option setup complete.\n");

	mvffbi = (struct mvffb_info *)fbi->par;
	ret = mvffb_dispdrv_init(pdev, fbi);
	if (ret < 0)
		goto init_dispdrv_failed;

	ret = dcu_set_usage(mvffbi->dcu_id);
	if (ret < 0) {
		dev_err(&pdev->dev, "dcu%d already in use\n",
				mvffbi->dcu_id);
		goto dcu_in_busy;
	}

	printk("Framebuffer: disp drv init complete.\n");

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (res && res->start && res->end) {
		fbi->fix.smem_len = res->end - res->start + 1;
		fbi->fix.smem_start = res->start;
		fbi->screen_base = ioremap(fbi->fix.smem_start, fbi->fix.smem_len);
		memset(fbi->screen_base, 0, fbi->fix.smem_len);
	}

	mvffbi->dcu = dcu_get_soc(mvffbi->dcu_id);
	if (IS_ERR(mvffbi->dcu)) {
		ret = -ENODEV;
		goto get_dcu_failed;
	}

	printk("Framebuffer: got dcu soc handle.\n");

	mvffbi->vsync_irq = DCU_IRQ_DMA_TRANS_FINISH;
	if (dcu_request_irq(mvffbi->dcu, mvffbi->vsync_irq, mvffb_vsync_irq_handler, 0, MVFFB_NAME, fbi) != 0) {
		dev_err(fbi->device, "Error registering DMA TRANS FINISH irq handler.\n");
		ret = -EBUSY;
		goto irq_request_failed;
	}
	dcu_disable_irq(mvffbi->dcu, mvffbi->vsync_irq);

	printk("Framebuffer: Requested the DMA TRANS FINISH IRQ\n");

	/* first user uses DP(display processor) with alpha feature */
	if (!g_dp_in_use[mvffbi->dcu_id]) {
		mvffbi->cur_blank = mvffbi->next_blank = FB_BLANK_UNBLANK;

		if(dcu_config_layer_alpha(mvffbi->dcu, fbi->node, 0x80, ALPHA_BLEND_ENABLED)) {
						ret = -EINVAL;
						goto mvffb_register_failed;
		}
		//dcu_disp_set_color_key(mvffbi->dcu, mvffbi->dcu_ch, false, 0);

		ret = mvffb_register(fbi);
		if (ret < 0)
			goto mvffb_register_failed;

		printk("Framebuffer: registered fb with alpha complete.\n");

		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		for(i = 1; i <= (CONFIG_FB_MVF_NUM_FBS-1); i++)
		{
			printk("Framebuffer: Registering Layer %d\n", i);
			ret = mvffb_setup_overlay(pdev, fbi, res);

			if (ret < 0) {
				mvffb_unregister(fbi);
				goto mvffb_setupoverlay_failed;
			}
		}

		g_dp_in_use[mvffbi->dcu_id] = true;
	} else {
		mvffbi->cur_blank = mvffbi->next_blank = FB_BLANK_POWERDOWN;

		ret = mvffb_register(fbi);
		if (ret < 0)
			goto mvffb_register_failed;
		printk("Framebuffer: registred fb complete.\n");

	}

	platform_set_drvdata(pdev, fbi);

#ifdef CONFIG_LOGO
	printk("Framebuffer: showing logo...\n");
	fb_prepare_logo(fbi, 0);
	fb_show_logo(fbi, 0);
#endif

	return 0;

irq_request_failed:
	dcu_free_irq(mvffbi->dcu, mvffbi->vsync_irq, fbi);
mvffb_setupoverlay_failed:
mvffb_register_failed:
get_dcu_failed:
	dcu_clear_usage(mvffbi->dcu_id);
dcu_in_busy:
init_dispdrv_failed:
	fb_dealloc_cmap(&fbi->cmap);
	framebuffer_release(fbi);
init_fbinfo_failed:
	return ret;
}

static int mvffb_remove(struct platform_device *pdev)
{
	struct fb_info *fbi = platform_get_drvdata(pdev);
	struct mvffb_info *mvf_fbi = fbi->par;
	int i = 0;

	if (!fbi)
		return 0;

	mvffb_blank(FB_BLANK_POWERDOWN, fbi);
	mvffb_unregister(fbi);
	mvffb_unmap_video_memory(fbi);

	for(i = 1; i <= (CONFIG_FB_MVF_NUM_FBS-1); i++)
	{
		if (mvf_fbi->ovfbi[i]) {
			mvffb_blank(FB_BLANK_POWERDOWN, mvf_fbi->ovfbi[i]);
			mvffb_unsetup_overlay(fbi, i);
			mvffb_unmap_video_memory(mvf_fbi->ovfbi[i]);
		}
	}

	dcu_clear_usage(mvf_fbi->dcu_id);
	if (&fbi->cmap)
		fb_dealloc_cmap(&fbi->cmap);
	framebuffer_release(fbi);
	return 0;
}

/*!
 * This structure contains pointers to the power management callback functions.
 */
static struct platform_driver mvffb_driver = {
	.driver = {
		   .name = MVFFB_NAME,
		   },
	.probe = mvffb_probe,
	.remove = mvffb_remove,
	.suspend = mvffb_suspend,
	.resume = mvffb_resume,
};

/*!
 * Main entry function for the framebuffer. The function registers the power
 * management callback functions with the kernel and also registers the MVFFB
 * callback functions with the core Linux framebuffer driver \b fbmem.c
 *
 * @return      Error code indicating success or failure
 */
int __init mvffb_init(void)
{
	return platform_driver_register(&mvffb_driver);
}

void mvffb_exit(void)
{
	platform_driver_unregister(&mvffb_driver);
}

module_init(mvffb_init);
module_exit(mvffb_exit);

MODULE_AUTHOR("Freescale Semiconductor, Inc.");
MODULE_DESCRIPTION("MVF frame buffer driver");
MODULE_LICENSE("GPL");
MODULE_SUPPORTED_DEVICE("fb");
