/*
 * The code contained herein is licensed under the GNU General Public
 * License. You may obtain a copy of the GNU General Public License
 * Version 2 or later at the following locations:
 *
 * http://www.opensource.org/licenses/gpl-license.html
 * http://www.gnu.org/copyleft/gpl.html
 */

/*!
 * @file mvf_dispdrv.c
 * @brief mvf display driver framework.
 *
 * A display device driver could call mvf_dispdrv_register(drv) in its dev_probe() function.
 * Move all dev_probe() things into mvf_dispdrv_driver->init(), init() function should init
 * and feedback setting;
 * Move all dev_remove() things into mvf_dispdrv_driver->deinit();
 * Move all dev_suspend() things into fb_notifier for SUSPEND, if there is;
 * Move all dev_resume() things into fb_notifier for RESUME, if there is;
 *
 * DCU4 fb driver could call mvf_dispdrv_gethandle(name, setting) before a fb
 * need be added, with fbi param passing by setting, after
 * mvf_dispdrv_gethandle() return, FB driver should get the basic setting
 * about fbi info and dcuv4-hw.
 *
 * @ingroup Framebuffer
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/mutex.h>
#include <linux/slab.h>
#include <linux/err.h>
#include <linux/string.h>
#include "mvf_dispdrv.h"

static LIST_HEAD(dispdrv_list);
static DEFINE_MUTEX(dispdrv_lock);

struct mvf_dispdrv_entry {
	/* Note: drv always the first element */
	struct mvf_dispdrv_driver *drv;
	bool active;
	void *priv;
	struct list_head list;
};

struct mvf_dispdrv_handle *mvf_dispdrv_register(struct mvf_dispdrv_driver *drv)
{
	struct mvf_dispdrv_entry *new;

	mutex_lock(&dispdrv_lock);

	new = kzalloc(sizeof(struct mvf_dispdrv_entry), GFP_KERNEL);
	if (!new) {
		mutex_unlock(&dispdrv_lock);
		return ERR_PTR(-ENOMEM);
	}

	new->drv = drv;
	list_add_tail(&new->list, &dispdrv_list);

	mutex_unlock(&dispdrv_lock);

	return (struct mvf_dispdrv_handle *)new;
}
EXPORT_SYMBOL_GPL(mvf_dispdrv_register);

int mvf_dispdrv_unregister(struct mvf_dispdrv_handle *handle)
{
	struct mvf_dispdrv_entry *entry = (struct mvf_dispdrv_entry *)handle;

	if (entry) {
		mutex_lock(&dispdrv_lock);
		list_del(&entry->list);
		mutex_unlock(&dispdrv_lock);
		kfree(entry);
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(mvf_dispdrv_unregister);

struct mvf_dispdrv_handle *mvf_dispdrv_gethandle(char *name,
	struct mvf_dispdrv_setting *setting)
{
	int ret, found = 0;
	struct mvf_dispdrv_entry *entry;

	mutex_lock(&dispdrv_lock);
	list_for_each_entry(entry, &dispdrv_list, list) {
		if (!strcmp(entry->drv->name, name) && (entry->drv->init)) {
			ret = entry->drv->init((struct mvf_dispdrv_handle *)
				entry, setting);
			if (ret >= 0) {
				entry->active = true;
				found = 1;
				break;
			}
		}
	}
	mutex_unlock(&dispdrv_lock);

	return found ? (struct mvf_dispdrv_handle *)entry:ERR_PTR(-ENODEV);
}
EXPORT_SYMBOL_GPL(mvf_dispdrv_gethandle);

void mvf_dispdrv_puthandle(struct mvf_dispdrv_handle *handle)
{
	struct mvf_dispdrv_entry *entry = (struct mvf_dispdrv_entry *)handle;

	mutex_lock(&dispdrv_lock);
	if (entry && entry->active && entry->drv->deinit) {
		entry->drv->deinit(handle);
		entry->active = false;
	}
	mutex_unlock(&dispdrv_lock);

}
EXPORT_SYMBOL_GPL(mvf_dispdrv_puthandle);

int mvf_dispdrv_setdata(struct mvf_dispdrv_handle *handle, void *data)
{
	struct mvf_dispdrv_entry *entry = (struct mvf_dispdrv_entry *)handle;

	if (entry) {
		entry->priv = data;
		return 0;
	} else
		return -EINVAL;
}
EXPORT_SYMBOL_GPL(mvf_dispdrv_setdata);

void *mvf_dispdrv_getdata(struct mvf_dispdrv_handle *handle)
{
	struct mvf_dispdrv_entry *entry = (struct mvf_dispdrv_entry *)handle;

	if (entry) {
		return entry->priv;
	} else
		return ERR_PTR(-EINVAL);
}
EXPORT_SYMBOL_GPL(mvf_dispdrv_getdata);
