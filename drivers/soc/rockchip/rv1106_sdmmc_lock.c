// SPDX-License-Identifier: GPL-2.0
#include <linux/mutex.h>
#include <linux/soc/rockchip/rk_sdmmc.h>

static DEFINE_MUTEX(rv1106_sdmmc_lock);

void rv1106_sdmmc_get_lock(void)
{
	mutex_lock(&rv1106_sdmmc_lock);
}
EXPORT_SYMBOL_GPL(rv1106_sdmmc_get_lock);

void rv1106_sdmmc_put_lock(void)
{
	mutex_unlock(&rv1106_sdmmc_lock);
}
EXPORT_SYMBOL_GPL(rv1106_sdmmc_put_lock);
