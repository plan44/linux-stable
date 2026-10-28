// SPDX-License-Identifier: GPL-2.0-only
/*
 * Rockchip OTP-derived ARM serial number.
 * Based on the Rockchip vendor CPU information driver.
 * Copyright (C) 2017 Rockchip Electronics Co. Ltd.
 */

#include <linux/crc32.h>
#include <linux/err.h>
#include <linux/module.h>
#include <linux/nvmem-consumer.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <asm/system_info.h>

static int rockchip_cpuinfo_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct nvmem_cell *cell;
	u8 *id, buf[16];
	size_t len;
	int i;

	if (!of_machine_is_compatible("rockchip,rv1103b"))
		return -ENODEV;

	cell = devm_nvmem_cell_get(dev, "id");
	if (IS_ERR(cell))
		return dev_err_probe(dev, PTR_ERR(cell), "failed to get SoC id cell\n");

	id = nvmem_cell_read(cell, &len);
	if (IS_ERR(id))
		return dev_err_probe(dev, PTR_ERR(id), "failed to read SoC id\n");

	if (len != sizeof(buf)) {
		kfree(id);
		return dev_err_probe(dev, -EINVAL, "invalid SoC id length: %zu\n", len);
	}

	/* Keep the vendor's odd-byte/even-byte CRC ordering for stable serials. */
	for (i = 0; i < 8; i++) {
		buf[i] = id[1 + (i << 1)];
		buf[i + 8] = id[i << 1];
	}
	kfree(id);

	system_serial_low = crc32(0, buf, 8);
	system_serial_high = crc32(system_serial_low, buf + 8, 8);
	dev_info(dev, "Serial: %08x%08x\n", system_serial_high, system_serial_low);

	return 0;
}

static const struct of_device_id rockchip_cpuinfo_of_match[] = {
	{ .compatible = "rockchip,cpuinfo" },
	{ }
};
MODULE_DEVICE_TABLE(of, rockchip_cpuinfo_of_match);

static struct platform_driver rockchip_cpuinfo_driver = {
	.probe = rockchip_cpuinfo_probe,
	.driver = {
		.name = "rockchip-cpuinfo",
		.of_match_table = rockchip_cpuinfo_of_match,
	},
};
builtin_platform_driver(rockchip_cpuinfo_driver);

MODULE_DESCRIPTION("Rockchip RV1103B OTP-derived ARM serial number");
MODULE_LICENSE("GPL");
