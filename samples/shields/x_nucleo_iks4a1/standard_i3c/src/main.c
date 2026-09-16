/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <inttypes.h>

static const struct {
	const struct device *dev;
	enum sensor_channel channel;
	int frequency;
	const char *unit;
	bool vector;
} sensors[] = {
	{ DEVICE_DT_GET(DT_ALIAS(accel1)), SENSOR_CHAN_ACCEL_XYZ, 60, "mm/s^2", true },
	{ DEVICE_DT_GET(DT_ALIAS(press0)), SENSOR_CHAN_PRESS, 10, "Pa", false },
	{ DEVICE_DT_GET(DT_ALIAS(accel0)), SENSOR_CHAN_ACCEL_XYZ, 104, "mm/s^2", true },
	{ DEVICE_DT_GET(DT_ALIAS(magn0)), SENSOR_CHAN_MAGN_XYZ, 10, "mgauss", true },
};

int main(void)
{
	int ret;

	for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
		struct sensor_value odr = { .val1 = sensors[i].frequency };

		if (!device_is_ready(sensors[i].dev)) {
			printk("%s is not ready\n", sensors[i].dev->name);
			return 0;
		}

		ret = sensor_attr_set(sensors[i].dev, sensors[i].channel,
				      SENSOR_ATTR_SAMPLING_FREQUENCY, &odr);
		if (ret < 0) {
			printk("%s configuration failed: %d\n", sensors[i].dev->name, ret);
			return 0;
		}
	}

	while (true) {
		for (size_t i = 0; i < ARRAY_SIZE(sensors); i++) {
			struct sensor_value value[3];

			ret = sensor_sample_fetch(sensors[i].dev);
			if (ret == 0) {
				ret = sensor_channel_get(sensors[i].dev, sensors[i].channel, value);
			}
			if (ret < 0) {
				printk("%s sampling failed: %d\n", sensors[i].dev->name, ret);
				return 0;
			}

			printk("%s: %" PRId64, sensors[i].dev->name,
			       sensor_value_to_milli(&value[0]));
			if (sensors[i].vector) {
				printk(", %" PRId64 ", %" PRId64, sensor_value_to_milli(&value[1]),
				       sensor_value_to_milli(&value[2]));
			}
			printk(" %s\n", sensors[i].unit);
		}
		k_sleep(K_SECONDS(1));
	}

	return 0;
}
