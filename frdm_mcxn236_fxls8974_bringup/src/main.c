/*
 * Copyright 2026
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define FXLS8974_NODE DT_NODELABEL(fxls8974)

#if !DT_NODE_EXISTS(FXLS8974_NODE)
#error "FXLS8974 devicetree node label fxls8974 is missing"
#endif

static const struct device *const fxls8974 = DEVICE_DT_GET(FXLS8974_NODE);

static uint64_t isqrt64(uint64_t value)
{
	uint64_t bit = 1ULL << 62;
	uint64_t result = 0U;

	while (bit > value) {
		bit >>= 2;
	}

	while (bit != 0U) {
		if (value >= result + bit) {
			value -= result + bit;
			result = (result >> 1) + bit;
		} else {
			result >>= 1;
		}

		bit >>= 2;
	}

	return result;
}

static int64_t square_i64(int64_t value)
{
	return value * value;
}

int main(void)
{
	struct sensor_value accel[3];
	struct sensor_value temp;
	int ret;

	printk("FRDM-MCXN236 FXLS8974 polling bring-up\n");
	printk("device: %s\n", fxls8974->name);

	if (!device_is_ready(fxls8974)) {
		printk("device_is_ready() failed for %s\n", fxls8974->name);
		return 0;
	}

	printk("device_is_ready() succeeded\n");

	while (true) {
		ret = sensor_sample_fetch(fxls8974);
		if (ret < 0) {
			printk("sensor_sample_fetch() failed: %d\n", ret);
			k_sleep(K_MSEC(500));
			continue;
		}

		ret = sensor_channel_get(fxls8974, SENSOR_CHAN_ACCEL_XYZ, accel);
		if (ret < 0) {
			printk("SENSOR_CHAN_ACCEL_XYZ failed: %d\n", ret);
			k_sleep(K_MSEC(500));
			continue;
		}

		const int64_t x_milli = sensor_value_to_milli(&accel[0]);
		const int64_t y_milli = sensor_value_to_milli(&accel[1]);
		const int64_t z_milli = sensor_value_to_milli(&accel[2]);
		const uint64_t mag_milli = isqrt64(square_i64(x_milli) +
						   square_i64(y_milli) +
						   square_i64(z_milli));

		printk("accel milli-m/s^2: x=%lld y=%lld z=%lld |a|=%llu",
		       (long long)x_milli, (long long)y_milli, (long long)z_milli,
		       (unsigned long long)mag_milli);

		ret = sensor_channel_get(fxls8974, SENSOR_CHAN_AMBIENT_TEMP, &temp);
		if (ret < 0) {
			printk(" temp failed: %d\n", ret);
		} else {
			printk(" temp milli-C=%lld\n", (long long)sensor_value_to_milli(&temp));
		}

		k_sleep(K_MSEC(500));
	}

	return 0;
}
