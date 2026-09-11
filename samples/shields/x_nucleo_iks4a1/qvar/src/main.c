/*
 * Copyright (c) 2026 STMicroelectronics
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include <zephyr/device.h>
#include <zephyr/devicetree.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/sensor/lsm6dsvxxx.h>
#include <zephyr/kernel.h>

#define LSM6DSV16X_NODE DT_ALIAS(accel1)
#define QVAR_BURST_SAMPLES 32
#define QVAR_BURST_SAMPLE_DELAY K_MSEC(8)
#define QVAR_RAW_RAIL_MICRO_MV (400LL * 1000000LL)
#define QVAR_USABLE_LIMIT_MICRO_MV (390LL * 1000000LL)
#define QVAR_FILTER_DIV 4

BUILD_ASSERT(DT_NODE_HAS_STATUS(LSM6DSV16X_NODE, okay),
	     "X-NUCLEO-IKS4A1 accel1 alias must be enabled");
BUILD_ASSERT(DT_NODE_HAS_COMPAT(LSM6DSV16X_NODE, st_lsm6dsv16x),
	     "X-NUCLEO-IKS4A1 accel1 alias must point to an LSM6DSV16X");

static int64_t qvar_abs(int64_t value)
{
	return value < 0 ? -value : value;
}

static int64_t qvar_limit(int64_t value)
{
	if (value > QVAR_USABLE_LIMIT_MICRO_MV) {
		return QVAR_USABLE_LIMIT_MICRO_MV;
	}

	if (value < -QVAR_USABLE_LIMIT_MICRO_MV) {
		return -QVAR_USABLE_LIMIT_MICRO_MV;
	}

	return value;
}

static void print_qvar_micro_mv(int64_t micro_mv, uint8_t clipped)
{
	uint64_t magnitude = micro_mv < 0 ? (uint64_t)-micro_mv : (uint64_t)micro_mv;

	printf("QVAR: %s%" PRIu64 ".%06" PRIu64 " mV", micro_mv < 0 ? "-" : "",
	       magnitude / 1000000U, magnitude % 1000000U);

	if (clipped != 0) {
		printf(" clipped=%u/%u", clipped, QVAR_BURST_SAMPLES);
	}

	printf("\n");
}

static int read_qvar_micro_mv(const struct device *lsm6dsv16x, int64_t *micro_mv)
{
	struct sensor_value qvar;

	if (sensor_sample_fetch_chan(lsm6dsv16x, SENSOR_CHAN_LSM6DSVXXX_QVAR) < 0) {
		printf("LSM6DSV16X QVAR sample update error\n");
		return -EIO;
	}

	if (sensor_channel_get(lsm6dsv16x, SENSOR_CHAN_LSM6DSVXXX_QVAR, &qvar) < 0) {
		printf("LSM6DSV16X QVAR channel get error\n");
		return -EIO;
	}

	*micro_mv = sensor_value_to_micro(&qvar);

	return 0;
}

static int read_qvar_envelope(const struct device *lsm6dsv16x, int64_t *micro_mv,
			      uint8_t *clipped)
{
	int64_t sum = 0;

	*clipped = 0;

	for (uint8_t i = 0; i < QVAR_BURST_SAMPLES; i++) {
		int64_t sample;

		if (read_qvar_micro_mv(lsm6dsv16x, &sample) < 0) {
			return -EIO;
		}

		if (qvar_abs(sample) >= QVAR_RAW_RAIL_MICRO_MV) {
			(*clipped)++;
		}

		sum += qvar_abs(qvar_limit(sample));
		k_sleep(QVAR_BURST_SAMPLE_DELAY);
	}

	*micro_mv = sum / QVAR_BURST_SAMPLES;

	return 0;
}

int main(void)
{
	const struct device *const lsm6dsv16x = DEVICE_DT_GET(LSM6DSV16X_NODE);
	int64_t filtered_micro_mv = 0;
	bool filtered_valid = false;

	if (!device_is_ready(lsm6dsv16x)) {
		printf("%s: device not ready\n", lsm6dsv16x->name);
		return 0;
	}

	printf("LSM6DSV16X QVAR polling\n");

	while (1) {
		int64_t envelope_micro_mv;
		uint8_t clipped;

		if (read_qvar_envelope(lsm6dsv16x, &envelope_micro_mv, &clipped) < 0) {
			return 0;
		}

		if (!filtered_valid) {
			filtered_micro_mv = envelope_micro_mv;
			filtered_valid = true;
		} else {
			filtered_micro_mv += (envelope_micro_mv - filtered_micro_mv) / QVAR_FILTER_DIV;
		}

		print_qvar_micro_mv(filtered_micro_mv, clipped);
	}
}
