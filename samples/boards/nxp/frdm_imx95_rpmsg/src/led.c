/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define BLINK_INTERVAL_MS 500

static void blink_d19(void *arg1, void *arg2, void *arg3)
{
	static const struct pwm_dt_spec led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
	uint32_t pulse = led.period;
	bool started = false;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (!pwm_is_ready_dt(&led)) {
		printk("D19 green PWM is unavailable\n");
		return;
	}

	for (;;) {
		int ret = pwm_set_pulse_dt(&led, pulse);

		if (ret != 0) {
			printk("D19 green PWM update failed: %d\n", ret);
			return;
		}
		if (!started) {
			printk("D19 green blinking via TPM3 channel 0 (500 ms on, 500 ms off)\n");
			started = true;
		}
		pulse = led.period - pulse;
		k_msleep(BLINK_INTERVAL_MS);
	}
}

K_THREAD_DEFINE(d19_blink, 1024, blink_d19, NULL, NULL, NULL, 0, 0, 0);
