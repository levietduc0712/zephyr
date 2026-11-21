/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/drivers/pwm.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/atomic.h>
#include <zephyr/sys/printk.h>

#include "led.h"

#define BLINK_INTERVAL_MS 500

static K_SEM_DEFINE(heartbeat_changed, 0, 1);
static atomic_t heartbeat_online;

void d19_set_heartbeat(bool online)
{
	atomic_set(&heartbeat_online, online);
	k_sem_give(&heartbeat_changed);
}

static void blink_d19(void *arg1, void *arg2, void *arg3)
{
	static const struct pwm_dt_spec led = PWM_DT_SPEC_GET(DT_ALIAS(pwm_led0));
	k_timeout_t wait = K_NO_WAIT;
	uint32_t pulse = 0U;
	bool blinking = false;
	bool first_update = true;

	ARG_UNUSED(arg1);
	ARG_UNUSED(arg2);
	ARG_UNUSED(arg3);

	if (!pwm_is_ready_dt(&led)) {
		printk("D19 green PWM is unavailable\n");
		return;
	}

	for (;;) {
		bool online;
		int ret;

		/* A heartbeat transition wakes the thread even during a blink interval. */
		k_sem_take(&heartbeat_changed, wait);
		online = atomic_get(&heartbeat_online) != 0;
		pulse = online ? led.period - pulse : 0U;
		ret = pwm_set_pulse_dt(&led, pulse);

		if (ret != 0) {
			printk("D19 green PWM update failed: %d\n", ret);
			return;
		}
		if (first_update || online != blinking) {
			printk("D19 green %s\n", online ? "blinking via TPM3 channel 0" :
							 "off (no A55 heartbeat)");
			blinking = online;
			first_update = false;
		}
		wait = online ? K_MSEC(BLINK_INTERVAL_MS) : K_FOREVER;
	}
}

K_THREAD_DEFINE(d19_blink, 1024, blink_d19, NULL, NULL, NULL, 0, 0, 0);
