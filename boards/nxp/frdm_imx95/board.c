/*
 * SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr/device.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/init.h>
#include <zephyr/sys/util.h>

#define UART_CLOCK_INIT_PRIORITY UTIL_INC(CONFIG_CLOCK_CONTROL_SCMI_INIT_PRIORITY)

BUILD_ASSERT(UART_CLOCK_INIT_PRIORITY < CONFIG_SERIAL_INIT_PRIORITY,
	     "LPUART3 clock must be configured before the serial driver");

static int frdm_imx95_uart_clock_init(void)
{
	const struct device *clk = DEVICE_DT_GET(DT_CLOCKS_CTLR(DT_NODELABEL(lpuart3)));
	clock_control_subsys_t root = UINT_TO_POINTER(DT_CLOCKS_CELL(DT_NODELABEL(lpuart3), name));

	if (!device_is_ready(clk)) {
		return -ENODEV;
	}

	return clock_control_set_rate(clk, root, UINT_TO_POINTER(24000000U));
}

SYS_INIT(frdm_imx95_uart_clock_init, PRE_KERNEL_1, UART_CLOCK_INIT_PRIORITY);
