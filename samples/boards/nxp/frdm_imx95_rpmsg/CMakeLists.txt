# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.28.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(frdm_imx95_rpmsg)

target_sources(app PRIVATE src/main.c)
target_sources_ifdef(CONFIG_PWM app PRIVATE src/led.c)
