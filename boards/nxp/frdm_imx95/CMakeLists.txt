# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_UART_MCUX_LPUART)
  zephyr_library()
  zephyr_library_sources(board.c)
endif()
