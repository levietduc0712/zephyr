# SPDX-FileCopyrightText: Copyright The Zephyr Project Contributors
# SPDX-License-Identifier: Apache-2.0

if(CONFIG_BOARD_FRDM_IMX95_SPSDK)
  board_runner_args(spsdk "--family=mimx9596" "--containers=two")
  include(${ZEPHYR_BASE}/boards/common/spsdk.board.cmake)
endif()
