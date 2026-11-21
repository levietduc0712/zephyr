# SPDX-License-Identifier: Apache-2.0

# Supply the matching FRDM BSP bootloader and assembled image to west flash.
board_runner_args(spsdk "--family=mimx9596" "--containers=two")
include(${ZEPHYR_BASE}/boards/common/spsdk.board.cmake)
