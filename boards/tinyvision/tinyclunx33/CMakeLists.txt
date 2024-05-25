# SPDX-License-Identifier: Apache-2.0

# This SoC does not allow every memory region to be reached via DMA
# so an extra linker script is required to place some buffers in a
# DMA-accessible memory.
set(BOARD_LINKER_SCRIPT ${CMAKE_CURRENT_SOURCE_DIR}/linker.ld CACHE INTERNAL "Board linker script, ${BOARD}")
