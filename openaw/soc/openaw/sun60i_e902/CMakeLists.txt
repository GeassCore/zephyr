set(SOC_LINKER_SCRIPT ${ZEPHYR_BASE}/include/zephyr/arch/riscv/common/linker.ld CACHE INTERNAL "")
zephyr_sources(start.S soc.c timer.c)
zephyr_include_directories(${CMAKE_CURRENT_LIST_DIR}/../../../hardware)
