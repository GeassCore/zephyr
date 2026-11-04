#include <zephyr/toolchain.h>
GTEXT(__start)
SECTION_FUNC(vectors, __start)
 csrci mstatus,8
 .option push
 .option norelax
 la gp,__global_pointer$
 .option pop
 la t0,_isr_wrapper
 ori t0,t0,3
 csrw mtvec,t0
 tail __reset
.globl __soc_handle_irq
__soc_handle_irq:
 ret
