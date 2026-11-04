#include <zephyr/kernel.h>
#include <zephyr/arch/riscv/csr.h>
#include "debug.h"
#define R8(a) (*(volatile uint8_t *)(a))
void arch_irq_enable(unsigned int irq) { R8(0xe0801001+4*irq)=1; }
void arch_irq_disable(unsigned int irq) { R8(0xe0801001+4*irq)=0; }
int arch_irq_is_enabled(unsigned int irq) { return R8(0xe0801001+4*irq)&1; }
void soc_interrupt_init(void) {
 for (unsigned i=0;i<208;i++) arch_irq_disable(i);
 volatile uint32_t *p=(volatile uint32_t *)e902_debug;
 for(unsigned i=0;i<sizeof(*e902_debug)/4;i++) p[i]=0;
 e902_debug->magic=E902_DEBUG_MAGIC; e902_debug->abi=2; e902_debug->os_id=1; e902_debug->stage=2;
}
void k_sys_fatal_error_handler(unsigned int reason,const struct arch_esf *esf) {
 csr_clear(mstatus,8);
 e902_debug->fault=reason+0xdead1000;
 e902_debug->fault_pc=esf ? esf->mepc : 0;
 e902_debug->stage=0xff;
 for(;;) __asm__ volatile("nop");
}
