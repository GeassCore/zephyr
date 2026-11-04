#include <zephyr/kernel.h>
#include <zephyr/drivers/timer/system_timer.h>
#include "debug.h"
#define R32(a) (*(volatile uint32_t *)(a))
#define R8(a) (*(volatile uint8_t *)(a))
static volatile uint32_t ticks;
static void tick(const void *arg) {
 ARG_UNUSED(arg); R32(0x07091004)=1; ++ticks;
 e902_debug->tick=ticks; ++e902_debug->timer_irq;
 sys_clock_announce(1);
}
static int timer_init(void) {
 static const uint32_t hz[]={24000000,19200000,26000000,24000000};
 IRQ_CONNECT(20,0,tick,NULL,0);
 R32(0x0701011c)|=0x10001; R32(0x07010100)=1;
 R32(0x07091020)=0; R32(0x07091024)=hz[(R32(0x07090160)>>14)&3]/1000;
 R32(0x0709102c)=0; R32(0x07091020)=2;
 unsigned n=100000; while((R32(0x07091020)&2)&&--n);
 if(!n) return -1;
 R32(0x07091004)=1; R32(0x07091000)|=1;
 R32(0xe0800008)=0; R8(0xe0801002+4*20)=0; R8(0xe0801003+4*20)=255;
 arch_irq_enable(20); R32(0x07091020)=1; return 0;
}
SYS_INIT(timer_init,PRE_KERNEL_2,CONFIG_SYSTEM_CLOCK_INIT_PRIORITY);
void sys_clock_set_timeout(int32_t t,bool idle) { ARG_UNUSED(t);ARG_UNUSED(idle); }
uint32_t sys_clock_elapsed(void) { return 0; }
uint32_t sys_clock_cycle_get_32(void) { return ticks; }
