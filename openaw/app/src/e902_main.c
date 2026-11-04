#include <zephyr/kernel.h>
#include "debug.h"
#include "platform.h"
#include <string.h>
#include <zephyr/kernel_structs.h>
static void snapshot(const struct k_thread *thread, void *arg)
{
 ARG_UNUSED(arg);
 unsigned i=e902_debug->task_count;
 if(i>=E902_DEBUG_TASKS) return;
 volatile struct e902_task_info *t=&e902_debug->tasks[i];
 const char *name=k_thread_name_get((k_tid_t)thread);
 if(!name) name="unnamed";
 for(unsigned j=0;j<16;j++) t->name[j]=j<strlen(name)?name[j]:0;
 t->priority=k_thread_priority_get((k_tid_t)thread);
 t->state=thread==k_current_get()?0:
  thread->base.thread_state&_THREAD_SUSPENDED?3:
  thread->base.thread_state&(_THREAD_PENDING|_THREAD_SLEEPING)?2:1;
 size_t free_bytes=0;
 k_thread_stack_space_get(thread,&free_bytes);
 t->stack_words=free_bytes/4; t->number=i;
 e902_debug->task_count=i+1;
}
static void heartbeat(void *a, void *b, void *c)
{
 ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
 for (;;) { e902_debug->heartbeat++; k_sleep(K_SECONDS(1)); }
}
static void busy(void *a, void *b, void *c)
{
 ARG_UNUSED(a); ARG_UNUSED(b); ARG_UNUSED(c);
 for (;;) e902_debug->busy_counter++;
}
K_THREAD_DEFINE(heartbeat_task,1024,heartbeat,NULL,NULL,NULL,2,0,0);
K_THREAD_DEFINE(busy_task,1024,busy,NULL,NULL,NULL,10,0,0);
K_MSGQ_DEFINE(messages,sizeof(uint32_t),8,4);
static void producer(void *a,void *b,void *c)
{
 ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
 uint32_t n=0;
 for(;;) {
  if(k_msgq_put(&messages,&n,K_FOREVER)==0){n++;e902_debug->queue_tx++;}
  k_sleep(K_MSEC(20));
 }
}
static void consumer(void *a,void *b,void *c)
{
 ARG_UNUSED(a);ARG_UNUSED(b);ARG_UNUSED(c);
 uint32_t n;
 for(;;) if(k_msgq_get(&messages,&n,K_FOREVER)==0)e902_debug->queue_rx++;
}
K_THREAD_DEFINE(producer_task,1024,producer,NULL,NULL,NULL,3,0,0);
K_THREAD_DEFINE(consumer_task,1024,consumer,NULL,NULL,NULL,2,0,0);
static void timer(struct k_timer *t) { ARG_UNUSED(t); e902_debug->software_timer++; }
K_TIMER_DEFINE(probe_timer,timer,NULL);
int main(void)
{
 e902_debug->stage=4;
 platform_mailbox_init();
 k_thread_name_set(k_current_get(),"zephyr-monitor");
 k_thread_name_set(heartbeat_task,"zephyr-heartbeat");
 k_thread_name_set(busy_task,"zephyr-busy");
 k_thread_name_set(producer_task,"zephyr-producer");
 k_thread_name_set(consumer_task,"zephyr-consumer");
 k_timer_start(&probe_timer,K_MSEC(250),K_MSEC(250));
 unsigned n=0;
 for (;;) {
  platform_mailbox_echo();
  if(++n==100) {
   unsigned key=irq_lock();
   e902_debug->sequence++;
   e902_debug->task_count=0;
   k_thread_foreach(snapshot,NULL);
   e902_debug->sequence++;
   irq_unlock(key); n=0;
  }
  k_sleep(K_MSEC(10));
 }
 return 0;
}
