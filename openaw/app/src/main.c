/* SPDX-License-Identifier: Apache-2.0 */
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
int main(void)
{
 unsigned int heartbeat = 0;
 k_thread_name_set(k_current_get(), "heartbeat");
 for (;;) {
  printk("Zephyr RV32E architecture baseline heartbeat %u\n", ++heartbeat);
  k_sleep(K_SECONDS(1));
 }
 return 0;
}
