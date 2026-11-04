# A733 E902 Zephyr board

`openaw/a733_e902` uses the out-of-tree `sun60i_e902` SoC. The firmware
starts at 0x4000a000, uses 92 KiB SRAM, and exposes ABI 2 Linux telemetry
at 0x40021000 (ARM PA 0x61000). The ISA is RV32EMC/ILP32E, no A/F/D.

The SoC implements direct non-vectored CLIC handling with mtvec mode 3,
208 interrupt inputs, exception-code mask 0xfff and 64-byte trap alignment.
S_TIMER0 IRQ 20 supplies periodic 1 kHz ticks; the DCXO frequency is read from
the RTC selector, including the DUT's 26 MHz source. The Zephyr kernel's RV32E
trap/context implementation handles scheduler ECALL and preemption.

Six threads exercise heartbeat, a non-yielding busy loop, message queue producer
and consumer, monitor/mailbox polling, and idle. A 250 ms kernel timer supplies
an independent callback counter. Linux reads task names, state, priority and
stack high-water free space. Mailbox diagnostic channel 3 uses the same XOR
protocol as FreeRTOS; this is not RPMsg. UART, PMP, userspace and SMP are not
part of this bringup profile. See ../../docs/support-status.md for validation.
