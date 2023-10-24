/*
 * Copyright (c) 2017 Jean-Paul Etienne <fractalclone@gmail.com>
 *
 * SPDX-License-Identifier: Apache-2.0
 */

/**
 * @brief UART driver for the duo Freedom Processor
 */

#define DT_DRV_COMPAT duo_uart0

#include <zephyr/kernel.h>
#include <zephyr/arch/cpu.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/pinctrl.h>
#include <soc.h>
#include <zephyr/irq.h>

#define UART0_BASE                              0x04140000

#define RXDATA_MASK    0xFF        /* Receive Data Mask */

#define UART_LCR_WLS_MSK 0x03 /* character length select mask */
#define UART_LCR_WLS_5 0x00 /* 5 bit character length */
#define UART_LCR_WLS_6 0x01 /* 6 bit character length */
#define UART_LCR_WLS_7 0x02 /* 7 bit character length */
#define UART_LCR_WLS_8 0x03 /* 8 bit character length */
#define UART_LCR_STB 0x04 /* # stop Bits, off=1, on=1.5 or 2) */
#define UART_LCR_PEN 0x08 /* Parity eneble */
#define UART_LCR_EPS 0x10 /* Even Parity Select */
#define UART_LCR_STKP 0x20 /* Stick Parity */
#define UART_LCR_SBRK 0x40 /* Set Break */
#define UART_LCR_BKSE 0x80 /* Bank select enable */
#define UART_LCR_DLAB 0x80 /* Divisor latch access bit */

#define UART_MCR_DTR 0x01 /* DTR   */
#define UART_MCR_RTS 0x02 /* RTS   */

#define UART_LSR_THRE 0x20 /* Transmit-hold-register empty */
#define UART_LSR_DR 0x01 /* Receiver data ready */
#define UART_LSR_TEMT 0x40 /* Xmitter empty */

#define UART_FCR_FIFO_EN 0x01 /* Fifo enable */
#define UART_FCR_RXSR 0x02 /* Receiver soft reset */
#define UART_FCR_TXSR 0x04 /* Transmitter soft reset */

#define UART_MCRVAL (UART_MCR_DTR | UART_MCR_RTS) /* RTS/DTR */
#define UART_FCR_DEFVAL (UART_FCR_FIFO_EN | UART_FCR_RXSR | UART_FCR_TXSR)
#define UART_LCR_8N1 0x03

struct uart_duo_regs_t {
	volatile uint32_t rbr; /* 0x00 Data register */
	volatile uint32_t ier; /* 0x04 Interrupt Enable Register */
	volatile uint32_t fcr; /* 0x08 FIFO Control Register */
	volatile uint32_t lcr; /* 0x0C Line control register */
	volatile uint32_t mcr; /* 0x10 Line control register */
	volatile uint32_t lsr; /* 0x14 Line Status Register */
	volatile uint32_t msr; /* 0x18 Modem Status Register */
	volatile uint32_t spr; /* 0x20 Scratch Register */
};

struct uart_duo_device_config {
	uintptr_t	port;
	uint32_t	baud_rate;
	const uint32_t tx_pin;
	const uint32_t rx_pin;
	const struct	pinctrl_dev_config *pcfg;
};

struct uart_duo_data {
	uint32_t	baud_rate;
};

#define DEV_UART(dev)						\
	((struct uart_duo_regs_t *)				\
	 ((const struct uart_duo_device_config * const)(dev)->config)->port)

static void uart_duo_poll_out(const struct device *dev,
					 unsigned char c)
{
	volatile struct uart_duo_regs_t *uart = DEV_UART(dev);

	while (!(uart->lsr & UART_LSR_THRE))
		;
	uart->rbr = c;
}

static int uart_duo_poll_in(const struct device *dev, unsigned char *c)
{
	volatile struct uart_duo_regs_t *uart = DEV_UART(dev);

	uint32_t val = uart->rx;
	while (!(uart->lsr & UART_LSR_DR))
		val = (int)uart->rbr;

	*c = (unsigned char)(val & RXDATA_MASK);

	return 0;
}

static int uart_duo_init(const struct device *dev)
{
	const struct uart_duo_device_config * const cfg = dev->config;
	volatile struct uart_duo_regs_t *uart = DEV_UART(dev);

	uart = (struct uart_duo_regs_t *)UART0_BASE;
	uart->lcr = uart->lcr | UART_LCR_DLAB | UART_LCR_8N1;
	uart->dll = divisor & 0xff;
	uart->dlm = (divisor >> 8) & 0xff;
	uart->lcr = uart->lcr & (~UART_LCR_DLAB);

	uart->ier = 0;
	uart->mcr = UART_MCRVAL;
	uart->fcr = UART_FCR_DEFVAL;

	uart->lcr = 3;

	return 0;
}

static const struct uart_driver_api uart_duo_driver_api = {
	.poll_in          = uart_duo_poll_in,
	.poll_out         = uart_duo_poll_out,
};

static struct uart_duo_data uart_duo_data_x;

PINCTRL_DT_INST_DEFINE(0);

static const struct uart_duo_device_config uart_duo_dev_cfg = {
	.port         = DT_INST_REG_ADDR(0),
	.sys_clk_freq = 2500000,
	.baud_rate    = DT_INST_PROP(0, current_speed),
	.pcfg	      = PINCTRL_DT_INST_DEV_CONFIG_GET(0),
};

DEVICE_DT_INST_DEFINE(0,
		    uart_duo_init,
		    NULL,
		    &uart_duo_data_x, &uart_duo_dev_cfg,
		    POST_KERNEL, CONFIG_SERIAL_INIT_PRIORITY,
		    (void *)&uart_duo_driver_api);

