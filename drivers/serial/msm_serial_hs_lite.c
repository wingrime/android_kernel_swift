/*
 * drivers/serial/msm_serial.c - driver for msm7k serial device and console
 *
 * Copyright (C) 2007 Google, Inc.
 * Copyright (c) 2010-2011, Code Aurora Forum. All rights reserved.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

/* Acknowledgements:
 * This file is based on msm_serial.c, originally
 * Written by Robert Love <rlove@google.com>  */

#if defined(CONFIG_SERIAL_MSM_HSL_CONSOLE) && defined(CONFIG_MAGIC_SYSRQ)
#define SUPPORT_SYSRQ
#endif

#include <linux/hrtimer.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/irq.h>
#include <linux/init.h>
#include <linux/delay.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/nmi.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/gpio.h>
#include <linux/debugfs.h>
#include <mach/board.h>
#include <mach/msm_serial_hs_lite.h>
#include <asm/mach-types.h>
#include "msm_serial_hs_hwreg.h"

struct msm_hsl_port {
	struct uart_port	uart;
	char			name[16];
	struct clk		*clk;
	struct clk		*pclk;
	struct dentry		*loopback_dir;
	unsigned int		imr;
	unsigned int		*uart_csr_code;
	unsigned int            *gsbi_mapbase;
	unsigned int            *mapped_gsbi;
	unsigned int            old_snap_state;
};

#define UART_TO_MSM(uart_port)	((struct msm_hsl_port *) uart_port)
#define is_console(port)	((port)->cons && \
				(port)->cons->index == (port)->line)
static struct dentry *debug_base;
static inline void wait_for_xmitr(struct uart_port *port, int bits);
static inline void msm_hsl_write(struct uart_port *port,
				 unsigned int val, unsigned int off)
{
	iowrite32(val, port->membase + off);
}
static inline unsigned int msm_hsl_read(struct uart_port *port,
		     unsigned int off)
{
	return ioread32(port->membase + off);
}

static unsigned int msm_serial_hsl_has_gsbi(void)
{
#if defined(CONFIG_ARCH_MSM8X60) || defined(CONFIG_ARCH_MSM8960)
	return 1;
#else
	return 0;
#endif
}

static int clk_en(struct uart_port *port, int enable)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	int ret = 0;

	if (enable) {
		ret = clk_enable(msm_hsl_port->clk);
		if (ret)
			goto err;
		if (msm_hsl_port->pclk) {
			ret = clk_enable(msm_hsl_port->pclk);
			if (ret) {
				clk_disable(msm_hsl_port->clk);
				goto err;
			}
		}
	} else {
		clk_disable(msm_hsl_port->clk);
		if (msm_hsl_port->pclk)
			clk_disable(msm_hsl_port->pclk);
	}
err:
	return ret;
}
static int msm_hsl_loopback_enable_set(void *data, u64 val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned long flags;
	int ret = 0;

	ret = clk_set_rate(msm_hsl_port->clk, 7372800);
	if (!ret)
		clk_en(port, 1);
	else {
		pr_err("%s(): Error: Setting the clock rate\n", __func__);
		return -EINVAL;
	}

	if (val) {
		spin_lock_irqsave(&port->lock, flags);
		ret = msm_hsl_read(port, UARTDM_MR2_ADDR);
		ret |= UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, ret, UARTDM_MR2_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);
	} else {
		spin_lock_irqsave(&port->lock, flags);
		ret = msm_hsl_read(port, UARTDM_MR2_ADDR);
		ret &= ~UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, ret, UARTDM_MR2_ADDR);
		spin_unlock_irqrestore(&port->lock, flags);
	}

	clk_en(port, 0);
	return 0;
}
static int msm_hsl_loopback_enable_get(void *data, u64 *val)
{
	struct msm_hsl_port *msm_hsl_port = data;
	struct uart_port *port = &(msm_hsl_port->uart);
	unsigned long flags;
	int ret = 0;

	ret = clk_set_rate(msm_hsl_port->clk, 7372800);
	if (!ret)
		clk_en(port, 1);
	else {
		pr_err("%s(): Error setting clk rate\n", __func__);
		return -EINVAL;
	}

	spin_lock_irqsave(&port->lock, flags);
	ret = msm_hsl_read(port, UARTDM_MR2_ADDR);
	spin_unlock_irqrestore(&port->lock, flags);
	clk_en(port, 0);

	*val = (ret & UARTDM_MR2_LOOP_MODE_BMSK) ? 1 : 0;
	return 0;
}
DEFINE_SIMPLE_ATTRIBUTE(loopback_enable_fops, msm_hsl_loopback_enable_get,
	msm_hsl_loopback_enable_set, "%llu\n");
/*
 * msm_serial_hsl debugfs node: <debugfs_root>/msm_serial_hsl/loopback.<id>
 * writing 1 turns on internal loopback mode in HW. Useful for automation
 * test scripts.
 * writing 0 disables the internal loopback mode. Default is disabled.
 */
static void msm_hsl_debugfs_init(struct msm_hsl_port *msm_uport, int id)
{
	char node_name[15];

	snprintf(node_name, sizeof(node_name), "loopback.%d", id);
	msm_uport->loopback_dir = debugfs_create_file(node_name,
					S_IRUGO | S_IWUSR,
					debug_base,
					msm_uport,
					&loopback_enable_fops);

	if (IS_ERR_OR_NULL(msm_uport->loopback_dir))
		pr_err("%s(): Cannot create loopback.%d debug entry",
							__func__, id);
}
static void msm_hsl_stop_tx(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	clk_en(port, 1);

	msm_hsl_port->imr &= ~UARTDM_ISR_TXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);

	clk_en(port, 0);
}

static void msm_hsl_start_tx(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	clk_en(port, 1);

	msm_hsl_port->imr |= UARTDM_ISR_TXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);

	clk_en(port, 0);
}

static void msm_hsl_stop_rx(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	clk_en(port, 1);

	msm_hsl_port->imr &= ~(UARTDM_ISR_RXLEV_BMSK |
			       UARTDM_ISR_RXSTALE_BMSK);
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);

	clk_en(port, 0);
}

static void msm_hsl_enable_ms(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	clk_en(port, 1);

	msm_hsl_port->imr |= UARTDM_ISR_DELTA_CTS_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);

	clk_en(port, 0);
}

static void handle_rx(struct uart_port *port, unsigned int misr)
{
	struct tty_struct *tty = port->state->port.tty;
	unsigned int sr;
	int count = 0;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	/*
	 * Handle overrun. My understanding of the hardware is that overrun
	 * is not tied to the RX buffer, so we handle the case out of band.
	 */
	if ((msm_hsl_read(port, UARTDM_SR_ADDR) & UARTDM_SR_OVERRUN_BMSK)) {
		port->icount.overrun++;
		tty_insert_flip_char(tty, 0, TTY_OVERRUN);
		msm_hsl_write(port, RESET_ERROR_STATUS, UARTDM_CR_ADDR);
	}

	if (misr & UARTDM_ISR_RXSTALE_BMSK) {
		count = msm_hsl_read(port, UARTDM_RX_TOTAL_SNAP_ADDR) -
			msm_hsl_port->old_snap_state;
		msm_hsl_port->old_snap_state = 0;
	} else {
		count = 4 * (msm_hsl_read(port, UARTDM_RFWR_ADDR));
		msm_hsl_port->old_snap_state += count;
	}

	/* and now the main RX loop */
	while (count > 0) {
		unsigned int c;
		char flag = TTY_NORMAL;

		sr = msm_hsl_read(port, UARTDM_SR_ADDR);
		if ((sr &
		     UARTDM_SR_RXRDY_BMSK) == 0) {
			msm_hsl_port->old_snap_state -= count;
			break;
		}
		c = msm_hsl_read(port, UARTDM_RF_ADDR);
		if (sr & UARTDM_SR_RX_BREAK_BMSK) {
			port->icount.brk++;
			if (uart_handle_break(port))
				continue;
		} else if (sr & UARTDM_SR_PAR_FRAME_BMSK) {
			port->icount.frame++;
		} else {
			port->icount.rx++;
		}

		/* Mask conditions we're ignorning. */
		sr &= port->read_status_mask;
		if (sr & UARTDM_SR_RX_BREAK_BMSK)
			flag = TTY_BREAK;
		else if (sr & UARTDM_SR_PAR_FRAME_BMSK)
			flag = TTY_FRAME;

		/* TODO: handle sysrq */
		/* if (!uart_handle_sysrq_char(port, c)) */
		tty_insert_flip_string(tty, (char *) &c,
				       (count > 4) ? 4 : count);
		count -= 4;
	}

	tty_flip_buffer_push(tty);
}

static void handle_tx(struct uart_port *port)
{
	struct circ_buf *xmit = &port->state->xmit;
	int sent_tx;
	int tx_count;
	int x;
	unsigned int tf_pointer = 0;

	tx_count = uart_circ_chars_pending(xmit);

	if (tx_count > (UART_XMIT_SIZE - xmit->tail))
		tx_count = UART_XMIT_SIZE - xmit->tail;
	if (tx_count >= port->fifosize)
		tx_count = port->fifosize;

	/* Handle x_char */
	if (port->x_char) {
		wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
		msm_hsl_write(port, tx_count + 1, UARTDM_NCF_TX_ADDR);
		msm_hsl_write(port, port->x_char, UARTDM_TF_ADDR);
		port->icount.tx++;
		port->x_char = 0;
	} else if (tx_count) {
		wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
		msm_hsl_write(port, tx_count, UARTDM_NCF_TX_ADDR);
	}
	if (!tx_count) {
		msm_hsl_stop_tx(port);
		return;
	}

	while (tf_pointer < tx_count)  {
		if (unlikely(!(msm_hsl_read(port, UARTDM_SR_ADDR) &
			       UARTDM_SR_TXRDY_BMSK)))
			continue;
		switch (tx_count - tf_pointer) {
		case 1: {
			x = xmit->buf[xmit->tail];
			port->icount.tx++;
			break;
		}
		case 2: {
			x = xmit->buf[xmit->tail]
				| xmit->buf[xmit->tail+1] << 8;
			port->icount.tx += 2;
			break;
		}
		case 3: {
			x = xmit->buf[xmit->tail]
				| xmit->buf[xmit->tail+1] << 8
				| xmit->buf[xmit->tail + 2] << 16;
			port->icount.tx += 3;
			break;
		}
		default: {
			x = *((int *)&(xmit->buf[xmit->tail]));
			port->icount.tx += 4;
			break;
		}
		}
		msm_hsl_write(port, x, UARTDM_TF_ADDR);
		xmit->tail = ((tx_count - tf_pointer < 4) ?
			      (tx_count - tf_pointer + xmit->tail) :
			      (xmit->tail + 4)) & (UART_XMIT_SIZE - 1);
		tf_pointer += 4;
		sent_tx = 1;
	}

	if (uart_circ_empty(xmit))
		msm_hsl_stop_tx(port);

	if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
		uart_write_wakeup(port);

}

static void handle_delta_cts(struct uart_port *port)
{
	msm_hsl_write(port, RESET_CTS, UARTDM_CR_ADDR);
	port->icount.cts++;
	wake_up_interruptible(&port->state->port.delta_msr_wait);
}

static irqreturn_t msm_hsl_irq(int irq, void *dev_id)
{
	struct uart_port *port = dev_id;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	unsigned int misr;
	unsigned long flags;

	spin_lock_irqsave(&port->lock, flags);
	clk_en(port, 1);
	misr = msm_hsl_read(port, UARTDM_MISR_ADDR);
	msm_hsl_write(port, 0, UARTDM_IMR_ADDR); /* disable interrupt */

	if (misr & (UARTDM_ISR_RXSTALE_BMSK | UARTDM_ISR_RXLEV_BMSK)) {
		handle_rx(port, misr);
		if (misr & (UARTDM_ISR_RXSTALE_BMSK))
			msm_hsl_write(port, RESET_STALE_INT, UARTDM_CR_ADDR);
		msm_hsl_write(port, 6500, UARTDM_DMRX_ADDR);
		msm_hsl_write(port, STALE_EVENT_ENABLE, UARTDM_CR_ADDR);
	}
	if (misr & UARTDM_ISR_TXLEV_BMSK)
		handle_tx(port);

	if (misr & UARTDM_ISR_DELTA_CTS_BMSK)
		handle_delta_cts(port);

	/* restore interrupt */
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);
	clk_en(port, 0);
	spin_unlock_irqrestore(&port->lock, flags);

	return IRQ_HANDLED;
}

static unsigned int msm_hsl_tx_empty(struct uart_port *port)
{
	unsigned int ret;

	clk_en(port, 1);
	ret = (msm_hsl_read(port, UARTDM_SR_ADDR) &
	       UARTDM_SR_TXEMT_BMSK) ? TIOCSER_TEMT : 0;
	clk_en(port, 0);

	return ret;
}

static void msm_hsl_reset(struct uart_port *port)
{
	/* reset everything */
	msm_hsl_write(port, RESET_RX, UARTDM_CR_ADDR);
	msm_hsl_write(port, RESET_TX, UARTDM_CR_ADDR);
	msm_hsl_write(port, RESET_ERROR_STATUS, UARTDM_CR_ADDR);
	msm_hsl_write(port, RESET_BREAK_INT, UARTDM_CR_ADDR);
	msm_hsl_write(port, RESET_CTS, UARTDM_CR_ADDR);
	msm_hsl_write(port, RFR_LOW, UARTDM_CR_ADDR);
}

static unsigned int msm_hsl_get_mctrl(struct uart_port *port)
{
	return TIOCM_CAR | TIOCM_CTS | TIOCM_DSR | TIOCM_RTS;
}

static void msm_hsl_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	unsigned int mr;
	unsigned int loop_mode;

	clk_en(port, 1);

	mr = msm_hsl_read(port, UARTDM_MR1_ADDR);

	if (!(mctrl & TIOCM_RTS)) {
		mr &= ~UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hsl_write(port, mr, UARTDM_MR1_ADDR);
		msm_hsl_write(port, RFR_HIGH, UARTDM_CR_ADDR);
	} else {
		mr |= UARTDM_MR1_RX_RDY_CTL_BMSK;
		msm_hsl_write(port, mr, UARTDM_MR1_ADDR);
	}

	loop_mode = TIOCM_LOOP & mctrl;
	if (loop_mode) {
		mr = msm_hsl_read(port, UARTDM_MR2_ADDR);
		mr |= UARTDM_MR2_LOOP_MODE_BMSK;
		msm_hsl_write(port, mr, UARTDM_MR2_ADDR);

		/* Reset TX */
		msm_hsl_reset(port);

		/* Turn on Uart Receiver & Transmitter*/
		msm_hsl_write(port, UARTDM_CR_RX_EN_BMSK
			      | UARTDM_CR_TX_EN_BMSK, UARTDM_CR_ADDR);
	}

	clk_en(port, 0);
}

static void msm_hsl_break_ctl(struct uart_port *port, int break_ctl)
{
	clk_en(port, 1);

	if (break_ctl)
		msm_hsl_write(port, START_BREAK, UARTDM_CR_ADDR);
	else
		msm_hsl_write(port, STOP_BREAK, UARTDM_CR_ADDR);

	clk_en(port, 0);
}

static void msm_hsl_set_baud_rate(struct uart_port *port, unsigned int baud)
{
	unsigned int baud_code, rxstale, watermark;
	unsigned int data;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	switch (baud) {
	case 300:
		baud_code = UARTDM_CSR_75;
		rxstale = 1;
		break;
	case 600:
		baud_code = UARTDM_CSR_150;
		rxstale = 1;
		break;
	case 1200:
		baud_code = UARTDM_CSR_300;
		rxstale = 1;
		break;
	case 2400:
		baud_code = UARTDM_CSR_600;
		rxstale = 1;
		break;
	case 4800:
		baud_code = UARTDM_CSR_1200;
		rxstale = 1;
		break;
	case 9600:
		baud_code = UARTDM_CSR_2400;
		rxstale = 2;
		break;
	case 14400:
		baud_code = UARTDM_CSR_3600;
		rxstale = 3;
		break;
	case 19200:
		baud_code = UARTDM_CSR_4800;
		rxstale = 4;
		break;
	case 28800:
		baud_code = UARTDM_CSR_7200;
		rxstale = 6;
		break;
	case 38400:
		baud_code = UARTDM_CSR_9600;
		rxstale = 8;
		break;
	case 57600:
		baud_code = UARTDM_CSR_14400;
		rxstale = 16;
		break;
	case 115200:
		baud_code = UARTDM_CSR_28800;
		rxstale = 31;
		break;
	case 230400:
		baud_code = UARTDM_CSR_57600;
		rxstale = 31;
		break;
	case 460800:
		baud_code = UARTDM_CSR_115200;
		rxstale = 31;
		break;
	default: /* 115200 baud rate */
		baud_code = UARTDM_CSR_28800;
		rxstale = 31;
		break;
	}

	msm_hsl_write(port, baud_code, UARTDM_CSR_ADDR);

	/* RX stale watermark */
	watermark = UARTDM_IPR_STALE_LSB_BMSK & rxstale;
	watermark |= UARTDM_IPR_STALE_TIMEOUT_MSB_BMSK & (rxstale << 2);
	msm_hsl_write(port, watermark, UARTDM_IPR_ADDR);

	/* set RX watermark */
	watermark = (port->fifosize * 3) / 4;
	msm_hsl_write(port, watermark, UARTDM_RFWR_ADDR);

	/* set TX watermark */
	msm_hsl_write(port, 0, UARTDM_TFWR_ADDR);

	msm_hsl_write(port, CR_PROTECTION_EN, UARTDM_CR_ADDR);
	msm_hsl_reset(port);

	data = UARTDM_CR_TX_EN_BMSK;
	data |= UARTDM_CR_RX_EN_BMSK;
	/* enable TX & RX */
	msm_hsl_write(port, data, UARTDM_CR_ADDR);

	msm_hsl_write(port, RESET_STALE_INT, UARTDM_CR_ADDR);
	/* turn on RX and CTS interrupts */
	msm_hsl_port->imr = UARTDM_ISR_RXSTALE_BMSK
		| UARTDM_ISR_DELTA_CTS_BMSK | UARTDM_ISR_RXLEV_BMSK;
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);
	msm_hsl_write(port, 6500, UARTDM_DMRX_ADDR);
	msm_hsl_write(port, STALE_EVENT_ENABLE, UARTDM_CR_ADDR);
}

static void msm_hsl_init_clock(struct uart_port *port)
{
	clk_en(port, 1);
}

static void msm_hsl_deinit_clock(struct uart_port *port)
{
	clk_en(port, 0);
}

static int msm_hsl_startup(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;
	unsigned int data, rfr_level;
	int ret;
	unsigned long flags;

	snprintf(msm_hsl_port->name, sizeof(msm_hsl_port->name),
		 "msm_serial_hsl%d", port->line);

	if (!(is_console(port)) || (!port->cons) ||
		(port->cons && (!(port->cons->flags & CON_ENABLED)))) {
		if (msm_serial_hsl_has_gsbi())
			if ((ioread32(msm_hsl_port->mapped_gsbi +
				GSBI_CONTROL_ADDR) & GSBI_PROTOCOL_I2C_UART)
					!= GSBI_PROTOCOL_I2C_UART)
				iowrite32(GSBI_PROTOCOL_I2C_UART,
					msm_hsl_port->mapped_gsbi +
						GSBI_CONTROL_ADDR);

		if (pdata && pdata->config_gpio) {
			ret = gpio_request(pdata->uart_tx_gpio,
							"UART_TX_GPIO");
			if (unlikely(ret)) {
				pr_err("%s: gpio request failed for:%d\n",
						 __func__, pdata->uart_tx_gpio);
				return ret;
			}

			ret = gpio_request(pdata->uart_rx_gpio, "UART_RX_GPIO");
			if (unlikely(ret)) {
				pr_err("%s: gpio request failed for:%d\n",
						__func__, pdata->uart_rx_gpio);
				gpio_free(pdata->uart_tx_gpio);
				return ret;
			}
		}
	}
#ifndef CONFIG_PM_RUNTIME
	msm_hsl_init_clock(port);
#endif
	pm_runtime_get_sync(port->dev);

	if (likely(port->fifosize > 12))
		rfr_level = port->fifosize - 12;
	else
		rfr_level = port->fifosize;

	spin_lock_irqsave(&port->lock, flags);

	/* set automatic RFR level */
	data = msm_hsl_read(port, UARTDM_MR1_ADDR);
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK;
	data &= ~UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK;
	data |= UARTDM_MR1_AUTO_RFR_LEVEL1_BMSK & (rfr_level << 2);
	data |= UARTDM_MR1_AUTO_RFR_LEVEL0_BMSK & rfr_level;
	msm_hsl_write(port, data, UARTDM_MR1_ADDR);
	spin_unlock_irqrestore(&port->lock, flags);

	ret = request_irq(port->irq, msm_hsl_irq, IRQF_TRIGGER_HIGH,
			  msm_hsl_port->name, port);
	if (unlikely(ret)) {
		printk(KERN_ERR "%s: failed to request_irq\n", __func__);
		return ret;
	}
	return 0;
}

static void msm_hsl_shutdown(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	const struct msm_serial_hslite_platform_data *pdata =
					pdev->dev.platform_data;

	clk_en(port, 1);

	msm_hsl_port->imr = 0;
	msm_hsl_write(port, 0, UARTDM_IMR_ADDR); /* disable interrupts */

	clk_en(port, 0);

	free_irq(port->irq, port);

#ifndef CONFIG_PM_RUNTIME
	msm_hsl_deinit_clock(port);
#endif
	pm_runtime_put_sync(port->dev);
	if (!(is_console(port)) || (!port->cons) ||
		(port->cons && (!(port->cons->flags & CON_ENABLED)))) {
		if (pdata && pdata->config_gpio) {
			gpio_free(pdata->uart_tx_gpio);
			gpio_free(pdata->uart_rx_gpio);
		}
	}
}

static void msm_hsl_set_termios(struct uart_port *port,
				struct ktermios *termios,
				struct ktermios *old)
{
	unsigned long flags;
	unsigned int baud, mr;

	spin_lock_irqsave(&port->lock, flags);
	clk_en(port, 1);

	/* calculate and set baud rate */
	baud = uart_get_baud_rate(port, termios, old, 300, 460800);

	msm_hsl_set_baud_rate(port, baud);

	/* calculate parity */
	mr = msm_hsl_read(port, UARTDM_MR2_ADDR);
	mr &= ~UARTDM_MR2_PARITY_MODE_BMSK;
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			mr |= ODD_PARITY;
		else if (termios->c_cflag & CMSPAR)
			mr |= SPACE_PARITY;
		else
			mr |= EVEN_PARITY;
	}

	/* calculate bits per char */
	mr &= ~UARTDM_MR2_BITS_PER_CHAR_BMSK;
	switch (termios->c_cflag & CSIZE) {
	case CS5:
		mr |= FIVE_BPC;
		break;
	case CS6:
		mr |= SIX_BPC;
		break;
	case CS7:
		mr |= SEVEN_BPC;
		break;
	case CS8:
	default:
		mr |= EIGHT_BPC;
		break;
	}

	/* calculate stop bits */
	mr &= ~(STOP_BIT_ONE | STOP_BIT_TWO);
	if (termios->c_cflag & CSTOPB)
		mr |= STOP_BIT_TWO;
	else
		mr |= STOP_BIT_ONE;

	/* set parity, bits per char, and stop bit */
	msm_hsl_write(port, mr, UARTDM_MR2_ADDR);

	/* calculate and set hardware flow control */
	mr = msm_hsl_read(port, UARTDM_MR1_ADDR);
	mr &= ~(UARTDM_MR1_CTS_CTL_BMSK | UARTDM_MR1_RX_RDY_CTL_BMSK);
	if (termios->c_cflag & CRTSCTS) {
		mr |= UARTDM_MR1_CTS_CTL_BMSK;
		mr |= UARTDM_MR1_RX_RDY_CTL_BMSK;
	}
	msm_hsl_write(port, mr, UARTDM_MR1_ADDR);

	/* Configure status bits to ignore based on termio flags. */
	port->read_status_mask = 0;
	if (termios->c_iflag & INPCK)
		port->read_status_mask |= UARTDM_SR_PAR_FRAME_BMSK;
	if (termios->c_iflag & (BRKINT | PARMRK))
		port->read_status_mask |= UARTDM_SR_RX_BREAK_BMSK;

	uart_update_timeout(port, termios->c_cflag, baud);

	clk_en(port, 0);
	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *msm_hsl_type(struct uart_port *port)
{
	return "MSM";
}

static void msm_hsl_release_port(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	resource_size_t size;

	uart_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "uartdm_resource");
	if (unlikely(!uart_resource))
		return;
	size = uart_resource->end - uart_resource->start + 1;

	release_mem_region(port->mapbase, size);
	iounmap(port->membase);
	port->membase = NULL;

	if (msm_serial_hsl_has_gsbi()) {
		iowrite32(GSBI_PROTOCOL_IDLE, msm_hsl_port->mapped_gsbi +
			  GSBI_CONTROL_ADDR);
		gsbi_resource = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							     "gsbi_resource");

		size = gsbi_resource->end - gsbi_resource->start + 1;
		iounmap(msm_hsl_port->mapped_gsbi);
		msm_hsl_port->mapped_gsbi = NULL;
	}
}

static int msm_hsl_request_port(struct uart_port *port)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	struct platform_device *pdev = to_platform_device(port->dev);
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	resource_size_t size;

	uart_resource = platform_get_resource_byname(pdev, IORESOURCE_MEM,
						     "uartdm_resource");
	if (unlikely(!uart_resource)) {
		pr_err("%s: can't get uartdm resource\n", __func__);
		return -ENXIO;
	}
	size = uart_resource->end - uart_resource->start + 1;

	if (unlikely(!request_mem_region(port->mapbase, size,
					 "msm_serial_hsl"))) {
		pr_err("%s: can't get mem region for uartdm\n", __func__);
		return -EBUSY;
	}

	port->membase = ioremap(port->mapbase, size);
	if (!port->membase) {
		release_mem_region(port->mapbase, size);
		return -EBUSY;
	}

	if (msm_serial_hsl_has_gsbi()) {
		gsbi_resource = platform_get_resource_byname(pdev,
							     IORESOURCE_MEM,
							     "gsbi_resource");
		if (unlikely(!gsbi_resource)) {
			pr_err("%s: can't get gsbi resource\n", __func__);
			return -ENXIO;
		}

		size = gsbi_resource->end - gsbi_resource->start + 1;
		msm_hsl_port->mapped_gsbi = ioremap(gsbi_resource->start,
						    size);
		if (!msm_hsl_port->mapped_gsbi) {
			return -EBUSY;
		}
	}

	return 0;
}

static void msm_hsl_config_port(struct uart_port *port, int flags)
{
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);
	if (flags & UART_CONFIG_TYPE) {
		port->type = PORT_MSM;
		if (msm_hsl_request_port(port))
			return;
	}
	if (msm_serial_hsl_has_gsbi())
		if ((ioread32(msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR) &
			GSBI_PROTOCOL_I2C_UART) != GSBI_PROTOCOL_I2C_UART)
			iowrite32(GSBI_PROTOCOL_I2C_UART,
				msm_hsl_port->mapped_gsbi + GSBI_CONTROL_ADDR);
}

static int msm_hsl_verify_port(struct uart_port *port,
			       struct serial_struct *ser)
{
	if (unlikely(ser->type != PORT_UNKNOWN && ser->type != PORT_MSM))
		return -EINVAL;
	if (unlikely(port->irq != ser->irq))
		return -EINVAL;
	return 0;
}

static void msm_hsl_power(struct uart_port *port, unsigned int state,
			  unsigned int oldstate)
{
	int ret;
	struct msm_hsl_port *msm_hsl_port = UART_TO_MSM(port);

	switch (state) {
	case 0:
		ret = clk_set_rate(msm_hsl_port->clk, 7372800);
		if (ret)
			pr_err("%s(): Error setting UART clock rate\n",
							__func__);
		clk_en(port, 1);
		break;
	case 3:
		clk_en(port, 0);
		ret = clk_set_rate(msm_hsl_port->clk, 0);
		if (ret)
			pr_err("%s(): Error setting UART clock rate to zero.\n",
							__func__);
		break;
	default:
		pr_err("%s(): msm_serial_hsl: Unknown PM state %d\n",
							__func__, state);
	}
}

static struct uart_ops msm_hsl_uart_pops = {
	.tx_empty = msm_hsl_tx_empty,
	.set_mctrl = msm_hsl_set_mctrl,
	.get_mctrl = msm_hsl_get_mctrl,
	.stop_tx = msm_hsl_stop_tx,
	.start_tx = msm_hsl_start_tx,
	.stop_rx = msm_hsl_stop_rx,
	.enable_ms = msm_hsl_enable_ms,
	.break_ctl = msm_hsl_break_ctl,
	.startup = msm_hsl_startup,
	.shutdown = msm_hsl_shutdown,
	.set_termios = msm_hsl_set_termios,
	.type = msm_hsl_type,
	.release_port = msm_hsl_release_port,
	.request_port = msm_hsl_request_port,
	.config_port = msm_hsl_config_port,
	.verify_port = msm_hsl_verify_port,
	.pm = msm_hsl_power,
};

static struct msm_hsl_port msm_hsl_uart_ports[] = {
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 0,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 1,
		},
	},
	{
		.uart = {
			.iotype = UPIO_MEM,
			.ops = &msm_hsl_uart_pops,
			.flags = UPF_BOOT_AUTOCONF,
			.fifosize = 64,
			.line = 2,
		},
	},
};

#define UART_NR	ARRAY_SIZE(msm_hsl_uart_ports)

static inline struct uart_port *get_port_from_line(unsigned int line)
{
	return &msm_hsl_uart_ports[line].uart;
}

/*
 *  Wait for transmitter & holding register to empty
 *  Derived from wait_for_xmitr in 8250 serial driver by Russell King  */
void wait_for_xmitr(struct uart_port *port, int bits)
{
	if (!(msm_hsl_read(port, UARTDM_SR_ADDR) & UARTDM_SR_TXEMT_BMSK)) {
		while ((msm_hsl_read(port, UARTDM_ISR_ADDR) & bits) != bits) {
			udelay(1);
			touch_nmi_watchdog();
			cpu_relax();
		}
		msm_hsl_write(port, CLEAR_TX_READY, UARTDM_CR_ADDR);
	}
}

#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
static void msm_hsl_console_putchar(struct uart_port *port, int ch)
{
	wait_for_xmitr(port, UARTDM_ISR_TX_READY_BMSK);
	msm_hsl_write(port, 1, UARTDM_NCF_TX_ADDR);

	while (!(msm_hsl_read(port, UARTDM_SR_ADDR) & UARTDM_SR_TXRDY_BMSK)) {
		udelay(1);
		touch_nmi_watchdog();
	}

	msm_hsl_write(port, ch, UARTDM_TF_ADDR);
}

static void msm_hsl_console_write(struct console *co, const char *s,
				  unsigned int count)
{
	struct uart_port *port;
	struct msm_hsl_port *msm_hsl_port;
	int locked;

	BUG_ON(co->index < 0 || co->index >= UART_NR);

	port = get_port_from_line(co->index);
	msm_hsl_port = UART_TO_MSM(port);

	/* not pretty, but we can end up here via various convoluted paths */
	if (port->sysrq || oops_in_progress)
		locked = spin_trylock(&port->lock);
	else {
		locked = 1;
		spin_lock(&port->lock);
	}
	msm_hsl_write(port, 0, UARTDM_IMR_ADDR);
	uart_console_write(port, s, count, msm_hsl_console_putchar);
	msm_hsl_write(port, msm_hsl_port->imr, UARTDM_IMR_ADDR);
	if (locked == 1)
		spin_unlock(&port->lock);
}

static int __init msm_hsl_console_setup(struct console *co, char *options)
{
	struct uart_port *port;
	int baud, flow, bits, parity;
	int ret;

	if (unlikely(co->index >= UART_NR || co->index < 0))
		return -ENXIO;

	port = get_port_from_line(co->index);

	if (unlikely(!port->membase))
		return -ENXIO;

	port->cons = co;

	pm_runtime_get_noresume(port->dev);

#ifndef CONFIG_PM_RUNTIME
	msm_hsl_init_clock(port);
#endif
	pm_runtime_resume(port->dev);

	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	bits = 8;
	parity = 'n';
	flow = 'n';
	msm_hsl_write(port, UARTDM_MR2_BITS_PER_CHAR_8 | STOP_BIT_ONE,
		      UARTDM_MR2_ADDR);	/* 8N1 */

	if (baud < 300 || baud > 115200)
		baud = 115200;
	msm_hsl_set_baud_rate(port, baud);

	ret = uart_set_options(port, co, baud, parity, bits, flow);
	msm_hsl_reset(port);
	/* Enable transmitter */
	msm_hsl_write(port, CR_PROTECTION_EN, UARTDM_CR_ADDR);
	msm_hsl_write(port, UARTDM_CR_TX_EN_BMSK, UARTDM_CR_ADDR);

	printk(KERN_INFO "msm_serial_hsl: console setup on port #%d\n",
	       port->line);

	return ret;
}

static struct uart_driver msm_hsl_uart_driver;

static struct console msm_hsl_console = {
	.name = "ttyHSL",
	.write = msm_hsl_console_write,
	.device = uart_console_device,
	.setup = msm_hsl_console_setup,
	.flags = CON_PRINTBUFFER,
	.index = -1,
	.data = &msm_hsl_uart_driver,
};

#define MSM_HSL_CONSOLE	(&msm_hsl_console)

#else
#define MSM_HSL_CONSOLE	NULL
#endif

static struct uart_driver msm_hsl_uart_driver = {
	.owner = THIS_MODULE,
	.driver_name = "msm_serial_hsl",
	.dev_name = "ttyHSL",
	.nr = UART_NR,
	.cons = MSM_HSL_CONSOLE,
};

static int __devinit msm_serial_hsl_probe(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port;
	struct resource *uart_resource;
	struct resource *gsbi_resource;
	struct uart_port *port;
	int ret;

	if (unlikely(pdev->id < 0 || pdev->id >= UART_NR))
		return -ENXIO;

	printk(KERN_INFO "msm_serial_hsl: detected port #%d\n", pdev->id);

	port = get_port_from_line(pdev->id);
	port->dev = &pdev->dev;
	msm_hsl_port = UART_TO_MSM(port);

	if (msm_serial_hsl_has_gsbi()) {
		gsbi_resource =
			platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "gsbi_resource");
		if (unlikely(!gsbi_resource))
			return -ENXIO;
		msm_hsl_port->clk = clk_get(&pdev->dev, "gsbi_uart_clk");
		msm_hsl_port->pclk = clk_get(&pdev->dev, "gsbi_pclk");
	} else {
		msm_hsl_port->clk = clk_get(&pdev->dev, "uartdm_clk");
		msm_hsl_port->pclk = NULL;
	}

	if (unlikely(IS_ERR(msm_hsl_port->clk))) {
		printk(KERN_ERR "%s: Error getting clk\n", __func__);
		return PTR_ERR(msm_hsl_port->clk);
	}
	if (unlikely(IS_ERR(msm_hsl_port->pclk))) {
		printk(KERN_ERR "%s: Error getting pclk\n", __func__);
		return PTR_ERR(msm_hsl_port->pclk);
	}

	uart_resource = platform_get_resource_byname(pdev,
						     IORESOURCE_MEM,
						     "uartdm_resource");
	if (unlikely(!uart_resource)) {
		printk(KERN_ERR "getting uartdm_resource failed\n");
		return -ENXIO;
	}
	port->mapbase = uart_resource->start;

	port->irq = platform_get_irq(pdev, 0);
	if (unlikely(port->irq < 0)) {
		printk(KERN_ERR "%s: getting irq failed\n", __func__);
		return -ENXIO;
	}

	device_set_wakeup_capable(&pdev->dev, 1);
	platform_set_drvdata(pdev, port);
	pm_runtime_enable(port->dev);
	msm_hsl_debugfs_init(msm_hsl_port, pdev->id);
	ret = uart_add_one_port(&msm_hsl_uart_driver, port);

	return ret;
}

static int __devexit msm_serial_hsl_remove(struct platform_device *pdev)
{
	struct msm_hsl_port *msm_hsl_port = platform_get_drvdata(pdev);
	struct uart_port *port;

	port = get_port_from_line(pdev->id);
	pm_runtime_put_sync(&pdev->dev);
	pm_runtime_disable(&pdev->dev);

	device_set_wakeup_capable(&pdev->dev, 0);
	platform_set_drvdata(pdev, NULL);
	uart_remove_one_port(&msm_hsl_uart_driver, port);

	clk_put(msm_hsl_port->pclk);
	clk_put(msm_hsl_port->clk);
	debugfs_remove(msm_hsl_port->loopback_dir);

	return 0;
}

#ifdef CONFIG_PM
static int msm_serial_hsl_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	if (port) {
		uart_suspend_port(&msm_hsl_uart_driver, port);
		if (device_may_wakeup(dev))
			enable_irq_wake(port->irq);

		if (is_console(port))
			msm_hsl_deinit_clock(port);
	}

	return 0;
}

static int msm_serial_hsl_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	if (port) {
		if (is_console(port))
			msm_hsl_init_clock(port);
		uart_resume_port(&msm_hsl_uart_driver, port);

		if (device_may_wakeup(dev))
			disable_irq_wake(port->irq);
	}

	return 0;
}
#else
#define msm_serial_hsl_suspend NULL
#define msm_serial_hsl_resume NULL
#endif

static int msm_hsl_runtime_suspend(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	dev_dbg(dev, "pm_runtime: suspending\n");
	msm_hsl_deinit_clock(port);
	return 0;
}

static int msm_hsl_runtime_resume(struct device *dev)
{
	struct platform_device *pdev = to_platform_device(dev);
	struct uart_port *port;
	port = get_port_from_line(pdev->id);

	dev_dbg(dev, "pm_runtime: resuming\n");
	msm_hsl_init_clock(port);
	return 0;
}

static struct dev_pm_ops msm_hsl_dev_pm_ops = {
	.suspend = msm_serial_hsl_suspend,
	.resume = msm_serial_hsl_resume,
	.runtime_suspend = msm_hsl_runtime_suspend,
	.runtime_resume = msm_hsl_runtime_resume,
};

static struct platform_driver msm_hsl_platform_driver = {
	.probe	= msm_serial_hsl_probe,
	.remove = __devexit_p(msm_serial_hsl_remove),
	.driver = {
		.name = "msm_serial_hsl",
		.owner = THIS_MODULE,
		.pm = &msm_hsl_dev_pm_ops,
	},
};

static int msm_serial_hsl_init(void)
{
	int ret;

	ret = uart_register_driver(&msm_hsl_uart_driver);
	if (unlikely(ret))
		return ret;

	debug_base = debugfs_create_dir("msm_serial_hsl", NULL);
	if (IS_ERR_OR_NULL(debug_base))
		pr_err("%s():Cannot create debugfs dir\n", __func__);

	ret = platform_driver_register(&msm_hsl_platform_driver);
	if (unlikely(ret))
		uart_unregister_driver(&msm_hsl_uart_driver);

	printk(KERN_INFO "msm_serial_hsl: driver initialized\n");

	return ret;
}

static void msm_serial_hsl_exit(void)
{
	debugfs_remove_recursive(debug_base);
#ifdef CONFIG_SERIAL_MSM_HSL_CONSOLE
	unregister_console(&msm_hsl_console);
#endif
	platform_driver_unregister(&msm_hsl_platform_driver);
	uart_unregister_driver(&msm_hsl_uart_driver);
}

module_init(msm_serial_hsl_init);
module_exit(msm_serial_hsl_exit);

MODULE_DESCRIPTION("Driver for msm HSUART serial device");
MODULE_LICENSE("GPL v2");
