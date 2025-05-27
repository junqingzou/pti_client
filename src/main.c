/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <zephyr/kernel.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <modem/nrf_modem_lib.h>
#include <nrf_modem_at.h>
#include <modem/at_cmd_custom.h>
#include <modem/at_parser.h>
#include <zephyr/drivers/clock_control.h>
#include <zephyr/drivers/clock_control/nrf_clock_control.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/pm/device.h>
#include <zephyr/sys/reboot.h>
#include <hal/nrf_gpio.h>
#include <hal/nrf_regulators.h>
#include <hal/nrf_uarte.h>
#include <hal/nrf_power.h>
#include <hal/nrf_clock.h>
#include <hal/nrf_ipc.h>
#include "ncs_version.h"

#define VERSION "1.0.0"
#if defined(CONFIG_BOARD_NRF9151DK)
#define GPIO_INPUT_PIN 19  /** Button 4 on nRF9151DK */
#elif defined(CONFIG_BOARD_NRF9160DK)
#define GPIO_INPUT_PIN 7   /** Button 2 on nRF9160DK */
#else
#define GPIO_INPUT_PIN 99  /** Invalid GPIO pin */
#endif
#define RTC_TIMER_INTERVAL_SEC  30

#if DT_HAS_CHOSEN(ncs_at_host_uart)
static const struct device *const uart_dev = DEVICE_DT_GET(DT_CHOSEN(ncs_at_host_uart));
#else
static const struct device *const uart_dev = DEVICE_DT_GET(DT_NODELABEL(uart0));
#endif
static const struct device *gpio_dev = DEVICE_DT_GET(DT_NODELABEL(gpio0));
static struct gpio_callback gpio_cb;

static struct at_parser parser;
static int64_t start_time;

/* call thi function to dispatch like modem URC */
extern void at_monitor_dispatch(const char *notif);

/* To parse custom AT commands */
static int parse_at_command(char *at_cmd)
{
	int err = at_parser_init(&parser, at_cmd);
	if (err) {
		printk("AT command parsing failed: %d\n", err);
		return err;
	}

	enum at_parser_cmd_type type;
	err = at_parser_cmd_type_get(&parser, &type);
	if (err) {
		printk("AT command type error: %d\n", err);
		return err;
	}

	return type;
}

/* To send URC from app as like from modem */
static void send_at_urc(const char *fmt, ...)
{
	static char rsp_buf[256];

	va_list arg_ptr;

	va_start(arg_ptr, fmt);
	vsnprintf(rsp_buf, sizeof(rsp_buf), fmt, arg_ptr);
	va_end(arg_ptr);

	at_monitor_dispatch(rsp_buf);
}

/* To strictly comply with UART timing, enable external XTAL oscillator */
void enable_xtal(bool on)
{
	static struct onoff_manager *clk_mgr;

	if (on) {
		int err = 0;
		struct onoff_client cli = {};

		clk_mgr = z_nrf_clock_control_get_onoff(CLOCK_CONTROL_NRF_SUBSYS_HF);
		sys_notify_init_spinwait(&cli.notify);
		err = onoff_request(clk_mgr, &cli);
		if (err < 0) {
			printk("Clock request failed: %d\n", err);
			return;
		}
		while (sys_notify_fetch_result(&cli.notify, &err) < 0) {
			/*empty*/
		}
		printk("Request clock result: %d\n", err);
	} else {
		(void)onoff_release(clk_mgr);
	}
}

/* To shutdown serial interface */
static void power_off_uart(void)
{
	(void)pm_device_action_run(uart_dev, PM_DEVICE_ACTION_SUSPEND);

#if 0
	const struct uarte_nrfx_config {
		NRF_UARTE_Type *uarte_regs; /* Instance address */
		uint32_t flags;
		bool disable_rx;
		const struct pinctrl_dev_config *pcfg;
	} *config = uart_dev->config;
	NRF_UARTE_Type *uarte = config->uarte_regs;

	nrf_uarte_disable(uarte);
#endif
	enable_xtal(false);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/* AT+SVERSION */
static int cb_at_command_sversion(char *buf, size_t len, char *at_cmd)
{
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);

	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		return at_cmd_custom_respond(buf, len, "%s,\"%s\"\r\nOK\r\n", STRINGIFY(NCS_VERSION_STRING), VERSION);
	} else {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
}

/* AT+SRESET */
FUNC_NORETURN static int cb_at_command_sreset(char *buf, size_t len, char *at_cmd)
{
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);

	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		sys_reboot(SYS_REBOOT_COLD);
	} else {
		(void)at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////

/* AT+SSYSOFF */
FUNC_NORETURN static int cb_at_command_ssysoff(char *buf, size_t len, char *at_cmd)
{
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);

	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		power_off_uart();
		(void)nrf_modem_lib_shutdown();
		k_sleep(K_MSEC(100));
		nrf_regulators_system_off(NRF_REGULATORS_NS);
		assert(false);
	} else {
		(void)at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
}

/* AT+SUARTOFF */
static int cb_at_command_suartoff(char *buf, size_t len, char *at_cmd)
{
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);

	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		power_off_uart();
		k_sleep(K_MSEC(100));
		return 0;
	} else {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
}

NRF_MODEM_LIB_ON_INIT(mdm_init_hook, on_modem_lib_init, NULL);
NRF_MODEM_LIB_ON_SHUTDOWN(mdm_shutdown_hook, on_modem_lib_shutdown, NULL);

static void on_modem_lib_init(int ret, void *ctx)
{
	const int64_t delta = k_uptime_delta(&start_time);

	ARG_UNUSED(ctx);

	send_at_urc("+SMDMINIT: %d, %d\r\n", ret, (int)(delta%1000));
}

static void on_modem_lib_shutdown(void *ctx)
{
	ARG_UNUSED(ctx);

	// cannot send URC anymore as libmodem is down
	printk("Modem shutdown, reset device!\n");
}

/* AT+SMDMOFF */
static int cb_at_command_smdmoff(char *buf, size_t len, char *at_cmd)
{
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);

	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		power_off_uart();
		(void)nrf_modem_lib_shutdown();
	} else {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}

	return 0;
}

/* nRF9120 PS section 5.6.1.2 Sleep
 Symbol 	Description 
 ========	======================================
 IMCUOFF0	MCU off, modem off, wake on GPIO and reset
 IMCUON0 	MCU on IDLE, modem off, RTC off
 IMCUON1 	MCU on IDLE, modem off, RTC on
 IMCUON2 	MCU on IDLE, modem off, wake on GPIOTE input (event mode), Constant latency System ON mode
 IMCUON3 	MCU on IDLE, modem off, wake on GPIOTE input (event mode), Low power System ON mode
 IMCUON4 	MCU on IDLE, modem off, wake on GPIOTE input (port event) 
 IRAM	 	RAM retention leakage current of a 32kB block   // NO SUPPORT
*/
static void rtc_timer_handler(struct k_timer *timer);
K_TIMER_DEFINE(rtc_timer, rtc_timer_handler, NULL);

static void rtc_timer_handler(struct k_timer *timer)
{
	ARG_UNUSED(timer);
	static int count;

	if (count < 10) {
		k_timer_start(&rtc_timer, K_SECONDS(RTC_TIMER_INTERVAL_SEC), K_NO_WAIT);
		count++;
	}
}

enum gpio_input_mode {
	GPIO_EVENT_IN,
	GPIO_EVENT_PORT
};

static void gpio_cb_func(const struct device *dev, struct gpio_callback *gpio_callback,
			 uint32_t pins)
{
	//gpio_pin_interrupt_configure(gpio_dev, INPUT_PIN, GPIO_INT_DISABLE);
	//gpio_pin_configure(gpio_dev, INPUT_PIN, GPIO_DISCONNECTED);
	//gpio_remove_callback(gpio_dev, gpio_callback);
}

static int sleep_gpio_cfg(gpio_pin_t pin, int input_mode)
{
	int err;

	err = gpio_pin_configure(gpio_dev, pin, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
	if (err) {
		return err;
	}

	gpio_init_callback(&gpio_cb, gpio_cb_func, BIT(pin));
	err = gpio_add_callback(gpio_dev, &gpio_cb);
	if (err) {
		return err;
	}

	if (input_mode == GPIO_EVENT_IN) {
		err = gpio_pin_interrupt_configure(gpio_dev, pin, GPIO_INT_EDGE_RISING);
	} else if (input_mode == GPIO_EVENT_PORT) {
		err = gpio_pin_interrupt_configure(gpio_dev, pin, GPIO_INT_LEVEL_LOW);
	} else {
		return -EINVAL;
	}
	k_sleep(K_MSEC(100));
	return err;
}

/* Sleep test command
   AT+SSLEEP=<symbol>[,<input_pin>]
   AT+SSLEEP=?
*/
static int cb_at_command_ssleep(char *buf, size_t len, char *at_cmd)
{
	int err;
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);
	uint16_t symbol;
	uint16_t input_pin = GPIO_INPUT_PIN;
	int count;

	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		err = at_parser_num_get(&parser, 1, &symbol);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		err = at_parser_cmd_count_get(&parser, &count);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		if (count > 2) {
			err = at_parser_num_get(&parser, 2, &input_pin);
			if (err < 0) {
				return at_cmd_custom_respond(buf, len, "ERROR\r\n");
			}
		}
		if (symbol == 9) {//IMCUOFF0
			gpio_pin_configure(gpio_dev, input_pin, GPIO_INPUT | GPIO_PULL_UP | GPIO_ACTIVE_LOW);
			nrf_gpio_cfg_sense_set(input_pin, NRF_GPIO_PIN_SENSE_LOW);
			(void)nrf_modem_lib_shutdown();
			k_sleep(K_MSEC(100));
			nrf_regulators_system_off(NRF_REGULATORS_NS);
		} else if (symbol == 0) {//IMCUON0
			(void)nrf_modem_lib_shutdown();
		} else if (symbol == 1) {//IMCUON1
			(void)nrf_modem_lib_shutdown();
			k_timer_start(&rtc_timer, K_SECONDS(RTC_TIMER_INTERVAL_SEC), K_NO_WAIT);
		} else if (symbol == 2) {//IMCUON2
			(void)nrf_modem_lib_shutdown();
			sleep_gpio_cfg(input_pin, GPIO_EVENT_IN);
			nrf_power_task_trigger(NRF_POWER_NS, NRF_POWER_TASK_CONSTLAT);
		} else if (symbol == 3) {//IMCUON3
			(void)nrf_modem_lib_shutdown();
			sleep_gpio_cfg(input_pin, GPIO_EVENT_IN);
			//nrf_power_task_trigger(NRF_POWER_NS, NRF_POWER_TASK_LOWPWR);  // default SYSTEM ON mode
		} else if (symbol == 4) {//IMCUON4
			(void)nrf_modem_lib_shutdown();
			sleep_gpio_cfg(input_pin, GPIO_EVENT_PORT);
		} else {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		power_off_uart();
	} else if (cmd_type == AT_PARSER_CMD_TYPE_TEST) {
		return at_cmd_custom_respond(buf, len, "+SSLEEP=(0,1,2,3,4,9),<input_pin>\r\nOK\r\n");
	} else {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}

	return 0;
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////
enum at_gpio_operations {
	GPIO_OP_DISABLE,
	GPIO_OP_WRITE,
	GPIO_OP_READ,
	GPIO_OP_TOGGLE
};

static int handle_at_command_sgpiocfg(char *buf, size_t len, uint16_t pin, uint16_t direct, uint16_t config)
{
	int err = -EINVAL;
	gpio_flags_t gpio_flags = UINT32_MAX;

	if (direct == 1) {
		gpio_flags = GPIO_INPUT;
	} else if (direct == 2 || direct == 3) {
		if (config == 1) {			/* initializes it to a low state. */
			gpio_flags = GPIO_OUTPUT_LOW;
		} else if (config == 2 ) {		/* initializes it to a high state. */
			gpio_flags = GPIO_OUTPUT_HIGH;
		} else if (config == 3 ) {		/* initializes it to a logic 0. */
			gpio_flags = GPIO_OUTPUT_INACTIVE;
		} else if (config == 4 ) {		/* initializes it to a logic 1. */
			gpio_flags = GPIO_OUTPUT_ACTIVE; 
		} else {
			return err;
		}
		if (direct == 3) {
			gpio_flags |= GPIO_INPUT; 
		}
	} else {
		return err;
	}

	printk("GPIO config %d direct %d mask 0x%08x\n", pin, direct, gpio_flags);
	err = gpio_pin_configure(gpio_dev, pin, gpio_flags);
	if (err) {
		printk("GPIO_0 config error: %d\n", err);
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	} else {
		return at_cmd_custom_respond(buf, len, "OK\r\n");
	}
}

static int handle_at_command_sgpiotest(char *buf, size_t len, uint16_t op, uint16_t pin, uint16_t value)
{
	int err = -EINVAL;
	int gpio_value;

	if (op == GPIO_OP_DISABLE) {
		(void)gpio_pin_interrupt_configure(gpio_dev, pin, GPIO_INT_DISABLE);
		err = gpio_pin_configure(gpio_dev, pin, GPIO_DISCONNECTED);
	} else if (op == GPIO_OP_WRITE) {
		err = gpio_pin_set(gpio_dev, pin, value);
	} else if (op == GPIO_OP_TOGGLE) {
		err = gpio_pin_toggle(gpio_dev, pin);
	} else if (op ==  GPIO_OP_READ) {
		gpio_value = gpio_pin_get(gpio_dev, pin);
		if (gpio_value < 0) {
			err = gpio_value;
		} else {
			err = 0;
		}
	} else {
		err = -EINVAL;
	}
	if (err) {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	} else if (op ==  GPIO_OP_READ) {
		return at_cmd_custom_respond(buf, len, "%d\r\nOK\r\n", gpio_value);
	} else {
		return at_cmd_custom_respond(buf, len, "OK\r\n");
	}
}

/* GPIO config command
   AT+SGPIOCFG=<pin>,<direct>[,<config>]
   AT+SGPIOCFG=?
*/
static int cb_at_command_sgpiocfg(char *buf, size_t len, char *at_cmd)
{
	int err;
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);
	int count;

	if (!device_is_ready(gpio_dev)) {
		printk("GPIO controller not ready\n");
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");;
	}
	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		uint16_t pin, direct, config = 0;

		err = at_parser_num_get(&parser, 1, &pin);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		err = at_parser_num_get(&parser, 2, &direct);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		err = at_parser_cmd_count_get(&parser, &count);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		if (count > 3) {
			err = at_parser_num_get(&parser, 3, &config);
			if (err < 0) {
				return at_cmd_custom_respond(buf, len, "ERROR\r\n");
			}
		}
		return handle_at_command_sgpiocfg(buf, len, pin, direct, config);
	} else if (cmd_type == AT_PARSER_CMD_TYPE_TEST) {
		return at_cmd_custom_respond(buf, len, "+SGPIOCFG=<pin>,<direct>[,<config>]\r\nOK\r\n");
	} else if (cmd_type > 0) {
		printk("AT command type error: %d\n", cmd_type);
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	} else {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
}

/* GPIO test command
   AT+SGPIOTEST=<pin>,<op>[,<value>]
   AT+SGPIOTEST=?
*/
static int cb_at_command_sgpiotest(char *buf, size_t len, char *at_cmd)
{
	int err;
	enum at_parser_cmd_type cmd_type = parse_at_command(at_cmd);
	int count;

	if (!device_is_ready(gpio_dev)) {
		printk("GPIO controller not ready\n");
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");;
	}
	if (cmd_type == AT_PARSER_CMD_TYPE_SET) {
		uint16_t op, pin, value = 1;

		err = at_parser_num_get(&parser, 1, &pin);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		err = at_parser_num_get(&parser, 2, &op);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		err = at_parser_cmd_count_get(&parser, &count);
		if (err < 0) {
			return at_cmd_custom_respond(buf, len, "ERROR\r\n");
		}
		if (count > 3) {
			err = at_parser_num_get(&parser, 3, &value);
			if (err < 0) {
				return at_cmd_custom_respond(buf, len, "ERROR\r\n");
			}
		}
		return handle_at_command_sgpiotest(buf, len, op, pin, value);
	} else if (cmd_type == AT_PARSER_CMD_TYPE_TEST) {
		return at_cmd_custom_respond(buf, len, "+SGPIOTEST=<pin>,<op>[,<value>]\r\nOK\r\n");
	} else if (cmd_type > 0) {
		printk("AT command type error: %d\n", cmd_type);
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	} else {
		return at_cmd_custom_respond(buf, len, "ERROR\r\n");
	}
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////
/***************************** AT commands table *********************************/
AT_CMD_CUSTOM(sversion_filter,     "AT+SVERSION",     cb_at_command_sversion);
AT_CMD_CUSTOM(sreset_filter,       "AT+SRESET",       cb_at_command_sreset);
AT_CMD_CUSTOM(ssysoff_filter,      "AT+SSYSOFF",      cb_at_command_ssysoff);
AT_CMD_CUSTOM(smdmoff_filter,      "AT+SMDMOFF",      cb_at_command_smdmoff);
AT_CMD_CUSTOM(suartoff_filter,     "AT+SUARTOFF",     cb_at_command_suartoff);
AT_CMD_CUSTOM(ssleep_filter,       "AT+SSLEEP",       cb_at_command_ssleep);
AT_CMD_CUSTOM(sgpiocfg_filter,     "AT+SGPIOCFG",     cb_at_command_sgpiocfg);
AT_CMD_CUSTOM(sgpiotest_filter,    "AT+SGPIOTEST",    cb_at_command_sgpiotest);
///////////////////////////////////////////////////////////////////////////////////////////////////////////

int main(void)
{
	int err;

	printk("The AT client for MFW-PTI started\n");
	printk("Reset cause: 0x%08x\n", nrf_power_resetreas_get(NRF_POWER));

	start_time = k_uptime_get();
	err = nrf_modem_lib_init();
	if (err) {
		printk("Modem library initialization failed, error: %d\n", err);
		return 0;
	}

	enable_xtal(true);
	printk("Ready\n");

	return 0;
}
