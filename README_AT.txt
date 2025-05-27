## General AT commands

### Read NCS and test app version

AT+SVERSION
<NCS_version>,<APP_version>
OK

### Soft reset the module

AT+SRESET

## Power measurement commands

Must reset device to get back AT channel after issuing these commands.

### Put the module in SYSTEM OFF power mode

AT+SSYSOFF

### Shutdown modem and power off serial interface, APP core SYSTEM ON Idle

AT+SMDMOFF

### Power off serial interface only

AT+SUARTOFF

### PS section 5.6.1.2 Sleep test cases

AT+SSLEEP=<symbol>
AT+SSLEEP=?

<symbol>
0 - MCU on IDLE, modem off, RTC off
1 - MCU on IDLE, modem off, RTC on // time out every 30 seconds, max 5 rounds
2 - MCU on IDLE, modem off, wake on GPIOTE input (event mode), Constant latency System ON mode
3 - MCU on IDLE, modem off, wake on GPIOTE input (event mode), Low power System ON mode  // default
4 - MCU on IDLE, modem off, wake on GPIOTE input (port event) // default Low power System ON mode
9 - MCU off, modem off, wake on GPIO and reset

## GPIO commands

### GPIO configuration

AT+SGPIOCFG=<pin>,<direct>[,<config>]
AT+SGPIOCFG=?

<pin>: Port 0 pin number, 0 ~ 31
<direct>: 1 - Input; 2 - Output; 3 - Input & Output
<config>:
  Required when <direct> is 2 or 3.
  1 - Configures GPIO pin as output and initializes it to a low state.
  2 - Configures GPIO pin as output and initializes it to a high state.
  3 - Configures GPIO pin as output and initializes it to a logic 0.
  4 - Configures GPIO pin as output and initializes it to a logic 1.

NOTE1 - Internal pull-up or pull-down is configured by DTS
NOTE2 - GPIO active level is configured by DTS

### GPIO test

AT+SGPIOTEST=<pin>,<op>[,<value>]
AT+SGPIOTEST=?

<pin>: Port 0 pin number, 0 ~ 31
<op>: 0 - Disable; 1 - Write; 2 - Read; 3- Toggle
<value>:
  0 - Logical 0.
  1 - Logical 1, default value if not specified.

AT+SGPIOTEST=<pin>,2
<value>
OK

AT+SGPIOTEST=<pin>,(0,1,3)[,<value>]
OK
