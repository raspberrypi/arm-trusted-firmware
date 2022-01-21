/*
 * Copyright (c) 2019, Linaro Limited
 * Copyright (c) 2019, Ying-Chun Liu (PaulLiu) <paul.liu@linaro.org>
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <assert.h>
#include <lib/mmio.h>
#include <drivers/delay_timer.h>
#include <drivers/rpi3/gpio/rpi5_gpio.h>

static uintptr_t reg_base;

static int rpi5_gpio_get_direction(int gpio);
static void rpi5_gpio_set_direction(int gpio, int direction);
static int rpi5_gpio_get_value(int gpio);
static void rpi5_gpio_set_value(int gpio, int value);
static void rpi5_gpio_set_pull(int gpio, int pull);

static const gpio_ops_t rpi5_gpio_ops = {
	.get_direction  = rpi5_gpio_get_direction,
	.set_direction  = rpi5_gpio_set_direction,
	.get_value      = rpi5_gpio_get_value,
	.set_value      = rpi5_gpio_set_value,
	.set_pull       = rpi5_gpio_set_pull,
};


static void rpi5_gpio_set_direction(int gpio, int direction)
{
	switch (direction) {
	case GPIO_DIR_IN:
		rpi5_gpio_set_select(gpio, RPI5_GPIO_FUNC_INPUT);
		break;
	case GPIO_DIR_OUT:
		rpi5_gpio_set_select(gpio, RPI5_GPIO_FUNC_OUTPUT);
		break;
	}
}

static int rpi5_gpio_get_value(int gpio)
{
	int regN = gpio / 32;
	int shift = gpio % 32;
	uintptr_t reg_lev = reg_base + RPI5_GPIO_GPLEV(regN);
	uint32_t value = mmio_read_32(reg_lev);

	if ((value >> shift) & 0x01)
		return GPIO_LEVEL_HIGH;
	return GPIO_LEVEL_LOW;
}

static void rpi5_gpio_set_value(int gpio, int value)
{
	int regN = gpio / 32;
	int shift = gpio % 32;
	uintptr_t reg_set = reg_base + RPI5_GPIO_GPSET(regN);
	uintptr_t reg_clr = reg_base + RPI5_GPIO_GPSET(regN);

	switch (value) {
	case GPIO_LEVEL_LOW:
		mmio_write_32(reg_clr, U(1) << shift);
		break;
	case GPIO_LEVEL_HIGH:
		mmio_write_32(reg_set, U(1) << shift);
		break;
	}
}

static void rpi5_gpio_set_pull(int gpio, int pull)
{
	int regN = gpio / 32;
	int shift = gpio % 32;
	uintptr_t reg_pud = reg_base + RPI5_GPIO_GPPUD;
	uintptr_t reg_clk = reg_base + RPI5_GPIO_GPPUDCLK(regN);

	switch (pull) {
	case GPIO_PULL_NONE:
		mmio_write_32(reg_pud, 0x0);
		break;
	case GPIO_PULL_UP:
		mmio_write_32(reg_pud, 0x2);
		break;
	case GPIO_PULL_DOWN:
		mmio_write_32(reg_pud, 0x1);
		break;
	}
	mdelay(150);
	mmio_write_32(reg_clk, U(1) << shift);
	mdelay(150);
	mmio_write_32(reg_clk, 0x0);
	mmio_write_32(reg_pud, 0x0);
}

void rpi5_gpio_init(void)
{
	reg_base = RPI5_GPIO_BASE;
	gpio_init(&rpi5_gpio_ops);
}
