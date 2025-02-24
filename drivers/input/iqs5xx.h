#pragma once

#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/input/input.h>

#define IQS5XX_DEFAULT_I2C_ADDR 0x74

#define IQS5XX_WRITE 0x00
#define IQS5XX_READ 0x01

#define IQS5XX_REG_STATUS 0x0000
#define IQS5XX_REG_REL_X 0x0002
#define IQS5XX_REG_REL_Y 0x0004
#define IQS5XX_REG_GESTURE 0x0006
#define IQS5XX_REG_END_WINDOW 0xEEEE

#define IQS5XX_STATUS_DRDY_BIT 0x01

struct iqs5xx_config;
struct iqs5xx_data;

struct iqs5xx_data {
    const struct device *dev;

    uint8_t btn_cache;
    bool in_int;

    struct gpio_callback gpio_cb;
    struct k_work work;
};

struct iqs5xx_config {
    struct i2c_dt_spec i2c;

    struct gpio_dt_spec dr;

    bool no_taps;
    bool x_invert;
    bool y_invert;
};

int iqs5xx_set_sleep(const struct device *dev, bool enabled);
