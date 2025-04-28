// char_driver.c
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/delay.h>       // for msleep()

#define DEVICE_NAME      "accel"
#define I2C_BUS_NUM      1
#define LSM9DS1_AG_ADDR  0x6B

/* Accelerometer registers */
#define WHO_AM_I_AG      0x0F
#define CTRL_REG6_XL     0x20
#define ODR_XL_119HZ     (0x7 << 5)
#define FS_XL_2G         (0x0 << 3)
#define OUT_X_L_XL       0x28

#define LED_GPIO         17
#define BUTTON_GPIO      27
#define THRESHOLD_RAW    5000   /* tune this */

struct accel_sample {
    int16_t x, y, z;
};

static struct i2c_client  *accel_client;
static bool                sampling_enabled = true;
static int                 irq_number;

/* Button IRQ: toggle sampling on/off */
static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
    sampling_enabled = !sampling_enabled;
    if (!sampling_enabled)
        gpio_set_value(LED_GPIO, 0);
    pr_info("accel: sampling %s\n",
            sampling_enabled ? "ENABLED" : "DISABLED");
    return IRQ_HANDLED;
}

/* /dev/accel read: pull one sample */
static ssize_t accel_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
    struct accel_sample sample;
    int ret;
    uint8_t raw[6];

    if (count < sizeof(sample))
        return -EINVAL;

    if (!sampling_enabled)
        return 0;

    /* burst-read X/Y/Z LSB+MSB */
    ret = i2c_smbus_read_i2c_block_data(accel_client,
                                        OUT_X_L_XL | 0x80,
                                        6, raw);
    if (ret < 0)
        return ret;

    sample.x = (int16_t)(raw[1] << 8 | raw[0]);
    sample.y = (int16_t)(raw[3] << 8 | raw[2]);
    sample.z = (int16_t)(raw[5] << 8 | raw[4]);

    /* LED on if any axis exceeds threshold */
    if (abs(sample.x) > THRESHOLD_RAW ||
        abs(sample.y) > THRESHOLD_RAW ||
        abs(sample.z) > THRESHOLD_RAW)
        gpio_set_value(LED_GPIO, 1);
    else
        gpio_set_value(LED_GPIO, 0);

    if (copy_to_user(buf, &sample, sizeof(sample)))
        return -EFAULT;

    return sizeof(sample);
}

static const struct file_operations accel_fops = {
    .owner = THIS_MODULE,
    .read  = accel_read,
};

static struct miscdevice accel_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME,
    .fops  = &accel_fops,
};

static int __init accel_init(void)
{
    struct i2c_adapter *adap;
    int ret;
    uint8_t who;

    /* 1) register /dev/accel */
    ret = misc_register(&accel_misc_device);
    if (ret) {
        pr_err("accel: misc_register failed (%d)\n", ret);
        return ret;
    }

    /* 2) bind to I²C sensor */
    adap = i2c_get_adapter(I2C_BUS_NUM);
    if (!adap) {
        ret = -ENODEV;
        pr_err("accel: get_adapter failed\n");
        goto err_misc;
    }
    accel_client = i2c_new_dummy_device(adap, LSM9DS1_AG_ADDR);
    i2c_put_adapter(adap);
    if (IS_ERR(accel_client)) {
        ret = PTR_ERR(accel_client);
        pr_err("accel: new_dummy_device failed (%d)\n", ret);
        goto err_misc;
    }

    /* 2a) verify chip ID */
    who = i2c_smbus_read_byte_data(accel_client, WHO_AM_I_AG);
    if (who != 0x68) {
        pr_err("accel: bad WHO_AM_I = 0x%02x\n", who);
        ret = -ENODEV;
        goto err_i2c;
    }

    /* 2b) init accelerometer: 119Hz, 2g, enable XYZ */
    ret = i2c_smbus_write_byte_data(accel_client,
                                    CTRL_REG6_XL,
                                    ODR_XL_119HZ | FS_XL_2G | 0x38);
    if (ret < 0) {
        pr_err("accel: CTRL_REG6_XL write failed (%d)\n", ret);
        goto err_i2c;
    }
    msleep(20);

    /* 3) request LED GPIO */
    ret = gpio_request(LED_GPIO, "accel_led");
    if (ret) {
        pr_err("accel: gpio_request LED failed (%d)\n", ret);
        goto err_i2c;
    }
    gpio_direction_output(LED_GPIO, 0);

    /* 4) request Button GPIO & IRQ */
    ret = gpio_request(BUTTON_GPIO, "accel_btn");
    if (ret) {
        pr_err("accel: gpio_request BTN failed (%d)\n", ret);
        goto err_led;
    }
    gpio_direction_input(BUTTON_GPIO);

    irq_number = gpio_to_irq(BUTTON_GPIO);
    ret = request_irq(irq_number, button_irq_handler,
                      IRQF_TRIGGER_RISING,
                      "accel_button", NULL);
    if (ret) {
        pr_err("accel: request_irq failed (%d)\n", ret);
        goto err_btn;
    }

    pr_info("accel: driver loaded\n");
    return 0;

err_btn:
    gpio_free(BUTTON_GPIO);
err_led:
    gpio_free(LED_GPIO);
err_i2c:
    i2c_unregister_device(accel_client);
err_misc:
    misc_deregister(&accel_misc_device);
    return ret;
}

static void __exit accel_exit(void)
{
    free_irq(irq_number,   NULL);
    gpio_free(BUTTON_GPIO);
    gpio_free(LED_GPIO);
    i2c_unregister_device(accel_client);
    misc_deregister(&accel_misc_device);
    pr_info("accel: driver unloaded\n");
}

module_init(accel_init);
module_exit(accel_exit);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Iyona Lynn Noronha");
MODULE_DESCRIPTION("I2C-based accelerometer char driver (miscdevice)");

