/*
 * char_driver.c
 *
 * Kernel-space I2C accelerometer driver that toggles an LED on threshold
 * and uses a button to enable/disable sampling.
 * Tested user-space via gpio_test.c (libgpiod) before porting to kernel.
 * Setup Reference: https://www.youtube.com/watch?v=D6bhLTP_mNI
 * Code reference: https://www.ics.com/blog/gpio-programming-exploring-libgpiod-library
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/fs.h>
#include <linux/miscdevice.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/gpio/consumer.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/mm.h>
#include <linux/highmem.h>
#include <linux/mman.h>
#include <linux/mutex.h>

/* I2C and accelerometer registers */
#define DEVICE_NAME       "accel"
#define WHO_AM_I_REG      0x0F
#define CTRL_REG6_XL      0x20
#define ODR_XL_119HZ      (0x7 << 5)
#define FS_XL_2G          (0x0 << 3)
#define OUT_X_L_XL        0x28

/* Ring-buffer in one page */
struct accel_sample {
    int16_t x, y, z;
    int16_t _pad;    /* pad to 8 bytes */
};
#define SAMPLES_PER_PAGE  (PAGE_SIZE / sizeof(struct accel_sample))

/* IOCTL definitions */
#define ACCEL_IOC_MAGIC   'a'
#define ACCEL_IOC_GET_STATE  _IOR(ACCEL_IOC_MAGIC, 1, int)
#define ACCEL_IOC_SET_THRESH _IOW(ACCEL_IOC_MAGIC, 2, int)
#define ACCEL_IOC_GET_THRESH _IOR(ACCEL_IOC_MAGIC, 3, int)

static struct i2c_client    *accel_client;
static struct gpio_desc     *led_gpiod;
static struct gpio_desc     *btn_gpiod;
static struct page          *buffer_page;
static struct accel_sample  *buffer_virt;
static unsigned int          write_pos;
static bool                  sampling_enabled = true;
static int                   threshold = 5000;
static int                   btn_irq;
static DEFINE_MUTEX          buf_lock;

/* Button IRQ toggles sampling */
static irqreturn_t button_irq_handler(int irq, void *dev_id)
{
    sampling_enabled = !sampling_enabled;
    /* turn LED off when disabling */
    if (!sampling_enabled)
        gpiod_set_value(led_gpiod, 0);
    dev_info(&accel_client->dev,
             "sampling %s\n",
             sampling_enabled ? "ENABLED" : "DISABLED");
    return IRQ_HANDLED;
}

/* Read one sample via I2C, store in buffer, copy to user */
static ssize_t accel_read(struct file *file, char __user *buf,
                          size_t count, loff_t *ppos)
{
    struct accel_sample sample;
    uint8_t raw[6];
    int ret;

    if (count < sizeof(sample))
        return -EINVAL;
    if (!sampling_enabled)
        return 0;

    ret = i2c_smbus_read_i2c_block_data(accel_client,
                                        OUT_X_L_XL | 0x80,
                                        6, raw);
    if (ret < 0)
        return ret;

    sample.x = (raw[1] << 8) | raw[0];
    sample.y = (raw[3] << 8) | raw[2];
    sample.z = (raw[5] << 8) | raw[4];

    /* update ring buffer */
    mutex_lock(&buf_lock);
    buffer_virt[write_pos] = sample;
    write_pos = (write_pos + 1) % SAMPLES_PER_PAGE;
    mutex_unlock(&buf_lock);

    /* LED on if threshold exceeded */
    if (abs(sample.x) > threshold ||
        abs(sample.y) > threshold ||
        abs(sample.z) > threshold)
        gpiod_set_value(led_gpiod, 1);
    else
        gpiod_set_value(led_gpiod, 0);

    if (copy_to_user(buf, &sample, sizeof(sample)))
        return -EFAULT;
    return sizeof(sample);
}

/* mmap(): map the single page ring buffer into user space */
static int accel_mmap(struct file *file, struct vm_area_struct *vma)
{
    unsigned long pfn = page_to_pfn(buffer_page);
    if (vma->vm_end - vma->vm_start != PAGE_SIZE)
        return -EINVAL;
    /* read-only mapping of our buffer */
    vma->vm_flags |= VM_DONTEXPAND | VM_DONTDUMP;
    if (remap_pfn_range(vma,
                        vma->vm_start,
                        pfn,
                        PAGE_SIZE,
                        vma->vm_page_prot))
        return -EAGAIN;
    return 0;
}

/* ioctl(): get/set sampling state and threshold */
static long accel_ioctl(struct file *file,
                        unsigned int cmd,
                        unsigned long arg)
{
    int tmp;
    switch (cmd) {
    case ACCEL_IOC_GET_STATE:
        return put_user(sampling_enabled, (int __user *)arg);
    case ACCEL_IOC_SET_THRESH:
        if (get_user(tmp, (int __user *)arg))
            return -EFAULT;
        threshold = tmp;
        return 0;
    case ACCEL_IOC_GET_THRESH:
        return put_user(threshold, (int __user *)arg);
    default:
        return -ENOTTY;
    }
}

static const struct file_operations accel_fops = {
    .owner          = THIS_MODULE,
    .read           = accel_read,
    .mmap           = accel_mmap,
    .unlocked_ioctl = accel_ioctl,
    .llseek         = default_llseek,
};

static struct miscdevice accel_misc_device = {
    .minor = MISC_DYNAMIC_MINOR,
    .name  = DEVICE_NAME,
    .fops  = &accel_fops,
};

static int accel_probe(struct i2c_client *client,
                       const struct i2c_device_id *id)
{
    int ret;
    uint8_t who;

    accel_client = client;

    /* allocate one page for ring buffer */
    buffer_page = alloc_page(GFP_KERNEL);
    if (!buffer_page)
        return -ENOMEM;
    buffer_virt = page_address(buffer_page);
    memset(buffer_virt, 0, PAGE_SIZE);
    write_pos = 0;

    /* register /dev/accel */
    ret = misc_register(&accel_misc_device);
    if (ret) {
        dev_err(&client->dev,
                "misc_register failed: %d\n", ret);
        goto err_page;
    }

    /* verify WHO_AM_I */
    who = i2c_smbus_read_byte_data(client, WHO_AM_I_REG);
    if (who != 0x68) {
        dev_err(&client->dev,
                "bad WHO_AM_I: 0x%02x\n", who);
        ret = -ENODEV;
        goto err_misc;
    }

    /* init accelerometer */
    ret = i2c_smbus_write_byte_data(client,
                                    CTRL_REG6_XL,
                                    ODR_XL_119HZ | FS_XL_2G | 0x38);
    if (ret < 0) {
        dev_err(&client->dev,
                "CTRL_REG6_XL write failed: %d\n", ret);
        goto err_misc;
    }
    msleep(20);

    /* get GPIO descriptors */
    led_gpiod = devm_gpiod_get(&client->dev,
                               "led",
                               GPIOD_OUT_LOW);
    if (IS_ERR(led_gpiod)) {
        ret = PTR_ERR(led_gpiod);
        dev_err(&client->dev,
                "LED GPIO failed: %d\n", ret);
        goto err_misc;
    }
    btn_gpiod = devm_gpiod_get(&client->dev,
                               "button",
                               GPIOD_IN);
    if (IS_ERR(btn_gpiod)) {
        ret = PTR_ERR(btn_gpiod);
        dev_err(&client->dev,
                "Button GPIO failed: %d\n", ret);
        goto err_misc;
    }

    /* map button GPIO to IRQ and register handler */
    btn_irq = gpiod_to_irq(btn_gpiod);
    if (btn_irq < 0) {
        dev_err(&client->dev,
                "gpiod_to_irq failed: %d\n", btn_irq);
        ret = btn_irq;
        goto err_misc;
    }
    ret = devm_request_threaded_irq(&client->dev,
                                    btn_irq,
                                    NULL,
                                    button_irq_handler,
                                    IRQF_TRIGGER_RISING |
                                    IRQF_ONESHOT,
                                    "accel_button",
                                    NULL);
    if (ret) {
        dev_err(&client->dev,
                "request_irq failed: %d\n", ret);
        goto err_misc;
    }

    dev_info(&client->dev, "accel driver loaded\n");
    return 0;

err_misc:
    misc_deregister(&accel_misc_device);
err_page:
    __free_page(buffer_page);
    return ret;
}

static int accel_remove(struct i2c_client *client)
{
    misc_deregister(&accel_misc_device);
    __free_page(buffer_page);
    dev_info(&client->dev, "accel driver unloaded\n");
    return 0;
}

static const struct i2c_device_id accel_id[] = {
    { DEVICE_NAME, 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, accel_id);

static struct i2c_driver accel_driver = {
    .driver = {
        .name = DEVICE_NAME,
    },
    .probe    = accel_probe,
    .remove   = accel_remove,
    .id_table = accel_id,
};
module_i2c_driver(accel_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Iyona Lynn Noronha");
MODULE_DESCRIPTION("I2C accel driver");

