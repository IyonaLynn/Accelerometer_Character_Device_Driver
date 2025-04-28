#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/uaccess.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/ioctl.h>
#include <linux/delay.h>
#include <linux/poll.h>

#define DRIVER_NAME "lsm9ds1_accel"
#define DEVICE_NAME "accel"
#define CLASS_NAME "accel"

// I2C addresses
#define LSM9DS1_AG_ADDR 0x6B
#define LSM9DS1_M_ADDR  0x1E

// Register definitions
#define WHO_AM_I_AG     0x0F
#define CTRL_REG6_XL    0x20
#define OUT_X_L_XL      0x28

// Configuration values
#define ODR_XL_119HZ    (0x7 << 5)
#define FS_XL_2G        (0x0 << 3)

// GPIO definitions
#define LED_GPIO        17
#define BUTTON_GPIO    27

// IOCTL definitions
#define ACCEL_IOCTL_BASE 'A'
#define ACCEL_GET_DATA   _IOR(ACCEL_IOCTL_BASE, 0, struct accel_data)
#define ACCEL_SET_LED    _IOW(ACCEL_IOCTL_BASE, 1, int)
#define ACCEL_TOGGLE_SAMPLING _IO(ACCEL_IOCTL_BASE, 2)

// Data structure for accelerometer readings
struct accel_data {
    short x;
    short y;
    short z;
};

// Device structure
struct accel_device {
    struct cdev cdev;
    struct class *class;
    struct device *device;
    dev_t devno;
    struct i2c_client *client_ag;
    struct i2c_client *client_m;
    struct accel_data data;
    int led_state;
    int sampling_enabled;
    wait_queue_head_t poll_wait;
    struct mutex lock;
};

static struct accel_device *accel_dev;
static int button_irq;

// Function prototypes
static int accel_open(struct inode *inode, struct file *file);
static int accel_release(struct inode *inode, struct file *file);
static ssize_t accel_read(struct file *file, char __user *buf, size_t count, loff_t *offset);
static unsigned int accel_poll(struct file *file, poll_table *wait);
static long accel_ioctl(struct file *file, unsigned int cmd, unsigned long arg);

// File operations structure
static const struct file_operations accel_fops = {
    .owner = THIS_MODULE,
    .open = accel_open,
    .release = accel_release,
    .read = accel_read,
    .poll = accel_poll,
    .unlocked_ioctl = accel_ioctl,
};

// I2C probe function
static int accel_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
    int ret;
    uint8_t reg;
    uint8_t init_data[2];
    
    // Check if we have the right device
    reg = WHO_AM_I_AG;
    ret = i2c_master_send(client, &reg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to read WHO_AM_I register\n");
        return ret;
    }
    
    ret = i2c_master_recv(client, &reg, 1);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to receive WHO_AM_I value\n");
        return ret;
    }
    
    if (reg != 0x68) {
        dev_err(&client->dev, "Unexpected WHO_AM_I value: 0x%02x\n", reg);
        return -ENODEV;
    }
    
    // Configure accelerometer
    init_data[0] = CTRL_REG6_XL;
    init_data[1] = ODR_XL_119HZ | FS_XL_2G | 0x38; // Enable X, Y, Z axes
    
    ret = i2c_master_send(client, init_data, 2);
    if (ret < 0) {
        dev_err(&client->dev, "Failed to configure accelerometer\n");
        return ret;
    }
    
    // Store the client pointer
    if (client->addr == LSM9DS1_AG_ADDR) {
        accel_dev->client_ag = client;
    } else if (client->addr == LSM9DS1_M_ADDR) {
        accel_dev->client_m = client;
    }
    
    return 0;
}

// I2C remove function
static int accel_remove(struct i2c_client *client)
{
    return 0;
}

// I2C device ID table
static const struct i2c_device_id accel_id[] = {
    { "lsm9ds1_ag", 0 },
    { "lsm9ds1_m", 0 },
    { }
};
MODULE_DEVICE_TABLE(i2c, accel_id);

// I2C driver structure
static struct i2c_driver accel_i2c_driver = {
    .driver = {
        .name = "lsm9ds1_i2c",
    },
    .probe = accel_probe,
    .remove = accel_remove,
    .id_table = accel_id,
};

// Button interrupt handler
static irqreturn_t button_isr(int irq, void *dev_id)
{
    struct accel_device *dev = (struct accel_device *)dev_id;
    
    // Toggle sampling state
    mutex_lock(&dev->lock);
    dev->sampling_enabled = !dev->sampling_enabled;
    mutex_unlock(&dev->lock);
    
    // Wake up any waiting processes
    wake_up_interruptible(&dev->poll_wait);
    
    pr_info("Button pressed. Sampling %s\n", dev->sampling_enabled ? "enabled" : "disabled");
    return IRQ_HANDLED;
}

// Read accelerometer data
static int read_accel_data(struct accel_device *dev)
{
    int ret;
    uint8_t reg;
    uint8_t data[6];
    
    if (!dev->client_ag)
        return -ENODEV;
    
    // Set register pointer to OUT_X_L_XL with auto-increment
    reg = OUT_X_L_XL | 0x80;
    ret = i2c_master_send(dev->client_ag, &reg, 1);
    if (ret < 0) {
        dev_err(&dev->client_ag->dev, "Failed to set register pointer\n");
        return ret;
    }
    
    // Read 6 bytes of data (X, Y, Z)
    ret = i2c_master_recv(dev->client_ag, data, 6);
    if (ret < 0) {
        dev_err(&dev->client_ag->dev, "Failed to read accelerometer data\n");
        return ret;
    }
    
    // Update device data
    mutex_lock(&dev->lock);
    dev->data.x = (short)(data[1] << 8) | data[0];
    dev->data.y = (short)(data[3] << 8) | data[2];
    dev->data.z = (short)(data[5] << 8) | data[4];
    mutex_unlock(&dev->lock);
    
    return 0;
}

// File open operation
static int accel_open(struct inode *inode, struct file *file)
{
    file->private_data = accel_dev;
    return 0;
}

// File release operation
static int accel_release(struct inode *inode, struct file *file)
{
    return 0;
}

// File read operation
static ssize_t accel_read(struct file *file, char __user *buf, size_t count, loff_t *offset)
{
    struct accel_device *dev = (struct accel_device *)file->private_data;
    struct accel_data data;
    int ret;
    
    if (count < sizeof(struct accel_data))
        return -EINVAL;
    
    // Read accelerometer data
    ret = read_accel_data(dev);
    if (ret < 0)
        return ret;
    
    // Copy data to user space
    mutex_lock(&dev->lock);
    data = dev->data;
    mutex_unlock(&dev->lock);
    
    if (copy_to_user(buf, &data, sizeof(data)))
        return -EFAULT;
    
    return sizeof(data);
}

// Poll operation
static unsigned int accel_poll(struct file *file, poll_table *wait)
{
    struct accel_device *dev = (struct accel_device *)file->private_data;
    unsigned int mask = 0;
    
    poll_wait(file, &dev->poll_wait, wait);
    
    mutex_lock(&dev->lock);
    if (dev->sampling_enabled)
        mask |= POLLIN | POLLRDNORM;
    mutex_unlock(&dev->lock);
    
    return mask;
}

// IOCTL operation
static long accel_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
    struct accel_device *dev = (struct accel_device *)file->private_data;
    struct accel_data data;
    int ret;
    
    switch (cmd) {
    case ACCEL_GET_DATA:
        ret = read_accel_data(dev);
        if (ret < 0)
            return ret;
        
        mutex_lock(&dev->lock);
        data = dev->data;
        mutex_unlock(&dev->lock);
        
        if (copy_to_user((void __user *)arg, &data, sizeof(data)))
            return -EFAULT;
        break;
        
    case ACCEL_SET_LED:
        mutex_lock(&dev->lock);
        dev->led_state = arg ? 1 : 0;
        gpio_set_value(LED_GPIO, dev->led_state);
        mutex_unlock(&dev->lock);
        break;
        
    case ACCEL_TOGGLE_SAMPLING:
        mutex_lock(&dev->lock);
        dev->sampling_enabled = !dev->sampling_enabled;
        mutex_unlock(&dev->lock);
        
        // Wake up any waiting processes
        wake_up_interruptible(&dev->poll_wait);
        break;
        
    default:
        return -ENOTTY;
    }
    
    return 0;
}

// Module initialization
static int __init accel_init(void)
{
    int ret;
    
    // Allocate device structure
    accel_dev = kzalloc(sizeof(struct accel_device), GFP_KERNEL);
    if (!accel_dev)
        return -ENOMEM;
    
    // Initialize mutex and wait queue
    mutex_init(&accel_dev->lock);
    init_waitqueue_head(&accel_dev->poll_wait);
    
    // Initialize sampling state
    accel_dev->sampling_enabled = 1;
    
    // Register character device
    ret = alloc_chrdev_region(&accel_dev->devno, 0, 1, DEVICE_NAME);
    if (ret < 0) {
        pr_err("Failed to allocate device number\n");
        goto err_alloc;
    }
    
    // Initialize cdev structure
    cdev_init(&accel_dev->cdev, &accel_fops);
    accel_dev->cdev.owner = THIS_MODULE;
    
    // Add cdev to the system
    ret = cdev_add(&accel_dev->cdev, accel_dev->devno, 1);
    if (ret < 0) {
        pr_err("Failed to add character device\n");
        goto err_cdev;
    }
    
    // Create device class
    accel_dev->class = class_create(THIS_MODULE, CLASS_NAME);
    if (IS_ERR(accel_dev->class)) {
        ret = PTR_ERR(accel_dev->class);
        pr_err("Failed to create device class\n");
        goto err_class;
    }
    
    // Create device node
    accel_dev->device = device_create(accel_dev->class, NULL, accel_dev->devno, NULL, DEVICE_NAME);
    if (IS_ERR(accel_dev->device)) {
        ret = PTR_ERR(accel_dev->device);
        pr_err("Failed to create device node\n");
        goto err_device;
    }
    
    // Register I2C driver
    ret = i2c_add_driver(&accel_i2c_driver);
    if (ret < 0) {
        pr_err("Failed to register I2C driver\n");
        goto err_i2c;
    }
    
    // Setup GPIO
    ret = gpio_request(LED_GPIO, "accel_led");
    if (ret < 0) {
        pr_err("Failed to request LED GPIO\n");
        goto err_gpio_led;
    }
    
    ret = gpio_direction_output(LED_GPIO, 0);
    if (ret < 0) {
        pr_err("Failed to set LED GPIO direction\n");
        goto err_gpio_dir;
    }
    
    ret = gpio_request(BUTTON_GPIO, "accel_button");
    if (ret < 0) {
        pr_err("Failed to request button GPIO\n");
        goto err_gpio_button;
    }
    
    ret = gpio_direction_input(BUTTON_GPIO);
    if (ret < 0) {
        pr_err("Failed to set button GPIO direction\n");
        goto err_gpio_dir_button;
    }
    
    // Setup button interrupt
    button_irq = gpio_to_irq(BUTTON_GPIO);
    ret = request_irq(button_irq, button_isr, IRQF_TRIGGER_RISING, "accel_button", accel_dev);
    if (ret < 0) {
        pr_err("Failed to request button interrupt\n");
        goto err_irq;
    }
    
    pr_info("LSM9DS1 accelerometer driver loaded\n");
    return 0;
    
err_irq:
err_gpio_dir_button:
    gpio_free(BUTTON_GPIO);
err_gpio_button:
err_gpio_dir:
    gpio_free(LED_GPIO);
err_gpio_led:
    i2c_del_driver(&accel_i2c_driver);
err_i2c:
    device_destroy(accel_dev->class, accel_dev->devno);
err_device:
    class_destroy(accel_dev->class);
err_class:
    cdev_del(&accel_dev->cdev);
err_cdev:
    unregister_chrdev_region(accel_dev->devno, 1);
err_alloc:
    kfree(accel_dev);
    return ret;
}

// Module cleanup
static void __exit accel_exit(void)
{
    // Free IRQ
    free_irq(button_irq, accel_dev);
    
    // Free GPIOs
    gpio_set_value(LED_GPIO, 0);
    gpio_free(LED_GPIO);
    gpio_free(BUTTON_GPIO);
    
    // Remove I2C driver
    i2c_del_driver(&accel_i2c_driver);
    
    // Destroy device node
    device_destroy(accel_dev->class, accel_dev->devno);
    
    // Destroy device class
    class_destroy(accel_dev->class);
    
    // Remove cdev
    cdev_del(&accel_dev->cdev);
    
    // Unregister device number
    unregister_chrdev_region(accel_dev->devno, 1);
    
    // Free device structure
    kfree(accel_dev);
    
    pr_info("LSM9DS1 accelerometer driver unloaded\n");
}

module_init(accel_init);
module_exit(accel_exit);
