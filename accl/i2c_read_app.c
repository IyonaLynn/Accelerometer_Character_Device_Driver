#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define I2C_DEVICE "/dev/i2c-1"
#define BMA400_ADDR 0x14

// BMA400 register addresses
#define BMA400_PWR_CTRL         0x7D
#define BMA400_PWR_MODE_NORMAL  0x02
#define BMA400_CHIP_ID_REG      0x00
#define BMA400_EXPECTED_CHIP_ID 0x90

#define BMA400_ACC_X_LSB  0x04
#define BMA400_ACC_X_MSB  0x05
#define BMA400_ACC_Y_LSB  0x06
#define BMA400_ACC_Y_MSB  0x07
#define BMA400_ACC_Z_LSB  0x08
#define BMA400_ACC_Z_MSB  0x09

// Convert raw 12-bit accelerometer data to signed integer
int16_t convert_accel_data(uint8_t lsb, uint8_t msb) {
    int16_t raw = ((int16_t)msb << 8) | lsb;
    raw = raw >> 4; // 12-bit data is left-aligned
    if (raw & (1 << 11))  // Handle negative values
        raw |= 0xF000;
    return raw;
}

int main() {
    int i2c_fd;
    uint8_t data[6];

    // Open I2C device
    printf("[INFO] Opening I2C device: %s\n", I2C_DEVICE);
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("Failed to open I2C device");
        return 1;
    }

    // Set slave address
    printf("[INFO] Setting I2C slave address to 0x%X\n", BMA400_ADDR);
    if (ioctl(i2c_fd, I2C_SLAVE, BMA400_ADDR) < 0) {
        perror("Failed to set I2C slave address");
        close(i2c_fd);
        return 1;
    }

    // Wait after power-up
    usleep(2000);
    printf("[INFO] Writing 0x%02X to PWR_CTRL (0x%02X) to enter normal mode\n", BMA400_PWR_MODE_NORMAL, BMA400_PWR_CTRL);

    // Set sensor to normal mode
    uint8_t pwr_ctrl[2] = {BMA400_PWR_CTRL, BMA400_PWR_MODE_NORMAL};
    if (write(i2c_fd, pwr_ctrl, 2) != 2) {
        perror("Failed to write to PWR_CTRL register");
        close(i2c_fd);
        return 1;
    }

    // Verify the power control value
    uint8_t verify_reg = BMA400_PWR_CTRL;
    if (write(i2c_fd, &verify_reg, 1) == 1) {
        uint8_t verify_val = 0;
        if (read(i2c_fd, &verify_val, 1) == 1) {
            printf("[DEBUG] Read PWR_CTRL = 0x%02X\n", verify_val);
        } else {
            perror("[WARN] Failed to read back PWR_CTRL");
        }
    } else {
        perror("[WARN] Failed to write to PWR_CTRL for readback");
    }

    // Read CHIP ID to verify communication
    uint8_t chip_id_reg = BMA400_CHIP_ID_REG;
    if (write(i2c_fd, &chip_id_reg, 1) == 1) {
        uint8_t chip_id;
        if (read(i2c_fd, &chip_id, 1) == 1) {
            printf("[DEBUG] BMA400 CHIP ID = 0x%02X\n", chip_id);
            if (chip_id != BMA400_EXPECTED_CHIP_ID)
                printf("[WARN] Unexpected CHIP ID! Check wiring or address.\n");
        } else {
            perror("[WARN] Failed to read CHIP ID");
        }
    }

    // Wait for measurement data to be ready
    usleep(10000);  // 10ms


    // Read 6 bytes from XYZ registers
    printf("[INFO] Setting register pointer to 0x%02X to read XYZ\n", BMA400_ACC_X_LSB);
    uint8_t start_reg = BMA400_ACC_X_LSB;
    if (write(i2c_fd, &start_reg, 1) != 1) {
        perror("Failed to set register pointer");
        close(i2c_fd);
        return 1;
    }

    printf("[INFO] Reading 6 bytes from accelerometer\n");
    if (read(i2c_fd, data, 6) != 6) {
        perror("Failed to read acceleration data");
        close(i2c_fd);
        return 1;
    }

    printf("[DEBUG] Raw data bytes: X=0x%02X%02X, Y=0x%02X%02X, Z=0x%02X%02X\n",
        data[1], data[0], data[3], data[2], data[5], data[4]);

    int16_t x = convert_accel_data(data[0], data[1]);
    int16_t y = convert_accel_data(data[2], data[3]);
    int16_t z = convert_accel_data(data[4], data[5]);

    printf("[INFO] Acceleration [g]:\n");
    printf("X: %.3f\tY: %.3f\tZ: %.3f\n", x / 1024.0, y / 1024.0, z / 1024.0);

    close(i2c_fd);
    printf("[INFO] I2C device closed.\n");

    return 0;
}

