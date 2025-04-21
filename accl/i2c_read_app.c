#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define I2C_DEVICE "/dev/i2c-1"
#define BMA400_ADDR 0x14

// Register definitions
#define BMA400_PWR_CTRL        0x7D
#define BMA400_PWR_MODE_SLEEP  0x00
#define BMA400_PWR_MODE_NORMAL 0x02
#define BMA400_ACC_CONFIG0     0x7C
#define BMA400_ODR_12_5HZ      0x06
#define BMA400_CHIP_ID_REG     0x00
#define BMA400_EXPECTED_CHIP_ID 0x90

#define BMA400_ACC_X_LSB  0x04
#define BMA400_ACC_Y_LSB  0x06
#define BMA400_ACC_Z_LSB  0x08

int16_t convert_accel_data(uint8_t lsb, uint8_t msb) {
    int16_t raw = ((int16_t)msb << 8) | lsb;
    raw = raw >> 4;
    if (raw & (1 << 11))
        raw |= 0xF000;
    return raw;
}

int main() {
    int i2c_fd;
    uint8_t data[6];

    printf("[INFO] Opening I2C device: %s\n", I2C_DEVICE);
    i2c_fd = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd < 0) {
        perror("[ERROR] Failed to open I2C device");
        return 1;
    }

    printf("[INFO] Setting I2C slave address to 0x%X\n", BMA400_ADDR);
    if (ioctl(i2c_fd, I2C_SLAVE, BMA400_ADDR) < 0) {
        perror("[ERROR] Failed to set I2C slave address");
        close(i2c_fd);
        return 1;
    }

    usleep(2000);  // Wait for power stabilization

    // Verify CHIP ID
    uint8_t chip_id_reg = BMA400_CHIP_ID_REG;
    if (write(i2c_fd, &chip_id_reg, 1) == 1) {
        uint8_t chip_id;
        if (read(i2c_fd, &chip_id, 1) == 1) {
            printf("[DEBUG] BMA400 CHIP ID = 0x%02X\n", chip_id);
            if (chip_id != BMA400_EXPECTED_CHIP_ID)
                printf("[WARN] Unexpected CHIP ID! Check sensor or address\n");
        } else {
            perror("[WARN] Failed to read CHIP ID");
        }
    }

    // Step 1: Set to SLEEP mode
    uint8_t sleep_mode[2] = {BMA400_PWR_CTRL, BMA400_PWR_MODE_SLEEP};
    if (write(i2c_fd, sleep_mode, 2) != 2) {
        perror("[ERROR] Failed to set SLEEP mode");
        close(i2c_fd);
        return 1;
    }
    printf("[INFO] BMA400 set to SLEEP mode\n");
    usleep(100000);

    // Step 2: Set to NORMAL mode
    uint8_t normal_mode[2] = {BMA400_PWR_CTRL, BMA400_PWR_MODE_NORMAL};
    if (write(i2c_fd, normal_mode, 2) != 2) {
        perror("[ERROR] Failed to set NORMAL mode");
        close(i2c_fd);
        return 1;
    }
    printf("[INFO] BMA400 set to NORMAL mode\n");
    usleep(100000);

    // Step 3: Set ODR = 12.5Hz
    uint8_t odr_config[2] = {BMA400_ACC_CONFIG0, BMA400_ODR_12_5HZ};
    if (write(i2c_fd, odr_config, 2) != 2) {
        perror("[ERROR] Failed to write ACC_CONFIG0 (ODR)");
    } else {
        printf("[INFO] Set ODR to 12.5Hz in ACC_CONFIG0\n");
    }
    usleep(100000);

    // Step 4: Read XYZ acceleration values
    uint8_t start_reg = BMA400_ACC_X_LSB;
    if (write(i2c_fd, &start_reg, 1) != 1) {
        perror("[ERROR] Failed to set register pointer to XYZ");
        close(i2c_fd);
        return 1;
    }

    if (read(i2c_fd, data, 6) != 6) {
        perror("[ERROR] Failed to read XYZ data");
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

