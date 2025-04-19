#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>

#define I2C_BUS "/dev/i2c-1"
#define BMA400_ADDR 0x14

#define BMA400_DATA_START_REG 0x04  // X_LSB register, auto-increments for X, Y, Z

int main() {
    int fd;
    char buf[6];

    // Open I2C device
    fd = open(I2C_BUS, O_RDWR);
    if (fd < 0) {
        perror("open I2C bus");
        return 1;
    }

    // Set I2C slave address
    if (ioctl(fd, I2C_SLAVE, BMA400_ADDR) < 0) {
        perror("Failed to set I2C address");
        close(fd);
        return 1;
    }

    // Select starting register (X_LSB)
    char reg = BMA400_DATA_START_REG;
    if (write(fd, &reg, 1) != 1) {
        perror("Failed to write register address");
        close(fd);
        return 1;
    }

    // Read 6 bytes: X_LSB, X_MSB, Y_LSB, Y_MSB, Z_LSB, Z_MSB
    if (read(fd, buf, 6) != 6) {
        perror("Failed to read sensor data");
        close(fd);
        return 1;
    }

    // Combine bytes into signed 16-bit values
    int16_t x = (buf[1] << 8) | buf[0];
    int16_t y = (buf[3] << 8) | buf[2];
    int16_t z = (buf[5] << 8) | buf[4];

    printf("Accel Raw Values:\n");
    printf("X: %d\n", x);
    printf("Y: %d\n", y);
    printf("Z: %d\n", z);

    close(fd);
    return 0;
}

