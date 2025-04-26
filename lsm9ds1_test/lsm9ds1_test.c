#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>

#define I2C_DEVICE "/dev/i2c-1"
#define LSM9DS1_AG_ADDR 0x6B  // Accelerometer/Gyroscope I2C address
#define LSM9DS1_M_ADDR  0x1E  // Magnetometer I2C address

// Accelerometer/Gyroscope Registers
#define WHO_AM_I_AG     0x0F
#define CTRL_REG1_G     0x10
#define CTRL_REG6_XL    0x20
#define OUT_X_L_XL      0x28
#define OUT_X_L_G       0x18

// Magnetometer Registers
#define WHO_AM_I_M      0x0F
#define CTRL_REG1_M     0x20
#define CTRL_REG2_M     0x21
#define OUT_X_L_M       0x28

// Expected chip IDs
#define EXPECTED_WHO_AM_I_AG  0x68
#define EXPECTED_WHO_AM_I_M   0x3D

// Configuration values
#define ODR_G_119HZ     (0x7 << 5)
#define FS_G_245DPS     (0x0 << 3)
#define ODR_XL_119HZ    (0x7 << 5)
#define FS_XL_2G        (0x0 << 3)
#define M_ULTRA_HIGH    (0x0 << 5)  // Ultra-high performance mode
#define M_ODR_80HZ      (0x7 << 2)  // 80Hz output data rate

float convert_accel(int16_t raw) {
    return raw * 0.061 / 1000.0;  // 2g range: 0.061 mg/LSB
}

float convert_gyro(int16_t raw) {
    return raw * 8.75 / 1000.0;   // 245 dps range: 8.75 mdps/LSB
}

float convert_mag(int16_t raw) {
    return raw * 0.14;            // 4 gauss range: 0.14 mgauss/LSB
}

int main() {
    int i2c_fd_ag, i2c_fd_m;
    uint8_t data[6];

    // Open I2C device for accelerometer/gyroscope
    printf("[INFO] Opening I2C device: %s\n", I2C_DEVICE);
    i2c_fd_ag = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd_ag < 0) {
        perror("[ERROR] Failed to open I2C device for AG");
        return 1;
    }

    // Open I2C device for magnetometer (same bus, different address)
    i2c_fd_m = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd_m < 0) {
        perror("[ERROR] Failed to open I2C device for M");
        close(i2c_fd_ag);
        return 1;
    }

    // Set slave addresses
    if (ioctl(i2c_fd_ag, I2C_SLAVE, LSM9DS1_AG_ADDR) < 0) {
        perror("[ERROR] Failed to set I2C slave address for AG");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    if (ioctl(i2c_fd_m, I2C_SLAVE, LSM9DS1_M_ADDR) < 0) {
        perror("[ERROR] Failed to set I2C slave address for M");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    usleep(2000);  // Wait for power stabilization

    // Verify AG CHIP ID
    uint8_t reg = WHO_AM_I_AG;
    write(i2c_fd_ag, &reg, 1);
    uint8_t chip_id_ag;
    read(i2c_fd_ag, &chip_id_ag, 1);
    printf("[DEBUG] LSM9DS1 AG CHIP ID = 0x%02X\n", chip_id_ag);

    if (chip_id_ag != EXPECTED_WHO_AM_I_AG) {
        printf("[WARN] Unexpected AG CHIP ID! Check sensor or address\n");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    // Verify M CHIP ID
    reg = WHO_AM_I_M;
    write(i2c_fd_m, &reg, 1);
    uint8_t chip_id_m;
    read(i2c_fd_m, &chip_id_m, 1);
    printf("[DEBUG] LSM9DS1 M CHIP ID = 0x%02X\n", chip_id_m);

    if (chip_id_m != EXPECTED_WHO_AM_I_M) {
        printf("[WARN] Unexpected M CHIP ID! Check sensor or address\n");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    // Configure Gyroscope
    uint8_t init_ag[][2] = {
        {CTRL_REG1_G, ODR_G_119HZ | FS_G_245DPS | 0x01},  // Enable X, Y, Z axes
        {CTRL_REG6_XL, ODR_XL_119HZ | FS_XL_2G | 0x38}    // Enable X, Y, Z axes
    };

    for (int i = 0; i < 2; i++) {
        if (write(i2c_fd_ag, init_ag[i], 2) != 2) {
            perror("[ERROR] AG initialization failed");
            close(i2c_fd_ag);
            close(i2c_fd_m);
            return 1;
        }
        usleep(20000);
    }

    // Configure Magnetometer
    uint8_t init_m[][2] = {
        {CTRL_REG1_M, M_ULTRA_HIGH | M_ODR_80HZ},  // Ultra-high performance, 80Hz
        {CTRL_REG2_M, 0x00}                        // +/-4 gauss
    };

    for (int i = 0; i < 2; i++) {
        if (write(i2c_fd_m, init_m[i], 2) != 2) {
            perror("[ERROR] M initialization failed");
            close(i2c_fd_ag);
            close(i2c_fd_m);
            return 1;
        }
        usleep(20000);
    }

    // Read Accelerometer data
    reg = OUT_X_L_XL | 0x80;  // Set MSB for auto-increment
    if (write(i2c_fd_ag, &reg, 1) != 1) {
        perror("[ERROR] Failed to set register pointer for accel");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    if (read(i2c_fd_ag, data, 6) != 6) {
        perror("[ERROR] Failed to read accel data");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    int16_t ax = (int16_t)(data[1] << 8) | data[0];
    int16_t ay = (int16_t)(data[3] << 8) | data[2];
    int16_t az = (int16_t)(data[5] << 8) | data[4];

    // Read Gyroscope data
    reg = OUT_X_L_G | 0x80;  // Set MSB for auto-increment
    if (write(i2c_fd_ag, &reg, 1) != 1) {
        perror("[ERROR] Failed to set register pointer for gyro");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    if (read(i2c_fd_ag, data, 6) != 6) {
        perror("[ERROR] Failed to read gyro data");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    int16_t gx = (int16_t)(data[1] << 8) | data[0];
    int16_t gy = (int16_t)(data[3] << 8) | data[2];
    int16_t gz = (int16_t)(data[5] << 8) | data[4];

    // Read Magnetometer data
    reg = OUT_X_L_M | 0x80;  // Set MSB for auto-increment
    if (write(i2c_fd_m, &reg, 1) != 1) {
        perror("[ERROR] Failed to set register pointer for mag");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    if (read(i2c_fd_m, data, 6) != 6) {
        perror("[ERROR] Failed to read mag data");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }

    int16_t mx = (int16_t)(data[1] << 8) | data[0];
    int16_t my = (int16_t)(data[3] << 8) | data[2];
    int16_t mz = (int16_t)(data[5] << 8) | data[4];

    // Print results
    printf("\n[INFO] Sensor Data:\n");
    printf("Acceleration [g]:\n");
    printf("X: %.3f\tY: %.3f\tZ: %.3f\n", convert_accel(ax), convert_accel(ay), convert_accel(az));
    
    printf("\nGyroscope [dps]:\n");
    printf("X: %.3f\tY: %.3f\tZ: %.3f\n", convert_gyro(gx), convert_gyro(gy), convert_gyro(gz));
    
    printf("\nMagnetometer [gauss]:\n");
    printf("X: %.3f\tY: %.3f\tZ: %.3f\n", convert_mag(mx), convert_mag(my), convert_mag(mz));

    close(i2c_fd_ag);
    close(i2c_fd_m);
    printf("\n[INFO] I2C devices closed.\n");
    return 0;
}
