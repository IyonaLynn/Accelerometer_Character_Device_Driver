#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>
#include <stdint.h>

struct accel_sample {
    int16_t x, y, z;
};

int main(void)
{
    int fd = open("/dev/accel", O_RDONLY);
    if (fd < 0) {
        perror("open /dev/accel");
        return 1;
    }

    struct accel_sample s;
    for (;;) {
        ssize_t r = read(fd, &s, sizeof(s));
        if (r < 0) {
            perror("read");
            break;
        } else if (r == 0) {
            printf("Sampling disabled (press button to re-enable)\n");
        } else {
            printf("Raw accel: X=%6d  Y=%6d  Z=%6d\n",
                   s.x, s.y, s.z);
        }
        sleep(1);
    }

    close(fd);
    return 0;
}

