#include <stdio.h>
#include <fcntl.h>
#include <unistd.h>

int main() {
    int fd = open("/dev/accel", O_RDONLY);
    if (fd < 0) {
        perror("open");
        return 1;
    }

    char buffer[6]; // Assume 3-axis x 2 bytes
    if (read(fd, buffer, sizeof(buffer)) < 0) {
        perror("read");
        close(fd);
        return 1;
    }

    printf("Raw data: %02x %02x %02x %02x %02x %02x\n",
           buffer[0], buffer[1], buffer[2], buffer[3], buffer[4], buffer[5]);

    close(fd);
    return 0;
}

