#include <gpiod.h>
#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <signal.h>

#define CONSUMER "gpio_app"
#define GPIO_CHIP "/dev/gpiochip0"
#define LED_GPIO 17
#define BUTTON_GPIO 27

volatile sig_atomic_t stop = 0;

void sigint_handler(int signum) {
    stop = 1;
}

int main(void) {
    struct gpiod_chip *chip;
    struct gpiod_line *led_line, *button_line;
    struct gpiod_line_event event;
    int ret;
    int led_state = 0;

    // Handle Ctrl+C
    signal(SIGINT, sigint_handler);

    // Open GPIO chip
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("gpiod_chip_open");
        return 1;
    }

    // Get LED line (output)
    led_line = gpiod_chip_get_line(chip, LED_GPIO);
    if (!led_line || gpiod_line_request_output(led_line, CONSUMER, 0) < 0) {
        perror("LED line setup failed");
        gpiod_chip_close(chip);
        return 1;
    }

    // Get Button line (input with event)
    button_line = gpiod_chip_get_line(chip, BUTTON_GPIO);
    if (!button_line || gpiod_line_request_both_edges_events(button_line, CONSUMER) < 0) {
        perror("Button line setup failed");
        gpiod_chip_close(chip);
        return 1;
    }

    printf("Monitoring button on GPIO %d, toggling LED on GPIO %d\n", BUTTON_GPIO, LED_GPIO);

    while (!stop) {
        ret = gpiod_line_event_wait(button_line, NULL);
        if (ret < 0) {
            perror("gpiod_line_event_wait");
            break;
        } else if (ret == 0) {
            continue;  // timeout
        }

        if (gpiod_line_event_read(button_line, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
                led_state = !led_state;
                gpiod_line_set_value(led_line, led_state);
                printf("Button Press Detected! LED %s\n", led_state ? "ON" : "OFF");
            }
        }
    }

    gpiod_line_release(led_line);
    gpiod_line_release(button_line);
    gpiod_chip_close(chip);
    return 0;
}

