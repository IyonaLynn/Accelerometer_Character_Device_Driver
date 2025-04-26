#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <stdint.h>
#include <gpiod.h>
#include <gtk/gtk.h>
#include <glib.h>
#include <math.h>
#include <time.h>

// I2C Configuration
#define I2C_DEVICE "/dev/i2c-1"
#define LSM9DS1_AG_ADDR 0x6B
#define LSM9DS1_M_ADDR  0x1E

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
#define M_ULTRA_HIGH    (0x0 << 5)
#define M_ODR_80HZ      (0x7 << 2)

// GPIO Configuration
#define CONSUMER "accel_gui"
#define GPIO_CHIP "/dev/gpiochip0"
#define LED_GPIO 17
#define BUTTON_GPIO 27

typedef struct {
    GtkLabel *label;
    const gchar *text;
} LabelUpdateData;

// Global variables
static int i2c_fd_ag = -1;
static int i2c_fd_m = -1;
static struct gpiod_chip *chip = NULL;
static struct gpiod_line *led_line = NULL;
static struct gpiod_line *button_line = NULL;
static gboolean sampling_enabled = TRUE;
static gboolean running = TRUE;
static GtkWidget *accel_x_label, *accel_y_label, *accel_z_label;
static GtkWidget *gyro_x_label, *gyro_y_label, *gyro_z_label;
static GtkWidget *mag_x_label, *mag_y_label, *mag_z_label;
static GtkWidget *led_status_label, *sampling_status_label;
static GtkWidget *tilt_indicator;

// Conversion functions
float convert_accel(int16_t raw) {
    return raw * 0.061 / 1000.0;
}

float convert_gyro(int16_t raw) {
    return raw * 8.75 / 1000.0;
}

float convert_mag(int16_t raw) {
    return raw * 0.14;
}

// Initialize I2C communication
int init_i2c() {
    // Open I2C device for accelerometer/gyroscope
    i2c_fd_ag = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd_ag < 0) {
        perror("Failed to open I2C device for AG");
        return -1;
    }

    // Open I2C device for magnetometer
    i2c_fd_m = open(I2C_DEVICE, O_RDWR);
    if (i2c_fd_m < 0) {
        perror("Failed to open I2C device for M");
        close(i2c_fd_ag);
        return -1;
    }

    // Set slave addresses
    if (ioctl(i2c_fd_ag, I2C_SLAVE, LSM9DS1_AG_ADDR) < 0) {
        perror("Failed to set I2C slave address for AG");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return -1;
    }

    if (ioctl(i2c_fd_m, I2C_SLAVE, LSM9DS1_M_ADDR) < 0) {
        perror("Failed to set I2C slave address for M");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return -1;
    }

    usleep(2000);  // Wait for power stabilization

    // Verify AG CHIP ID
    uint8_t reg = WHO_AM_I_AG;
    write(i2c_fd_ag, &reg, 1);
    uint8_t chip_id_ag;
    read(i2c_fd_ag, &chip_id_ag, 1);

    if (chip_id_ag != EXPECTED_WHO_AM_I_AG) {
        printf("Unexpected AG CHIP ID! Check sensor or address\n");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return -1;
    }

    // Verify M CHIP ID
    reg = WHO_AM_I_M;
    write(i2c_fd_m, &reg, 1);
    uint8_t chip_id_m;
    read(i2c_fd_m, &chip_id_m, 1);

    if (chip_id_m != EXPECTED_WHO_AM_I_M) {
        printf("Unexpected M CHIP ID! Check sensor or address\n");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return -1;
    }

    // Configure Gyroscope
    uint8_t init_ag[][2] = {
        {CTRL_REG1_G, ODR_G_119HZ | FS_G_245DPS | 0x01},
        {CTRL_REG6_XL, ODR_XL_119HZ | FS_XL_2G | 0x38}
    };

    for (int i = 0; i < 2; i++) {
        if (write(i2c_fd_ag, init_ag[i], 2) != 2) {
            perror("AG initialization failed");
            close(i2c_fd_ag);
            close(i2c_fd_m);
            return -1;
        }
        usleep(20000);
    }

    // Configure Magnetometer
    uint8_t init_m[][2] = {
        {CTRL_REG1_M, M_ULTRA_HIGH | M_ODR_80HZ},
        {CTRL_REG2_M, 0x00}
    };

    for (int i = 0; i < 2; i++) {
        if (write(i2c_fd_m, init_m[i], 2) != 2) {
            perror("M initialization failed");
            close(i2c_fd_ag);
            close(i2c_fd_m);
            return -1;
        }
        usleep(20000);
    }

    return 0;
}

// Initialize GPIO
int init_gpio() {
    // Open GPIO chip
    chip = gpiod_chip_open(GPIO_CHIP);
    if (!chip) {
        perror("gpiod_chip_open");
        return -1;
    }

    // Get LED line (output)
    led_line = gpiod_chip_get_line(chip, LED_GPIO);
    if (!led_line || gpiod_line_request_output(led_line, CONSUMER, 0) < 0) {
        perror("LED line setup failed");
        gpiod_chip_close(chip);
        return -1;
    }

    // Get Button line (input with event)
    button_line = gpiod_chip_get_line(chip, BUTTON_GPIO);
    if (!button_line || gpiod_line_request_both_edges_events(button_line, CONSUMER) < 0) {
        perror("Button line setup failed");
        gpiod_chip_close(chip);
        return -1;
    }

    return 0;
}

// Read sensor data
void read_sensors(float *accel, float *gyro, float *mag) {
    uint8_t data[6];
    uint8_t reg;

    if (!sampling_enabled) {
        return;
    }

    // Read Accelerometer data
    reg = OUT_X_L_XL | 0x80;
    if (write(i2c_fd_ag, &reg, 1) != 1) {
        perror("Failed to set register pointer for accel");
        return;
    }

    if (read(i2c_fd_ag, data, 6) != 6) {
        perror("Failed to read accel data");
        return;
    }

    accel[0] = convert_accel((int16_t)(data[1] << 8) | data[0]);
    accel[1] = convert_accel((int16_t)(data[3] << 8) | data[2]);
    accel[2] = convert_accel((int16_t)(data[5] << 8) | data[4]);

    // Read Gyroscope data
    reg = OUT_X_L_G | 0x80;
    if (write(i2c_fd_ag, &reg, 1) != 1) {
        perror("Failed to set register pointer for gyro");
        return;
    }

    if (read(i2c_fd_ag, data, 6) != 6) {
        perror("Failed to read gyro data");
        return;
    }

    gyro[0] = convert_gyro((int16_t)(data[1] << 8) | data[0]);
    gyro[1] = convert_gyro((int16_t)(data[3] << 8) | data[2]);
    gyro[2] = convert_gyro((int16_t)(data[5] << 8) | data[4]);

    // Read Magnetometer data
    reg = OUT_X_L_M | 0x80;
    if (write(i2c_fd_m, &reg, 1) != 1) {
        perror("Failed to set register pointer for mag");
        return;
    }

    if (read(i2c_fd_m, data, 6) != 6) {
        perror("Failed to read mag data");
        return;
    }

    mag[0] = convert_mag((int16_t)(data[1] << 8) | data[0]);
    mag[1] = convert_mag((int16_t)(data[3] << 8) | data[2]);
    mag[2] = convert_mag((int16_t)(data[5] << 8) | data[4]);
}

gboolean update_label_idle(gpointer user_data) {
    LabelUpdateData *data = (LabelUpdateData *)user_data;
    gtk_label_set_text(GTK_LABEL(data->label), data->text);
    free(data);  // Free the dynamically allocated memory
    return FALSE; // FALSE = run only once
}

// Check button state
void check_button() {
    struct gpiod_line_event event;
    int ret = gpiod_line_event_wait(button_line, &(struct timespec){0, 10000000}); // 10ms timeout
    
    if (ret > 0) {
        if (gpiod_line_event_read(button_line, &event) == 0) {
            if (event.event_type == GPIOD_LINE_EVENT_RISING_EDGE) {
                sampling_enabled = !sampling_enabled;
                gpiod_line_set_value(led_line, sampling_enabled ? 1 : 0);
		LabelUpdateData *data = malloc(sizeof(LabelUpdateData));
		data->label = GTK_LABEL(sampling_status_label);
		data->text = sampling_enabled ? "Sampling: Enabled" : "Sampling: Disabled";
            }
        }
    }
}

// Update GUI with sensor data
gboolean update_gui(gpointer data) {
    (void)data;
    if (!running) return FALSE;

    float accel[3], gyro[3], mag[3];
    char buffer[32];
    
    // Read sensors
    read_sensors(accel, gyro, mag);
    
    // Update accelerometer labels
    snprintf(buffer, sizeof(buffer), "X: %.3f g", accel[0]);
    gtk_label_set_text(GTK_LABEL(accel_x_label), buffer);
    
    snprintf(buffer, sizeof(buffer), "Y: %.3f g", accel[1]);
    gtk_label_set_text(GTK_LABEL(accel_y_label), buffer);
    
    snprintf(buffer, sizeof(buffer), "Z: %.3f g", accel[2]);
    gtk_label_set_text(GTK_LABEL(accel_z_label), buffer);
    
    // Update gyroscope labels
    snprintf(buffer, sizeof(buffer), "X: %.3f dps", gyro[0]);
    gtk_label_set_text(GTK_LABEL(gyro_x_label), buffer);
    
    snprintf(buffer, sizeof(buffer), "Y: %.3f dps", gyro[1]);
    gtk_label_set_text(GTK_LABEL(gyro_y_label), buffer);
    
    snprintf(buffer, sizeof(buffer), "Z: %.3f dps", gyro[2]);
    gtk_label_set_text(GTK_LABEL(gyro_z_label), buffer);
    
    // Update magnetometer labels
    snprintf(buffer, sizeof(buffer), "X: %.3f gauss", mag[0]);
    gtk_label_set_text(GTK_LABEL(mag_x_label), buffer);
    
    snprintf(buffer, sizeof(buffer), "Y: %.3f gauss", mag[1]);
    gtk_label_set_text(GTK_LABEL(mag_y_label), buffer);
    
    snprintf(buffer, sizeof(buffer), "Z: %.3f gauss", mag[2]);
    gtk_label_set_text(GTK_LABEL(mag_z_label), buffer);
    
    // Update LED status
    gtk_label_set_text(GTK_LABEL(led_status_label), 
                      gpiod_line_get_value(led_line) ? "LED: ON" : "LED: OFF");
    
    // Update tilt indicator based on acceleration
    double tilt = sqrt(accel[0]*accel[0] + accel[1]*accel[1]);
    if (tilt > 0.5) {
        gtk_widget_set_name(tilt_indicator, "tilted");
    } else {
        gtk_widget_set_name(tilt_indicator, "not-tilted");
    }
    
    // Check button state
    check_button();
    
    return TRUE;
}

// Cleanup function
void cleanup() {
    running = FALSE;
    
    if (led_line) {
        gpiod_line_set_value(led_line, 0);
        gpiod_line_release(led_line);
    }
    
    if (button_line) {
        gpiod_line_release(button_line);
    }
    
    if (chip) {
        gpiod_chip_close(chip);
    }
    
    if (i2c_fd_ag >= 0) {
        close(i2c_fd_ag);
    }
    
    if (i2c_fd_m >= 0) {
        close(i2c_fd_m);
    }
}

// Window close handler
gboolean on_window_close(GtkWidget *widget, GdkEvent *event, gpointer data) {
    (void)widget;
    (void)event;
    (void)data;
    cleanup();
    gtk_main_quit();
    return FALSE;
}

// Toggle sampling callback
void on_toggle_sampling(GtkButton *button, gpointer data) {
    (void)button;
    (void)data;
    sampling_enabled = !sampling_enabled;
    gpiod_line_set_value(led_line, sampling_enabled ? 1 : 0);
    gtk_label_set_text(GTK_LABEL(sampling_status_label), 
                      sampling_enabled ? "Sampling: Enabled" : "Sampling: Disabled");
}

int main(int argc, char *argv[]) {
    GtkWidget *window;
    GtkWidget *grid;
    GtkWidget *frame;
    GtkWidget *label;
    GtkWidget *button;
    GtkCssProvider *css_provider;
    GtkStyleContext *context;
    
    // Initialize GTK
    gtk_init(&argc, &argv);
    
    // Initialize hardware
    if (init_i2c() < 0) {
        fprintf(stderr, "Failed to initialize I2C\n");
        return 1;
    }
    
    if (init_gpio() < 0) {
        fprintf(stderr, "Failed to initialize GPIO\n");
        close(i2c_fd_ag);
        close(i2c_fd_m);
        return 1;
    }
    
    // Create main window
    window = gtk_window_new(GTK_WINDOW_TOPLEVEL);
    gtk_window_set_title(GTK_WINDOW(window), "Accelerometer Data Monitor");
    gtk_window_set_default_size(GTK_WINDOW(window), 400, 500);
    g_signal_connect(window, "delete-event", G_CALLBACK(on_window_close), NULL);
    
    // Create grid layout
    grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(window), grid);
    gtk_grid_set_row_spacing(GTK_GRID(grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(grid), 5);
    gtk_container_set_border_width(GTK_CONTAINER(grid), 10);
    
    // Add CSS for tilt indicator
    css_provider = gtk_css_provider_new();
    gtk_css_provider_load_from_data(css_provider,
        "#tilted { background-color: red; border-radius: 50px; }"
        "#not-tilted { background-color: green; border-radius: 50px; }", -1, NULL);
    
    // Accelerometer frame
    frame = gtk_frame_new("Accelerometer Data (g)");
    gtk_grid_attach(GTK_GRID(grid), frame, 0, 0, 2, 1);
    
    GtkWidget *accel_grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(frame), accel_grid);
    gtk_grid_set_row_spacing(GTK_GRID(accel_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(accel_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(accel_grid), 10);
    
    label = gtk_label_new("X:");
    gtk_grid_attach(GTK_GRID(accel_grid), label, 0, 0, 1, 1);
    accel_x_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(accel_grid), accel_x_label, 1, 0, 1, 1);
    
    label = gtk_label_new("Y:");
    gtk_grid_attach(GTK_GRID(accel_grid), label, 0, 1, 1, 1);
    accel_y_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(accel_grid), accel_y_label, 1, 1, 1, 1);
    
    label = gtk_label_new("Z:");
    gtk_grid_attach(GTK_GRID(accel_grid), label, 0, 2, 1, 1);
    accel_z_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(accel_grid), accel_z_label, 1, 2, 1, 1);
    
    // Gyroscope frame
    frame = gtk_frame_new("Gyroscope Data (dps)");
    gtk_grid_attach(GTK_GRID(grid), frame, 0, 1, 2, 1);
    
    GtkWidget *gyro_grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(frame), gyro_grid);
    gtk_grid_set_row_spacing(GTK_GRID(gyro_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(gyro_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(gyro_grid), 10);
    
    label = gtk_label_new("X:");
    gtk_grid_attach(GTK_GRID(gyro_grid), label, 0, 0, 1, 1);
    gyro_x_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(gyro_grid), gyro_x_label, 1, 0, 1, 1);
    
    label = gtk_label_new("Y:");
    gtk_grid_attach(GTK_GRID(gyro_grid), label, 0, 1, 1, 1);
    gyro_y_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(gyro_grid), gyro_y_label, 1, 1, 1, 1);
    
    label = gtk_label_new("Z:");
    gtk_grid_attach(GTK_GRID(gyro_grid), label, 0, 2, 1, 1);
    gyro_z_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(gyro_grid), gyro_z_label, 1, 2, 1, 1);
    
    // Magnetometer frame
    frame = gtk_frame_new("Magnetometer Data (gauss)");
    gtk_grid_attach(GTK_GRID(grid), frame, 0, 2, 2, 1);
    
    GtkWidget *mag_grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(frame), mag_grid);
    gtk_grid_set_row_spacing(GTK_GRID(mag_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(mag_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(mag_grid), 10);
    
    label = gtk_label_new("X:");
    gtk_grid_attach(GTK_GRID(mag_grid), label, 0, 0, 1, 1);
    mag_x_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(mag_grid), mag_x_label, 1, 0, 1, 1);
    
    label = gtk_label_new("Y:");
    gtk_grid_attach(GTK_GRID(mag_grid), label, 0, 1, 1, 1);
    mag_y_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(mag_grid), mag_y_label, 1, 1, 1, 1);
    
    label = gtk_label_new("Z:");
    gtk_grid_attach(GTK_GRID(mag_grid), label, 0, 2, 1, 1);
    mag_z_label = gtk_label_new("0.000");
    gtk_grid_attach(GTK_GRID(mag_grid), mag_z_label, 1, 2, 1, 1);
    
    // Status frame
    frame = gtk_frame_new("Status");
    gtk_grid_attach(GTK_GRID(grid), frame, 0, 3, 2, 1);
    
    GtkWidget *status_grid = gtk_grid_new();
    gtk_container_add(GTK_CONTAINER(frame), status_grid);
    gtk_grid_set_row_spacing(GTK_GRID(status_grid), 5);
    gtk_grid_set_column_spacing(GTK_GRID(status_grid), 10);
    gtk_container_set_border_width(GTK_CONTAINER(status_grid), 10);
    
    led_status_label = gtk_label_new("LED: OFF");
    gtk_grid_attach(GTK_GRID(status_grid), led_status_label, 0, 0, 1, 1);
    
    sampling_status_label = gtk_label_new("Sampling: Enabled");
    gtk_grid_attach(GTK_GRID(status_grid), sampling_status_label, 0, 1, 1, 1);
    
    // Tilt indicator
    tilt_indicator = gtk_label_new("");
    gtk_widget_set_size_request(tilt_indicator, 50, 50);
    gtk_widget_set_name(tilt_indicator, "not-tilted");
    context = gtk_widget_get_style_context(tilt_indicator);
    gtk_style_context_add_provider(context, GTK_STYLE_PROVIDER(css_provider), 
                                  GTK_STYLE_PROVIDER_PRIORITY_USER);
    gtk_grid_attach(GTK_GRID(grid), tilt_indicator, 0, 4, 1, 1);
    
    label = gtk_label_new("Tilt Indicator");
    gtk_grid_attach(GTK_GRID(grid), label, 1, 4, 1, 1);
    
    // Control button
    button = gtk_button_new_with_label("Toggle Sampling");
    g_signal_connect(button, "clicked", G_CALLBACK(on_toggle_sampling), NULL);
    gtk_grid_attach(GTK_GRID(grid), button, 0, 5, 2, 1);
    
    // Show all widgets
    gtk_widget_show_all(window);
    
    // Add CSS provider to screen
    gtk_style_context_add_provider_for_screen(gdk_screen_get_default(),
                                            GTK_STYLE_PROVIDER(css_provider),
                                            GTK_STYLE_PROVIDER_PRIORITY_USER);
    
    // Set up timer to update GUI
    g_timeout_add(100, update_gui, NULL);
    
    // Start main loop
    gtk_main();
    
    return 0;
}
