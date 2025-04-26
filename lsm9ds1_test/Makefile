CC      ?= $(CROSS_COMPILE)gcc
CFLAGS  ?= -g -Wall -Werror
TARGET  ?= i2c_read_app

all: $(TARGET)

# Build the i2c_read_app binary
$(TARGET): i2c_read_app.c
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDES) $(LDFLAGS)

# Clean target
clean:
	-rm -f $(TARGET)

.PHONY: all clean

