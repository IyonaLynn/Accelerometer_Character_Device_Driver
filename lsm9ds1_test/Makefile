CC      ?= $(CROSS_COMPILE)gcc
CFLAGS  ?= -g -Wall -Werror
TARGET  ?= lsm9ds1_test

all: $(TARGET)

# Build the i2c_read_app binary
$(TARGET): lsm9ds1_test.c
	$(CC) $(CFLAGS) $^ -o $@ $(INCLUDES) $(LDFLAGS)

# Clean target
clean:
	-rm -f $(TARGET)

.PHONY: all clean

