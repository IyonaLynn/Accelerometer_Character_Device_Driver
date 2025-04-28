# Allow Buildroot to override CC/CFLAGS on the command line.
CC     ?= gcc
CFLAGS ?= -Wall -O2
TARGET ?= accel_app
SRCS   := main.c

.PHONY: all clean

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)

