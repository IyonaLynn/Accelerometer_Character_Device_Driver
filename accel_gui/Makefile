# Makefile for Accelerometer GUI Application

# Compiler and flags
CC ?= $(CROSS_COMPILE)gcc
PKG_CONFIG ?= pkg-config

CFLAGS ?= -Wall -Wextra -g $(shell $(PKG_CONFIG) --cflags gtk+-3.0)
LDFLAGS ?= $(shell $(PKG_CONFIG) --libs gtk+-3.0) -lgpiod -lm

# Source files
SRC = accel_gui.c
OBJ = $(SRC:.c=.o)
TARGET = accel_gui

# Default target
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Install the application
install: $(TARGET)
	install -d $(DESTDIR)$(PREFIX)/bin
	install -m 755 $(TARGET) $(DESTDIR)$(PREFIX)/bin

# Uninstall the application
uninstall:
	rm -f $(DESTDIR)$(PREFIX)/bin/$(TARGET)

# Clean up
clean:
	rm -f $(OBJ) $(TARGET)

# Run the application
run: $(TARGET)
	./$(TARGET)

.PHONY: all clean install uninstall run

