# Makefile for Accelerometer GUI Application

# Compiler and flags
CC = gcc
CFLAGS = -Wall -Wextra -g `pkg-config --cflags gtk+-3.0`
LDFLAGS = `pkg-config --libs gtk+-3.0` -lgpiod -lm

# Source files
SRC = accel_gui.c
OBJ = $(SRC:.c=.o)
TARGET = accel_gui

# Installation directories (if needed)
PREFIX = /usr/local
BINDIR = $(PREFIX)/bin

# Default target
all: $(TARGET)

$(TARGET): $(OBJ)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

%.o: %.c
	$(CC) $(CFLAGS) -c $< -o $@

# Install the application
install: $(TARGET)
	install -d $(BINDIR)
	install -m 755 $(TARGET) $(BINDIR)

# Uninstall the application
uninstall:
	rm -f $(BINDIR)/$(TARGET)

# Clean up
clean:
	rm -f $(OBJ) $(TARGET)

# Run the application (for testing)
run: $(TARGET)
	./$(TARGET)

# Dependencies (GTK3 and libgpiod)
deps:
	sudo apt-get install libgtk-3-dev libgpiod-dev

.PHONY: all clean install uninstall run deps
