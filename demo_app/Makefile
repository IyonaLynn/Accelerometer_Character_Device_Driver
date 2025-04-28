CC       := gcc
CFLAGS   := -Wall -O2
TARGET   := accel_app
SRCS     := main.c

all: $(TARGET)

$(TARGET): $(SRCS)
	$(CC) $(CFLAGS) -o $@ $^

clean:
	rm -f $(TARGET)

