CC      ?= $(CROSS_COMPILE)gcc
CFLAGS  ?= -g -Wall -Werror
TARGET  ?= char_driver.ko
KDIR    ?= /lib/modules/$(shell uname -r)/build
PWD     := $(shell pwd)

obj-m := lsm9ds1_accel.o
lsm9ds1_accel-objs := char_driver.o

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	rm -f *.o *.mod.c Module.symvers modules.order

install:
	sudo insmod $(TARGET)

uninstall:
	sudo rmmod lsm9ds1_accel

.PHONY: all clean install uninstall
