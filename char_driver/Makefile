# Kernel build directory (Buildroot will set this via KERNEL_DIR or by exporting ARCH/CROSS_COMPILE)
KDIR   ?= $(LINUX_DIR)
PWD    := $(shell pwd)

# Build char_driver.ko from char_driver.o
obj-m  := char_driver.o

.PHONY: all clean

all:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean

