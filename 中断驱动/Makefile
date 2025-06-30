KERNELDIR := /home/fanelu/linux/linux-kernel
CURRENT_PATH := $(shell pwd)

obj-m := irq_driver_m3.o

build: kernel_modules

kernel_modules:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) modules

clean:
	$(MAKE) -C $(KERNELDIR) M=$(CURRENT_PATH) clean
