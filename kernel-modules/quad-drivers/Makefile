# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
    ccflags-y := -I/usr/xenomai/include
    obj-m := gpe.o mpu9150.o
    gpe-y := gpe-drv.o gpe-pwm.o
    mpu9150-y := mpu9150-drv.o mpu9150-irq.o

# Othewise, we are called directly from the command
# line; invoke the kernel build system.
else

KDIR    ?= /opt/embedded/bbb/linux-dev/KERNEL
PWD     := $(shell pwd)
ARCH    := arm
CROSS   := arm-linux-gnueabihf-

default:
	@echo 'Invoking build system'
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) clean

endif
