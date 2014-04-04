# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE),)
    ccflags-y := -I/usr/xenomai/include
    obj-m := gpe.o mpu9150.o
    gpe-y := gpe-drv.o gpe-pwm.o
    mpu9150-y := i2c-mpu9150.o

# Othewise, we are called directly from the command
# line; invoke the kernel build system.
else

KDIR    ?= /home/donny3000/workspace/beaglebone-orig/linux-beaglebone/kernel
PWD     := $(shell pwd)
ARCH    := arm
CROSS   := arm-linux-gnueabihf-

default:
	@echo 'Invoking build system'
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS) clean

endif
