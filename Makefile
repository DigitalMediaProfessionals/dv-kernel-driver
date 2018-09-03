ARCH=$(shell gcc -print-multiarch)

ifeq ($(ARCH), arm-linux-gnueabihf)

# For on-board compiling (32-bit ARM)
MAKEARCH = $(MAKE)

else

ifeq ($(ARCH), aarch64-linux-gnu)

# For on-board compiling (64-bit ARM)
MAKEARCH = $(MAKE)

else

#ARCH ?= arm
ARCH ?= arm64
#CROSS_COMPILE ?= arm-linux-gnueabihf-
CROSS_COMPILE ?= aarch64-linux-gnu-
MAKEARCH = $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

endif

endif

BUILD_DIR := $(shell pwd)
VERBOSE = 0
PWD = $(shell pwd)
MOD = dmp_dv
#KERNELDIR ?= /lib/modules/`uname -r`/build
KERNELDIR ?= /home/ubuntu/kernel/source

obj-m := $(MOD).o
$(MOD)-objs  := ./src/dmp-dv.o ./src/command.o

all:
	$(MAKEARCH) -C $(KERNELDIR) M=$(PWD) modules

clean:
	rm -rf *.ko *.o *.order src/*.o .tmp_vers* *.symvers *.mod.c .dmp* src/.*.cmd
