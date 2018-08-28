KERNELDIR ?= /lib/modules/`uname -r`/build
CROSS_COMPILE ?= arm-linux-gnueabihf-
BUILD_DIR := $(shell pwd)
VERBOSE = 0
PWD = $(shell pwd)
MOD = dmp_dv
ARCH ?= arm

MAKEARCH = $(MAKE) ARCH=$(ARCH) CROSS_COMPILE=$(CROSS_COMPILE)

obj-m := $(MOD).o 
$(MOD)-objs  := ./src/dmp-dv.o ./src/command.o

all: 
	$(MAKEARCH) -C $(KERNELDIR) M=$(PWD) modules

clean: 
	rm -rf *.ko *.o *.order src/*.o .tmp_vers* *.symvers *.mod.c .dmp* src/.*.cmd

