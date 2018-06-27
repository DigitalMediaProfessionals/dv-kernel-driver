
# board dependencies:
ifeq ($(board),arria10)
  BDDEF=DMP_ARRIA10
  KERNELDIR=/home/share/steven/arria10/kernel/linux-socfpga-rel_socfpga-3.10-ltsi-rt_16.03.02_pr
  CROSS_COMPILE=arm-linux-gnueabi-
else
  BDDEF=DMP_ZC706
  KERNELDIR=/home/cnn-hw1/users/steven/zc706_SWEnv/linux-xlnx-xilinx-v2017.2
  CROSS_COMPILE=/usr/local/tools/xilinx/Vivado_2017.2_0616_1/SDK/2017.2/gnu/arm/lin/bin/arm-xilinx-linux-gnueabi-
endif

BUILD_DIR := $(shell pwd)
VERBOSE = 0
PWD=$(shell pwd)
MOD = DMP_drm
ARCH=arm

MAKEARCH=$(MAKE) ARCH=$(ARCH) KCPPFLAGS="-D$(BDDEF)" CROSS_COMPILE=$(CROSS_COMPILE)

obj-m := $(MOD).o 
$(MOD)-objs  := ./src/CNV_km.o 
$(MOD)-objs  += ./src/pdc.o 

all: 
	$(MAKEARCH) -C $(KERNELDIR) M=$(PWD) modules

clean: 
	rm -rf *.ko *.o *.order src/*.o .tmp_vers* *.symvers *.mod.c .DMP* src/.*.cmd

