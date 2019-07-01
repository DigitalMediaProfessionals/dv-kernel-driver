# DV Kernel Driver

## HOWTO Build

Edit Makefile and modify `KERNELDIR` and `CROSS_COMPILE` to point to the kernel directory and cross tool respectively. Then run `make`.

It is also OK to pass them from the command line directly. Like this:

```
CROSS_COMPILE=<path to cross tool>/arm-linux-gnueabihf- KERNELDIR=<path to kernel> make
```

## HOWTO Install
Run `sudo make install` on your AI FPGA Module.
