####---Makefile----#####
OBJ = module_frame
obj-m := $(OBJ).o
#KERNELDIR = /lib/modules/`uname -r`/build
KERNELDIR = /home/storm/source/curl/2440/utu-Linux2.6.24_for_utu2440_2009-07-18
PWD = `pwd`

default:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules ARCH=arm


install:
	insmod $(OBJ).ko

uninstall:
	rmmod $(OBJ).ko

clean:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) clean

