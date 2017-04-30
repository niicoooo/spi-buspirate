obj-m+= spi-buspirate.o
ccflags-y := -std=gnu99 -Wno-declaration-after-statement
 
KERNELDIR = /lib/modules/$(shell uname -r)/build
PWD       ?= $(shell pwd) 

all:
	make -C $(KERNELDIR) SUBDIRS=$(PWD) modules

clean:
#	find . -type f | xargs touch
	make -C $(KERNELDIR) M=$(PWD) clean

.PHONY: clean
