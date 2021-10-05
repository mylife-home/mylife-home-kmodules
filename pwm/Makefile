# http://lxr.free-electrons.com/source/Documentation/kbuild/modules.txt

obj-m = dma_pwm.o
dma_pwm-y := management.o hw.o mbox.o
ccflags-y := -D$(MYLIFE_ARCH)

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

modules_install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
