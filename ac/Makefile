# http://lxr.free-electrons.com/source/Documentation/kbuild/modules.txt
obj-m = ac_zc.o ac_dimmer.o ac_button.o
ac_zc-y := ac_zc_main.o
ac_dimmer-y := ac_dimmer_main.o
ac_button-y := ac_button_main.o

all:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules

modules_install:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) modules_install

clean:
	make -C /lib/modules/$(shell uname -r)/build M=$(PWD) clean
