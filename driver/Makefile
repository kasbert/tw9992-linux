obj-m := tw9992.o

KERNELDIR ?= /lib/modules/$(shell uname -r)/build
PWD       := $(shell pwd)

EXTRA_CFLAGS	:= -DDEBUG=0

all: stuff tw9992.dtbo

stuff:
	$(MAKE) -C $(KERNELDIR) M=$(PWD)

modules_install:
	$(MAKE) -C $(KERNELDIR) M=$(PWD) modules_install

clean:
	rm -rf *.o *~ core .depend .*.cmd *.ko *.mod.c .tmp_versions vtty *.dtbo *.mod modules.order Module.symvers

install:
	sudo make modules_install
	sudo cp -v tw9992.dtbo /boot/firmware/overlays/

tw9992.dtbo: tw9992-overlay.dts
	dtc -@ -I dts -O dtb -o tw9992.dtbo tw9992-overlay.dts

crash: all install
	sudo dtoverlay -r tw9992; sudo rmmod tw9992; sync ; sudo dtoverlay tw9992

# Peek i2c register 0x02
peek:
	v4l2-ctl -c setaddr=0x02; v4l2-ctl -C getreg
