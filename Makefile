#------------------ App
CC := arm-none-linux-gnueabi-gcc
#APP := ./app/main.c 
#APP_TARGET := tst-pwm



#------------------- Device Driver
obj-m := g2450_pwm.o

#KDIR	:= /lib/modules/$(shell uname -r)/build
#KDIR	:= /work/REBIS_BSP/linux-2.6.17.13-rebis
KDIR	:= /work/kernel-mds2450
#KDIR	:= /project/repo/008_mds/linux_fw_2450/linux/mango_bsp/kernel

all:
	make -C $(KDIR) SUBDIRS=$(PWD) modules ARCH=arm
#	$(CC) $(APP) -o $(APP_TARGET)

clean:
	make -C $(KDIR) SUBDIRS=$(PWD) clean

