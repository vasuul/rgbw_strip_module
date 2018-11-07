KERNELSRCDIR ?= /lib/modules/$(shell uname -r)/build

TARGET_BASE = rgbw_strip

obj-m := ${TARGET_BASE}.o

all: ${TARGET_BASE}.ko ${TARGET_BASE}.dtbo

${TARGET_BASE}.dtbo: ${TARGET_BASE}.dts
	dtc -@ -I dts -O dtb -o $@ $<

${TARGET_BASE}.ko: ${TARGET_BASE}.c ${TARGET_BASE}.h
	$(MAKE) -C ${KERNELSRCDIR} M=$(PWD) modules

clean:
	make -C ${KERNELSRCDIR} M=$(PWD) clean

install:
	cp ${TARGET_BASE}.dtbo /boot/overlays
	cp ${TARGET_BASE}.ko /lib/modules

.PHONY: clean all
