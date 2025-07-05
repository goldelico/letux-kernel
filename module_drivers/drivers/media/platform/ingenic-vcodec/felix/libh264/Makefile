ifneq ($(KERNELRELEASE),)
obj-y += api/jzm_h264_dec.o

obj-y += src/

else
#KDIR := "/home1/pzqi/Manhatton-halley2-v2.0-release/kernel/"
KDIR := "/home/boysic/work/dorado/platform/kernel"
EXTRA_CFLAGS := -I`pwd` -I`pwd`/include
all:
	$(MAKE) -C $(KDIR) M=`pwd` modules EXTRA_CFLAGS+=$(EXTRA_CFLAGS)

clean:
	rm -f libavcodec/api/.*.cmd
	rm -f libavcodec/api/*.o
	rm -f libavcodec/.*.cmd
	rm -f libavcodec/*.mod.c
	rm -f libavcodec/*.order
	rm -f libavcodec/*.o
	rm -f libavcodec/*.ko
	rm -f libavutil/.*.cmd
	rm -f libavutil/.*.o
	rm -f libavutil/*.o
endif

