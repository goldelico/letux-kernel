TOPDIR = .

CC = mips-linux-gnu-gcc
LD = mips-linux-gnu-ld
OBJCOPY = mips-linux-gnu-objcopy
OBJDUMP	= mips-linux-gnu-objdump
drop-sections := .reginfo .mdebug .oomment .note .pdr .options .MIPS.options
strip-flags   := $(addprefix --remove-section=,$(drop-sections))

CFLAGS += -I$(TOPDIR)/include
CFLAGS += -nostdinc -Wall -Wundef -Werror-implicit-function-declaration \
	 -fno-common -EL -Os -march=mips32 -mabi=32 -G 0 -mno-abicalls -fno-pic\
	 -msoft-float

LDFLAGS	:= -nostdlib -EL -T target.ld
OBJCOPY_ARGS  := -O elf32-tradlittlemips

OBJS := $(TOPDIR)/src/fastboot_resume.o	\
	$(TOPDIR)/src/uart.o


all: $(TOPDIR)/firmware.bin
	@hexdump -v -e '"0x" 1/4 "%08x" "," "\n"' $< > $(TOPDIR)/fastboot_resume_code.hex

$(TOPDIR)/firmware.bin:$(TOPDIR)/firmware.o
	@$(LD) -nostdlib -EL -T $(TOPDIR)/target.ld $(OBJS) -Map $(TOPDIR)/fastboot.map -o $(TOPDIR)/fastboot.elf
	@$(OBJCOPY) $(strip-flags) $(OBJCOPY_ARGS) -O binary $(TOPDIR)/fastboot.elf $@
	@$(OBJDUMP) $(TOPDIR)/fastboot.elf -D > fastboot.dump
$(TOPDIR)/firmware.o:$(OBJS)


%.o:%.c
	$(CC) $(CFLAGS) -o $@ -c $^

%.o:%.S
	$(CC) $(CFLAGS) -o $@ -c $^

clean:
	find . -name "*.o" | xargs rm -vf
	find . -name "*.o.cmd" | xargs rm -vf
	rm *.dump *.map *.elf *.bin *.hex

