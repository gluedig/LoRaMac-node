PROJECT = lora-mote

BOARD=GrinnMote
RADIO=sx1272
HAL=STM32L1xx_HAL_Driver
GCC_BIN =
OBJECTS = ./main.o
TOPDIR = ../../
HALDIR=$(TOPDIR)boards/mcu/stm32/
BOARDDIR=$(TOPDIR)boards/$(BOARD)/

BOARD_OBJECTS = $(patsubst %.c,%.o,$(wildcard $(BOARDDIR)*.c)) $(BOARDDIR)cmsis/system_stm32l1xx.o $(BOARDDIR)cmsis/arm-gcc/startup_stm32l1xx_md.o
BOARD_OBJECTS += $(patsubst %.c,%.o,$(wildcard $(TOPDIR)radio/*.c)) $(patsubst %.c,%.o,$(wildcard $(TOPDIR)radio/$(RADIO)/*.c))
BOARD_OBJECTS += $(patsubst %.c,%.o,$(wildcard $(TOPDIR)system/*.c)) $(patsubst %.c,%.o,$(wildcard $(TOPDIR)peripherals/*.c))
BOARD_OBJECTS += $(patsubst %.c,%.o,$(wildcard $(TOPDIR)mac/*.c)) $(patsubst %.c,%.o,$(wildcard $(TOPDIR)system/crypto/*.c))



HAL_OBJECTS = $(patsubst %.c,%.o,$(wildcard $(HALDIR)*.c)) $(patsubst %.c,%.o,$(wildcard $(HALDIR)$(HAL)/Src/*.c))
USB_OBJECTS = $(patsubst %.c,%.o,$(wildcard $(HALDIR)STM32_USB_Device_Library/Core/Src/*.c)) $(patsubst %.c,%.o,$(wildcard $(HALDIR)STM32_USB_Device_Library/Class/CDC/Src/*.c))

OBJECTS += $(BOARD_OBJECTS)
SYS_OBJECTS = $(HAL_OBJECTS)

INCLUDE_PATHS = -I$(HALDIR)cmsis -I$(HALDIR)$(HAL)/Inc -I$(HALDIR) -I. -I$(BOARDDIR) -I$(BOARDDIR)cmsis -I$(TOPDIR)system -I$(TOPDIR)radio -I$(TOPDIR)radio/$(RADIO) -I$(TOPDIR)peripherals -I$(TOPDIR)mac -I$(TOPDIR)system/crypto

LIBRARY_PATHS = -L.
LINKER_SCRIPT = $(BOARDDIR)/cmsis/arm-gcc/stm32l1xx_md_flash.ld

###############################################################################
AS      = $(GCC_BIN)arm-none-eabi-as
CC      = $(GCC_BIN)arm-none-eabi-gcc
CPP     = $(GCC_BIN)arm-none-eabi-g++
LD      = $(GCC_BIN)arm-none-eabi-gcc
OBJCOPY = $(GCC_BIN)arm-none-eabi-objcopy
OBJDUMP = $(GCC_BIN)arm-none-eabi-objdump
SIZE    = $(GCC_BIN)arm-none-eabi-size


CPU = -mcpu=cortex-m3 -mthumb
CC_FLAGS = $(CPU) -c -g -fno-common -fmessage-length=0 -Wall -Wextra -fno-exceptions -ffunction-sections -fdata-sections -fomit-frame-pointer -MMD -MP
CC_FLAGS += -Wimplicit-function-declaration
CC_SYMBOLS = -DSTM32L151xB -DUSE_BAND_868 -DUSE_HAL_DRIVER -DUSE_DEBUGGER

LD_FLAGS = $(CPU) -Wl,--gc-sections --specs=nano.specs -Wl,-Map=$(PROJECT).map,--cref
#LD_FLAGS += -Wl,--wrap,main -u _printf_float -u _scanf_float
LD_SYS_LIBS = -lstdc++ -lsupc++ -lm -lc -lgcc -lnosys


ifeq ($(DEBUG), 1)
  CC_FLAGS += -DDEBUG -O0
else
  CC_FLAGS += -DNDEBUG -Os
endif

.PHONY: all clean lst size

all: $(PROJECT).bin $(PROJECT).hex size


clean:
	rm -f $(PROJECT).bin $(PROJECT).elf $(PROJECT).hex $(PROJECT).map $(PROJECT).lst $(OBJECTS) $(DEPS)

mrproper: clean
	rm -f $(SYS_OBJECTS)

.asm.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<
.s.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<
.S.o:
	$(CC) $(CPU) -c -x assembler-with-cpp -o $@ $<

.c.o:
	$(CC)  $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu99   $(INCLUDE_PATHS) -o $@ $<

.cpp.o:
	$(CPP) $(CC_FLAGS) $(CC_SYMBOLS) -std=gnu++98 -fno-rtti $(INCLUDE_PATHS) -o $@ $<

$(PROJECT).elf: $(OBJECTS) $(SYS_OBJECTS)
	$(LD) $(LD_FLAGS) -T$(LINKER_SCRIPT) $(LIBRARY_PATHS) -o $@ $^ $(LIBRARIES) $(LD_SYS_LIBS)

$(PROJECT).bin: $(PROJECT).elf
	$(OBJCOPY) -O binary $< $@

$(PROJECT).hex: $(PROJECT).elf
	@$(OBJCOPY) -O ihex $< $@

$(PROJECT).lst: $(PROJECT).elf
	@$(OBJDUMP) -Sdh $< > $@

lst: $(PROJECT).lst

size: $(PROJECT).elf
	$(SIZE) $(PROJECT).elf

DEPS = $(OBJECTS:.o=.d) $(SYS_OBJECTS:.o=.d)
-include $(DEPS)


