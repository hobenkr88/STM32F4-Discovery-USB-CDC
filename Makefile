TARGET = prog

# Take a look into $(CUBE_DIR)/Drivers/BSP for available BSPs
BOARD = STM32F4-Discovery
BSP_BASE = stm32f4_discovery

# MCU family and type in various capitalizations o_O
MCU_FAMILY = stm32f4xx
MCU_LC = stm32f401xc
MCU_MC = STM32F407xx
MCU_UC = STM32F407VG

SRCS = main.c \
	stm32f4xx_hal_msp.c \
	stm32f4xx_it.c \
	usb_device.c \
	usb_usr.c \
	usbd_cdc_if.c \
	usbd_conf.c \
	usbd_desc.c

SRCS += system_stm32f4xx.c

# USB
SRCS      +=  usbd_core.c usbd_ctlreq.c usbd_ioreq.c
#SRCS      += usbd_hid.c
SRCS      += usbd_cdc.c
#SRCS      += stm32f4xx_hal_pcd.c

CUBE_DIR = .
HAL_DIR = $(CUBE_DIR)/Drivers/STM32F4xx_HAL_Driver
CMSIS_DIR = $(CUBE_DIR)/Drivers/CMSIS
DEV_DIR = $(CMSIS_DIR)/Device/ST/STM32F4xx

###############################################################################
# Toolchain

PREFIX     = arm-none-eabi
CC         = $(PREFIX)-gcc
AR         = $(PREFIX)-ar
OBJCOPY    = $(PREFIX)-objcopy
OBJDUMP    = $(PREFIX)-objdump
SIZE       = $(PREFIX)-size
GDB        = $(PREFIX)-gdb

# Include search paths (-I)
INCS       = -IInc
#INCS      += -I$(BSP_DIR)
INCS      += -I$(CMSIS_DIR)/Include
INCS      += -I$(DEV_DIR)/Include
INCS      += -I$(HAL_DIR)/Inc

INCS      += -I$(CMSIS_DIR)/Device/ST/STM32F4xx/Include

# USB .h
INCS      += -I$(CUBE_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Inc
INCS      += -I$(CUBE_DIR)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Inc

# Library search paths
LIBS       = -L$(CMSIS_DIR)/Lib

# Compiler flags
CFLAGS     = -Wall -g -std=c99 -Os
CFLAGS    += -mlittle-endian -mcpu=cortex-m4 -march=armv7e-m -mthumb
CFLAGS    += -mfpu=fpv4-sp-d16 -mfloat-abi=hard
CFLAGS    += -ffunction-sections -fdata-sections
CFLAGS += -DSTM32F407xx
CFLAGS    += $(INCS)
CFLAGS += $(DEFS)

CFLAGS    += -mthumb-interwork
CFLAGS += -Wno-pointer-sign

# Linker flags
LDFLAGS    = -Wl,--gc-sections -Wl,-Map=$(TARGET).map $(LIBS) -TLinkerScript/stm32f401xc.ld

# Enable Semihosting
LDFLAGS   += --specs=rdimon.specs -lc -lrdimon

vpath %.c Src
vpath %.c $(CMSIS_DIR)/Device/ST/STM32F4xx/Source/Templates
vpath %.c $(CUBE_DIR)/Middlewares/ST/STM32_USB_Device_Library/Core/Src
vpath %.c $(CUBE_DIR)/Middlewares/ST/STM32_USB_Device_Library/Class/CDC/Src

OBJS = $(addprefix Obj/,$(SRCS:.c=.o))
DEPS = $(addprefix Dep/,$(SRCS:.c=.d))

LIBSTM32F4_HAL = Drivers/STM32F4xx_HAL_Driver
MY_LIBS = $(LIBSTM32F4_HAL)

# Prettify output
V = 0
ifeq ($V, 0)
	Q = @
	P = > /dev/null
endif

###################################################

.PHONY: all $(MY_LIBS) $(LIBSTM32F4_HAL) dirs program debug clean

all: $(TARGET).elf

-include $(DEPS)

$(TARGET).elf: $(OBJS) $(LIBSTM32F4_HAL)
	@echo "[LD]      $(TARGET).elf"
	$Q$(CC) $(CFLAGS) $(LDFLAGS) ASM/startup_$(MCU_LC).s $(OBJS) -o $@ -L$(HAL_DIR) -lstm32f4_hal
	@echo "[OBJDUMP] $(TARGET).lst"
	$Q$(OBJDUMP) -St $(TARGET).elf >$(TARGET).lst
	@echo "[SIZE]    $(TARGET).elf"
	$(SIZE) $(TARGET).elf

$(LIBSTM32F4_HAL):
	@$(MAKE) -C $@

dirs:
	@mkdir -p Obj Dep

Obj/%.o: %.c dirs
	$Q$(CC) $(CFLAGS) -c -o $@ $< -MMD -MF Dep/$(*F).d -L$(HAL_DIR) -lstm32f4_hal

program: all
	#$(OCD) -s $(OCD_DIR) $(OCDFLAGS) -c "program $(TARGET).elf verify reset"
	arm-none-eabi-objcopy -O ihex $(TARGET).elf $(TARGET).hex
	arm-none-eabi-objcopy -O binary $(TARGET).elf $(TARGET).bin
	~/stlink/st-flash write $(TARGET).bin 0x8000000

debug:
	terminator -e "~/stlink/st-util" &
	arm-none-eabi-gdb -tui $(TARGET).elf

clean:
	@echo "[RM]      $(TARGET).elf"; rm -f $(TARGET).elf
	@echo "[RM]      $(TARGET).map"; rm -f $(TARGET).map
	@echo "[RM]      $(TARGET).lst"; rm -f $(TARGET).lst
	@echo "[RMDIR]   Dep"          ; rm -fr Dep
	@echo "[RMDIR]   Obj"          ; rm -fr Obj
