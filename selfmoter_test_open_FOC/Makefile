TARGET := selfmoter
BUILD_DIR := build

PREFIX ?= arm-none-eabi-
CC := $(PREFIX)gcc
AS := $(PREFIX)gcc
OBJCOPY := $(PREFIX)objcopy
SIZE := $(PREFIX)size

DEBUG ?= 1
OPT ?= -Og
USE_CMSIS_DSP_FALLBACK ?= 1
CMSIS_DSP_LIB ?=

LDSCRIPT := STM32F407IGTx_FLASH.ld

MCU := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard

DEFS := \
  -DUSE_HAL_DRIVER \
  -DSTM32F407xx \
  -DARM_MATH_CM4 \
  -D__FPU_PRESENT=1U

INCLUDES := \
  -ICore/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc \
  -IDrivers/STM32F4xx_HAL_Driver/Inc/Legacy \
  -IDrivers/CMSIS/Device/ST/STM32F4xx/Include \
  -IDrivers/CMSIS/Include \
  -IFoc/bsp \
  -IFoc/bsp/cc6920so \
  -IFoc/bsp/voltage \
  -IFoc/debug \
  -IFoc/dsp/Include \
  -IFoc/foc \
  -IFoc/system

C_SOURCES := \
  Core/Src/main.c \
  Core/Src/gpio.c \
  Core/Src/adc.c \
  Core/Src/spi.c \
  Core/Src/tim.c \
  Core/Src/stm32f4xx_it.c \
  Core/Src/stm32f4xx_hal_msp.c \
  Core/Src/system_stm32f4xx.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_adc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_rcc_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_flash_ramfunc.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_gpio.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_dma.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_pwr_ex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_cortex.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_exti.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_spi.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim.c \
  Drivers/STM32F4xx_HAL_Driver/Src/stm32f4xx_hal_tim_ex.c \
  Foc/bsp/cc6920so/cc6920so.c \
  Foc/bsp/voltage/voltage.c \
  Foc/bsp/bsp.c \
  Foc/bsp/bsp_irq.c \
  Foc/debug/vofa_debug.c \
  Foc/system/delay.c \
  Foc/foc/foc_as5047p.c \
  Foc/foc/foc_app.c \
  Foc/foc/foc_current.c \
  Foc/foc/foc_filter.c \
  Foc/foc/foc_flux.c \
  Foc/foc/foc_hfi.c \
  Foc/foc/foc_hw.c \
  Foc/foc/foc_identify.c \
  Foc/foc/foc_math.c \
  Foc/foc/foc_motor.c \
  Foc/foc/foc_pid.c \
  Foc/foc/foc_pwm.c \
  Foc/foc/foc_sensorless.c \
  Foc/foc/foc_smo.c \
  Foc/foc/foc_vbus.c \
  Foc/foc/foc_vf.c

ifeq ($(USE_CMSIS_DSP_FALLBACK),1)
C_SOURCES += Foc/dsp/cmsis_dsp_fallback.c
endif

ASM_SOURCES := Core/Startup/startup_stm32f407xx.s

OBJECTS := $(addprefix $(BUILD_DIR)/,$(C_SOURCES:.c=.o))
OBJECTS += $(addprefix $(BUILD_DIR)/,$(ASM_SOURCES:.s=.o))
DEPS := $(OBJECTS:.o=.d)

CFLAGS := $(MCU) $(OPT) -std=gnu11 -ffunction-sections -fdata-sections -Wall -Wextra
CFLAGS += $(DEFS) $(INCLUDES)
ifeq ($(DEBUG),1)
CFLAGS += -g3 -gdwarf-2
endif

ASFLAGS := $(MCU) -x assembler-with-cpp $(DEFS) $(INCLUDES)
ifeq ($(DEBUG),1)
ASFLAGS += -g3 -gdwarf-2
endif

LDFLAGS := $(MCU) -T$(LDSCRIPT) --specs=nano.specs --specs=nosys.specs
LDFLAGS += -Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref
LDFLAGS += -Wl,--gc-sections

LIBS := $(CMSIS_DSP_LIB) -lm

.PHONY: all clean flash size

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin size

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile $(LDSCRIPT)
	$(CC) $(OBJECTS) $(LDFLAGS) $(LIBS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O ihex $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(OBJCOPY) -O binary -S $< $@

$(BUILD_DIR)/%.o: %.c Makefile
	@mkdir -p $(dir $@)
	$(CC) -c $(CFLAGS) -MMD -MP -MF"$(@:.o=.d)" $< -o $@

$(BUILD_DIR)/%.o: %.s Makefile
	@mkdir -p $(dir $@)
	$(AS) -c $(ASFLAGS) -MMD -MP -MF"$(@:.o=.d)" $< -o $@

size: $(BUILD_DIR)/$(TARGET).elf
	$(SIZE) $<

flash: $(BUILD_DIR)/$(TARGET).elf
	openocd -f interface/stlink.cfg -f target/stm32f4x.cfg -c "adapter speed 4000" -c "program $(BUILD_DIR)/$(TARGET).elf verify reset exit"

clean:
	rm -rf $(BUILD_DIR)

-include $(DEPS)
