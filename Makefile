OBJDIR = obj
OUTDIR = build
TARGET = $(OUTDIR)/STM32F4Discovery


SOURCES += src/main.c
SOURCES += src/startup_stm32f4xx.s
SOURCES += src/STM32F4-Discovery_callback.c
SOURCES += src/stm32f4xx_it.c
SOURCES += src/system_stm32f4xx.c
#SOURCES += src/tiny_printf.c
SOURCES += src/printf-stdarg.c
SOURCES += src/pwrtrain_steering.c
SOURCES += src/communication.c
SOURCES += src/accelerometer.c

SOURCES += Utilities/STM32F4-Discovery/stm32f4_discovery.c

SOURCES += FreeRTOS_Source/list.c
SOURCES += FreeRTOS_Source/queue.c
SOURCES += FreeRTOS_Source/tasks.c  
SOURCES += FreeRTOS_Source/timers.c
SOURCES += FreeRTOS_Source/portable/MemMang/heap_1.c
SOURCES += FreeRTOS_Source/portable/GCC/ARM_CM4F/port.c

SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/misc.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_adc.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_can.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_crc.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_aes.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_des.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_cryp_tdes.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dac.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dbgmcu.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dcmi.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_dma.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_exti.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_flash.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_fsmc.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_gpio.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_md5.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_hash_sha1.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_i2c.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_iwdg.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_pwr.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rcc.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rng.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_rtc.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_sdio.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_spi.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_syscfg.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_tim.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_usart.c
SOURCES += Libraries/STM32F4xx_StdPeriph_Driver/src/stm32f4xx_wwdg.c

SOURCES += Utilities/STM32F4-Discovery/stm32f4_discovery_lis302dl.c

OBJECTS = $(addprefix $(OBJDIR)/,$(addsuffix .o,$(basename $(SOURCES))))

INCLUDES += -Iinc
INCLUDES += -IFreeRTOS_Source/include
INCLUDES += -IFreeRTOS_Source/portable/GCC/ARM_CM4F
INCLUDES += -ILibraries/CMSIS/Include
INCLUDES += -ILibraries/Device/STM32F4xx/Include
INCLUDES += -ILibraries/STM32F4xx_StdPeriph_Driver/inc
INCLUDES += -IUtilities/STM32F4-Discovery


CFLAGS += -std=gnu99 
CFLAGS += -g
CFLAGS += -Wall
#CFLAGS += -mlittle-endian 
CFLAGS += -mthumb 
#CFLAGS += -mthumb-interwork 
#CFLAGS += -nostartfiles 
CFLAGS += -mcpu=cortex-m4
#CFLAGS += -fsingle-precision-constant 
#CFLAGS += -Wdouble-promotion
CFLAGS += -mfpu=fpv4-sp-d16 
CFLAGS += -mfloat-abi=softfp
CFLAGS += -DUSE_STM32F4_DISCOVERY -DHSE_VALUE=8000000 -DSTM32F4XX -DUSE_STDPERIPH_DRIVER -O0 -ffunction-sections -fdata-sections

LDFLAGS += -Wl,-Map=$(TARGET).map
LDFLAGS += -Tstm32f4_flash.ld -Wl,--start-group -lc -lm -Wl,--end-group -static -Wl,-cref,-u,Reset_Handler -Wl,-Map=STM32F4_Discovery_FreeRTOS_Simple_Demo.map -Wl,--gc-sections -Wl,--defsym=malloc_getpagesize_P=0x1000

TOOLCHAIN = arm-none-eabi
CC				= $(TOOLCHAIN)-gcc
ST_FLASH  = st-flash
OBJCOPY		= $(TOOLCHAIN)-objcopy



all:	dirs build link flash

test:
	@echo $(OBJECTS)

dirs:
	$(shell mkdir -p $(OBJDIR) 2>/dev/null)
	$(shell mkdir -p $(OUTDIR) 2>/dev/null)
	$(shell mkdir -p $(OBJDIR)/src 2>/dev/null)
	$(shell mkdir -p $(OBJDIR)/Utilities/STM32F4-Discovery/ 2>/dev/null)
	$(shell mkdir -p $(OBJDIR)/FreeRTOS_Source/portable/MemMang/ 2>/dev/null)
	$(shell mkdir -p $(OBJDIR)/FreeRTOS_Source/portable/GCC/ARM_CM4F/ 2>/dev/null)
	$(shell mkdir -p $(OBJDIR)/Libraries/STM32F4xx_StdPeriph_Driver/src/ 2>/dev/null)
	
build: $(OBJECTS)
	@echo
	@echo Build finished.

$(OBJDIR)/%.o : %.c
	@echo
	@echo Compiling CPP: $<
	$(CC) -c $(INCLUDES) $(CFLAGS) $< -o $@

$(OBJDIR)/%.o : %.s
	@echo
	@echo Assembling: $<
	$(CC) -c $(INCLUDES) $(CFLAGS) -Wa,--warn -x assembler-with-cpp $< -o $@

link: $(OBJECTS)
	@echo
	@echo Linking.
	$(CC) $^ $(LDFLAGS) $(CFLAGS) --output $(TARGET).elf
	$(OBJCOPY) -O binary $(TARGET).elf $(TARGET).bin

clean: clobber
	$(shell rm -rf $(OUTDIR) 2>/dev/null)
	
clobber:
	$(shell rm -rf $(OBJDIR) 2>/dev/null)
	
flash: $(TARGET).bin
	$(ST_FLASH) write $< 0x8000000
	

