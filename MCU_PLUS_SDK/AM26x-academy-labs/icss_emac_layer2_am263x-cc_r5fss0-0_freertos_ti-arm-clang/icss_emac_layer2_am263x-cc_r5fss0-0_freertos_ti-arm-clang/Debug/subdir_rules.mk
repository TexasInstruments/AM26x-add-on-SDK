################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
build-1184554161: ../example.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1.20.0/sysconfig_cli.bat" --script "C:/Users/A0503581/workspace_v12/icss_emac_layer2_am263x-cc_r5fss0-0_freertos_ti-arm-clang/example.syscfg" -o "syscfg" -s "C:/ti/mcu_plus_sdk_am263x_09_02_00_56/.metadata/product.json" --context "r5fss0-0" --part AM263x --package ZCZ --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/ti_dpl_config.c: build-1184554161 ../example.syscfg
syscfg/ti_dpl_config.h: build-1184554161
syscfg/ti_drivers_config.c: build-1184554161
syscfg/ti_drivers_config.h: build-1184554161
syscfg/ti_drivers_open_close.c: build-1184554161
syscfg/ti_drivers_open_close.h: build-1184554161
syscfg/ti_pinmux_config.c: build-1184554161
syscfg/ti_power_clock_config.c: build-1184554161
syscfg/ti_board_config.c: build-1184554161
syscfg/ti_board_config.h: build-1184554161
syscfg/ti_board_open_close.c: build-1184554161
syscfg/ti_board_open_close.h: build-1184554161
syscfg/ti_enet_config.c: build-1184554161
syscfg/ti_enet_config.h: build-1184554161
syscfg/ti_enet_open_close.c: build-1184554161
syscfg/ti_enet_open_close.h: build-1184554161
syscfg/ti_enet_soc.c: build-1184554161
syscfg/ti_enet_lwipif.c: build-1184554161
syscfg/ti_enet_lwipif.h: build-1184554161
syscfg/linker_defines.h: build-1184554161
syscfg: build-1184554161

syscfg/%.o: ./syscfg/%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -I"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/kernel/freertos/FreeRTOS-Kernel/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/kernel/freertos/config/am263x/r5f" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/lwipif/inc" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/source" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-stack/src/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-stack/src/include/lwip" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-port/freertos/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-port/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-stack/contrib" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-config/am263x/icss_emac" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware/icss_dual_emac" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware/icss_switch" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware/source" -DSOC_AM263X -DAM263X_CC -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"syscfg/$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/A0503581/workspace_v12/icss_emac_layer2_am263x-cc_r5fss0-0_freertos_ti-arm-clang/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -I"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/include/c" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/kernel/freertos/FreeRTOS-Kernel/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/kernel/freertos/portable/TI_ARM_CLANG/ARM_CR5F" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/kernel/freertos/config/am263x/r5f" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/lwipif/inc" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/source" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-stack/src/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-stack/src/include/lwip" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-port/freertos/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-port/include" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-stack/contrib" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/lwip/lwip-config/am263x/icss_emac" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware/icss_dual_emac" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware/icss_switch" -I"C:/ti/mcu_plus_sdk_am263x_09_02_00_56/source/networking/icss_emac/firmware/source" -DSOC_AM263X -DAM263X_CC -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/A0503581/workspace_v12/icss_emac_layer2_am263x-cc_r5fss0-0_freertos_ti-arm-clang/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '


