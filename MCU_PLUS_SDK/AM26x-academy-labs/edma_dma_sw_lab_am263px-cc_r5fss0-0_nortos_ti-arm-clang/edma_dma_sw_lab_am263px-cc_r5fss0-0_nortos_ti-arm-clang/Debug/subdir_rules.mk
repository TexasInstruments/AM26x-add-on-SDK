################################################################################
# Automatically-generated file. Do not edit!
################################################################################

SHELL = cmd.exe

# Each subdirectory must supply rules for building sources it contributes
%.o: ../%.c $(GEN_OPTS) | $(GEN_FILES) $(GEN_MISC_FILES)
	@echo 'Building file: "$<"'
	@echo 'Invoking: Arm Compiler'
	"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/bin/tiarmclang.exe" -c -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 -mlittle-endian -mthumb -I"C:/ti/ccs1280/ccs/tools/compiler/ti-cgt-armllvm_3.2.2.LTS/include/c" -I"C:/ti/mcu_plus_sdk/source" -DSOC_AM263PX -D_DEBUG_=1 -g -Wall -Wno-gnu-variable-sized-type-not-at-end -Wno-unused-function -MMD -MP -MF"$(basename $(<F)).d_raw" -MT"$(@)" -I"C:/Users/A0503581/workspace_v12/edma_dma_sw_lab_am263px-cc_r5fss0-0_nortos_ti-arm-clang/Debug/syscfg"  $(GEN_OPTS__FLAG) -o"$@" "$<"
	@echo 'Finished building: "$<"'
	@echo ' '

build-462443649: ../example.syscfg
	@echo 'Building file: "$<"'
	@echo 'Invoking: SysConfig'
	"C:/ti/sysconfig_1.20.0/sysconfig_cli.bat" --script "C:/Users/A0503581/workspace_v12/edma_dma_sw_lab_am263px-cc_r5fss0-0_nortos_ti-arm-clang/example.syscfg" -o "syscfg" -s "C:/ti/mcu_plus_sdk/.metadata/product.json" --context "r5fss0-0" --part AM263P4 --package ZCZ_S --compiler ticlang
	@echo 'Finished building: "$<"'
	@echo ' '

syscfg/error.h: build-462443649 ../example.syscfg
syscfg: build-462443649


