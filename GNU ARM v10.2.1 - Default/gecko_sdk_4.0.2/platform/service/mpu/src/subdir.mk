################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2/platform/service/mpu/src/sl_mpu.c 

OBJS += \
./gecko_sdk_4.0.2/platform/service/mpu/src/sl_mpu.o 

C_DEPS += \
./gecko_sdk_4.0.2/platform/service/mpu/src/sl_mpu.d 


# Each subdirectory must supply rules for building sources it contributes
gecko_sdk_4.0.2/platform/service/mpu/src/sl_mpu.o: /Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2/platform/service/mpu/src/sl_mpu.c
	@echo 'Building file: $<'
	@echo 'Invoking: GNU ARM C Compiler'
	arm-none-eabi-gcc -g3 -gdwarf-2 -mcpu=cortex-m33 -mthumb -std=c99 '-DEFR32BG22C224F512IM40=1' '-DSL_BOARD_NAME="BRD4184A"' '-DSL_BOARD_REV="A02"' '-DSL_COMPONENT_CATALOG_PRESENT=1' '-DMBEDTLS_CONFIG_FILE=<mbedtls_config.h>' '-DMBEDTLS_PSA_CRYPTO_CONFIG_FILE=<psa_crypto_config.h>' '-DSL_RAIL_LIB_MULTIPROTOCOL_SUPPORT=0' '-DSL_RAIL_UTIL_PA_CONFIG_HEADER=<sl_rail_util_pa_config.h>' '-DSLI_RADIOAES_REQUIRES_MASKING=1' -I"/Users/clfilho/SimplicityStudio/v5_workspace/SoilMoistureSensorCentral" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/Device/SiliconLabs/EFR32BG22/Include" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//app/common/util/app_assert" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//app/common/util/app_log" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/common/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//protocol/bluetooth/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//hardware/board/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/bootloader" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/bootloader/api" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/CMSIS/Include" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/sl_cryptoacc_library/include" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/device_init/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/emdrv/common/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/emlib/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/emlib/host/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/hfxo_manager/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/iostream/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/sl_mbedtls_support/config" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/mbedtls/include" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/mbedtls/library" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/sl_mbedtls_support/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/mpu/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//hardware/driver/mx25_flash_shutdown/inc/sl_mx25_flash_shutdown_usart" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/emdrv/nvm3/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//app/bluetooth/common/ota_dfu" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/power_manager/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/sl_psa_driver/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/common" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/protocol/ble" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/protocol/ieee802154" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/protocol/zwave" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/chip/efr32/efr32xg2x" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/plugin/pa-conversions" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/plugin/pa-conversions/efr32xg22" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/radio/rail_lib/plugin/rail_util_pti" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/se_manager/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/se_manager/src" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/silicon_labs/silabs_core/memory_manager" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/common/toolchain/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/system/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/sleeptimer/inc" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//util/third_party/crypto/sl_component/sl_protocol_crypto/src" -I"/Users/clfilho/SimplicityStudio/SDKs/gecko_sdk2//platform/service/udelay/inc" -I"/Users/clfilho/SimplicityStudio/v5_workspace/SoilMoistureSensorCentral/autogen" -I"/Users/clfilho/SimplicityStudio/v5_workspace/SoilMoistureSensorCentral/config" -Os -Wall -Wextra -fno-builtin -ffunction-sections -fdata-sections -imacrossl_gcc_preinclude.h -mfpu=fpv5-sp-d16 -mfloat-abi=hard -c -fmessage-length=0 -MMD -MP -MF"gecko_sdk_4.0.2/platform/service/mpu/src/sl_mpu.d" -MT"gecko_sdk_4.0.2/platform/service/mpu/src/sl_mpu.o" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


