#!/bin/bash

export CROSS_COMPILE=../arm-eabi-4.8/bin/arm-eabi-

make -C . TOOLCHAIN_PREFIX=$CROSS_COMPILE BOOTLOADER_OUT=. msm8974 EMMC_BOOT=1
# TARGET_BUILD_VARIANT=user

cp ./build-msm8974/emmc_appsboot.mbn /home/user/blackberry/passport/imggen/files/aboot.mbn

fastboot flash aboot ./build-msm8974/emmc_appsboot.mbn
fastboot reboot-bootloader
fastboot oem bootlog