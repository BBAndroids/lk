# top level project rules for the msm8974 project
#
LOCAL_DIR := $(GET_LOCAL_DIR)

TARGET := msm8974

MODULES += app/aboot

ifeq ($(TARGET_BUILD_VARIANT),user)
DEBUG := 0
else
DEBUG := 2
endif

EMMC_BOOT := 1
ENABLE_SDHCI_SUPPORT := 1
ENABLE_USB30_SUPPORT := 0
ENABLE_FBCON_DISPLAY_MSG := 1

#enable power on vibrator feature
ENABLE_PON_VIB_SUPPORT := true

#DEFINES += WITH_DEBUG_DCC=1
DEFINES += WITH_DEBUG_UART=1
#DEFINES += WITH_DEBUG_FBCON=1
DEFINES += DEVICE_TREE=1
#DEFINES += MMC_BOOT_BAM=1
DEFINES += CRYPTO_BAM=1
DEFINES += CRYPTO_REG_ACCESS=1
DEFINES += ABOOT_IGNORE_BOOT_HEADER_ADDRS=1
DEFINES += WITH_DEBUG_LOG_BUF=1
DEFINES += LK_LOG_BUF_SIZE=40960
DEFINES += NO_SCM_V8_SUPPORT=1

#Disable thumb mode
ENABLE_THUMB := false

DEFINES += ABOOT_FORCE_KERNEL_ADDR=0x00008000
DEFINES += ABOOT_FORCE_RAMDISK_ADDR=0x02000000
DEFINES += ABOOT_FORCE_TAGS_ADDR=0x01e00000
DEFINES += ABOOT_FORCE_KERNEL64_ADDR=0x00080000

ifeq ($(ENABLE_FBCON_DISPLAY_MSG),1)
DEFINES += FBCON_DISPLAY_MSG=1
endif

ifeq ($(EMMC_BOOT),1)
DEFINES += _EMMC_BOOT=1
endif

ifeq ($(ENABLE_PON_VIB_SUPPORT),true)
DEFINES += PON_VIB_SUPPORT=1
endif

ifeq ($(ENABLE_SDHCI_SUPPORT),1)
DEFINES += MMC_SDHCI_SUPPORT=1
endif

ifeq ($(ENABLE_USB30_SUPPORT),1)
DEFINES += USB30_SUPPORT=1
endif
