## HOW TO BUILD KERNEL FOR GT-S7568(kyletdcmcc)

## 1. How to Build
 - get Toolchain
 download and install arm-eabi-4.4.3 toolchain for ARM EABI.
 Extract kernel source and move into the top directory.
 
## 1.1 Modify your makefile
CROSS_COMPILE=../arm-eabi-4.4.3/bin/arm-eabi-      //*It's your path for arm-eabi-4.4.3*//
## 1.2 Build your kernel
 $ make kyletd-vlx_defconfig
 $ make
// This model used Image
// $ make zImage
 $ make Image
## 2. Output files
// - Kernel : Kernel/arch/arm/boot/zImage
 - Kernel : Kernel/arch/arm/boot/Image
 - module : Kernel/drivers/*/*.ko
