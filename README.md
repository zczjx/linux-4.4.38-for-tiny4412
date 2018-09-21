# linux-4.4.38-for-tiny4412
linux-4.4.38-for-tiny4412 made by FriendlyARM Corp.

- how to boot and flash the uboot
    
    * use the uboot modified by myself
    https://github.com/zczjx/uboot_tiny4412
    
    * do sign manually if you modify the uboot
    before flash it to gen bl2.bin
    cd sd_fuse/tiny4412
   ../mkbl2 ../../u-boot.bin bl2.bin 14336

    * follow this article to flash uboot.bin to emmc
    http://www.cnblogs.com/pengdonglin137/p/4161084.html

- how to build the linux-4.4.38-for-tiny4412 
    
    1. download the correct version of arm-linux-gcc
    I use arm-linux-gcc version 4.9.4 download from Linaro
    https://releases.linaro.org/components/toolchain/binaries/4.9-2017.01/arm-linux-gnueabi/
    
    2. cp tiny4412_linux_4_4_defconfig .config

    3. make LOADADDR=0x40008000 uImage

    4. make dtbs

- how to flash the linux kernel and dtb image

    - use uboot fastboot run fastboot in uboot

    - fastboot flash kernel-4-4 arch/arm/boot/uImage

    - fastboot flash dtb arch/arm/boot/dts/exynos4412-tiny4412.dtb

    - run reset in uboot

- set your user partition and flash your rootfs to your emmc

 
