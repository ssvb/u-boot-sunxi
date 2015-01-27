#!/bin/sh

make CROSS_COMPILE=arm-none-linux-gnueabi- Cubietruck_defconfig
make CROSS_COMPILE=arm-none-linux-gnueabi- -j2 || exit 1

fel write 0x2000 spl/u-boot-spl.bin
fel exe   0x2000

sleep 1 # Wait for DRAM initialization to complete

fel write 0x4a000000 u-boot.bin
fel exe   0x4a000000
