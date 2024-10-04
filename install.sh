#!/usr/bin/env bash

export BOOTFS=/run/media/tperrot/bootfs
export ROOTFS=/run/media/tperrot/rootfs

if [ ! -d ${BOOTFS} ]; then
    echo "${BOOTFS} isn't available"
    exit 11
fi
if [ ! -d ${ROOTFS} ]; then
    echo "${ROOTFS} isn't available"
    exit 12
fi

export KERNEL=kernel8

make -j$(nproc) INSTALL_MOD_PATH=${ROOTFS} modules_install || exit 20

cp ${BOOTFS}/${KERNEL}.img ${BOOTFS}/${KERNEL}-backup.img || exit 21
cp arch/arm64/boot/Image.gz ${BOOTFS}/${KERNEL}.img || exit 3
cp arch/arm64/boot/dts/broadcom/*.dtb ${BOOTFS}/ || exit 4
cp arch/arm64/boot/dts/overlays/*.dtb* ${BOOTFS}/overlays/ || exit 5
cp arch/arm64/boot/dts/overlays/README ${BOOTFS}/overlays/ || exit 6

exit 0
