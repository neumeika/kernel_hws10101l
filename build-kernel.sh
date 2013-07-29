#!/bin/bash

###############################################################################
# To all DEV around the world :)                                              #
# to build this kernel you need to be ROOT and to have bash as script loader  #
# do this:                                                                    #
# cd /bin                                                                     #
# rm -f sh                                                                    #
# ln -s bash sh                                                               #
# now go back to kernel folder and run:                                       #
# sh load_config.sh                                                           #
# sh clean_kernel.sh                                                          #
#                                                                             #
# Now you can build my kernel.                                                #
# using bash will make your life easy. so it's best that way.                 #
# Have fun and update me if something nice can be added to my source.         #
###############################################################################

# location
export KERNELDIR=`readlink -f .`
export PARENT_DIR=`readlink -f ..`
export INITRAMFS_SOURCE=`readlink -f ${KERNELDIR}/initramfs`
export INITRAMFS_TMP="/tmp/initramfs-source"
export PATH=${KERNELDIR}/arm-eabi-4.6/bin/:${PATH}

# ccache
#export USE_CCACHE=1
#export CCACHE_DIR=~/toolchain/ccache

# kernel
export ARCH=arm
#export USE_SEC_FIPS_MODE=true
export KERNEL_CONFIG="hisi-s10_A_lte_defconfig"

# build script							
export USER=`whoami`
export HOST_CHECK=`uname -n`
export OLDMODULES=`find -name *.ko`

# system compiler
export CROSS_COMPILE=${KERNELDIR}/arm-eabi-4.6/bin/arm-eabi-

NUMBEROFCPUS=`grep 'processor' /proc/cpuinfo | wc -l`


if [ "${1}" != "" ]; then
	export KERNELDIR=`readlink -f ${1}`
fi;

if [ ! -f ${KERNELDIR}/.config ]; then
	cp ${KERNELDIR}/arch/arm/configs/${KERNEL_CONFIG}.config
	make ${KERNEL_CONFIG}
fi;

. ${KERNELDIR}/.config

# remove previous zImage files
if [ -e ${KERNELDIR}/zImage ]; then
	rm ${KERNELDIR}/zImage
fi;
if [ -e ${KERNELDIR}/arch/arm/boot/zImage ]; then
	rm ${KERNELDIR}/arch/arm/boot/zImage
fi;

# remove all old modules before compile
cd ${KERNELDIR}
for i in $OLDMODULES; do
	rm -f $i
done;

# remove previous initramfs files
if [ -d ${INITRAMFS_TMP} ]; then
	echo "removing old temp iniramfs"
	rm -rf ${INITRAMFS_TMP}
fi;
if [ -f "/tmp/cpio*" ]; then
	echo "removing old temp iniramfs_tmp.cpio"
	rm -rf /tmp/cpio*
fi;

# clean initramfs old compile data
rm -f usr/initramfs_data.cpio
rm -f usr/initramfs_data.o

cd ${KERNELDIR}/
cp .config arch/arm/configs/${KERNEL_CONFIG}

if [ $USER != "root" ]; then
	make -j${NUMBEROFCPUS} modules || exit 1
else
	nice -n -15 make -j${NUMBEROFCPUS} modules || exit 1
fi;

# copy initramfs files to tmp directory
cp -ax $INITRAMFS_SOURCE ${INITRAMFS_TMP}

# clear git repositories in initramfs
if [ -e ${INITRAMFS_TMP}/.git ]; then
	rm -rf /tmp/initramfs-source/.git
fi;

# remove empty directory placeholders
find ${INITRAMFS_TMP} -name EMPTY_DIRECTORY -exec rm -rf {} \;

# remove mercurial repository
if [ -d ${INITRAMFS_TMP}/.hg ]; then
	rm -rf ${INITRAMFS_TMP}/.hg
fi;

# remove update initramfs scripts
rm -f ${INITRAMFS_TMP}/update*

# copy modules into initramfs
mkdir -p $INITRAMFS/lib/modules
mkdir -p ${INITRAMFS_TMP}/lib/modules
find -name '*.ko' -exec cp -av {} ${INITRAMFS_TMP}/lib/modules/ \;
${CROSS_COMPILE}strip --strip-debug ${INITRAMFS_TMP}/lib/modules/*.ko
chmod 755 ${INITRAMFS_TMP}/lib/modules/*


# make initramfs package
find ${INITRAMFS_TMP}/. | cpio -o -H newc | gzip > ${KERNELDIR}/initrd.img

# make zImage
if [ $USER != "root" ]; then
	time make -j${NUMBEROFCPUS} zImage

else
	time nice -n -15 make -j${NUMBEROFCPUS} zImage

fi;

# restore clean arch/arm/boot/compressed/Makefile_clean till next time
#cp ${KERNELDIR}/arch/arm/boot/compressed/Makefile_clean ${KERNELDIR}/arch/arm/boot/compressed/Makefile

if [ -e ${KERNELDIR}/arch/arm/boot/zImage ]; then
#	${KERNELDIR}/mkshbootimg.py ${KERNELDIR}/zImage ${KERNELDIR}/arch/arm/boot/zImage ${KERNELDIR}/payload.tar.xz ${KERNELDIR}/recovery.tar.xz

	cp ${KERNELDIR}/arch/arm/boot/zImage ${KERNELDIR}/zImage

	#abootimg --create ${KERNELDIR}/boot.img -f ${KERNELDIR}/bootimg.cfg  -k ${KERNELDIR}/zImage -r ${KERNELDIR}/initrd.img
	${KERNELDIR}/mkbootimg --kernel zImage --ramdisk initrd.img  --base 0x00008000 --pagesize 2048 --ramdiskaddr 0x01000000  --cmdline 'console=ttyS0 k3v2_pmem=1 vmalloc=512M androidboot.hardware=hws10101l mmcparts=mmcblk0:p1(xloader),p3(nvme),p4(misc),p5(splash),p6(oeminfo),p7(logo),p8(vrcb),p9(recovery2),p10(recovery),p11(boot),p12(modemimage),p13(modemnv),p14(modemnvm2),p15(cache),p16(system),p17(cust),p18(userdata),p19(reserve1),p20(reserve2),p21(reserve3),p22(data);mmcblk1:p1(sdcard2)' -o ${KERNELDIR}/boot.img

	# copy all needed to ready kernel folder
	cp ${KERNELDIR}/.config ${KERNELDIR}/arch/arm/configs/${KERNEL_CONFIG}
	cp ${KERNELDIR}/.config ${KERNELDIR}/READY-JB/

#	rm ${KERNELDIR}/READY-JB/zImage
#	rm ${KERNELDIR}/READY-JB/Kernel_*

	stat ${KERNELDIR}/boot.img
	#GETVER=`grep 'Siyah-.*-V' arch/arm/configs/${KERNEL_CONFIG} | sed 's/.*".//g' | sed 's/-J.*//g'`
	cp ${KERNELDIR}/boot.img /${KERNELDIR}/READY-JB/boot/
	cd ${KERNELDIR}/READY-JB/

	zip -r Kernel_${GETVER}-`date +"[%H-%M]-[%d-%m]-JB-ICS"`.zip .
	echo "8A BC 62 56 3D 39 19 46 FA 6F 82 45 62 86 ED 2E D3 9F 96 66"
	echo "90 8C 28 A9 BD 33 76 88 B9 6F F5 EF 84 8B 69 01 AD B8 57 C5"

else
	echo "Kernel STUCK in BUILD! no zImage exist"
fi;
