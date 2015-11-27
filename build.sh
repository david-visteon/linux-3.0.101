#!/bin/bash
HOME_DIR=`pwd`
BUILD_DIR="./.build"
BUILD_CMD=""
BOARD_NAME="imx6q-visteon-sx5"
MENU_CONFIG=0
BOARD_CONFIG="imx6q-visteon_defconfig"
ARCH=arm
QUIET=0

USAGE_EXAMPLE=" -O .build -b imx6q-visteon-sx5 -c all -C imx6q-visteon_defconfig"
while getopts "O:c:mC:qh" OPT; do
	case "$OPT" in
		O)
			BUILD_DIR=$OPTARG
			;;
		c)
            BUILD_CMD=$OPTARG
			;;			
		m)
			MENU_CONFIG=1
			;;
		C)
			BOARD_CONFIG=$OPTARG
			if [[ -z "$BOARD_CONFIG" ]]; then
				BOARD_CONFIG="imx6q-visteon_defconfig"
			fi
			;;
		q)
			QUIET=1
			;;
		h | \?)
			echo
			echo "USAGE:            - $(basename 0) $USAGE"
			echo "USAGE:            - $(basename 0) $USAGE_EXAMPLE"
			echo
			echo " clean            - Delete build output directory."
			echo " all              - Build and install kernel."
			echo " uImage           - Build kernel as uImage."
			echo " defconfig        - Copy defconfig into build directory."
			echo " menuconfig       - Configure the kernel with menuconfig."
			echo " distclean        - Delete build and delivery directory."
			exit 0
			;;
	esac
done

if [[ -z "$BOARD_CONFIG" || ! -e arch/$ARCH/configs/$BOARD_CONFIG ]]; then
	echo
	echo "ERROR: Specified BOARD_CONFIG ($BOARD_CONFIG) does not exist"
	$0 -h
	exit 1
fi

DEBUG_SECTION_MISMATCH=""
DELIVER_DIR="$HOME_DIR/delivery/target/$ARCH/$BOARD_NAME"
#KRNL_DEV_DIR="$DELIVER_DIR/kernel_dev"
KRNL_VER=$(make -s kernelversion)
CROSS_SDK=/opt/freescale/usr/local/gcc-4.6.2-glibc-2.13-linaro-multilib-2011.12/fsl-linaro-toolchain/bin/arm-none-linux-gnueabi-

if [[ "$QUIET" != "1" ]]; then
	echo
	echo "home directory   : $HOME_DIR"
	echo "Build directory  : $BUILD_DIR"
	echo "Build cmd        : $BUILD_CMD"
	echo "Board name       : $BOARD_NAME"
	echo "Board Config     : $BOARD_CONFIG"
	echo "Kernel Version   : $KRNL_VER"
	echo "Cross Compile    : $CROSS_SDK"
fi

launch_build() {
	local _build_cmd=$1

	case "${_build_cmd}" in

		"all" )
			# Create delivery directories
			mkdir -pv $HOME_DIR/delivery/target/$ARCH/$BOARD_NAME/boot
			mkdir -pv $HOME_DIR/delivery/target/$ARCH/usr
			mkdir -pv $HOME_DIR/$BUILD_DIR

			# Copy defconfig into build directory
			make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR $BOARD_CONFIG
			if [ "$MENU_CONFIG" = "1" ]; then
				# Edit / view the Kernel config
				make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR menuconfig
			elif [[ "$QUIET" != "0" ]]; then
				echo "Skipping 'menuconfig'"
			fi

			# Build kernel sources
			make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR

			# Build an image for u-boot
			make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR uImage

			# Copy Image and zImage to delivery directory
			[ -e $HOME_DIR/$BUILD_DIR/arch/$ARCH/boot/Image ] &&  cp $HOME_DIR/$BUILD_DIR/arch/$ARCH/boot/Image  $DELIVER_DIR/boot
			[ -e $HOME_DIR/$BUILD_DIR/arch/$ARCH/boot/zImage ] && cp $HOME_DIR/$BUILD_DIR/arch/$ARCH/boot/zImage $DELIVER_DIR/boot
			[ -e $HOME_DIR/$BUILD_DIR/arch/$ARCH/boot/uImage ] && cp $HOME_DIR/$BUILD_DIR/arch/$ARCH/boot/uImage $DELIVER_DIR/boot
			
			# Build kernel modules
			make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR INSTALL_MOD_PATH=$DELIVER_DIR modules
			;;

		"clean" )
			make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR  clean
			if [[ -n $BUILD_DIR ]]; then
				if [[ "$(readlink -f ${BUILD_DIR})" != "$(readlink -f ${PWD})" ]]; then
					echo "!!!!!! REMOVING $(readlink -f ${BUILD_DIR}) !!!!!!"
					rm -rfv $BUILD_DIR
				else
					echo "Not removing $(readlink -f ${BUILD_DIR})"
				fi
			else
				echo "BUILD_DIR is not defined"
			fi
			if [[ -n $KRNL_DEV_DIR ]]; then
				rm -rfv $KRNL_DEV_DIR
			else
				echo "KRNL_DEV_DIR is not defined"
			fi
			make $DEBUG_SECTION_MISMATCH ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR  mrproper
			;;

		"build" )
			make ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR
			;;

		"defconfig" )
			mkdir -pv $HOME_DIR/$BUILD_DIR
			make ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR $BOARD_CONFIG
			;;

		"menuconfig" )
			make ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR menuconfig
			;;

		"distclean" )
			# Use 'clean' to do most of the dirty work
			#$0 -b $BOARD_NAME -C $BOARD_CONFIG -O $BUILD_DIR -c clean -q
			launch_build clean

			rm -rfv $HOME_DIR/delivery
			;;

		"uImage" )
			make ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR uImage
			;;

		"mrproper" )
			make ARCH=$ARCH CROSS_COMPILE=$CROSS_SDK O=$HOME_DIR/$BUILD_DIR mrproper
			;;			
		* )
			echo "Unknown command: ${_build_cmd}"
			exit 1
			;;
	esac
}

for _cmd in ${BUILD_CMD}; do
	launch_build "${_cmd}"
done

exit 0

