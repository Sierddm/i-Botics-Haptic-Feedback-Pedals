SUMMARY = "Builds an external non real-time Ramstix Linux kernel module"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

inherit module

PR = "r0"
PV = "1.0"

INSANE_SKIP_${PN} = "installed-vs-shipped "
FILESEXTRAPATHS_prepend := "${THISDIR}/files/src/gpmc_fpga:${THISDIR}/files/include/gpmc_fpga:${THISDIR}/files/include/gpmc_fpga_user:"

SRC_URI = "file://Makefile \
	   file://Kbuild \
           file://gpmc_fpga.c \
	   file://gpmc_fpga.h \
           file://gpmc_fpga_user.h \
           file://COPYING \
          "
S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.
#KERNEL_MODULE_AUTOLOAD += "gpmc_fpga"
