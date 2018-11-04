
SUMMARY = "Builds the RaMstix IO userspace library and utilities"
LICENSE = "GPLv2"
LIC_FILES_CHKSUM = "file://COPYING;md5=12f884d2ae1ff87c09e5b7ccc2c4ca7e"

PR = "r0"
PV = "1.0"

INSANE_SKIP_${PN} = "installed-vs-shipped "

SRC_URI = "file://*"
S = "${WORKDIR}"

# The inherit of module.bbclass will automatically name module packages with
# "kernel-module-" prefix as required by the oe-core build environment.

# Define packages
PACKAGES = "${PN} ${PN}-dev ${PN}-dbg ${PN}-staticdev"
RDEPENDS_${PN}-staticdev = ""
RDEPENDS_${PN}-dev = ""
RDEPENDS_${PN}-dbg = ""

# Define which packages contains which files
FILES_${PN} += "/usr/ramstix_gpmc_fpga/bin/*"
FILES_${PN}-staticdev += "/usr/ramstix_gpmc_fpga/lib/*.a"

# Shared library settings
# see: https://wiki.yoctoproject.org/wiki/TipsAndTricks/Packaging_Prebuilt_Libraries
INSANE_SKIP_${PN} = "ldflags"
INHIBIT_PACKAGE_STRIP = "1"
INHIBIT_SYSROOT_STRIP = "1"
SOLIBS = ".so"
FILES_SOLIBSDEV = ""
 
# bitbake functions
do_compile() {
	oe_runmake libramstix staticlibramstix rw_util reg_util
}
 
do_install() {
	# Install library
	install -d ${D}/usr/ramstix_gpmc_fpga/lib
	install -m 0644 libramstix.a ${D}/usr/ramstix_gpmc_fpga/lib
	
	# Install utility applications
	install -d ${D}/usr/ramstix_gpmc_fpga/bin
	install -m 0744 ./build/bin/gpmc_fpga_user/utility/reg_util ${D}/usr/ramstix_gpmc_fpga/bin
	install -m 0744 ./build/bin/gpmc_fpga_user/utility/rw_util ${D}/usr/ramstix_gpmc_fpga/bin

	# Install shared library
	install -d ${D}${libdir}
	install -m 0755 ${WORKDIR}/libramstix.so ${D}${libdir}
}
