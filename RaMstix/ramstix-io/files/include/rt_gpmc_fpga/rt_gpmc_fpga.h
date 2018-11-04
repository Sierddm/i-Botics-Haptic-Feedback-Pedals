/**
 * @file rt_gpmc_fpga_internal.h
 * @brief Real-Time General Purpose Memory Controller kernel module for the RaMstix board.
 * @author Jan Jaap Kempenaar, University of Twente
 * @author Wilbert van de Ridder, University of Twente
 * @addtogroup gpmc_fpga
 * @{
 * @addtogroup c_gpmc_fpga_kernel_rt GPMC kernel module (real-time)
 * @brief Kernel space data definitions and read/write functions.
 * @{
 *
 * Kernel space device driver for the GPMC FPGA connection on the RaMstix board. This module is used by the userspace driver to exchange data with the FPGA hardware components.
 * The kernel module's implementation conforms to the [Real-Time Driver Model from Xenomai](http://www.xenomai.org/documentation/trunk/html/api/group__rtdm.html)
 */
#ifndef _RT_GPMC_FPGA_H_
#define _RT_GPMC_FPGA_H_

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/io.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <asm/uaccess.h>
#include <linux/types.h>
//#include <sys/ioctl.h>
#if !defined(CONFIG_LEDS_GPIO) && !defined(CONFIG_LEDS_GPIO_MODULE)
#define LED_ON  1       ///< Led on
#define LED_OFF 0       ///< Led off
#include <linux/gpio.h>
#endif
#include <omap34xx.h>
#include <gpmc.h>
#include <rtdm/driver.h>
/**
 * Module only definitions
 */
// #define DEBUG_MODULE

#ifdef DEBUG_MODULE
  #define DPRINTK    rtdm_printk    ///< Debug printk.
  #define TRACE_OBJECT(func) rtdm_printk(KERN_INFO "rt_gpmc_fpga: TRACE %s\n", func)    ///< Debug trace function.
#else
  #define DPRINTK(string, ...)      ///< Disabled debug printk
  #define TRACE_OBJECT(...)	        ///< Disabled trace function
#endif
 
#define SUCCESS 0                   ///< Successful return.
/*
 * GPMC device driver definitions.
 */
#define GPMC_REVISION           0x00    ///< GPMC revision number.
#define GPMC_SYSCONFIG          0x10    ///< GPMC System config.
#define GPMC_SYSSTATUS          0x14    ///< GPMC System status.
#define GPMC_IRQSTATUS          0x18    ///< GPMC IRQ status.
#define GPMC_IRQENABLE          0x1c    ///< GPMC IRQ enable.
#define GPMC_TIMEOUT_CONTROL    0x40    ///< GPMC TIMEOUT control
#define GPMC_ERR_ADDRESS        0x44    ///< GPMC error address.
#define GPMC_ERR_TYPE           0x48    ///< GPMC error type.
#define GPMC_CONFIG             0x50    ///< GPMC configuration.
#define GPMC_STATUS             0x54    ///< GPMC status.
#define GPMC_PREFETCH_CONFIG1   0x1e0   ///< .
#define GPMC_PREFETCH_CONFIG2   0x1e4   ///< .
#define GPMC_PREFETCH_CONTROL   0x1ec   ///< .
#define GPMC_PREFETCH_STATUS    0x1f0   ///< .
#define GPMC_ECC_CONFIG         0x1f4   ///< .
#define GPMC_ECC_CONTROL        0x1f8   ///< .
#define GPMC_ECC_SIZE_CONFIG    0x1fc   ///< .
#define GPMC_ECC1_RESULT        0x200   ///< .

#define GPMC_CS0_OFFSET         0x60    ///< GPMC chipselect 0 offset.
#define GPMC_CS_SIZE            0x30    ///< GPCM chipselect memory size.

#define VALIDATE_VERSION        3       ///< FPGA version register.
#define VALIDATE_REG1           7       ///< FPGA validation read/write register 1.
#define VALIDATE_REG2           11      ///< FPGA validation read/write register 2.


#define GET_MAJOR(val)			(0x7 & (val >> 13)) ///< Get major version number from register value.
#define GET_MINOR(val)			(0x1f & (val >> 8)) ///< Get minor version number from register value.
#define GET_BUILD(val)			(0xff & val)        ///< Get build version number from register value.
#define SET_VERSION(major, minor, build)	(((major & 0x7) << 13) | ((minor & 0x1f) << 8) | (build & oxff)) ///< Construct register value for major, minor and build version.


#define DRIVER_NAME         "RaMstix GPMC FPGA driver"                          ///< GPMC driver name.
#define PERIPHERAL_NAME     "RaMstix GPMC FPGA driver device"                   ///< GPMC peripheral name
#define PROVIDER_NAME       "Robotics and Mechatronics, University of Twente"   ///< GPMC device driver provider.
#define DEVICE_SUBCLASS     123                                                 ///< Device subclass (this value is made up).

/**
 * @brief Read 32-bit value from GPMC CS6 offset.
 *
 * @param offset Register offset.
 * @return Returns current 32-bit value from offset.
 */
u32 read32_from_fpga(int offset);

/**
 * @brief Write 32-bit value to GPMC CS6 offset.
 *
 * @param offset Register offset.
 * @param val 32-bit value.
 */
void write32_to_fpga(int offset, u32 val);

/**
 * @brief Read 16-bit value from GPMC CS6 offset.
 *
 * @param offset Register offset.
 * @return Returns current 16-bit value from GPMC CS6 offset.
 */
u16 read16_from_fpga(int offset);

/**
 * @brief Write 16-bit value to GPMC CS6 offset.
 *
 * @param offset Register offset
 * @param val 16-bit value.
 */
void write16_to_fpga(int offset, u16 val);
/**
 * @brief Write 32-bite value to GPMC base offset.
 *
 * @param idx GPMC base register offset.
 * @param val 32-bit value.
 */
static void rt_gpmc_write_reg(int idx, u32 val);


/**
 * @brief Write 32-bit value to specific CS offset.
 *
 * @param cs Chipselect (0 .. 7).
 * @param idx Register offset.
 * @param val 32-bit value.
 */
void rt_gpmc_cs_write_reg(int cs, int idx, u32 val);

 /**
 * @brief Open FPGA GPMC device.
 * 
 * Open the FPGA GPMC device. The function also validates if the FPGA is programmed properly.
 *
 * @param context RTDM context structure.
 * @param user_info RTDM user_info structure.
 * @param oflags RTDM device flags.
 * @return Returns 0 on success.
 */
static int nrt_gpmc_fpga_open(struct rtdm_fd *fd, int oflags);

/**
 * @brief Close FPGA GPMC device.
 * 
 * Close the FPGA GPMC device. Resets the validation registers used in the nrt_gpmc_fpga_open function.
 *
 * @param context RTDM context structure.
 * @param user_info RTDM user_info structure.
 * @return Return 0 on success.
 */
static void nrt_gpmc_fpga_close(struct rtdm_fd *fd);

/**
 * @brief ioctl_nrt Non-realtime read access dummy
 * This real-time driver should not be called from a non-realtime userspace application.
 * @return Returns -ENOSYS
 */
static int nrt_gpmc_fpga_ioctl(struct rtdm_fd *fd, unsigned int request, void __user *arg);


/**
 * @brief ioctl_rt Read register value based on request.
 * We misuse ioctl, normally this function should be used to control a stream device, however our device is not a stream device and we use this function to set specific register values.
 * 
 * Maybe for final implementation expand and use files for all hardware devices as linux does?
 * @param context RTDM context structure.
 * @param user_info RTDM user_info structure.
 * @param request Request id.
 * @param arg arg should be a pointer to a gpmc_fpga_data structure.
 * @return Returns 0 on success, -1 when an unknown request was submitted.
 */
static int rt_gpmc_fpga_ioctl(struct rtdm_fd *fd, unsigned int request, void __user *arg);

/**
 * @brief Validity check.
 *
 * Performs a few read/write actions to see if FPGA is programmed valid.
 *
 * @return Returns 0 if FPGA appears to have a valid applicatoin.
 */
static int rt_gpmc_valid_fpga(void);

/**
 * @brief Initialise GPMC FPGA RaMstix driver.
 * 
 * @return Returns 0 on success.
 */
int __init rt_gpmc_fpga_init(void);

/**
 * @brief Deregister GPMC FPGA RaMstix driver.
 *
 */
void __exit rt_gpmc_fpga_exit(void);

#endif // _RT_GPMC_FPGA_H_

/**
 * @}
 * @}
 */
