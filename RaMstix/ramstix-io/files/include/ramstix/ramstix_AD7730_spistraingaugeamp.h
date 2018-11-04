/**
 * @file ramstix_AD7730_spistraingaugeamp.h 
 * @brief AD7730 straingauge amplifier implementation
 * @author Douwe Dresscher, University of Twente
 *
 * @addtogroup spi
 * @{
 * @addtogroup spi_custom Sensors based on SPI module
 * @brief Userspace configuration and read function for the MS5611.
 * @{
 * 
 * Datasheet of the AD7730:[dataheet](http://www.analog.com/media/en/technical-documentation/data-sheets/AD7730_7730L.pdf)
 * 
**/

#ifndef _RAMSTIX_AD7730_SPISTRAINGAUGEAMP_H_
#define _RAMSTIX_AD7730_SPISTRAINGAUGEAMP_H_

#define RAMSTIX_DEBUG

#include "ramstix/ramstix_spi.h"
#include "ramstix/ramstix_gpio.h"
#include <time.h>
#include <rtdm/rtdm.h>
#include <inttypes.h>
#include <math.h>

//modes
#define WRITE_MODE ((uint8_t)0x00)
#define SINGLE_READ_MODE ((uint8_t)0x10)
#define START_CONTINUOUS_READ_MODE ((uint8_t)0x20)
#define END_CONTINUOUS_READ_MODE ((uint8_t)0x30)

//registers
#define STATUS_REGISTER ((uint8_t)0x00)
#define COMMUNICATIONS_REGISTER ((uint8_t)0x00)
#define DATA_REGISTER ((uint8_t)0x01)
#define MODE_REGISTER ((uint8_t)0x02)
#define FILTER_REGISTER ((uint8_t)0x03)
#define DAC_REGISTER ((uint8_t)0x04)
#define OFFSET_REGISTER ((uint8_t)0x05)
#define GAIN_REGISTER ((uint8_t)0x06)
#define TEST_REGISTER ((uint8_t)0x07)

//VALUE_TO_WRITE = MODE | REGISTER

//filter register bits
#define SF11    ((uint32_t)0x800000)
#define SF10    ((uint32_t)0x400000)
#define SF9     ((uint32_t)0x200000)
#define SF8     ((uint32_t)0x100000)
#define SF7     ((uint32_t)0x080000)
#define SF6     ((uint32_t)0x040000)
#define SF5     ((uint32_t)0x020000)
#define SF4     ((uint32_t)0x010000)
#define SF3     ((uint32_t)0x008000)
#define SF2     ((uint32_t)0x004000)
#define SF1     ((uint32_t)0x002000)
#define SF0     ((uint32_t)0x001000)
#define SKIP    ((uint32_t)0x000200)
#define FAST    ((uint32_t)0x000100)
#define AC      ((uint32_t)0x000020)
#define CHP     ((uint32_t)0x000010)
#define DL3     ((uint32_t)0x000008)
#define DL2     ((uint32_t)0x000004)
#define DL1     ((uint32_t)0x000002)
#define DL0     ((uint32_t)0x000001)

//mode register bits
#define MD2     ((uint16_t)0x8000)
#define MD1     ((uint16_t)0x4000)
#define MD0     ((uint16_t)0x2000)
#define NBU     ((uint16_t)0x1000)
#define DEN     ((uint16_t)0x0800)
#define D1      ((uint16_t)0x0400)
#define D0      ((uint16_t)0x0200)
#define WL      ((uint16_t)0x0100)
#define HIREF   ((uint16_t)0x0080)
#define RN1     ((uint16_t)0x0020)
#define RN0     ((uint16_t)0x0010)
#define CLKDIS  ((uint16_t)0x0008)
#define BO      ((uint16_t)0x0004)
#define CH1     ((uint16_t)0x0002)
#define CH0     ((uint16_t)0x0001)

/**
 * @brief Initialise AD7730.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param baud baud rate for SPI connection
 * @param ncsPin GPIO pin number used for chip select.
 * @param nresetPin GPIO pin number used rot reset.
 * @param nrdyPin GPIO pin number used for ready pin.
 * @param mode_register mode register value. Options: NBU|RN1|RN0 - refer to AD7730 datasheet for details
 * @param filter_register filter register value. Options: SF11..SF0|SKIP|FAST|AC|CHP|DL3...DL0 - refer to AD7730 datasheet for details
 */
void ramstixInitAD7730(int fd, uint32_t baud, unsigned int ncsPin, unsigned int nresetPin, unsigned int nrdyPin, uint16_t mode_register, uint32_t filter_register);

/**
 * @brief Get the latest measurement result, in mV
 *
 * @param fd GPMC memory controller file descriptor.
 * @param baud baud rate for SPI connection
 * @param csPin GPIO pin number used for chip select.
 * @param nrdyPin GPIO pin number used for ready pin.
 *
 * @return the latest measurement result, in mV
 */
double ramstixGetAD7730Value(int fd, uint32_t baud, unsigned int ncsPin, unsigned int nrdyPin);

/**
 * @brief Close the sensor.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param csPin GPIO pin number used for chip select.
 */
void ramstixCloseAD7730(int fd, unsigned int csPin);

#endif
