/**
 * @file ramstix_ms5611_spibarometer.h
 * @brief Driver for the MS5611 spi-interfaced barometer
 * @author Geert Folkertsma, University of Twente
 *
 * @addtogroup spi
 * @{
 * @addtogroup spi_custom Sensors based on SPI module
 * @brief Userspace configuration and read function for the MS5611.
 * @{
 * 
 * Datasheet of the MS5611: [Farnell datasheet](http://www.farnell.com/datasheets/1756128.pdf)
 * 
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.
#include "ramstix/ramstix_ms5611_spibarometer.h"

int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    //Init the sensor, using pin 3 as chip select (the first available output pin, next to CLK/MOSI)
    ramstixInitMS5611(fd, 3);

    pressure = ramstixGetMS5611Pressure(fd, 3);
    printf("[MS5611_barometer] Pressure: %f Pa\n",pressure);
    // Close SPI interface.
    ramstixCloseMS5611(fd, 3);
    close(fd);
    return 0;
}
 * @endcode
 */

#ifndef _RAMSTIX_MS5611_SPIBAROMETER_H_
#define _RAMSTIX_MS5611_SPIBAROMETER_H_

// #define RAMSTIX_DEBUG

#include "ramstix/ramstix_spi.h"
#include "ramstix/ramstix_gpio.h"
#include <time.h>
#include <rtdm/rtdm.h>
#include <inttypes.h>

// registers of the device
#define MS5611_D1 0x40
#define MS5611_D2 0x50
#define MS5611_RESET 0x1E

// D1 and D2 result size (bytes)
#define MS5611_D1D2_SIZE 3

// OSR (Over Sampling Ratio) constants
#define MS5611_OSR_256 0x00
#define MS5611_OSR_512 0x02
#define MS5611_OSR_1024 0x04
#define MS5611_OSR_2048 0x06
#define MS5611_OSR_4096 0x08

#define MS5611_PROM_BASE_ADDR 0xA2 // by adding ints from 0 to 6 we can read all the prom configuration values. 
// C1 will be at 0xA2 and all the subsequent are multiples of 2
#define MS5611_PROM_REG_COUNT 6 // number of registers in the PROM
#define MS5611_PROM_REG_SIZE 2 // size in bytes of a prom registry.

// conversion-in-progress status
enum MS5611_CONVERSION {
  MS5611_IDLE,
  MS5611_BUSY_T,
  MS5611_BUSY_P
};

enum MS5611_CONVERSION_TYPE{
  MS5611_CONVERT_P,
  MS5611_CONVERT_T
};

/**
 * @brief Utility function to compute the time passed in milliseconds
 *
 * @param from timespec (clock_gettime result) of start
 * @param to timespec (clock_gettime result) of end (typically "now")
 * 
 * @return difference in milliseconds between <from> and <to>
 */
int difftime_ms(struct timespec* from, struct timespec* to);

/**
 * @brief Utility function to execute a milliseconds delay
 *
 * @param wait_ms delay the specified number of milliseconds
 */
void delay_ms(const double wait_ms);

/**
 * @brief Utility function to easily switch over SPI's bit length
 *
 * @param fd GPMC memory controller file descriptor
 * @param numBits number of bits in the messages (1..32)
 */
void setSPIbits(int fd, unsigned int numBits);

/**
 * @brief Initialise MS5611 barometer senser; get calibration parameter.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param csPin GPIO pin number used for chip select.
 */
void ramstixInitMS5611(int fd, unsigned int csPin);

/**
 * @brief Start an ADC conversion on the chip
 *
 * @param fd GPMC memory controller file descriptor.
 * @param csPin GPIO pin number used for chip select.
 * @param type Do a P or T conversion (MS5611_CONVERT_P/MS5611_CONVERT_T)
 */
void ramstixMS5611StartConversion(int fd, unsigned int csPin, enum MS5611_CONVERSION_TYPE type);

/**
 * @brief Get the result of an ADC conversion off the chip
 *
 * @param fd GPMC memory controller file descriptor.
 * @param csPin GPIO pin number used for chip select.
 * 
 * @return ADC measurement result from the chip
 */
uint32_t ramstixMS5611GetConversionResult(int fd, unsigned int csPin);

/**
 * @brief Get a pressure reading from the sensor.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param csPin GPIO pin number used for chip select.
 * 
 * @return Pressure measured in Pa (100.000 Pa = 1 bar)
 */
double ramstixGetMS5611Pressure(int fd, unsigned int csPin);

/**
 * @brief Close the sensor.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param csPin GPIO pin number used for chip select.
 */
void ramstixCloseMS5611(int fd, unsigned int csPin);

#endif