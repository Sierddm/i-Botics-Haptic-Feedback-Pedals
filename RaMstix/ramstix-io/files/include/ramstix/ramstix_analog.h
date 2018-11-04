/**
 * @file ramstix_analog.h
 * @brief
 * @author Jan Jaap Kempenaar, University of Twente
 * @addtogroup analogIO
 * @{
 *
 * Requires the @ref c_gpmc_fpga_userdriver "GPMC userspace driver".
 *
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.


int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    // Open ADC 1 for reading:
    ramstixInitADC(fd, 0);
    // Get value from ADC 1:
    printf("Value measured on ADC1: %f V\n", ramstixGetADCValue(fd, 0));
    // Close ADC 1.
    ramstixCloseADC(fd,0);
    close(fd);
    return 0;
}
 * @endcode
 *
 */

#ifndef _RAMSTIX_ANALOG_H_
#define _RAMSTIX_ANALOG_H_

// Enable debug
// #define RAMSTIX_DEBUG
#include <stdint.h>

#include "ramstix/ramstix_definitions.h"
#include "gpmc_fpga_user/gpmc_fpga_user.h"

/**
 * @brief Initialise an ADC.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset ADC offset (0 for ADC1, 1 for ADC2, etc.)
 */
void ramstixInitADC(int fd, unsigned int component_offset);

/**
 * @brief Get measured voltage
 *
 * Get the measured voltage. The voltage is calculated based on a defined lower and upper bound, see ramstix_definitions.h. In the calculation it is assumed that the
 * voltage scale of the AD convertor is linear.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset ADC offset (0 for ADC1, 1 for ADC2, etc.)
 * @return Returns the measured voltage value.
 */
double ramstixGetADCValue(int fd, unsigned int component_offset);

/**
 * @brief Get measured integer value.
 *
 * Returns an unchanged measured integer value from the ADC. The ADC returns  16-bit value.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset ADC offset (0 for ADC1, 1 for ADC2, etc.)
 * @return Returns the current measured integer value.
 */
int16_t ramstixGetADCIntValue(int fd, unsigned int component_offset);

/**
 * @brief Close/Disable ADC.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset ADC offset (0 for ADC1, 1 for ADC2, etc.)
 */
void ramstixCloseADC(int fd, unsigned int component_offset);

/**
 * @brief Initialise/enable the DAC output.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset DAC offset (0 for ADC1, 1 for ADC2, etc.)
 */
void ramstixInitDAC(int fd, unsigned int component_offset);

/**
 * @brief Set output voltage on DAC.
 *
 * Set output voltage on DAC. The output is calculated based on the upper and lower limits defined in ramstix_definitions.h. The calculation
 * assumes that the voltage is a linear scale.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset DAC offset (0 for ADC1, 1 for ADC2, etc.)
 * @param volt Voltage set on the output.
 */
void ramstixSetDACValue(int fd, unsigned int component_offset, double volt);

/**
 * @brief Set output integer output value on DAC.
 *
 * This function assumes that the users' application takes care of calculating the correct output value. The output value is a 16-bit value.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset DAC offset (0 for ADC1, 1 for ADC2, etc.)
 * @param value Integer value set on the DAC.
 */
void ramstixSetDACIntValue(int fd, unsigned int component_offset, int value);

/**
 * @brief Close/disable DAC output.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset DAC offset (0 for ADC1, 1 for ADC2, etc.)
 */
void ramstixCloseDAC(int fd, unsigned int component_offset);




#endif // _RAMSTIX_ANALOG_H_
