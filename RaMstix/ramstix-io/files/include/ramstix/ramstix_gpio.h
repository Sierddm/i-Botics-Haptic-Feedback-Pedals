/**
 * @file ramstix_gpio.h
 * @brief General purpose input and output on the RaMstix.
 * @author Jan Jaap Kempenaar, University of Twente
 *
 * @addtogroup gpio
 * @{
 *
 * Functions to control the output or read the input from the Digital input and output pinds on the RaMstix device. Requires the @ref c_gpmc_fpga_userdriver "GPMC userspace driver".
 *
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.
#include "ramstix_gpio.h"

int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    // Set pin 1 on header SV5 to '1':
    ramstixSetPinValue(fd, 1, 1);
    // Get pin status from pin 1 on header SV7:
    if (ramstixGetPinValue(fd, 1) {
        printf("Pin SV7-1 is '1'\n");
    } else {
        printf("Pin SV7-1 is '0'\n");
    }
    close(fd);
    return 0;
}
 * @endcode
 */

#ifndef _RAMSTIX_GPIO_H_
#define _RAMSTIX_GPIO_H_

// Enable debug
// #define RAMSTIX_DEBUG
#include "ramstix/ramstix_definitions.h"
#include "gpmc_fpga_user/gpmc_fpga_user.h"

/**
 * @brief Set GPIO pin value.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param pin_number GPIO pin number.
 * @param val Value to put on the pin.
 */
void ramstixSetPinValue(int fd, int pin_number, double val);

/**
 * @brief Get GPIO pin value.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param pin_number GPIO pin number.
 * @return Returns the current value on the IO pin
 */
int ramstixGetPinValue(int fd, int pin_number);

#endif // _RAMSTIX_GPIO_H_

/**
 * @}
 */
