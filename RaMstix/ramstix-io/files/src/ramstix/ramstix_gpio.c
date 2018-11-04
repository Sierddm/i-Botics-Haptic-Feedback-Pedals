/**
 * @file ramstix_gpio.c
 * @brief GPIO implementation.
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include "ramstix/ramstix_gpio.h"


void ramstixSetPinValue(int fd, int pin_number, double val)
{
    unsigned int current = readGPMCFPGA16(fd, GPIO_OUTPUT_REG);
    // Pin value 1 should shift 0 times, subtract one.
    unsigned int mask = 0x01 << (pin_number - 1);
    RAMSTIX_PRINT("GPIO Mask: 0x%x\n", mask);
    if (0 < val)
    {
        // Set to 1.
        writeGPMCFPGA16(fd, GPIO_OUTPUT_REG, (current | mask));
    }
    else
    {
        // Set to 0.
        writeGPMCFPGA16(fd, GPIO_OUTPUT_REG, (current & ~mask));
    }
}

int ramstixGetPinValue(int fd, int pin_number)
{
    unsigned int current = readGPMCFPGA16(fd, GPIO_INPUT_REG);

    RAMSTIX_PRINT("GPIO pinshift: %i, current: 0x%x\n", pin_number-1, current);
    return ((current >> (pin_number-1)) & 0x01);
    return 0;
}
