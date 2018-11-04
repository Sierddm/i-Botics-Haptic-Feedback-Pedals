/**
 * @file ramstix_pwmout.h
 * @brief PWM and Stepper motor control on the RaMstix.
 * @author Jan Jaap Kempenaar, University of Twente
 *
 * @addtogroup pwm
 * @{
 *
 * Functions to control the pulse width modules on the RaMstix board. The PWMs can be configured to support different types of h-bridges. In addition the PWMs can be configured as a stepper motor with variable frequency. Requires the @ref c_gpmc_fpga_userdriver "GPMC userspace driver".
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.
#include "ramstix_pwmout.h"

int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    // configure PWM 2 for the VNH3SP30 H-Bridge as non-braking at 22 kHz.
    ramstixInitPWMOut(fd, 1, VNH3SP30, NO_BRAKING, NON_INTERLACED);
    // Set duty cycle to 50%.
    ramstixSetPWMValue(fd, 1, 0.5);
    // disable the PWM.
    ramstixClosePWMOut(fd, 1);
    close(fd);
    return 0;
}
 * @endcode
 */

#ifndef _RAMSTIX_PWMOUT_H_
#define _RAMSTIX_PWMOUT_H_

// enable debug.
// #define RAMSTIX_DEBUG
#define PWM_DEFAULT_FREQUENCY 16000	///< Default frequency for PWM, set at 16 kHz.
#include "ramstix/ramstix_definitions.h"
#include "gpmc_fpga_user/gpmc_fpga_user.h"

/**
 * @brief Pulse width modulator type/mode.
 */
enum BRIDGE_TYPE
{
    BERT = 0,       ///< Configure for Bert van den Berg h-bridge.
    THIEMO = 1,     ///< Configure for Thiemo van Engelen h-bridge.
    VNH3SP30 = 2,   ///< Configure for the VNH3SP30 h-bridge.
    NONE = 3        ///< Configure to only produce a PWM signal.
};
/**
 * @brief Braking or non braking mode.
 */
enum BRAKING
{
    NO_BRAKING = 0, ///< Configure no braking mode.
    BRAKING = 1     ///< Configure braking mode.
};
/**
 * @brief Interlaced or non interlaced mode.
 */
enum INTERLACED
{
    NON_INTERLACED = 0, ///< Disable interlaced PWM.
    INTERLACED = 1      ///< Enable interlaced PWM.
};

/* PWM generator */
/**
 * @brief Set PWM duty cycle. Range from -1 to 1.
 *
 * Set the duty cycle of the PWM. The input expects a value between -1 and 1 which is translated to a PWM signal. 0 equals 0% duty cycle and 1 results in 100%. The sign determines the direction of the PWM.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @param value Pulse width value. 1 equals 100%, 0 equals 0%. Negative values result in opposite direction.
 */
void ramstixSetPWMValue(int fd, unsigned int component_offset, double value);

/**
 * @brief Get PWM duty cycle.
 *
 * Returns the scalar value currently in the PWM value register.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @return Returns the current integer value in the PWM duty cycle register.
 */
int ramstixGetPWMValue(int fd, unsigned int component_offset);

/**
 * @brief Set PWM frequency.
 *
 * Set the frequency of the pulse width modulator.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @param value Frequency value, the correct integer value is calculated by the function.
 */
void ramstixSetPWMFrequency(int fd, unsigned int component_offset, double value);

/**
 * @brief Get PWM frequency prescaler.
 *
 * Get the PWM frequency prescaler value. To calculate the actual frequency: F = CLOCK_FREQUENCY * Scalar / 2^27
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @return Returns the current prescaler value.
 */
unsigned int ramstixGetPWMPrescaler(int fd, unsigned int component_offset);

/**
 * @brief Initialise PWM.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @param bridge Configuration mode, based on type of H-bridge.
 * @param brake Enable braking mode.
 * @param interl Enable interlacing.
 */
void ramstixInitPWMOut(int fd, unsigned int component_offset, enum BRIDGE_TYPE bridge, enum BRAKING brake, enum INTERLACED interl);

/**
 * @brief Close PWM.
 *
 * Disables the PWM and sets duty cycle to zero.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 */
void ramstixClosePWMOut(int fd, unsigned int component_offset);

// Frequency generator (Stepper motor mode)

/**
 * @brief Set Stepper frequency.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @param value Stepper frequency value.
 */
void ramstixSetFrequencyValue(int fd, unsigned int component_offset, double value);

/**
 * @brief Get stepper frequency.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @return Returns the current stepper frequency scalar.
 */
int ramstixGetFrequencyValue(int fd, unsigned int component_offset);

/**
 * @brief Initialise PWM in stepper mode.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @param brake Enable braking mode.
 */
void ramstixInitFrequencyOut(int fd, unsigned int component_offset, enum BRAKING brake);

/**
 * @brief Close PWM from stepper mode.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 */
void ramstixCloseFrequencyOut(int fd, unsigned int component_offset);


/**
 * @brief Init PWM block for servo motors.
 * 
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 */
void ramstixInitServo(int fd, unsigned int component_offset);

/**
 * @brief Set duty cycle for servo motors.
 * 
 * Duty cycle varies from 0 to 100% and is given as a value between 0 and 1;
 * The maximum and minimum pulsewidths for the resulting servo signal are
 * limited to 2100 and 600 us (3% and 10.5%)
 * 
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset PWM offset. (0 for PWM1, 1 for PWM2 etc.)
 * @param duty Duty cycle for the servo control. Range should be between 0.03 and 0.105.
 * 
 * @see pwm
 */
void ramstixSetServoPulseWidth(int fd, unsigned int component_offset, double duty);

#endif // _RAMSTIX_PWMOUT_H_

/**
 * @}
 */
