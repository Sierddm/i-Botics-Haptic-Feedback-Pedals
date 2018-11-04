/**
 * @file ramstix_timemeas.h
 * @brief Time measurement utilities.
 * @author Jan Jaap Kempenaar, University of Twente
 *
 * @addtogroup measurement
 * @{
 *
 * Measurement module to measure digital signals. The module is capable of measuring the frequency, duty cycle and edge count.
 * The actual calculation is performed in software.
 *
 * Requires the @ref c_gpmc_fpga_userdriver "GPMC userspace driver".
 *
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.
#include "ramstix_timemeas.h"

int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    // Configure a duty cycle measurement on Digital input 1:
    // The measurement will trigger on the rising edge.
    ramstixInitTimeMeasurement(fd, 0, RISING, DUTY_CYCLE_EDGE_RESET, FILTER_ENABLED, WAIT_FOR_EDGE);
    // NOTE: give some time to actually perform an initial measurement here.
    double cycle = ramstixGetDutyCycle(fd, 0);
    printf("Duty cycle is %f \%.\n", cycle * 100);
    ramstixCloseTimeMeasurement(fd, 0);
    close(fd);
    return 0;
}
 * @endcode
 */

#ifndef _RAMSTIX_TIMEMEAS_H_
#define _RAMSTIX_TIMEMEAS_H_

// enable debug.
//#define RAMSTIX_DEBUG

#include "ramstix/ramstix_definitions.h"
#include "gpmc_fpga_user/gpmc_fpga_user.h"

/**
 * @brief Edge trigger type
 */
enum EDGE_TYPE
{
    RISING=1,   ///< Trigger on rising edge.
    FALLING=0,  ///< Trigger on falling edge.
};

/**
 * @brief Measurement type.
 */
enum MEASUREMENT_TYPE
{
    FREQUENCY=0,                ///< Configure for frequency measurement.
    EDGE_COUNTING=1,            ///< Configure for edge counting measurement.
    DUTY_CYCLE_WINDOWED=2,      ///< Configure for windowed duty cycle measurement. @see ramstixInitTimeMeasurement
    DUTY_CYCLE_EDGE_RESET=3     ///< Configure for duty cycle measurement.
};

/**
 * @brief Enable filter.
 */
enum FILTERING
{
    FILTER_ENABLED=1,   ///< Enable filter on measurement.
    FILTER_DISABLED=0   ///< Disable filter on measurement.
};

/**
 * @brief Measurement start type. (unused, measurement starts immediatly.)
 */
enum START_TYPE
{
    WAIT_FOR_EDGE=1,    ///< Wait for edge to start measurement.
    IMMEDIATELY=0       ///< Start measurement immidiatly.
};

/**
 * @brief Initialise time measurement component.
 *
 * Initialises the measurement component for a particular measurement. The hardware components only contain counters and do not perform the actual result calculation, this is resolved in software.
 *
 * <strong>Note:</strong> When configuring the Windowed duty cycle measurement, this should only be used if measuring a very large duty cycle of several seconds. The counters can hold about 85 seconds of measurement data.
 * In this mode, when a counter overflow occurs, the value of the counters is written to the registers. In software these values should be stored and checked. For more information on obtaining status and counter data, see the "see also".
 * @see ramstixGetLowCount
 * @see ramstixGetTotalCount
 * @see ramstixGetEnded
 * @see ramstixGetOverflow
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @param edge Define on which edge type to trigger.
 * @param meas Define the measurement type.
 * @param filter Enable or disable measurement filter.
 * @param start_situation For now, this is unused and measurement is started immediatly.
 */
void ramstixInitTimeMeasurement(int fd, unsigned int component_offset, enum EDGE_TYPE edge, enum MEASUREMENT_TYPE meas, enum FILTERING filter, enum START_TYPE start_situation);

/**
 * @brief Disable time measurement component.
 *
 * Disables the measurement component by setting the control register to 0.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 */
void ramstixCloseTimeMeasurement(int fd, unsigned int component_offset);

/**
 * @brief Get duty cycle.
 *
 * Get the measured duty cycle. Do note that counters get updated only if an edge is detected.
 *
 * <strong>Note:</strong> Do not use this function to obtain the duty cycle when using windowed mode.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @return Returns the duty cyle. The range returned is between 0 and 1, where 1 equals 100% duty cycle.
 */
inline double ramstixGetDutyCycle(int fd, unsigned int component_offset);

/**
 * @brief Get measured frequency.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @return Returns the measure frequency.
 */
inline double ramstixGetMeasuredFrequency(int fd, unsigned int component_offset);

/**
 * @brief Get low count register value.
 *
 * Get the low count register value. The low count register contains the amount of clock pulses, that are counted while the input signal is low.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @return Returns the number of clock pulses while input signal is low.
 */
inline unsigned int ramstixGetLowCount(int fd, unsigned int component_offset);

/**
 * @brief Get total count register value.
 *
 * Get the total count register value. The total count register value contains the number of counts until the next edge is detected on which the measurement component is configured to trigger.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @return Returns the number of clock pulses until next edge trigger.
 */
inline unsigned int ramstixGetTotalCount(int fd, unsigned int component_offset);

/**
 * @brief Not implemented.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @param value Maximum value.
 */
inline void ramstixSetMaxValue(int fd, unsigned int component_offset, unsigned int value);

/**
 * @brief Not implemented.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measerement device number (eg. 1, 2, etc.).
 * @param minfreq Minimum frequency.
 */
void ramstixSetMinFrequency(int fd, unsigned int component_offset, double minfreq);

/**
 * @brief Get overflow bit status.
 *
 * Get overflow bit status. This can be used to check the current status of the register when performing a windowed duty cycle measurement.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 * @return Returns 1 if overflow occured, 0 when no overflow occured.
 */
inline int ramstixGetOverflow(int fd, unsigned int component_offset);

/**
 * @brief Get ended bit status.
 *
 * Get ended bit status. This bit can be used to determine if a valid measurement was performed by the measurement component.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measerement device number (eg. 1, 2, etc.).
 * @return Returns 1 if a measurement ended, 0 if no measurement ended yet.
 */
inline int ramstixGetEnded(int fd, unsigned int component_offset);

/**
 * @brief Not implemented.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Measurement device offset. (0 for device 1, 1 for device 2, etc.)
 */
void ramstixClearEnded(int fd, unsigned int component_offset);

#endif // _RAMSTIX_TIMEMEAS_H_

/**
 * @}
 */
