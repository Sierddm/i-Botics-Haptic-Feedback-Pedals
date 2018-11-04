/**
 * @file ramstix_qcntr.h
 * @brief Quadrature encoder and counter functions.
 * @author Jan Jaap Kempenaar, University of Twente
 *
 * @addtogroup encoders
 * @{
 * 
 * Functions to obtain information from the encoders on the RaMstix FPGA. Requires the @ref c_gpmc_fpga_userdriver "GPMC userspace driver".
 *
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.
#include "ramstix_qcntr.h"

int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    // Configure encoder 1 as Quadrature encoder with reset on positive index.
    ramstixInitQCounter(fd, 1, POSEDGE_RESET, QUADRATURE);
    // Reset encoder to zero.
    ramstixSetQCounterValue(fd, 1, 0);
    // Read values from the encoder.
    int counter = ramstixGetQCounterValue(fd, 1);
    // result:
    printf("Counter value is: %i\n", counter);
    // close encoder counter and rt_device.
    ramstixCloseQCounter(fd, 1);
    close(fd);
    return 0;
}
 * @endcode
 * 
 */



#ifndef _RAMSTIX_QCNTR_H_
#define _RAMSTIX_QCNTR_H_

// enable debug
// #define RAMSTIX_DEBUG

#include "ramstix/ramstix_definitions.h"
#include "gpmc_fpga_user/gpmc_fpga_user.h"

/**
 * @brief Encoder counter type.
 */
enum ENCODER_COUNTER_TYPE
{
    UP_DOWN_COUNTER = 0,    ///< Configure as up/down counter.
    QUADRATURE      = 1     ///< Configure as quadrature encoder.
};

/**
 * @brief Define how encoder responds to the index input EN#I.
 */
enum ENCODER_INDEX_TYPE
{
    NO_RESET        = -1,   ///< Do not reset on index input.
    NEGEDGE_RESET   = 1,    ///< Reset on falling edge on index input.
    POSEDGE_RESET   = 0     ///< Reset on rising edge on index input.
};

/**
 * @brief Initialise encoder counter.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 * @param index Encoder index input reset type.
 * @param count_type Counter type for encoder (Up/down, quadrature.)
 */
void ramstixInitQCounter(int fd, unsigned int component_offset, enum ENCODER_INDEX_TYPE index, enum ENCODER_COUNTER_TYPE count_type);

/**
 * @brief Close encoder counter.
 *
 * This function disables the encoder by setting the control register to 0.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 */
void ramstixCloseQCounter(int fd, unsigned int component_offset);

/**
 * @brief Reset encoder counter value.
 *
 * Use this function to reset the encoder counter to 0. The "value" is ignored as it is not yet possible to write values to the count register of the encoder.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 * @param value Counte value.
 */
void ramstixSetQCounterValue(int fd, unsigned int component_offset, int value);

/**
 * @brief Get encoder counter value.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 * @return Returns the current encoder counter value.
 */
int ramstixGetQCounterValue(int fd, unsigned int component_offset);

/**
 * @brief Get latch index.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 * @return Returns 1 if latch index is high, else 0 is returned.
 */
int ramstixGetLatchedIndex(int fd, unsigned int component_offset);

/**
 * @brief Get channel A status.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 * @return Returns 1 if channel A is high, else 0 is returned.
 */
int ramstixGetChannelA(int fd, unsigned int component_offset);

/**
 * @brief Get channel B status.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 * @return Returns 1 if channel B is high, else 0 is returned.
 */
int ramstixGetChannelB(int fd, unsigned int component_offset);

/**
 * @brief Unimplemented.
 *
 * @param fd GPMC memory controller file descriptor.
 * @param component_offset Encoder offset. (0 for Enc1, 1 for Enc2, etc.)
 */
int ramstixGetIndex(int fd, unsigned int component_offset);

#endif // _RAMSTIX_QCNTR_H_

/**
 * @}
 */
