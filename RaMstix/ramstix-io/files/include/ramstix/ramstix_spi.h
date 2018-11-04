/**
 * @file ramstix_spi.h
 * @brief Generic SPI drivers that can be used to create custom drivers.
 * @author Jan Jaap Kempenaar, University of Twente
 *
 * @addtogroup spi
 * @{
 *
 * Generic SPI interface to add custom SPI based sensors to the RaMstix. Configuration can be derived from the SPI timing diagram ([Source](http://commons.wikimedia.org/wiki/File:SPI_timing_diagram2.svg#mediaviewer/File:SPI_timing_diagram2.svg)).
 * \anchor spi_timing_image
 * \image html spi.svg <a href="http://commons.wikimedia.org/wiki/File:SPI_timing_diagram2.svg#mediaviewer/File:SPI_timing_diagram2.svg">SPI timing diagram</a>
 * \image latex spi.pdf "SPI timing diagram"
 *
 *
 * Requires the @ref c_gpmc_fpga_userdriver "GPMC userspace driver".
 *
 * General usage example:
 * @code
// System files
#include <stdio.h>

// Driver files.
#include "ramstix_spi.h"

int main(int argc, char* argv[])
{
    // Open real-time device.
    int fd = openGPMCFPGA();
    // Validate.
    if (0 < fd) {
        printf("error");
        return 1;
    }
    // Configure SPI: (See also image as reference)
    // Number of data bits: 10
    // Clock phase mode: 0
    // Clock polarity mode: 0
    // Send least significant bit first.
    // Expect to receive least significant bit first.
    // Clock frequency: 100 kHz.
    ramstixOpenSPI(fd, 10, 0, 0, LSB_FIRST, LSB_FIRST, 100000);

    // Send data value 20:
    ramstixSendSPI(fd, 20);
    // Wait for data:
    int data = ramstixReceiveSPI(fd);
    printf("Received data: %i\n", data);
    // Close SPI interface.
    ramstixCloseSPI(fd);
    close(fd);
    return 0;
}
 * @endcode
 *
 */

#ifndef _RAMSTIX_SPI_H_
#define _RAMSTIX_SPI_H_

#include "ramstix/ramstix_definitions.h"
#include "gpmc_fpga_user/gpmc_fpga_user.h"

/**
 * @brief Bit orientation
 *
 * Depending on the order which bits are received, you may be required to shift bytes appropriately.
 */
enum BIT_ORIENTATION
{
    LSB_FIRST = 0,  ///< Received/send least significant bit first.
    MSB_FIRST = 1   ///< Received/send most significant bit first.
};

/**
 * @brief SPI mode
 */
enum SPI_MODE
{
	MODE0 = 0,       ///< Clock polarity 0, Clock edge 1
	MODE1 = 1,       ///< Clock polarity 0, Clock edge 0
	MODE2 = 2,       ///< Clock polarity 1, Clock edge 0
	MODE3 = 3        ///< Clock polarity 1, Clock edge 1
};
/**
 * @brief Open and configure SPI bus.
 *
 * Configure and enable the SPI bus. You can configure (see also \ref spi_timing_image "SPI timing diagram"):
 *  - Number of data bits.
 *  - Clock phase mode.
 *  - Clock polarity mode.
 *  - Input and Output bit orientation.
 *  - Clock frequency of the SPI.
 *
 *
 * @param fd GPMC memory controller file descriptor.
 * @param bitcount Defines the amount of data bits that the SPI will transfer. Must be at least 1(bit) any value greater than 32, is regarded as 32.
 * @param mode SPI mode for clock polarity and clock phase.
 * @param mosi Bit orientation for the outgoing data.
 * @param miso Bit orientation for the incomming data.
 * @param frequency Clock frequency of the SPI given in Hz.
 */
void ramstixOpenSPI(int fd, int bitcount, enum SPI_MODE mode, enum BIT_ORIENTATION mosi, enum BIT_ORIENTATION miso, int frequency);

/**
 * @brief Close spi bus.
 *
 * @param fd GPMC memory controller file descriptor.
 */
void ramstixCloseSPI(int fd);

/**
 * @brief Send data to SPI bus.
 *
 * When calling this function while the SPI is sending, the current action is interrupted and the SPI will start with transmission of the
 * data in this function call. Use the ramstixStatusSPI function to check current status of the SPI bus.
 *
 * The function does not validate if the data value fits within the given amount of bits. Bits out of bounds of the databit count are ignored.
 *
 * For example: when the SPI is configured to send 4 bits msb first, sending 0xA000000B as data will result in 0xB being sent msb first.
 * 
 * @see ramstixStatusSPI
 * @param fd GPMC memory controller file descriptor.
 * @param data Data value send to the SPI bus.
 */
void ramstixSendSPI(int fd, unsigned int data);

/**
 * @brief Get SPI transmission status.
 *
 * @param fd GPMC memory controller file descriptor.
 * @return Returns 1 if transmission on the SPI bus is ready.
 */
inline int ramstixStatusSPI(int fd);

/**
 * @brief Receive data from SPI bus.
 *
 * Receive data from the SPI bus. Depending on the configuration of the SPI bus, it may be required to change the byte order of the received data.
 *
 * @param fd GPMC memory controller file descriptor.
 * @return Returns the received data.
 */
unsigned int ramstixReceiveSPI(int fd);

#endif // _RAMSTIX_SPI_H_

/**
 * @}
 */
