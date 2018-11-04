/**
 * @file ramstix_spi.c
 * @brief SPI implementation
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include "ramstix/ramstix_spi.h"

void ramstixOpenSPI(int fd, int bitcount, enum SPI_MODE mode, enum BIT_ORIENTATION mosi, enum BIT_ORIENTATION miso, int frequency)
{
    int ctrl_reg = SPI_ENABLE, clk_scale;
    // add the bitcount, it needs shift left and a mask:
    if (0 >= bitcount)
    {
        RAMSTIX_PRINT("Cannot send 0 or negative bits, should be at least 1.\n");
        return;
    }
    else if (32 <= bitcount)
    {
    	bitcount = 0; // Set to 0, this will result in 32 bits.
    }
    ctrl_reg |= ((bitcount << 1) & SPI_BIT_COUNT);
    // Set clock phase bit, if clock_phase is >0
	switch(mode)
	{
		case MODE0:
			ctrl_reg |= SPI_CLOCK_PHASE;
			break;
			
		case MODE1:
			// all zero.
			break;
			
		case MODE2:
			ctrl_reg |= SPI_CLOCK_POLARITY;
			break;
			
		case MODE3:
			ctrl_reg |= (SPI_CLOCK_PHASE | SPI_CLOCK_POLARITY);
			break;
			
		default:
			// nothing
			break;
	}
	
    // If MOSI orientation is msb first.
    if (mosi)
    {
        ctrl_reg |= SPI_MOSI_ORIENTATION;
    }
    // If MISO orientation if msb first.
    if (miso)
    {
        ctrl_reg |= SPI_MISO_ORIENTATION;
    }
    // calculate and write the clock frequency first.
    // prescale is (F_CLK/freq)/2 - 1
    clk_scale = ((CLOCK_FREQUENCY/frequency) >> 1) - 1;
    RAMSTIX_PRINT("SPI clock scalar: %i\n", clk_scale);
    writeGPMCFPGA16(fd, SPI_BASE + SPI_CLK_PRESCALE_REG, clk_scale);
    RAMSTIX_PRINT("SPI ctrl reg: %x\n", ctrl_reg);
    writeGPMCFPGA16(fd, SPI_BASE + SPI_CONTROL_REG, ctrl_reg);
}

void ramstixCloseSPI(int fd)
{
    RAMSTIX_PRINT("SPI set clk scalar and ctrl reg to 0\n");
    writeGPMCFPGA16(fd, SPI_BASE + SPI_CONTROL_REG, 0);
    writeGPMCFPGA16(fd, SPI_BASE + SPI_CLK_PRESCALE_REG, 0);
}

void ramstixSendSPI(int fd, unsigned int data)
{
    RAMSTIX_PRINT("SPI write data: 0x%X\n", data);
    writeGPMCFPGA32(fd, SPI_BASE + SPI_MOSI_REG, data);
}

inline int ramstixStatusSPI(int fd)
{
    return ((readGPMCFPGA16(fd, SPI_BASE + SPI_STATUS_REG) & SPI_READY_FLAG) > 0) ? 1 : 0;
}

unsigned int ramstixReceiveSPI(int fd)
{
    RAMSTIX_PRINT("SPI block until ready...");
    while (!ramstixStatusSPI(fd)); // Wait until transmission is ready.
    unsigned int data = readGPMCFPGA32(fd, SPI_BASE + SPI_MISO_REG);
    RAMSTIX_PRINT(" received 0x%X\n", data);
    return data;
}
