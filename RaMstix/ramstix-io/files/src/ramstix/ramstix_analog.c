/**
 * @file ramstix_analog.c
 * @brief
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include "ramstix/ramstix_analog.h"
#include "ramstix/ramstix_definitions.h"
#include <limits.h>


void ramstixInitADC(int fd, unsigned int component_offset)
{
    unsigned int offset = ADC_BASE + component_offset * ADC_REG_SIZE + ADC_CONTROL_REG;
    RAMSTIX_PRINT("ADC ctrl offset: %i\n", offset);
    writeGPMCFPGA16(fd, offset, ADC_ENABLE); // Enable.
}

double ramstixGetADCValue(int fd, unsigned int component_offset)
{
    int16_t ret; double voltage;
    
    ret = ramstixGetADCIntValue(fd, component_offset);
    // ret is a signed 16-bit integer            32768
/*    voltage = ( ((double)ret * ADC_MAX_VALUE) / SHRT_MAX );*/
    voltage = ( (ADC_MAX_VALUE / SHRT_MAX) * (double)ret);
    
    // Based on min and max values, calculate the measured voltages.
    // Assumes that the voltage measurement is based on a linear scale.
    RAMSTIX_PRINT("ADC value reg: %i, volts: %f\n", ret, voltage);
    return voltage;
}

int16_t ramstixGetADCIntValue(int fd, unsigned int component_offset)
{
	unsigned int offset = ADC_BASE + component_offset * ADC_REG_SIZE + ADC_VALUE_REG;
	RAMSTIX_PRINT("ADC value offset: %i\n", offset);
    uint16_t adc_2scompl = readGPMCFPGA16(fd, offset);
    RAMSTIX_PRINT("%d\n", adc_2scompl);
    // The ADC is 2's complement: if sign bit is clear, then use the number as-is (positive)
    //  -- if the sign bit (MSB) is set, convert to negative number by subtracting 1 and inverting
    if( adc_2scompl & (0x8000) ){
        return (int16_t) -(~(adc_2scompl)+1);
    } else {
        // No sign bit: positive number
        return (int16_t) adc_2scompl;
    }
}

void ramstixCloseADC(int fd, unsigned int component_offset)
{
    unsigned int offset = ADC_BASE + component_offset * ADC_REG_SIZE + ADC_CONTROL_REG;
    RAMSTIX_PRINT("ADC ctrl offset: %i\n", offset);
    writeGPMCFPGA16(fd, offset, 0); // Disable.
}


void ramstixInitDAC(int fd, unsigned int component_offset)
{
	unsigned int offset = DAC_BASE + component_offset * DAC_REG_SIZE + DAC_CONTROL_REG;
	RAMSTIX_PRINT("DAC ctrl offset: %i\n", offset);
	writeGPMCFPGA16(fd, offset, DAC_ENABLE);
}

void ramstixSetDACValue(int fd, unsigned int component_offset, double volt)
{
	unsigned int offset = DAC_BASE + component_offset * DAC_REG_SIZE + DAC_VALUE_REG;
	unsigned int value = (int) (((volt - DAC_MIN_VALUE)/(DAC_MAX_VALUE - DAC_MIN_VALUE))*USHRT_MAX);
	RAMSTIX_PRINT("DAC value offset: %i\n", offset);
	RAMSTIX_PRINT("DAC intval: %i\n", value);
	writeGPMCFPGA16(fd, offset, value);
}

void ramstixSetDACIntValue(int fd, unsigned int component_offset, int value)
{
	unsigned int offset = DAC_BASE + component_offset * DAC_REG_SIZE + DAC_VALUE_REG;
	RAMSTIX_PRINT("DAC value offset: %i\n", offset);
	RAMSTIX_PRINT("DAC intval: %i\n", value);
	writeGPMCFPGA16(fd, offset, value);
}

void ramstixCloseDAC(int fd, unsigned int component_offset)
{
	unsigned int offset = DAC_BASE + component_offset * DAC_REG_SIZE + DAC_CONTROL_REG;
	RAMSTIX_PRINT("DAC ctrl offset: %i\n", offset);
	writeGPMCFPGA16(fd, offset, 0);
}
