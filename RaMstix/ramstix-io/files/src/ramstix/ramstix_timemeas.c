/**
 * @file ramstix_timemeas.c
 * @brief
 * @author Jan Jaap Kempenaar, University of Twente
 */


#include "ramstix/ramstix_timemeas.h"


void ramstixInitTimeMeasurement(int fd, unsigned int component_offset, enum EDGE_TYPE edge, enum MEASUREMENT_TYPE meas, enum FILTERING filter, enum START_TYPE start_situation)
{
    // Turn on enable bit and clear bit for counters.
    unsigned int offset, ctrl_reg = (MEASURE_ENABLE | MEASURE_COUNT_CLEAR);

    switch(meas)
    {
        case FREQUENCY:
            // Mode = 0 and Edge triggered.
            ctrl_reg |= MEASURE_EDGE_TRIGGERED;
            break;

        case EDGE_COUNTING:
            // Mode = 1 and edge triggered.
            ctrl_reg |= (MEASURE_MODE | MEASURE_EDGE_TRIGGERED);
            break;

        case DUTY_CYCLE_EDGE_RESET:
            // Mode = 0 and edge triggered, write result on overflow.
            ctrl_reg |= (MEASURE_EDGE_TRIGGERED | MEASURE_SAVE_ON_OVERFLOW);
            break;

        case DUTY_CYCLE_WINDOWED:
        default:
            // mode = 0, not edge triggered save value on overflow.
            ctrl_reg |= MEASURE_SAVE_ON_OVERFLOW;
            break;
    }

    if (RISING == edge)
    {
        ctrl_reg |= MEASURE_EDGE; // Set 1 for rising edge.
    }

    if (FILTER_DISABLED == filter)
    {
        ctrl_reg |= MEASURE_FILTER_DISABLE; // Disable filter.
    }

    if (WAIT_FOR_EDGE == start_situation)
    {
        // Not implemented.
    }

    RAMSTIX_PRINT("Measure ctrl_reg = 0x%x\n", ctrl_reg);
    offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_CONTROL_REG;
    RAMSTIX_PRINT("Measure ctrl offset: %i\n", offset);
    writeGPMCFPGA16(fd, offset, ctrl_reg);
}

inline double ramstixGetDutyCycle(int fd, unsigned int component_offset)
{
    double low_count, total_count;
    int data[2];
    int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_LOW_COUNT_REG;
    RAMSTIX_PRINT("Measure cntr offset: %i\n", offset);
    readGPMCFPGA64(fd, offset, data);
    if (0 == data[1]) // if low count is equal to zero.
    {
        RAMSTIX_PRINT("Measure low count = 0\n");
        return 0;
    }
    else
    {
        low_count = (double) data[1];
        total_count = (double) data[0];
        RAMSTIX_PRINT("Measure low_count %f, total_count %f\n", low_count, total_count);
        return ((total_count - low_count) / total_count);
    }
}

inline double ramstixGetMeasuredFrequency(int fd, unsigned int component_offset)
{
    if ((0 == ramstixGetOverflow(fd, component_offset)) && (1 == ramstixGetEnded(fd, component_offset)))
    {
        // valid measurement.
        int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_TOTAL_COUNT_REG;
        RAMSTIX_PRINT("Measure total count offset: %i\n", offset);
        int total_count = readGPMCFPGA32(fd, offset);
        RAMSTIX_PRINT("Measure total_count %i, %x\n", total_count, total_count);
        if (total_count == 0)
        {
            return 0;
        }
        else
        {
            return ((double) CLOCK_FREQUENCY / (double) total_count);
        }
    }
    else
    {
        // invalid measurement:
        RAMSTIX_PRINT("Measure - GetMeasuredFrequencyInvalid\n");
        return 0;
    }
}
inline unsigned int ramstixGetLowCount(int fd, unsigned int component_offset)
{
    int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_LOW_COUNT_REG;
    RAMSTIX_PRINT("Measure low count offset: %i\n", offset);
    return readGPMCFPGA32(fd, offset);
}
inline unsigned int ramstixGetTotalCount(int fd, unsigned int component_offset)
{
    int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_TOTAL_COUNT_REG;
    RAMSTIX_PRINT("Measure total count offset: %i\n", offset);
    return readGPMCFPGA32(fd, offset);
}
inline void ramstixSetMaxValue(int fd, unsigned int component_offset, unsigned int value)
{
    // not implemented.
    // which max value?
}
void ramstixSetMinFrequency(int fd, unsigned int component_offset, double minfreq)
{
    // not implemented.
    // which minimum value
}

inline int ramstixGetOverflow(int fd, unsigned int component_offset)
{
    int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_FLAGS_REG;
    RAMSTIX_PRINT("Measure flags offset: %i\n", offset);
    return (readGPMCFPGA16(fd, offset) & MEASURE_OVERFLOW_FLAG) ? 1 : 0;
}
inline int ramstixGetEnded(int fd, unsigned int component_offset)
{
    int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_FLAGS_REG;
    RAMSTIX_PRINT("Measure flags offset: %i\n", offset);
    return (readGPMCFPGA16(fd, offset) & MEASURE_FINISHED_FLAG) ? 1 : 0;
}
void ramstixClearEnded(int fd, unsigned int component_offset)
{
    // not implemented
    // Cannot write to this flag.
}
void ramstixCloseTimeMeasurement(int fd, unsigned int component_offset)
{
    int offset = MEASURE_BASE + component_offset * MEASURE_REG_SIZE + MEASURE_CONTROL_REG;
    RAMSTIX_PRINT("Measure ctrl offset %i\n", offset);
    writeGPMCFPGA16(fd, offset, 0);
}
