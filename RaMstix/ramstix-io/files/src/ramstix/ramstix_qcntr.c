/**
 * @file ramstix_qcntr.c
 * @brief
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include "ramstix/ramstix_qcntr.h"

void ramstixSetQCounterValue(int fd, unsigned int component_offset, int value)
{
// Our function does not set a value, notify end-user.
#if defined _MSC_VER
//#pramga message ("warning: ramstixSetQCounterValue only resets the counter value to zero.")
#elif defined __GNUC__
//#warning 'ramstixSetQCounterValue' only resets the counter value to zero.
#endif

    // Calculate register offset:
    unsigned int ctrl_reg, offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_CONTROL_REG;
    RAMSTIX_PRINT("Encoder ctrl offset: %i, 0x%x\n", offset, offset);
    ctrl_reg = readGPMCFPGA16(fd, offset) | ENCODER_RESET;
    RAMSTIX_PRINT("Encoder ctrl VALUE: %i, 0x%x\n", ctrl_reg, ctrl_reg);
    writeGPMCFPGA16(fd, offset, ctrl_reg);
}

int ramstixGetQCounterValue(int fd, unsigned int component_offset)
{
    unsigned int offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_VALUE_REG;
/*    RAMSTIX_PRINT("Encoder value offset: %i, 0x%x\n", offset, offset);*/
    return readGPMCFPGA32(fd, offset);
}

void ramstixInitQCounter(int fd, unsigned int component_offset, enum ENCODER_INDEX_TYPE index, enum ENCODER_COUNTER_TYPE count_type)
{
    //@todo add register values.
    unsigned int ctrl_reg = 0x00, offset;
    // enable filter and counter
    ctrl_reg |= (ENCODER_ENABLE | ENCODER_INPUT_FILTER);
    switch (index)
    {
        case NEGEDGE_RESET:
            // clear on index and index pol
            ctrl_reg |= (ENCODER_CLEAR_ON_INDEX | ENCODER_INDEX_POLARITY);
            break;

        case POSEDGE_RESET:
            // clear on index.
            ctrl_reg |= ENCODER_CLEAR_ON_INDEX;
            break;
        default:
            // do nothing.
            break;
    }
    if (UP_DOWN_COUNTER == count_type)
    {
        ctrl_reg |= ENCODER_COUNTER_MODE;
    }
    offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_CONTROL_REG;
    RAMSTIX_PRINT("Encoder ctrl offset: %i, 0x%x\nCtrl: 0x%x\n", offset, offset, ctrl_reg);
    writeGPMCFPGA16(fd, offset, ctrl_reg);
}
void ramstixCloseQCounter(int fd, unsigned int component_offset)
{
    unsigned int offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_CONTROL_REG;
    RAMSTIX_PRINT("Encoder ctrl offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA16(fd, offset, 0);
}
int ramstixGetLatchedIndex(int fd, unsigned int component_offset)
{
    unsigned int offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_READ_REG;
    RAMSTIX_PRINT("Encoder read offset: %i, 0x%x\n", offset, offset);
    return ((ENCODER_LATCH_INDEX & readGPMCFPGA16(fd, offset)) > 0) ? 1 : 0;
}
int ramstixGetChannelA(int fd, unsigned int component_offset)
{
    unsigned int offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_READ_REG;
    RAMSTIX_PRINT("Encoder read offset: %i, 0x%x\n", offset, offset);
    return ((ENCODER_CHANNEL_A & readGPMCFPGA16(fd, offset)) > 0) ? 1 : 0;
}
int ramstixGetChannelB(int fd, unsigned int component_offset)
{
    unsigned int offset = ENCODER_BASE + component_offset * ENCODER_REG_SIZE + ENCODER_READ_REG;
    RAMSTIX_PRINT("Encoder read offset: %i, 0x%x\n", offset, offset);
    return ((ENCODER_CHANNEL_B & readGPMCFPGA16(fd, offset)) > 0) ? 1 : 0;
}
int ramstixGetIndex(int fd, unsigned int component_offset)
{
    return 0;
}
