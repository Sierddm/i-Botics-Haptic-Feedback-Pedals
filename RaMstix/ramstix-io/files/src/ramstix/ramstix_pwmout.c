/**
 * @file ramstix_pwmout.c
 * @brief
 * @author Jan Jaap Kempenaar, University of Twente
 */

#include "ramstix/ramstix_pwmout.h"

/* PWM generator */
void ramstixInitPWMOut(int fd, unsigned int component_offset, enum BRIDGE_TYPE bridge, enum BRAKING brake, enum INTERLACED interl)
{
    unsigned int offset, ctrl_reg = PWM_ENABLE;

    // Select PWM H-bridge mode.
    switch (bridge)
    {
        case THIEMO:
            ctrl_reg |= PWM_CONFIG_THIEMO;
            break;
        case VNH3SP30:
            ctrl_reg |= PWM_CONFIG_VNH3SP30;
            break;
        case NONE:
            ctrl_reg |= PWM_CONFIG_NONE;
            break;
        case BERT:
        default:
            ctrl_reg |= PWM_CONFIG_BERT; // Mode 00
            break;
    }

    // Enable braking
    if (BRAKING == brake)
    {
        ctrl_reg |= PWM_BRAKE_ENABLE;
    }

    // enable interlaced
    if (INTERLACED == interl)
    {
        ctrl_reg |= PWM_INTERLACE_ENABLE;
    }

    // Set control register.
    offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_CONTROL_REG;
    RAMSTIX_PRINT("PWM ctrl offset: %i, 0x%x\nControl: %x\n", offset, offset, ctrl_reg);
    writeGPMCFPGA16(fd, offset, ctrl_reg);

    // Set duty cycle to zero.
    offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_DUTY_REG;
    RAMSTIX_PRINT("PWM ctrl offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA16(fd, offset, 0);
}
void ramstixClosePWMOut(int fd, unsigned int component_offset)
{
    // Set duty cycle value to zero.
    ramstixSetPWMValue(fd, component_offset, 0);
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_CONTROL_REG;
    RAMSTIX_PRINT("PWM ctrl offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA16(fd, offset, 0);

}
void ramstixSetPWMValue(int fd, unsigned int component_offset, double value)
{
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_DUTY_REG;
    RAMSTIX_PRINT("PWM value offset: %i, 0x%x\nPWM value: %i\n", offset, offset, (int) (value * 32767));
    writeGPMCFPGA16(fd, offset, (int) (value * 32767));
}
int ramstixGetPWMValue(int fd, unsigned int component_offset)
{
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_DUTY_REG;
    RAMSTIX_PRINT("PWM value offset: %i, 0x%x\n", offset, offset);
    return readGPMCFPGA16(fd, offset);
}
void ramstixSetPWMFrequency(int fd, unsigned int component_offset, double value)
{
    double prescaler = (2048 * 65535 * value) / (double) CLOCK_FREQUENCY;
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_DUTY_FREQ_REG;
    RAMSTIX_PRINT("PWM freq offset: %i, 0x%x\nPWM scalar: %i\n", offset, offset, (int) prescaler);
    writeGPMCFPGA16(fd, offset, (int) prescaler);
}
unsigned int ramstixGetPWMPrescaler(int fd, unsigned int component_offset)
{
    int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_DUTY_FREQ_REG;
    RAMSTIX_PRINT("PWM freq offset: %i, 0x%x\n", offset, offset);
    return readGPMCFPGA16(fd, offset);
}

/* Frequency generator (Stepper motor mode) */

void ramstixInitFrequencyOut(int fd, unsigned int component_offset, enum BRAKING brake)
{
    unsigned int offset, ctrl_reg = (PWM_ENABLE | PWM_STEPPER_ENABLE);

    if (BRAKING == brake)
    {
        ctrl_reg |= PWM_BRAKE_ENABLE;
    }
    offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_CONTROL_REG;
    RAMSTIX_PRINT("PWM ctrl offset: %i, 0x%x\nControl: %x\n", offset, offset, ctrl_reg);
    writeGPMCFPGA16(fd, offset, ctrl_reg);

    offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_STEPPER_FREQ_REG;
    RAMSTIX_PRINT("PWM Stepper freq offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA16(fd, offset, 0);

}
void ramstixCloseFrequencyOut(int fd, unsigned int component_offset)
{
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_CONTROL_REG;
    RAMSTIX_PRINT("PWM ctrl offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA16(fd, offset, 0);

    offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_STEPPER_FREQ_REG;
    RAMSTIX_PRINT("PWM Stepper freq offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA16(fd, offset, 0);
}
void ramstixSetFrequencyValue(int fd, unsigned int component_offset, double value)
{
    long intval = (long) value * 20;
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_STEPPER_FREQ_REG;
    RAMSTIX_PRINT("PWM Stepper freq offset: %i, 0x%x\n", offset, offset);
    writeGPMCFPGA32(fd, offset, (unsigned int) intval);
}
int ramstixGetFrequencyValue(int fd, unsigned int component_offset)
{
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_STEPPER_FREQ_REG;
    RAMSTIX_PRINT("PWM Stepper freq offset: %i, 0x%x\n", offset, offset);
    return readGPMCFPGA32(fd, offset);
}

void ramstixInitServo(int fd, unsigned int component_offset)
{
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_SERVO_DUTY_REG;
    unsigned int ctrl_reg = (PWM_ENABLE | PWM_SERVO_ENABLE);
    // Set duty cycle to 0.
    RAMSTIX_PRINT("PWM Servo duty offset offset: %i\n", offset);
    writeGPMCFPGA32(fd, offset, 0);

    // set control register value.
    offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_CONTROL_REG;
    RAMSTIX_PRINT("PWM Servo ctrl: %x, offset: %i\n", ctrl_reg, offset);
    writeGPMCFPGA16(fd, offset, ctrl_reg);
}

void ramstixSetServoPulseWidth(int fd, unsigned int component_offset, double duty)
{
    unsigned int offset = PWM_BASE + component_offset * PWM_REG_SIZE + PWM_SERVO_DUTY_REG;
    double pulsewidth = (duty * 0.02) * CLOCK_FREQUENCY;
    RAMSTIX_PRINT("PWM servo offset: %i, 0x%x\nPWM duty: %.4f, pulsewidth: %f s\n",
                  offset, offset, pulsewidth / CLOCK_FREQUENCY / 0.02, pulsewidth*CLOCK_FREQUENCY);
    if (pulsewidth > 30000.0 || pulsewidth < 105000.0)  // 600 us < pulsewidth > 2100 us
    {
        writeGPMCFPGA32(fd, offset, (int) (pulsewidth));
    }
    else
    {
        RAMSTIX_PRINT("PWM sero offset is out of bounds. allowed = [600...2100] us\n");
    }
}