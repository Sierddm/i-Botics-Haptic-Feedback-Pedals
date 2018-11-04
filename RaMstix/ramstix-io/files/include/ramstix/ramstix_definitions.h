/**
 * @file ramstix_definitions.h
 * @brief File with address and register definitions and configuration masks for the RaMstix. Base addresses are aligned for 16-bit registers.
 * @author Jan Jaap Kempenaar, University of Twente
 */

#ifndef _RAMSTIX_DEFINITIONS_H_
#define _RAMSTIX_DEFINITIONS_H_
// Define debug print statement.
#ifdef RAMSTIX_DEBUG
#include <stdio.h>
#define RAMSTIX_PRINT   printf  ///< Enabled DEBUG print.
#else
#define RAMSTIX_PRINT(...)      ///< Disabled DEBUG print.
#endif

/*
 * Definition format:
 component_BASE      = Base address for the first component.
 component_REG_SIZE  = Number of register in component. (e.g. control + value register = 2.)
 component_regname   = Index of register, compared to base
 reg_offset = RAMSTIX_component_BASE + component_num * RAMSTIX_component_REG_SIZE + RAMSTIX_component_regname
 */
//
// General definitions.
//
#define CLOCK_FREQUENCY 50000000        ///< RaMstix clock frequency
//
// Pulse width modulator definitions
//
#define PWM_BASE                0       ///< Pulse width modulator base register.
#define PWM_REG_SIZE            4       ///< Number of 16-bit registers per PWM.
#define PWM_TOTAL_NUM           4       ///< Number of present PWM modules.
#define PWM_CONTROL_REG         0       ///< PWM control register (16-bits).
#define PWM_DUTY_REG            1       ///< PWM Duty cycle register (when configured as PWM) (16-bits), least significant 4 bits are ignored, value is 11 bit, msb is direction.
#define PWM_DUTY_FREQ_REG       2       ///< PWM Frequency register (when configured as PWM). (16-bits)
#define PWM_STEPPER_FREQ_REG    1       ///< Stepper frequency register (when configure as stepper), msb is direction. (32-bits)
#define PWM_SERVO_DUTY_REG      1       ///< Servo duty cycle register. (32-bits).
// Configuration masks:
#define PWM_ENABLE              0x01    ///< PWM enable. Set to '1' to enable the PWM.
#define PWM_INTERLACE_ENABLE    0x02    ///< PWM interlace enable.
#define PWM_STEPPER_ENABLE      0x04    ///< Stepper enable, set to '1' to set PWM as stepper control.
#define PWM_BRAKE_ENABLE        0x08    ///< PWM Break enable.
#define PWM_SERVO_ENABLE        0x40    ///< PWM servo enable
#define PWM_CONFIG_MODE         0x30    ///< PWM operation mode, set to 00 for Bert van de Bert, 01 for Thiemo van Engelen, 10 for VNH3SP30, 11 for only PWM.
#define PWM_CONFIG_BERT         0x00    ///< Bert van de Berg mode
#define PWM_CONFIG_THIEMO       0x10    ///< Thiemo van Engelen mode
#define PWM_CONFIG_VNH3SP30     0x20    ///< VNH3SP30 mode
#define PWM_CONFIG_NONE         0x30    ///< PWM only mode.
//
// Encoder definitions
//
#define ENCODER_BASE	         16      ///< Encoder base register.
#define ENCODER_REG_SIZE        4       ///< Number of 16-bit registers per encoder.
#define ENCODER_TOTAL_NUM       4       ///< Total number of encoder counter modules
#define ENCODER_CONTROL_REG     0       ///< Control register. (16-bits)
#define ENCODER_READ_REG        1       ///< Read only register with latch index, channel A/B etc. (16-bits)
#define ENCODER_VALUE_REG       2       ///< Value register. (32-bits)
// Configuration masks:
#define ENCODER_ENABLE          0x01    ///< Encoder enable. Set to '1' to enable the PWM.
#define ENCODER_LATCH_INDEX     0x01    ///< Encoder latch index bit.
#define ENCODER_CHANNEL_A       0x02    ///< Encoder channel A input.
#define ENCODER_CHANNEL_B       0x04    ///< Encoder channel B input.
#define ENCODER_INDEX_POLARITY  0x02    ///< Index polarity 0 is index active low, 1 is index active high.
#define ENCODER_CLEAR_ON_INDEX  0x04    ///< Clear on index, 0 is no CoI, 1 is clear counter on index.
#define ENCODER_INPUT_FILTER    0x08    ///< Input filter, 0 is no filter, 1 is ~2 MHz input filter.
#define ENCODER_COUNTER_MODE    0x10    ///< Counter mode, 0 is quadrature, 1 is up/down.
#define ENCODER_RESET           0x20    ///< Reset pin. Set this to '1' to reset the counter.
//
// GPIO definitions
//
#define GPIO_INPUT_REG          32      ///< GPIO input register. (16-bits)
#define GPIO_INPUT_NUM          16      ///< GPIO input pin total.
#define GPIO_OUTPUT_REG         33      ///< GPIO output register. (16-bits)
#define GPIO_OUTPUT_NUM         16      ///< GPIO output pin total.
//
// MEASURE definitions
//
#define MEASURE_BASE            34      ///< Measurement base register
#define MEASURE_REG_SIZE        6       ///< Number of 16-bit registers per measure component.
#define MEASURE_TOTAL_NUM       16      ///< Number of measurement modules on the RaMstix.
#define MEASURE_CONTROL_REG     0       ///< Measure control register. (16-bit)
#define MEASURE_FLAGS_REG       1       ///< Measure flag register. (16-bit) (Read only)
#define MEASURE_LOW_COUNT_REG   2       ///< Number of clock pulses when measure signal is low. (32-bit)
#define MEASURE_TOTAL_COUNT_REG 4       ///< Total number of clock pulses. (32-bit)
// Configuration masks:
#define MEASURE_ENABLE          0x01    ///< Measure enable. Set to '1' to enable the measure component.
#define MEASURE_EDGE            0x02    ///< Set on which edge the measure triggers. Set '1' for rising edge, '0' falling.
#define MEASURE_COUNT_CLEAR     0X04    ///< Clear counter values.
#define MEASURE_MODE            0x08    ///< Measure mode: '0' is frequency/duty cycle, '1' = pulse count.
#define MEASURE_FILTER_DISABLE  0x10    ///< Disable measure filter, set to '1' to disable the filter.
#define MEASURE_EDGE_TRIGGERED  0x20    ///< Write counter value when an edge is detected.
#define MEASURE_SAVE_ON_OVERFLOW    0x40    ///< Set to '1' to save on internal counter overflow, use this for very low frequency signals.
// flags
#define MEASURE_OVERFLOW_FLAG   0x01    ///< Meausure overflow flag.
#define MEASURE_FINISHED_FLAG   0x02    ///< Measure finished flag.
//
// SPI definitions
//
#define SPI_BASE                130     ///< SPI base register.
#define SPI_REG_SIZE            7       ///< Number of 16-bit registers per SPI component.
#define SPI_CONTROL_REG         0       ///< SPI configuration register.
#define SPI_CLK_PRESCALE_REG    1       ///< SPI clock prescale register.
#define SPI_MOSI_REG            2       ///< SPI MOSI register.
#define SPI_MISO_REG            4       ///< SPI MISO register.
#define SPI_STATUS_REG          6       ///< SPI status register.
// Configuration masks:
#define SPI_ENABLE              0x01    ///< SPI enable, set to 1 to enable component.
#define SPI_BIT_COUNT           0x3e    ///< Bit count mask.
#define SPI_CLOCK_PHASE         0x40    ///< SPI clock phase
#define SPI_CLOCK_POLARITY      0x80    ///< SPI clock polarity
#define SPI_MOSI_ORIENTATION    0x100   ///< MOSI orientation, '0' sends lsb first, '1' sends msb first.
#define SPI_MISO_ORIENTATION    0x200   ///< MISO orientation, '1' is first recieved msb, '0' means received lsb.
// flags
#define SPI_READY_FLAG          0x1     ///< Transmission ready flag.
//
// ADC definitions
//
#define ADC_BASE                137     ///< AD convertor base register.
#define ADC_REG_SIZE            2       ///< Number of 16-bit registers per ADC.
#define ADC_CONTROL_REG         0       ///< ADC control register.
#define ADC_VALUE_REG           1       ///< ADC value register.
// Configuration masks:
#define ADC_ENABLE              0x01    ///< Enable, set to '1' to enable the ADC.
// ADC values
#define ADC_MAX_VALUE           5.0     ///< Value when max_uint(16) is measured.
#define ADC_MIN_VALUE           -5.0    ///< Value when min_uint(16) is measured.
//
// DAC definitions.
//
#define DAC_BASE				141		///< DA convertor base register.
#define DAC_REG_SIZE			2		///< Number of 16-bit registers per DAC.
#define DAC_CONTROL_REG			0		///< DAC control register.
#define DAC_VALUE_REG			1		///< DAC value register.
// Configuration masks:
#define DAC_ENABLE				0x01	///< Enable, set to '1' to enable the DAC.
// DAC values
#define DAC_MAX_VALUE			5.00	///< Output value when max_uint16 is set.
#define DAC_MIN_VALUE			-5.00	///< Output value when min_uint16 is set.
#endif // _RAMSTIX_DEFINITIONS_H_
