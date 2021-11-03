/**
 * @file AD5933.cpp
 * @brief Library code for AD5933
 *
 * Library code for AD5933. Referenced the datasheet and code found at
 * https://github.com/WuMRC/drive
 *
 * @author Michael Meli
 */

#include "AD5933.h"
#include <Math.h>

#include "transfer_handler.h"

/**
 * Request to read a byte from the AD5933.
 *
 * @param address Address of register requesting data from
 * @param value Pointer to a byte where the return value should be stored, or
 *        where the error code will be stored if fail.
 * @return Success or failure
 */
int AD5933getByte(uint8_t address, uint8_t *value) {
    // Request to read a byte using the address pointer register
	
    iic_send(AD5933_ADDR, &address, 1, true);
	iic_read(AD5933_ADDR, value, 1);
	
    NRF_LOG_INFO("Read %x", *value);
    
	return true;
}

/**
 * Write a byte to a register on the AD5933.
 *
 * @param address The register address to write to
 * @param value The byte to write to the address
 * @return Success or failure of transmission
 */
bool AD5933sendByte(byte address, byte value) {
    // Send byte to address
    
	uint8_t configData[2];
	configData[0] = address;
    configData[1] = value;
	
	iic_send(AD5933_ADDR, configData, 2, false);
	
	return true;
}

/**
 * Set the control mode register, CTRL_REG1. This is the register where the
 * current command needs to be written to so this is used a lot.
 *
 * @param mode The control mode to set
 * @return Success or failure
 */
bool AD5933setControlMode(byte mode) {
    // Get the current value of the control register
    byte val;
    if (!AD5933getByte(CTRL_REG1, &val))
        return false;

    // Wipe out the top 4 bits...mode bits are bits 5 through 8.
    val &= 0x0F;

    // Set the top 4 bits appropriately
    val |= mode;

    // Write back to the register
    return AD5933sendByte(CTRL_REG1, val);
}

bool AD5933setRange(byte range)
{
    byte val;

    // Get the current value of the control register
    if(!AD5933getByte(CTRL_REG1, &val))
    {
        return false;
    }

    // Clear out the bottom bit, D9 and D10, which is the output voltage range set bit
    val &=  0xF9;

    // Determine what output voltage range was selected
    switch (range)
    {
        case CTRL_OUTPUT_RANGE_2:
            // Set output voltage range to 1.0 V p-p typical in CTRL_REG1
            val |= CTRL_OUTPUT_RANGE_2;
            break;

        case CTRL_OUTPUT_RANGE_3:
            // Set output voltage range to 400 mV p-p typical in CTRL_REG1
            val |= CTRL_OUTPUT_RANGE_3;
            break;
        
        case CTRL_OUTPUT_RANGE_4:
            // Set output voltage range to 200 mV p-p typical in CTRL_REG1
            val |= CTRL_OUTPUT_RANGE_4;
            break;

        default:
            // Set output voltage range to 200 mV p-p typical in CTRL_REG1
            val |= CTRL_OUTPUT_RANGE_1;
            break;
    }

    //Write to register
    return AD5933sendByte(CTRL_REG1, val);
}

/**
 * Reset the AD5933. This interrupts a sweep if one is running, but the start
 * frequency, number of increments, and frequency increment register contents
 * are not overwritten, but an initialize start frequency command is required
 * to restart a frequency sweep.
 *
 * @return Success or failure
 */
bool AD5933reset() {
    // Get the current value of the control register
    byte val;
    if (!AD5933getByte(CTRL_REG2, &val))
        return false;

    // Set bit D4 for restart
    val |= CTRL_RESET;

    // Send byte back
    return AD5933sendByte(CTRL_REG2, val);
}

/**
 * Set enable temperature measurement. This interferes with frequency sweep
 * operation, of course.
 *
 * @param enable Option to enable to disable temperature measurement.
 * @return Success or failure
 */
bool AD5933enableTemperature(byte enable) {
    // If enable, set temp measure bits. If disable, reset to no operation.
    if (enable == TEMP_MEASURE) {
        return AD5933setControlMode(CTRL_TEMP_MEASURE);
    } else {
        return AD5933setControlMode(CTRL_NO_OPERATION);
    }
}

/**
 * Get the temperature reading from the AD5933. Waits until a temperature is
 * ready. Also ensures temperature measurement mode is active.
 *
 * @return The temperature in celcius, or -1 if fail.
 */
double AD5933getTemperature() {
    // Set temperature mode
    if (AD5933enableTemperature(TEMP_MEASURE)) {
        // Wait for a valid temperature to be ready
        while((AD5933readStatusRegister() & STATUS_TEMP_VALID) != STATUS_TEMP_VALID) ;

        // Read raw temperature from temperature registers
        byte rawTemp[2];
        if (AD5933getByte(TEMP_DATA_1, &rawTemp[0]) &&
            AD5933getByte(TEMP_DATA_2, &rawTemp[1]))
        {
            // Combine raw temperature bytes into an interger. The ADC
            // returns a 14-bit 2's C value where the 14th bit is a sign
            // bit. As such, we only need to keep the bottom 13 bits.
            int rawTempVal = (rawTemp[0] << 8 | rawTemp[1]) & 0x1FFF;

            // Convert into celcius using the formula given in the
            // datasheet. There is a different formula depending on the sign
            // bit, which is the 5th bit of the byte in TEMP_DATA_1.
            if ((rawTemp[0] & (1<<5)) == 0) {
                return rawTempVal / 32.0;
            } else {
                return (rawTempVal - 16384) / 32.0;
            }
        }
    }
    return -1;
}


/**
 * Set the color source. Choices are between internal and external.
 *
 * @param source Internal or External clock
 * @return Success or failure
 */
bool AD5933setClockSource(byte source) {
    // Determine what source was selected and set it appropriately
    switch (source) {
        case CLOCK_EXTERNAL:
            return AD5933sendByte(CTRL_REG2, CTRL_CLOCK_EXTERNAL);
        case CLOCK_INTERNAL:
            return AD5933sendByte(CTRL_REG2, CTRL_CLOCK_INTERNAL);
        default:
            return false;
    }
}

/**
 * Set the color source to internal or not.
 *
 * @param internal Whether or not to set the clock source as internal.
 * @return Success or failure
 */
bool AD5933setInternalClock(bool internal) {
    // This function is mainly a wrapper for setClockSource()
    if (internal)
        return AD5933setClockSource(CLOCK_INTERNAL);
    else
        return AD5933setClockSource(CLOCK_EXTERNAL);
}

/**
 * Set the start frequency for a frequency sweep.
 *
 * @param start The initial frequency.
 * @return Success or failure
 */
bool AD5933setStartFrequency(unsigned long start) {
    // Page 24 of the Datasheet gives the following formula to represent the
    // start frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    long freqHex = (start / (clockSpeed / 4.0))*pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return false;   // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
    byte highByte = (freqHex >> 16) & 0xFF;
    byte midByte = (freqHex >> 8) & 0xFF;
    byte lowByte = freqHex & 0xFF;

    // Attempt sending all three bytes
    return AD5933sendByte(START_FREQ_1, highByte) &&
           AD5933sendByte(START_FREQ_2, midByte) &&
           AD5933sendByte(START_FREQ_3, lowByte);
}

/**
 * Set the increment frequency for a frequency sweep.
 *
 * @param start The frequency to increment by. Max of 0xFFFFFF.
 * @return Success or failure
 */
bool AD5933setIncrementFrequency(unsigned long increment) {
    // Page 25 of the Datasheet gives the following formula to represent the
    // increment frequency.
    // TODO: Precompute for better performance if we want to keep this constant.
    long freqHex = (increment / (clockSpeed / 4.0))*pow(2, 27);
    if (freqHex > 0xFFFFFF) {
        return false;   // overflow
    }

    // freqHex should be a 24-bit value. We need to break it up into 3 bytes.
    byte highByte = (freqHex >> 16) & 0xFF;
    byte midByte = (freqHex >> 8) & 0xFF;
    byte lowByte = freqHex & 0xFF;

    // Attempt sending all three bytes
    return AD5933sendByte(INC_FREQ_1, highByte) &&
           AD5933sendByte(INC_FREQ_2, midByte) &&
           AD5933sendByte(INC_FREQ_3, lowByte);
}

/**
 * Set the number of frequency increments for a frequency sweep.
 *
 * @param start The number of increments to use. Max 511.
 * @return Success or failure
 */
bool AD5933setNumberIncrements(unsigned int num) {
    // Check that the number sent in is valid.
    if (num > 511) {
        return false;
    }

    // Divide the 9-bit integer into 2 bytes.
    byte highByte = (num >> 8) & 0xFF;
    byte lowByte = num & 0xFF;

    // Write to register.
    return AD5933sendByte(NUM_INC_1, highByte) &&
           AD5933sendByte(NUM_INC_2, lowByte);
}

/**
 * Set the PGA gain factor.
 *
 * @param gain The gain factor to select. Use constants or 1/5.
 * @return Success or failure
 */
bool AD5933setPGAGain(byte gain) {
    // Get the current value of the control register
    byte val;
    if (!AD5933getByte(CTRL_REG1, &val))
        return false;

    // Clear out the bottom bit, D8, which is the PGA gain set bit
    val &= 0xFE;

    // Determine what gain factor was selected
    if (gain == PGA_GAIN_X1 || gain == 1) {
        // Set PGA gain to x1 in CTRL_REG1
        val |= PGA_GAIN_X1;
        return AD5933sendByte(CTRL_REG1, val);
    } else if (gain == PGA_GAIN_X5 || gain == 5) {
        // Set PGA gain to x5 in CTRL_REG1
        val |= PGA_GAIN_X5;
        return AD5933sendByte(CTRL_REG1, val);
    } else {
        return false;
    }
}

/**
 * Read the value of a register.
 *
 * @param reg The address of the register to read.
 * @return The value of the register. Returns 0xFF if can't read it.
 */
byte AD5933readRegister(byte reg) {
    // Read status register and return it's value. If fail, return 0xFF.
    byte val;
    if (AD5933getByte(reg, &val)) {
        return val;
    } else {
        return STATUS_ERROR;
    }
}

/**
 * Read the value of the status register.
 *
 * @return The value of the status register. Returns 0xFF if can't read it.
 */
byte AD5933readStatusRegister() {
    return AD5933readRegister(STATUS_REG);
}

/**
 * Read the value of the control register.
 *
 * @return The value of the control register. Returns 0xFFFF if can't read it.
 */
int AD5933readControlRegister() {
    return ((AD5933readRegister(CTRL_REG1) << 8) | AD5933readRegister(CTRL_REG2)) & 0xFFFF;
}

/**
 * Get a raw complex number for a specific frequency measurement.
 *
 * @param real Pointer to an int that will contain the real component.
 * @param imag Pointer to an int that will contain the imaginary component.
 * @return Success or failure
 */
bool AD5933getComplexData(int16_t *real, int16_t *imag) {
    // Wait for a measurement to be available
    while ((AD5933readStatusRegister() & STATUS_DATA_VALID) != STATUS_DATA_VALID);

    // Read the four data registers.
    // TODO: Do this faster with a block read
    byte realComp[2];
    byte imagComp[2];
    if (AD5933getByte(REAL_DATA_1, &realComp[0]) &&
        AD5933getByte(REAL_DATA_2, &realComp[1]) &&
        AD5933getByte(IMAG_DATA_1, &imagComp[0]) &&
        AD5933getByte(IMAG_DATA_2, &imagComp[1]))
    {
        // Combine the two separate bytes into a single 16-bit value and store
        // them at the locations specified.
        *real = (int16_t)(((realComp[0] << 8) | realComp[1]) & 0xFFFF);
        *imag = (int16_t)(((imagComp[0] << 8) | imagComp[1]) & 0xFFFF);

        return true;
    } else {
        *real = -1;
        *imag = -1;
        return false;
    }
}

/**
 * Set the power level of the AD5933.
 *
 * @param level The power level to choose. Can be on, standby, or down.
 * @return Success or failure
 */
bool AD5933setPowerMode(byte level) {
    // Make the appropriate switch. TODO: Does no operation even do anything?
    switch (level) {
        case POWER_ON:
            return AD5933setControlMode(CTRL_NO_OPERATION);
        case POWER_STANDBY:
            return AD5933setControlMode(CTRL_STANDBY_MODE);
        case POWER_DOWN:
            return AD5933setControlMode(CTRL_POWER_DOWN_MODE);
        default:
            return false;
    }
}

/**
 * Perform a complete frequency sweep.
 *
 * @param real An array of appropriate size to hold the real data.
 * @param imag An array of appropriate size to hold the imaginary data.
 * @param n Length of the array (or the number of discrete measurements)
 * @return Success or failure
 */
int AcqStateMachine = 0;

bool AD5933frequencySweepAsync(int16_t* real, int16_t* imag, int n) {
    // Begin by issuing a sequence of commands
    // If the commands aren't taking hold, add a brief delay
    
    static int i = 0;
    
    //NRF_LOG_INFO("state %d", AcqStateMachine);
    
    switch(AcqStateMachine)
    {
        case 0:
            if(AD5933setControlMode(CTRL_INIT_START_FREQ))
            {
                AcqStateMachine = 1;
                
            }
                return false;
        case 1:
            if(AD5933setControlMode(CTRL_START_FREQ_SWEEP))
            {
                AcqStateMachine = 2;
                i = 0;
            }
                return false;
        case 2:
            ;
            uint8_t ret = AD5933readStatusRegister();
            //NRF_LOG_INFO("Convert result %d", ret);
            if (ret & STATUS_DATA_VALID) 
            {
                NRF_LOG_INFO("Convert completed");
                //if (i < n)
                {
                    if (AD5933getComplexData(real + i, imag + i))
                    {
                        i++;
                        AcqStateMachine = 3;
                        return true;
                    }
                }
                
            }
            return false;
        case 3:
            
            if (i < n)
            {
                if(AD5933setControlMode(CTRL_INCREMENT_FREQ))
                {
                    AcqStateMachine = 2;
                    return false;
                }
                    
            } else 
            {
                i=0;
                AcqStateMachine = 4;
                return false;
            }
        
        case 4:
            
            AD5933setPowerMode(POWER_STANDBY);
            AcqStateMachine = 0;
            return false;
            
    
    }
    
}

bool AD5933_begin(void)
{
    
    //Communication init.
    iic_init();
    
    AD5933reset();
    AD5933setClockSource(CLOCK_INTERNAL);
    AD5933setPGAGain(PGA_GAIN_X1);
    AD5933setRange(CTRL_OUTPUT_RANGE_1);
    
    AD5933setStartFrequency(7000);
    AD5933setIncrementFrequency(100);
    AD5933setNumberIncrements(1);
    
    AD5933setPowerMode(POWER_STANDBY);
    
    return 0;


}

