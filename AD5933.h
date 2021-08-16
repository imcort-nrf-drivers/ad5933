#ifndef AD5933_h
#define AD5933_h

/**
 * Includes
 */
#include <stdint.h>
#include <stdbool.h>
typedef unsigned char byte;
/**
 * AD5933 Register Map
 *  Datasheet p23
 */
// Device address and address pointer
#define AD5933_ADDR     (0x0D)
#define ADDR_PTR        (0xB0)
// Control Register
#define CTRL_REG1       (0x80)
#define CTRL_REG2       (0x81)
// Start Frequency Register
#define START_FREQ_1    (0x82)
#define START_FREQ_2    (0x83)
#define START_FREQ_3    (0x84)
// Frequency increment register
#define INC_FREQ_1      (0x85)
#define INC_FREQ_2      (0x86)
#define INC_FREQ_3      (0x87)
// Number of increments register
#define NUM_INC_1       (0x88)
#define NUM_INC_2       (0x89)
// Number of settling time cycles register
#define NUM_SCYCLES_1   (0x8A)
#define NUM_SCYCLES_2   (0x8B)
// Status register
#define STATUS_REG      (0x8F)
// Temperature data register
#define TEMP_DATA_1     (0x92)
#define TEMP_DATA_2     (0x93)
// Real data register
#define REAL_DATA_1     (0x94)
#define REAL_DATA_2     (0x95)
// Imaginary data register
#define IMAG_DATA_1     (0x96)
#define IMAG_DATA_2     (0x97)

/**
 * Constants
 *  Constants for use with the AD5933 library class.
 */
// Temperature measuring
#define TEMP_MEASURE    (CTRL_TEMP_MEASURE)
#define TEMP_NO_MEASURE (CTRL_NO_OPERATION)
// Clock sources
#define CLOCK_INTERNAL  (CTRL_CLOCK_INTERNAL)
#define CLOCK_EXTERNAL  (CTRL_CLOCK_EXTERNAL)
// PGA gain options
#define PGA_GAIN_X1     (CTRL_PGA_GAIN_X1)
#define PGA_GAIN_X5     (CTRL_PGA_GAIN_X5)
// Power modes
#define POWER_STANDBY   (CTRL_STANDBY_MODE)
#define POWER_DOWN      (CTRL_POWER_DOWN_MODE)
#define POWER_ON        (CTRL_NO_OPERATION)
// I2C result success/fail
#define I2C_RESULT_SUCCESS       (0)
#define I2C_RESULT_DATA_TOO_LONG (1)
#define I2C_RESULT_ADDR_NAK      (2)
#define I2C_RESULT_DATA_NAK      (3)
#define I2C_RESULT_OTHER_FAIL    (4)
// Control register options
#define CTRL_NO_OPERATION       (0x00)
#define CTRL_INIT_START_FREQ    (0x10)
#define CTRL_START_FREQ_SWEEP   (0x20)
#define CTRL_INCREMENT_FREQ     (0x30)
#define CTRL_REPEAT_FREQ        (0x40)
#define CTRL_TEMP_MEASURE       (0x90)
#define CTRL_POWER_DOWN_MODE    (0xa0)
#define CTRL_STANDBY_MODE       (0xb0)
#define CTRL_RESET              (0x10)
#define CTRL_CLOCK_EXTERNAL     (0x08)
#define CTRL_CLOCK_INTERNAL     (0x00)
#define CTRL_PGA_GAIN_X1        (0x01)
#define CTRL_PGA_GAIN_X5        (0x00)
// Status register options
#define STATUS_TEMP_VALID       (0x01)
#define STATUS_DATA_VALID       (0x02)
#define STATUS_SWEEP_DONE       (0x04)
#define STATUS_ERROR            (0xFF)
// Frequency sweep parameters
#define SWEEP_DELAY             (1)
#define clockSpeed 16776000
/**
 * AD5933 Library class
 *  Contains mainly functions for interfacing with the AD5933.
 */

// Reset the board
bool AD5933reset(void);

// Temperature measuring
bool AD5933enableTemperature(byte);
double AD5933getTemperature(void);

// Clock
bool AD5933setClockSource(byte);
bool AD5933setInternalClock(bool);
//bool setSettlingCycles(int); // not implemented - not used yet

// Frequency sweep configuration
bool AD5933setStartFrequency(unsigned long);
bool AD5933setIncrementFrequency(unsigned long);
bool AD5933setNumberIncrements(unsigned int);

// Gain configuration
bool AD5933setPGAGain(byte);

// Excitation range configuration
//bool setRange(byte, int); // not implemented - not used yet

// Read registers
byte AD5933readRegister(byte);
byte AD5933readStatusRegister(void);
int AD5933readControlRegister(void);

// Impedance data
bool AD5933getComplexData(int16_t*, int16_t*);

// Set control mode register (CTRL_REG1)
bool AD5933setControlMode(byte);

// Power mode
bool AD5933setPowerMode(byte);

// Perform frequency sweeps
bool AD5933frequencySweep(int16_t real[], int16_t imag[], int);
bool AD5933calibrate(double gain[], int phase[], int ref, int n);
//static bool AD5933calibrate(double gain[], int phase[], int real[],
//                        int imag[], int ref, int n);

// Private data
//unsigned long clockSpeed = 16776000;

// Sending/Receiving byte method, for easy re-use
//int AD5933getByte(byte, byte*);
//bool AD5933sendByte(byte, byte);

#endif
