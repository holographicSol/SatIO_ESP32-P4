/*
  I2C Helper Functions - Written By Benjamin Jack Cullen.

  Intends to standardize I2C communication functions across
  multiple I2C buses, devices, and across multiple projects.
*/

#ifndef I2C_HELPER_H
#define I2C_HELPER_H

#include <stdint.h>
#include <stdbool.h>
#include <Arduino.h>
#include <Wire.h>

// #define SLAVE_ADDR_BUS0 0  // uncomment to set address if required.
#define SLAVE_ADDR_BUS1 12 // uncomment to set address if required.
// #define SLAVE_ADDR_BUS2 0  // uncomment to set address if required.

#define IIC_BUS0_SDA 21 // uncomment to set pin if required.
#define IIC_BUS0_SCL 22 // uncomment to set pin if required.

#define IIC_BUS1_SDA 18 // uncomment to set pin if required.
#define IIC_BUS1_SCL 19 // uncomment to set pin if required.

// #define IIC_BUS2_SDA 7 // uncomment to set pin if required.
// #define IIC_BUS2_SCL 8 // uncomment to set pin if required.

#define I2C_MAX_TOKENS 32
#define MAX_IIC_BUFFER_SIZE 32
#define I2C_ADDR_CONTROL_PAD 8
#define I2C_ADDR_OUTPUT_PORTCONTROLLER 9
#define I2C_ADDR_INPUT_PORTCONTROLLER    10

extern TwoWire iic_0;
extern TwoWire iic_1;
extern TwoWire iic_2;

typedef struct {
  int  i_token;
  char * token;
  long i_bytes;
  char INPUT_BUFFER[MAX_IIC_BUFFER_SIZE];
  char OUTPUT_BUFFER_CHARS[MAX_IIC_BUFFER_SIZE];
  byte OUTPUT_BUFFER_BYTES[MAX_IIC_BUFFER_SIZE];
  char TOKENS[I2C_MAX_TOKENS][MAX_IIC_BUFFER_SIZE];
  long REQUEST_ID;

} IICLink;

/** ----------------------------------------------------------------------------
 * @brief Global IIC wire instances and data struictures for each wire channel.
 * @warning Only to be used by receive event handlers! Make your own IICLink instances for other uses.
 */
extern IICLink I2CLinkBus0; // default data structure instance for I2C bus 0
extern IICLink I2CLinkBus1; // default data structure instance for I2C bus 1
extern IICLink I2CLinkBus2; // default data structure instance for I2C bus 2

/** ----------------------------------------------------------------------------
 * @brief Clears the output buffer chars of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputChars(IICLink &iic_link);

/** ----------------------------------------------------------------------------
 * @brief Clears the output buffer bytes of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputBytes(IICLink &iic_link);

/** ----------------------------------------------------------------------------
 * @brief Clears the input buffer chars of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkInputChars(IICLink &iic_link);

/** ----------------------------------------------------------------------------
 * @brief Writes data to an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void writeI2CToSlaveChars(TwoWire &wire,
                          IICLink &iic_link,
                          int address,
                          long delayMs,
                          String debugTag);

/** ----------------------------------------------------------------------------
 * @brief Writes data to an I2C master device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param delayMs Delay in milliseconds after writing.
 */
void writeI2CToMasterChars(TwoWire &wire,
                           IICLink &iic_link,
                           long delayMs);

/** ----------------------------------------------------------------------------
 * @brief Requests data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param request_id Request ID to send to the slave so that slave knows what is being requested.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void requestFromSlaveChars(TwoWire &wire,
                           IICLink &iic_link,
                           int address,
                           long request_id,
                           size_t len_expected,
                           long delayMs,
                           String debugTag);

#endif // I2C_HELPER_H