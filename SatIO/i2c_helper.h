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

#define IIC_BUS0_SDA 2
#define IIC_BUS0_SCL 3

#define IIC_BUS1_SDA 4
#define IIC_BUS1_SCL 5

#define IIC_BUS2_SDA 7
#define IIC_BUS2_SCL 8

#define MAX_TOKENS 32
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
  char TOKENS[MAX_TOKENS][MAX_IIC_BUFFER_SIZE];
  long REQUEST_ID;

} IICLink;

extern IICLink I2CLinkBus0;
extern IICLink I2CLinkBus1;
extern IICLink I2CLinkBus2;

/**
 * @brief Clears the output buffer chars of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputChars(IICLink &iic_link);

/**
 * @brief Clears the output buffer bytes of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputBytes(IICLink &iic_link);

/**
 * @brief Clears the input buffer chars of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkInputChars(IICLink &iic_link);

/**
 * @brief Writes data to an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void writeI2CToSlave(TwoWire &wire,
                     IICLink &iic_link,
                     int address,
                     long delayMs,
                     String debugTag);

/**
 * @brief Requests data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param request_id Request ID to send to the slave so that slave knows what is being requested.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void requestFromSlave(TwoWire &wire,
                     IICLink &iic_link,
                     int address,
                     long request_id,
                     size_t len_expected,
                     long delayMs,
                     String debugTag);

#endif // I2C_HELPER_H