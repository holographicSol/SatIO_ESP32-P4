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
// #define SLAVE_ADDR_BUS1 0 // uncomment to set address if required.
// #define SLAVE_ADDR_BUS2 0  // uncomment to set address if required.

#define IIC_BUS0_SDA 2 // uncomment to set pin if required.
#define IIC_BUS0_SCL 3 // uncomment to set pin if required.

#define IIC_BUS1_SDA 4 // uncomment to set pin if required.
#define IIC_BUS1_SCL 5 // uncomment to set pin if required.

#define IIC_BUS2_SDA 7 // uncomment to set pin if required.
#define IIC_BUS2_SCL 8 // uncomment to set pin if required.

#define I2C_MAX_TOKENS 32
#define MAX_IIC_BUFFER_SIZE 32
#define I2C_ADDR_CONTROL_PAD 8
#define I2C_ADDR_OUTPUT_PORTCONTROLLER 9
#define I2C_ADDR_INPUT_PORTCONTROLLER  10

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
  uint8_t INPUT_PACKET[MAX_IIC_BUFFER_SIZE];
  uint8_t OUTPUT_PACKET[MAX_IIC_BUFFER_SIZE];
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

/** ----------------------------------------------------------------------------
 * @brief Writes binary data to an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void writeI2CToSlaveBin(TwoWire &wire,
                        IICLink &iic_link,
                        int address,
                        long len_packet,
                        long delayMs,
                        String debugTag);

/** ----------------------------------------------------------------------------
 * @brief Writes binary data to an I2C master device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param len_packet Length of the packet to write in bytes.
 * @param delayMs Delay in milliseconds after writing.
 */
void writeI2CToMasterBin(TwoWire &wire,
                         IICLink &iic_link,
                         long len_packet,
                         long delayMs);

/** ----------------------------------------------------------------------------
 * @brief Requests binary data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param request_id Request ID to send to the slave so that slave knows what is being requested.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void requestFromSlaveBin(TwoWire &wire,
                         IICLink &iic_link,
                         int address,
                         long len_packet,
                         long request_id,
                         size_t len_expected,
                         long delayMs,
                         String debugTag);

/** ----------------------------------------------------------------------------
 * @brief Requests binary data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
bool requestFromSlaveBinNoID(TwoWire &wire,
                         IICLink &iic_link,
                         int address,
                         size_t len_expected,
                         long delayMs,
                         String debugTag);

/**
 * @brief Read uint8_t from I2C wire into specified value.
 * @warning Specified value is expected to be uint8_t.
 */
void read_uint8_FromWire(TwoWire &wire, uint8_t &value);

/**
 * @brief Read int8_t from I2C wire into specified value.
 * @warning Specified value is expected to be int8_t.
 */
void read_int8_FromWire(TwoWire &wire, int8_t &value);

/**
 * @brief Read uint16_t from I2C wire into specified value (2 bytes, little-endian).
 * @warning Specified value is expected to be uint16_t.
 */
void read_uint16_FromWire(TwoWire &wire, uint16_t &value);

/**
 * @brief Read int16_t from I2C wire into specified value (2 bytes, little-endian).
 * @warning Specified value is expected to be int16_t.
 */
void read_int16_FromWire(TwoWire &wire, int16_t &value);

/**
 * @brief Read uint32_t from I2C wire into specified value (4 bytes, little-endian).
 * @warning Specified value is expected to be uint32_t.
 */
void read_uint32_FromWire(TwoWire &wire, uint32_t &value);

/**
 * @brief Read int32_t from I2C wire into specified value (4 bytes, little-endian).
 * @warning Specified value is expected to be int32_t.
 */
void read_int32_FromWire(TwoWire &wire, int32_t &value);

/**
 * @brief Read uint64_t from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be uint64_t.
 */
void read_uint64_FromWire(TwoWire &wire, uint64_t &value);

/**
 * @brief Read int64_t from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be int64_t.
 */
void read_int64_FromWire(TwoWire &wire, int64_t &value);

/**
 * @brief Read float from I2C wire into specified value (4 bytes, little-endian).
 * @warning Specified value is expected to be float.
 */
void read_float_FromWire(TwoWire &wire, float &value);

/**
 * @brief Read double from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be double.
 */
void read_double_FromWire(TwoWire &wire, double &value);

/**
 * @brief Read long from I2C wire into specified value (little-endian).
 * @warning Specified value is expected to be long.
 */
void read_long_FromWire(TwoWire &wire, long &value);

/**
 * @brief Read long long from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be long long.
 */
void read_longlong_FromWire(TwoWire &wire, long long &value);

/**
 * @brief Read char from I2C wire into specified value.
 * @warning Specified value is expected to be char.
 */
void read_char_FromWire(TwoWire &wire, char &value);

/**
 * @brief Read N chars from I2C wire into specified char array.
 * @param wire Specify TwoWire instance.
 * @param value Pointer to char array to store the read values.
 * @param n_chars Number of chars to read.
 * @warning Specified value is expected to be a char array with at least n_chars size.
 * @warning Ensure the char array is large enough to hold n_chars values.
 */
void read_nchars_FromWire(TwoWire &wire, char *value, size_t n_chars);

/**
 * @brief Read bool from I2C wire into specified value.
 * @warning Specified value is expected to be bool.
 */
void read_bool_FromWire(TwoWire &wire, bool &value);

/**
 * @brief Read byte from I2C wire into specified value.
 * @warning Specified value is expected to be byte.
 */
void read_byte_FromWire(TwoWire &wire, byte &value);

#endif // I2C_HELPER_H