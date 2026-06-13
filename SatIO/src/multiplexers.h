/*
  Multiplexers Library. Written by Benjamin Jack Cullen.

  For TCA9548A & CD74HC4067.
*/

#ifndef MULTIPLEXERS_H
#define MULTIPLEXERS_H

#include <stdint.h>
#include <Arduino.h>
#include <Wire.h>

/* Do not modify */
#define MAX_ANALOG_DIGITAL_MULTIPLEXER_PINS             5
#define MAX_ANALOG_DIGITAL_MULTIPLEXER_CONTROL_PINS     4
#define MAX_ANALOG_DIGITAL_MULTIPLEXER_CHANNELS         16
#define INDEX_ANALOG_DIGITAL_MULTIPLEXER_S0             0
#define INDEX_ANALOG_DIGITAL_MULTIPLEXER_S1             1
#define INDEX_ANALOG_DIGITAL_MULTIPLEXER_S2             2
#define INDEX_ANALOG_DIGITAL_MULTIPLEXER_S3             3
#define INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG            4

/* Customize pins as required */
#define PIN_ANALOG_DIGITAL_MULTIPLEXER_0_SO  23
#define PIN_ANALOG_DIGITAL_MULTIPLEXER_0_S1  22
#define PIN_ANALOG_DIGITAL_MULTIPLEXER_0_S2  21
#define PIN_ANALOG_DIGITAL_MULTIPLEXER_0_S3  20
#define PIN_ANALOG_DIGITAL_MULTIPLEXER_0_SIG 53

/* Customize pins as required */
// #define PIN_ANALOG_DIGITAL_MULTIPLEXER_1_SO  12
// #define PIN_ANALOG_DIGITAL_MULTIPLEXER_1_S1  14
// #define PIN_ANALOG_DIGITAL_MULTIPLEXER_1_S2  27
// #define PIN_ANALOG_DIGITAL_MULTIPLEXER_1_S3  16
// #define PIN_ANALOG_DIGITAL_MULTIPLEXER_1_SIG 4

/* Do not modify */
#define I2C_MULTIPLEXER_TCA9548A_ADDRESS_0    0x70 // Default I2C address for TCA9548A
#define MAX_I2C_MULTIPLEXER_TCA9548A_CHANNELS 8

/**
 * @struct I2C Multiplexer
 * @brief Useful for creating multiple I2C multiplexer instances
 */
typedef struct {
  int address;
  double data[MAX_I2C_MULTIPLEXER_TCA9548A_CHANNELS];
} I2CMultiplexer;
extern I2CMultiplexer i2c_mux_0;

/**
 * @struct AnalogDigitalMultiplexer
 * @brief Useful for creating multiple analog/digital multiplexer instances
 */
typedef struct {
  int pins[MAX_ANALOG_DIGITAL_MULTIPLEXER_PINS];
  double data[MAX_ANALOG_DIGITAL_MULTIPLEXER_CHANNELS];
} AnalogDigitalMultiplexer;
extern AnalogDigitalMultiplexer ad_mux_0;
// extern AnalogDigitalMultiplexer ad_mux_1;

/**
 * Set IIC multiplexer channel
 * @param wire Specify TwoWire instance
 * @param mux_id Specify IIC multiplexer
 * @param channel Specify IIC multiplexer channel
 * @return None
 */
void setI2CMultiplexChannel(TwoWire &wire, I2CMultiplexer &mux_id, uint8_t channel);


/**
 * Initialize analog/digital multiplexer
 * @param mux_id Specify analog/digital multiplexer
 * @return None
 */
void initADMultiplexer(AnalogDigitalMultiplexer &mux_id);

/**
 * Set analog/digital multiplexer channel
 * @param mux_id Specify analog/digital multiplexer
 * @param channel Specify analog/digital multiplexer channel
 * @return None
 */
void setADMultiplexerChannel(AnalogDigitalMultiplexer &mux_id, int channel);

/**
 * Read from channel.
 * @param mux_id Specify analog/digital multiplexer
 * @param channel Specify analog/digital multiplexer channel
 * @return None
 */
void readADMultiplexerAnalogChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel);
void readADMultiplexerDigitalChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel);
void readAllADMultiplexerAnalogChannels(AnalogDigitalMultiplexer &mux_id);
void readAllADMultiplexerDigitalChannels(AnalogDigitalMultiplexer &mux_id);

/**
 * Write to channel.
 * @param mux_id Specify analog/digital multiplexer
 * @param channel Specify analog/digital multiplexer channel
 * @param data Data to be written to channel
 * @return None
 */
void writeADMultiplexerAnalogChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel, int data);
void writeADMultiplexerDigitalChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel, int data);

/**
 * NAN stored analog/digital multiplexer channel data
 */
void setADMultiplexerDataNAN(AnalogDigitalMultiplexer &mux_id);

/**
 * NAN stored IIC multiplexer channel data
 */
void setI2CMultiplexerDataNAN(I2CMultiplexer &mux_id);

#endif