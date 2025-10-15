/*
  Multiplexers Library. Written by Benjamin Jack Cullen.
*/

#ifndef MULTIPLEXERS_H
#define MULTIPLEXERS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include "config.h"

/**
 * @struct I2CMultiplexer
 */
struct I2CMultiplexer {
  // addresses for IIC multiplexers
    uint8_t address;
};

/**
 * @struct ADMultiplexer
 */
struct ADMultiplexer {
  // control pins of analog/digital multiplexers
  int control_pins[MAX_AD_MUX_CONTROL_PINS];

  // signal pin of analog/digital multiplexers
  int signal_pin;
};

/**
 * @struct MultiplexerDataStruct
 */
struct MultiplexerDataStruct {
  // Results from analog/digital multiplexer 0.
  float ADMPLEX_0_DATA[16]={};

  // Results from IIC multiplexer 0.
  float IICMPLEX_0_DATA_0[8]={};
};
extern struct MultiplexerDataStruct multiplexerData;

/**
 * Initialize IIC multiplexer
 * @param mux_id Specify IIC multiplexer
 * @return None
 */
void initMultiplexI2C(uint8_t mux_id);

/**
 * Set IIC multiplexer channel
 * @param mux_id Specify IIC multiplexer
 * @param channel Specify IIC multiplexer channel
 * @return None
 */
void setMultiplexChannel_I2C(uint8_t mux_id, uint8_t channel);

/**
 * Initialize analog/digital multiplexer
 * @param mux_id Specify analog/digital multiplexer
 * @return None
 */
void initMultiplexAD(uint8_t mux_id);

/**
 * Set analog/digital multiplexer channel
 * @param mux_id Specify analog/digital multiplexer
 * @param channel Specify analog/digital multiplexer channel
 * @return None
 */
void setMultiplexChannel_AD(uint8_t mux_id, int channel);

/**
 * NAN analog/digital multiplexer channel results
 */
void setADMPLEX_0_NAN(void);

/**
 * NAN IIC multiplexer channel results
 */
void setIICMPLEX_0_NAN(void);

#ifdef __cplusplus
}
#endif

#endif