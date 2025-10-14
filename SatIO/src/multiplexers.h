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

// ----------------------------------------------------------------------------------------
// I2C Multiplexer (TCA9548A) Configuration Struct.
// ----------------------------------------------------------------------------------------
struct I2CMultiplexer {
    uint8_t address;
};

// ----------------------------------------------------------------------------------------
// Analog/Digital Multiplexer (74HC4067) Configuration Struct.
// ----------------------------------------------------------------------------------------
struct ADMultiplexer {
    int control_pins[MAX_AD_MUX_CONTROL_PINS];
    int signal_pin;
};

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                             DATA: MULTIPLEXERS
// ------------------------------------------------------------------------------------------------------------------------------
struct MultiplexerDataStruct {
  // ----------------------------------------------------
  // ADMPLEX 0 x16 Analog/Digital Multiplexer.
  // ----------------------------------------------------
  float ADMPLEX_0_DATA[16]={};
  // ----------------------------------------------------
  // IICMPLEX 0 x8 i2C Multiplexer.
  // ----------------------------------------------------
  float IICMPLEX_0_DATA_0[8]={};
};
extern struct MultiplexerDataStruct multiplexerData;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
void initMultiplexI2C(uint8_t mux_id);
void setMultiplexChannel_I2C(uint8_t mux_id, uint8_t channel);
void initMultiplexAD(uint8_t mux_id);
void setMultiplexChannel_AD(uint8_t mux_id, int channel);
void setADMPLEX_0_NAN(void);
void setIICMPLEX_0_NAN(void);

#ifdef __cplusplus
}
#endif

#endif