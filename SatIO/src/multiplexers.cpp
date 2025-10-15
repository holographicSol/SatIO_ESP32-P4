/*
  Multiplexers Library. Written by Benjamin Jack Cullen.
*/

#include "multiplexers.h"
#include <Arduino.h>
#include <Wire.h>

static struct I2CMultiplexer i2c_muxes[MAX_I2C_MUX] = {
    { .address = 0x70 },  // Mux 0
};

static struct ADMultiplexer ad_muxes[MAX_AD_MUX] = {
    {
      .control_pins={ADMPLEX_0_S0,
                    ADMPLEX_0_S1,
                    ADMPLEX_0_S2,
                    ADMPLEX_0_S3},
      .signal_pin = ADMPLEX_0_SIG
    },
};

struct MultiplexerDataStruct multiplexerData = {
  .ADMPLEX_0_DATA={
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
  },
  .IICMPLEX_0_DATA_0={
    0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0,
  },
};

static const int AD_MUX_CHANNEL_TABLE[MAX_AD_MUX_CHANNELS][MAX_AD_MUX_CONTROL_PINS] = {
  {0,0,0,0}, //channel 0 
  {1,0,0,0}, //channel 1 
  {0,1,0,0}, //channel 2
  {1,1,0,0}, //channel 3
  {0,0,1,0}, //channel 4
  {1,0,1,0}, //channel 5
  {0,1,1,0}, //channel 6
  {1,1,1,0}, //channel 7
  {0,0,0,1}, //channel 8
  {1,0,0,1}, //channel 9
  {0,1,0,1}, //channel 10
  {1,1,0,1}, //channel 11
  {0,0,1,1}, //channel 12
  {1,0,1,1}, //channel 13
  {0,1,1,1}, //channel 14
  {1,1,1,1}  //channel 15
};

void initMultiplexI2C(uint8_t mux_id) {if (mux_id >= MAX_I2C_MUX) {return;}}

void setMultiplexChannel_I2C(uint8_t mux_id, uint8_t channel) {
    if (mux_id >= MAX_I2C_MUX || channel > 7) {return;}
    Wire.beginTransmission(i2c_muxes[mux_id].address);
    Wire.write(1 << channel);
    Wire.endTransmission();
}

void initMultiplexAD(uint8_t mux_id) {
    if (mux_id >= MAX_AD_MUX) {return;}
    for (int i = 0; i < MAX_AD_MUX_CONTROL_PINS; i++) {
        pinMode(ad_muxes[mux_id].control_pins[i], OUTPUT);
        digitalWrite(ad_muxes[mux_id].control_pins[i], LOW);
    }
    pinMode(ad_muxes[mux_id].signal_pin, INPUT);
}

void setMultiplexChannel_AD(uint8_t mux_id, int channel) {
    if (mux_id >= MAX_AD_MUX || channel >= MAX_AD_MUX_CHANNELS) {return;}
    for (int i = 0; i < MAX_AD_MUX_CONTROL_PINS; i++) {
        digitalWrite(ad_muxes[mux_id].control_pins[i], AD_MUX_CHANNEL_TABLE[channel][i]);
    }
}

void setADMPLEX_0_NAN(void) {
  for (int i=0; i<MAX_AD_MUX_CHANNELS; i++)
    {multiplexerData.ADMPLEX_0_DATA[i]=NAN;}
}

void setIICMPLEX_0_NAN(void) {
for (int i=0; i<MAX_IIC_MUX_CHANNELS; i++)
  {multiplexerData.IICMPLEX_0_DATA_0[i]=NAN;}
}