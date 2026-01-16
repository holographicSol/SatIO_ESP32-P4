/*
  Multiplexers Library. Written by Benjamin Jack Cullen.

  For TCA9548A & CD74HC4067.
*/

#include <Arduino.h>
#include <Wire.h>
#include "./multiplexers.h"

// ------------------------------------------------------------------------------------
// Add/Remove Multiplexer Instances
// ------------------------------------------------------------------------------------
AnalogDigitalMultiplexer ad_mux_0 = {
  .pins = {PIN_ANALOG_DIGITAL_MULTIPLEXER_0_SO,
           PIN_ANALOG_DIGITAL_MULTIPLEXER_0_S1,
           PIN_ANALOG_DIGITAL_MULTIPLEXER_0_S2,
           PIN_ANALOG_DIGITAL_MULTIPLEXER_0_S3,
           PIN_ANALOG_DIGITAL_MULTIPLEXER_0_SIG},
  .data = {},
};

// AnalogDigitalMultiplexer ad_mux_1 = {
//   .pins = {PIN_ANALOG_DIGITAL_MULTIPLEXER_1_SO,
//            PIN_ANALOG_DIGITAL_MULTIPLEXER_1_S1,
//            PIN_ANALOG_DIGITAL_MULTIPLEXER_1_S2,
//            PIN_ANALOG_DIGITAL_MULTIPLEXER_1_S3,
//            PIN_ANALOG_DIGITAL_MULTIPLEXER_1_SIG},
//   .data = {},
// };

I2CMultiplexer i2c_mux_0 = {
  .address = I2C_MULTIPLEXER_TCA9548A_ADDRESS_0,
  .data = {},
};

// ------------------------------------------------------------------------------------
// IIC
// ------------------------------------------------------------------------------------
void setI2CMultiplexChannel(TwoWire &wire, I2CMultiplexer &mux_id, uint8_t channel) {
  /* Set the channel on specified I2C multiplexer */
  wire.beginTransmission(mux_id.address);
  wire.write(1 << channel);
  wire.endTransmission();
}

void setI2CMultiplexerDataNAN(I2CMultiplexer &mux_id) {
  /* Set all I2C multiplexer channel data to NAN */
  for (int i=0; i<MAX_I2C_MULTIPLEXER_TCA9548A_CHANNELS; i++)
    {mux_id.data[i]=NAN;}
}

// ------------------------------------------------------------------------------------
// Analog/Digital
// ------------------------------------------------------------------------------------
static const int AD_MUX_CHANNEL_TABLE[MAX_ANALOG_DIGITAL_MULTIPLEXER_CHANNELS][MAX_ANALOG_DIGITAL_MULTIPLEXER_CONTROL_PINS] = {
  {0,0,0,0}, // channel 0 
  {1,0,0,0}, // channel 1 
  {0,1,0,0}, // channel 2
  {1,1,0,0}, // channel 3
  {0,0,1,0}, // channel 4
  {1,0,1,0}, // channel 5
  {0,1,1,0}, // channel 6
  {1,1,1,0}, // channel 7
  {0,0,0,1}, // channel 8
  {1,0,0,1}, // channel 9
  {0,1,0,1}, // channel 10
  {1,1,0,1}, // channel 11
  {0,0,1,1}, // channel 12
  {1,0,1,1}, // channel 13
  {0,1,1,1}, // channel 14
  {1,1,1,1}  // channel 15
};

void setADMultiplexerChannel(AnalogDigitalMultiplexer &mux_id, int channel) {
  /* Set the channel on specified analog/digital multiplexer */
  for (int i=0; i<MAX_ANALOG_DIGITAL_MULTIPLEXER_CONTROL_PINS; i++) {
      digitalWrite(mux_id.pins[i], AD_MUX_CHANNEL_TABLE[channel][i]);
  }
}

void readADMultiplexerAnalogChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel) {
  /* Default is to store raw data (customize as required). */
  setADMultiplexerChannel(mux_id, channel);
  pinMode(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], INPUT);
  mux_id.data[channel] = analogRead(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]);
}

void readAllADMultiplexerAnalogChannels(AnalogDigitalMultiplexer &mux_id) {
  for (int i=0; i<MAX_ANALOG_DIGITAL_MULTIPLEXER_CHANNELS; i++) {
    readADMultiplexerAnalogChannel(mux_id, i);
  }
}

void readADMultiplexerDigitalChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel) {
  /* Default is to store raw data (customize as required). */
  setADMultiplexerChannel(mux_id, channel);
  pinMode(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], INPUT);
  mux_id.data[channel] = digitalRead(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG]);
}

void readAllADMultiplexerDigitalChannels(AnalogDigitalMultiplexer &mux_id) {
  for (int i=0; i<MAX_ANALOG_DIGITAL_MULTIPLEXER_CHANNELS; i++) {
    readADMultiplexerDigitalChannel(mux_id, i);
  }
}

void writeADMultiplexerAnalogChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel, int data) {
  /* Write analog data to channel */
  setADMultiplexerChannel(mux_id, channel);
  pinMode(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], OUTPUT);
  analogWrite(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], data);
}

void writeADMultiplexerDigitalChannel(AnalogDigitalMultiplexer &mux_id, uint8_t channel, int data) {
  /* Write digital data to channel */
  setADMultiplexerChannel(mux_id, channel);
  pinMode(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], OUTPUT);
  digitalWrite(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], data);
}

void setADMultiplexerDataNAN(AnalogDigitalMultiplexer &mux_id) {
  /* Set all analog/digital multiplexer channel data to NAN */
  for (int i=0; i<MAX_ANALOG_DIGITAL_MULTIPLEXER_CHANNELS; i++) {mux_id.data[i]=NAN;}
}

void initADMultiplexer(AnalogDigitalMultiplexer &mux_id) {
  /* Initialize an analog/digital multiplexer instance */
  for (int i=0; i<MAX_ANALOG_DIGITAL_MULTIPLEXER_CONTROL_PINS; i++) {
    // control pins as outputs
      pinMode(mux_id.pins[i], OUTPUT);
      digitalWrite(mux_id.pins[i], LOW);
  }
  // signal pin as direction
  pinMode(mux_id.pins[INDEX_ANALOG_DIGITAL_MULTIPLEXER_SIG], INPUT);
}
