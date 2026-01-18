/*
Written by Benjamin Jack Cullen.

PortController - IIC I/O device.

                 Can send pin readings to master.

                 Can set pin levels acscording to master.

                 For perfomance reasons it is recommnended for single use as either input
                 or output device but can be used as both.

                 Modulation is currently done here but modulation values may be forwarded to
                 external MCU's on each IO in the future.
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>
#include <i2c_helper.h>

// ------------------------------------------------------------
// Set expected pins for a board
// ------------------------------------------------------------
#define PIN_MIN 2
#define PIN_MAX 69
#define MAX_MATRIX_SWITCHES 70

// -----------------------------------------------------------------------------------
// Analog input pins on Mega2560: A0–A15 → physical pins 54–69
// -----------------------------------------------------------------------------------
constexpr uint8_t ANALOG_PINS[]  = {54,55,56,57,58,59,60,61,62,63,64,65,66,67,68,69};
constexpr uint8_t NUM_ANALOG     = sizeof(ANALOG_PINS);

// -----------------------------------------------------------------------------------
// Digital pins on Mega2560: 0–53
// -----------------------------------------------------------------------------------
constexpr uint8_t DIGITAL_PINS[] = {
    0, 1, 2, 3, 4, 5, 6, 7, 8, 9,
    10,11,12,13,14,15,16,17,18,19,
    20,21,22,23,24,25,26,27,28,29,
    30,31,32,33,34,35,36,37,38,39,
    40,41,42,43,44,45,46,47,48,49,
    50,51,52,53
};
constexpr uint8_t NUM_DIGITAL    = sizeof(DIGITAL_PINS);

// --------------------------------------------------------------------
// Inline binary search – compiles to ~10–15 instructions
// --------------------------------------------------------------------
inline bool isAnalogPin(uint8_t pin) {
  for (int i=0; i<NUM_ANALOG; i++) {
    if (pin==ANALOG_PINS[i]) {return true;}
  }
  return false;
}
inline bool isDigitalPin(uint8_t pin) {
  for (int i=0; i<NUM_DIGITAL; i++) {
    if (pin==DIGITAL_PINS[i]) {return true;}
  }
  return false;
}

// ------------------------------------------------------------
// matrix switch ports (default no port)
// ------------------------------------------------------------
volatile signed int matrix_port_map[MAX_MATRIX_SWITCHES] = {
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 0-9
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 10-19
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 20-29
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 30-9
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 40-49
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 50-59
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, // 60-69
}; 
volatile signed int port_index;

// ------------------------------------------------------------
// Matrix switch logic modulation times.
// 0 : uS time off period (0uS = remain on).
// 1 : uS time on period  (0uS = remain off).
// 2 : uS previous time (set automatically).
// Example: if 0,1 both 0uS then remain on.
// Example: if 0=>0uS and 1=0uS then pulse, remain off.
// Example: if 0=>0uS and 1>0uS then keep modulating.
// Allows for multiple scenarios while remaining simple.
// ------------------------------------------------------------
volatile unsigned long matrix_modulation_time[MAX_MATRIX_SWITCHES][3]={
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, // 0-9
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, // 10-19
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, // 20-29
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, // 30-39
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, // 40-49
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, // 50-59
  {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}, {0,0,0}  // 60-69
};

// ------------------------------------------------------------
// Keep track of intended high/low state for modulation
// ------------------------------------------------------------
volatile bool matrix_modulation_switch_state[MAX_MATRIX_SWITCHES] = {
  false, false, false, false, false, false, false, false, false, false, // 0-9
  false, false, false, false, false, false, false, false, false, false, // 10-19
  false, false, false, false, false, false, false, false, false, false, // 20-29
  false, false, false, false, false, false, false, false, false, false, // 30-39
  false, false, false, false, false, false, false, false, false, false, // 40-49
  false, false, false, false, false, false, false, false, false, false, // 50-59
  false, false, false, false, false, false, false, false, false, false, // 60-69
};

// ------------------------------------------------------------
// Output Values: values to be written to a pin
// ------------------------------------------------------------
volatile int output_value[MAX_MATRIX_SWITCHES]={
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
};

// ------------------------------------------------------------
// Input Values: values read from a pin
// ------------------------------------------------------------
volatile int input_value[MAX_MATRIX_SWITCHES]={
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
};

volatile int current_input_value=0;
volatile uint8_t current_pin=0;

// ------------------------------------------------------------
// Clear Data
// ------------------------------------------------------------
void clearMatrixSwitch() {
  for (int i=PIN_MIN;i<PIN_MAX; i++) {digitalWrite(i, LOW);}
  for (int i=0;i<MAX_MATRIX_SWITCHES; i++) {
    output_value[i]=0;
    matrix_port_map[i]=-1;
    matrix_modulation_time[i][0]=0;
    matrix_modulation_time[i][1]=0;
    matrix_modulation_time[i][2]=0;
    matrix_modulation_switch_state[i]=false;
  }
}

/** ----------------------------------------------------------------------------
 * @brief Request binary event handler for Bus 0
 */

void requestEventBus0Bin() {
  // Serial.println("[requestEventBus0Bin] id: " + String(I2CLinkBus0.REQUEST_ID));
  switch (I2CLinkBus0.REQUEST_ID) {
    // send pin reading
    case 0x1E: {
        // Serial.println("[requestEventBus0Bin] preparing to send requested data: input value");
        clearI2CLinkOutputPacket(I2CLinkBus0);
        write_uint8_ToPacket(I2CLinkBus0.OUTPUT_PACKET, 0, (uint8_t)current_pin);
        write_float_ToPacket(I2CLinkBus0.OUTPUT_PACKET, 1, (float)input_value[current_pin]);
        writeI2CToMasterBin(Wire, I2CLinkBus0, 5, 0);
        if (++current_pin >= NUM_ANALOG + NUM_DIGITAL) {current_pin = 0;}
        break;
      }
    default: {
        Serial.println("[requestEventBus0Bin] event id is not defined: " + String(I2CLinkBus0.REQUEST_ID));
        while (Wire.available()) {Wire.read();} // drain
        break;
    }
  }
}

/** ----------------------------------------------------------------------------
 * @brief Receive binary event handler for Bus 0
*/
void receiveEventBus0Bin(int n_bytes_received) {
  if (n_bytes_received < 1) return;
  uint8_t cmd = Wire.read(); // expects uint8 command byte (up to 255 unique commands can be accepted). 
  Serial.println("[receiveEventBus0Bin] " + String(cmd) + " (" + String(n_bytes_received) + " bytes)");
  switch (cmd) {

    // Command 10
    case 0x0A: {
      for (int i = 0; i < MAX_MATRIX_SWITCHES; i++) {
        matrix_port_map[i] = -1;
        output_value[i] = 0;
        matrix_modulation_time[i][0] = 0;
        matrix_modulation_time[i][1] = 0;
        matrix_modulation_time[i][2] = 0;
        matrix_modulation_switch_state[i] = false;
      }
      current_pin = 0;
      while (Wire.available()) Wire.read();  // flush
      break;
    }

    // Command 20
    case 0x14: {
      if (n_bytes_received != 15) { while (Wire.available()) Wire.read(); Serial.println("!=15"); return;}

      uint8_t idx;
      read_uint8_FromWire(Wire, idx);

      int8_t pin;
      read_int8_FromWire(Wire, pin);

      int32_t value;
      read_int32_FromWire(Wire, value);

      uint32_t off_time;
      read_uint32_FromWire(Wire, off_time);

      uint32_t on_time;
      read_uint32_FromWire(Wire, on_time);

      if (idx >= MAX_MATRIX_SWITCHES) return;

      matrix_port_map[idx]           = pin;
      output_value[idx]              = value;
      matrix_modulation_time[idx][0] = off_time;
      matrix_modulation_time[idx][1] = on_time;

      // Debug: I2C timeouts may occur if blocking with serial prints 
      // Serial.println("pin  " + String(matrix_port_map[idx]));
      // Serial.println("outv " + String(output_value[idx]));
      // Serial.println("pwm0 " + String(matrix_modulation_time[idx][0]));
      // Serial.println("pwm1 " + String(matrix_modulation_time[idx][1]));
      // Serial.println("pwm2 " + String(matrix_modulation_time[idx][2]));
      // Serial.println("pwm3 " + String(matrix_modulation_switch_state[idx] ? "true" : "false"));

      // Set digital pin
      if (isDigitalPin(matrix_port_map[port_index])) {
        current_input_value=digitalRead(matrix_port_map[port_index]);
        if ( (current_input_value==1) && (output_value[port_index]==0) ) {
          pinMode(matrix_port_map[port_index], OUTPUT); // new
          digitalWrite(matrix_port_map[port_index], LOW);
          matrix_modulation_time[port_index][2]=0;
        }
        else if ( (current_input_value==0) && (output_value[port_index]==1) ) {
          pinMode(matrix_port_map[port_index], OUTPUT); // new
          digitalWrite(matrix_port_map[port_index], HIGH);
          matrix_modulation_time[port_index][2]=0;
        }
      }
      // Set analog pin
      else if (isAnalogPin(matrix_port_map[port_index])) {
        pinMode(matrix_port_map[port_index], OUTPUT); // new
        analogWrite(matrix_port_map[port_index], output_value[port_index]);
      }
      break;
    }

    // Command 30
    case 0x1E: {
      current_pin = 0;
      I2CLinkBus0.REQUEST_ID = 0x1E;
      while (Wire.available()) {Wire.read();}
      break;
    }
    // Default: flush
    default: {
      while (Wire.available()) {Wire.read();}
      break;
    }
  }
}

// ------------------------------------------------------------
// Output modulator
// ------------------------------------------------------------
void modulator() {
  // ------------------------------------------------------------
  // Logic modulator
  // Modulate output only if a switch state is already true.
  // Modulator values: time high, time low.
  // ------------------------------------------------------------
  for (int i=0; i<MAX_MATRIX_SWITCHES; i++) {
    if (output_value[i]>0) {
      if (matrix_modulation_time[i][0] != 0 || matrix_modulation_time[i][1] != 0) {
        // ------------------------------------------------------
        // handle currently low
        // ------------------------------------------------------
        if (matrix_modulation_switch_state[i]==false) {
          // ----------------------------------
          // modulate on
          // ----------------------------------
          if ((micros() - matrix_modulation_time[i][2]) >= matrix_modulation_time[i][0]) {
            // Serial.println("[t0 exceeded (mod on)] idx: " + String(i));
            if (matrix_port_map[i]<54) {digitalWrite(matrix_port_map[i], HIGH);}
            else {analogWrite(matrix_port_map[i], output_value[i]);}
            matrix_modulation_time[i][2]=micros();
            matrix_modulation_switch_state[i]=true;
          }
        }
        // -------------------------------------------------------
        // handle currently high
        // -------------------------------------------------------
        else if (matrix_modulation_switch_state[i]==true) {
          // ----------------------------------
          // remain off
          // ----------------------------------
          if (matrix_modulation_time[i][1]==0) {
            if ((micros() - matrix_modulation_time[i][2]) >= matrix_modulation_time[i][0]) {
              // Serial.println("[t1 exceeded (remain off)] idx: " + String(i));
              if (matrix_port_map[i]<54) {digitalWrite(matrix_port_map[i], LOW);}
              else {analogWrite(matrix_port_map[i], 0);}
              matrix_modulation_time[i][2]=micros();
              matrix_modulation_switch_state[i]=false;
              // change parent state off
              output_value[i]=0;
            }
          }
          // ----------------------------------
          // modulate off
          // ----------------------------------
          else {
            if ((micros() - matrix_modulation_time[i][2]) >= matrix_modulation_time[i][1]) {
              // Serial.println("[t1 exceeded (mod off)] idx: " + String(i));
              if (matrix_port_map[i]<54) {digitalWrite(matrix_port_map[i], LOW);}
              else {analogWrite(matrix_port_map[i], 0);}
              matrix_modulation_time[i][2]=micros();
              matrix_modulation_switch_state[i]=false;
            }
          }
        }
      }
    }
  }
}

// ------------------------------------------------------------
// Reads all analog and digital pins
// ------------------------------------------------------------
void readPins() {
  int i_counter=0;
  for (int i=0; i<NUM_DIGITAL; i++) {
    input_value[i_counter]=digitalRead(DIGITAL_PINS[i]);
    i_counter++;
  }
  for (int i=0; i<NUM_ANALOG; i++) {
    input_value[i_counter]=analogRead(ANALOG_PINS[i]);
    i_counter++;
  }
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                              SETUP
// ------------------------------------------------------------------------------------------------------------------

void setup() {
  // ------------------------------------------------------------
  // Serial
  // ------------------------------------------------------------
  Serial.setTimeout(50); // ensure this is set before begin()
  Serial.begin(115200);  while(!Serial);
  
  // ------------------------------------------------------------
  // I2C
  // ------------------------------------------------------------
  Wire.begin(SLAVE_ADDR_BUS0); 
  Serial.println("[IIC] Starting IIC as slave address: " + String(SLAVE_ADDR_BUS0));
  // ------------------------------------------------------------
  // Function to run when data requested from master
  // ------------------------------------------------------------
  Wire.onRequest(requestEventBus0Bin);
  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  Wire.onReceive(receiveEventBus0Bin);

  Serial.println("[READY] Waiting for instructions");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                          MAIN LOOP
// ------------------------------------------------------------------------------------------------------------------

void loop() {
  // modulator(); // for output: uncomment if required
  readPins();  // for input: // uncomment if required
}