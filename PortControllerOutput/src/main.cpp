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

#define SLAVE_ADDR 9 // set address as required.

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
volatile signed int current_pin=-1;
volatile bool multi_read_mode=false;

// ------------------------------------------------------------
// I2C Data
// ------------------------------------------------------------
struct I2CLinkStruct {
  volatile int i_token;
  char         * token;
  volatile byte OUTPUT_BUFFER[32];
  char          INPUT_BUFFER[32];
  char          TMP_BUFFER[32];
};
I2CLinkStruct I2CLink;


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

// ------------------------------------------------------------
// I2C Event: sends binary data to master
// ------------------------------------------------------------
void requestEvent() {
  // if (!multi_read_mode) return;
  float value = (float)input_value[current_pin];   // your original double → float (safe!)
  // Union = fastest way to get raw bytes of a float
  union {
    float    f;
    uint8_t  bytes[4];
  } u;
  u.f = value;
  uint8_t packet[5] = {
    (uint8_t)current_pin,
    u.bytes[0],
    u.bytes[1],
    u.bytes[2],
    u.bytes[3]
  };
  Wire.write(packet, 5);
  if (++current_pin >= NUM_ANALOG + NUM_DIGITAL) {
    current_pin = 0;
    multi_read_mode = false;
  }
}

// ------------------------------------------------------------
// I2C Event: expects binary data from master
// ------------------------------------------------------------
void receiveEvent(int howMany) {
  if (howMany < 1) return;
  uint8_t cmd = Wire.read();
  // Serial.println("cmd " + String(cmd) + " (" + String(howMany) + " bytes)");
  switch (cmd) {
    // ------------------------------------------------------------
    // Instruction: M0
    // ------------------------------------------------------------
    case 0xB0:
    // Serial.println("[Resuest] M0");
      for (int i = 0; i < MAX_MATRIX_SWITCHES; i++) {
        matrix_port_map[i] = -1;
        output_value[i] = 0;
        matrix_modulation_time[i][0] = 0;
        matrix_modulation_time[i][1] = 0;
        matrix_modulation_time[i][2] = 0;
        matrix_modulation_switch_state[i] = false;
      }
      multi_read_mode = false;
      current_pin = -1;
      while (Wire.available()) Wire.read();  // flush
      break;
    // ------------------------------------------------------------
    // Instruction: M1
    // ------------------------------------------------------------
    case 0xB1:
      // Serial.println("[Resuest] M1");
      if (howMany != 13) { while (Wire.available()) Wire.read(); Serial.println("!=12"); return; }
      uint8_t  idx      = Wire.read();
      int8_t   pin      = (int8_t)Wire.read();  // correctly handles -1
      int32_t  value    = (int32_t)(uint32_t)Wire.read() |
                          (int32_t)Wire.read() << 8 |
                          (int32_t)Wire.read() << 16 |
                          (int32_t)Wire.read() << 24;
      uint32_t off_time = (uint32_t)Wire.read() |
                          (uint32_t)Wire.read() << 8 |
                          (uint32_t)Wire.read() << 16 |
                          (uint32_t)Wire.read() << 24;
      uint16_t on_time  = (uint16_t)Wire.read() |
                          (uint16_t)Wire.read() << 8;
      if (idx >= MAX_MATRIX_SWITCHES) return;
      // ------------------------------------------------------------
      // Store
      // ------------------------------------------------------------
      matrix_port_map[idx]           = pin;
      output_value[idx]              = value;
      matrix_modulation_time[idx][0] = off_time;
      matrix_modulation_time[idx][1] = on_time;
      matrix_modulation_time[idx][2] = 0;
      matrix_modulation_switch_state[idx] = false;
      // ------------------------------------------------------------
      // Debug: I2C timeouts may occur if blocking with serial prints 
      // ------------------------------------------------------------
      // Serial.println("matrix_port_map[idx] "           + String(matrix_port_map[idx]));
      // Serial.println("output_value[idx]              " + String(output_value[idx]));
      // Serial.println("matrix_modulation_time[idx][0] " + String(matrix_modulation_time[idx][0]));
      // Serial.println("matrix_modulation_time[idx][1] " + String(matrix_modulation_time[idx][1]));
      // Serial.println("matrix_modulation_time[idx][2] " + String(matrix_modulation_time[idx][2]));
      // Serial.println("matrix_modulation_switch_state[idx] " + String(matrix_modulation_switch_state[idx] ? "true" : "false"));
      // ------------------------------------------------------------
      // Apply
      // ------------------------------------------------------------
      if (pin >= 0) {
        if (isDigitalPin(pin)) {
          pinMode(pin, OUTPUT);
          digitalWrite(pin, (value != 0) ? HIGH : LOW);
        }
        else if (isAnalogPin(pin)) {
          pinMode(pin, OUTPUT);
          analogWrite(pin, constrain(value, 0, 255));
        }
      }
      break;
    // ------------------------------------------------------------
    // Instruction: M2
    // ------------------------------------------------------------
    case 0xB2:  // M2 — Start multi-read mode
    // Serial.println("[Resuest] M2"); 
      multi_read_mode = true;
      current_pin = 0;
      while (Wire.available()) Wire.read();
      break;
    // ------------------------------------------------------------
    // Default case: flush
    // ------------------------------------------------------------
    default:
      while (Wire.available()) Wire.read();
      break;
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
  Wire.begin(SLAVE_ADDR); 
  Serial.println("[IIC] Starting IIC as slave address: " + String(SLAVE_ADDR));
  // ------------------------------------------------------------
  // Function to run when data requested from master
  // ------------------------------------------------------------
  Wire.onRequest(requestEvent);
  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  Wire.onReceive(receiveEvent);

  Serial.println("[READY] Waiting for instructions");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                          MAIN LOOP
// ------------------------------------------------------------------------------------------------------------------

void loop() {
  modulator(); // for output
  // readPins();  // for input
}