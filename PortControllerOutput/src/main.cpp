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

volatile int output_value[MAX_MATRIX_SWITCHES]={
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
};

volatile int input_value[MAX_MATRIX_SWITCHES]={
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 0-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 10-19
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 20-29
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 30-9
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 40-49
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 50-59
  0, 0, 0, 0, 0, 0, 0, 0, 0, 0, // 60-69
};

// ------------------------------------------------------------
// keep track of intended high/low state for modulation
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
// Current pin reading
// ------------------------------------------------------------
volatile int current_input_value=0;
volatile signed int current_pin=-1;
volatile bool multi_read_mode=false;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA
// ------------------------------------------------------------------------------------------------------------------------------
struct I2CLinkStruct {
  volatile int i_token;
  char         * token;
  volatile byte OUTPUT_BUFFER[32];
  char          INPUT_BUFFER[32];
  char          TMP_BUFFER[32];
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS
// ------------------------------------------------------------------------------------------------------------------------------
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

void requestEvent() {
  // if (multi_read_mode) {
    memset(I2CLink.TMP_BUFFER, 0, sizeof(I2CLink.TMP_BUFFER));
    strcpy(I2CLink.TMP_BUFFER, String(String(current_pin) + "," + String(input_value[current_pin])).c_str());
    Wire.write((uint8_t*)I2CLink.TMP_BUFFER, sizeof(I2CLink.TMP_BUFFER));
    current_pin++;
    if (current_pin >= NUM_ANALOG+NUM_DIGITAL) {
        current_pin = 0; // Reset pin index for next 'M' command
        multi_read_mode = false; // Turn off the mode after the last pin
    }
  // }
}

void receiveEvent(int) {
  // ------------------------------------------------------------
  // Read incoming data
  // ------------------------------------------------------------
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  // Serial.println("[RCV] " + String(I2CLink.INPUT_BUFFER));
  // ------------------------------------------------------------
  // Tokenize data tag
  // ------------------------------------------------------------
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");
  // ------------------------------------------------------------
  // Mode 0: Clear existing
  // ------------------------------------------------------------
  if (strcmp(I2CLink.INPUT_BUFFER, "M0")==0) {clearMatrixSwitch(); multi_read_mode=false;}
  // ------------------------------------------------------------
  /*
    Mode 1: Write Output.
    (1) Read instrunction from master.
    (2) Store instruction.
    (3) Write to pin.
    Instruction from master: M1,INDEX,PIN,OUTPUT_VALUE,OFF_uS_TIME,ON_TIME_uS
    Instruction to master:   Currently none.
  */
  // ------------------------------------------------------------
  else if (strncmp(I2CLink.INPUT_BUFFER, "M1", strlen("M1"))==0) {
    multi_read_mode=false;
    // -----------------------
    // index
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    port_index = atoi(I2CLink.token);
    // -----------------------
    // pin
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    matrix_port_map[port_index] = atoi(I2CLink.token);
    // -----------------------
    // output value
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    output_value[port_index] = atoi(I2CLink.token);
    // -----------------------
    // off time micros
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    matrix_modulation_time[port_index][0]=atol(I2CLink.token);
    // -----------------------
    // on time micros
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    matrix_modulation_time[port_index][1]=atol(I2CLink.token);
    // -----------------------
    // Digital
    // -----------------------
    if (isDigitalPin(matrix_port_map[port_index])) {
      // Serial.println("Digital Pin:");
      current_input_value=digitalRead(matrix_port_map[port_index]);
      if ( (current_input_value==1) && (output_value[port_index]==0) ) {
        // Serial.println("Analog Pin:");
        pinMode(matrix_port_map[port_index], OUTPUT); // new
        digitalWrite(matrix_port_map[port_index], LOW);
        matrix_modulation_time[port_index][2]=0;
      }
      else if ( (current_input_value==0) && (output_value[port_index]==1) ) {
        pinMode(matrix_port_map[port_index], OUTPUT); // new
        digitalWrite(matrix_port_map[port_index], HIGH);
        matrix_modulation_time[port_index][2]=0;
      }
      // else ignore unecessary
    }
    // -----------------------
    // Analog
    // -----------------------
    else if (isAnalogPin(matrix_port_map[port_index])) {
      pinMode(matrix_port_map[port_index], OUTPUT); // new
      analogWrite(matrix_port_map[port_index], output_value[port_index]);
    }
  }
  // ------------------------------------------------------------
  /*
    Mode 2: Read Input.
    (1) Read instrunction from master.
    (2) Read pin.
    (3) Send value to master.
    Instruction from master: M2,PIN
    Instruction to master:   M2,PIN,VALUE
  */
  // ------------------------------------------------------------
  else if (strncmp(I2CLink.INPUT_BUFFER, "M2", strlen("M2"))==0) {
    multi_read_mode=true;
  }
}

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
  readPins();  // for input
}