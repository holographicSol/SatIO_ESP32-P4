/*
Written by Benjamin Jack Cullen.

PortController - Receives messages over IIC and manipulates ports accordingly.
                 This software should be flashed to ATMEGA2560.

                 Modulation is currently done on the here but modulation values may be forwarded to
                 external MCU's on each IO in the future.
*/

#include <Arduino.h>
#include <stdio.h>
#include <string.h>
#include <limits.h>
#include <stdlib.h>
#include <Wire.h>

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    MATRIX DATA
// ------------------------------------------------------------------------------------------------------------------------------

signed int port_index;

// ------------------------------------------------------------
// Set expected matrix switch output pin number range
// Strongly recommended to stay within this range unless add
// further handling on setup(), clearMatrixSwitch(), etc.
// ------------------------------------------------------------
#define PIN_MIN 2
#define PIN_MAX 69
#define MAX_MATRIX_SWITCHES 70

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

int output_value[MAX_MATRIX_SWITCHES]={
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

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                    TIME STRUCT
// ------------------------------------------------------------------------------------------------------------------------------

struct TimeStruct {
  double mainLoopTimeTaken; // current main loop time
  unsigned long mainLoopTimeStart; // time recorded at the start of each iteration of main loop
};
TimeStruct timeData;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                       I2C DATA
// ------------------------------------------------------------------------------------------------------------------------------

#define SLAVE_ADDR 9

struct I2CLinkStruct {
  int    i_token;
  char * token;
  byte   OUTPUT_BUFFER[32];
  char   INPUT_BUFFER[32];
  char   TMP_BUFFER[32];
};
I2CLinkStruct I2CLink;

// ------------------------------------------------------------------------------------------------------------------------------
//                                                                                                                     I2C EVENTS
// ------------------------------------------------------------------------------------------------------------------------------

long t0=micros();

void print1PortStat(int idx) {
  Serial.println(
    "[IDX " + String(idx) + "] " +
    "[PIN " + String(matrix_port_map[idx]) + "] " +
    "[H/L " + String(output_value[idx]) + "] " +
    "[T0 "  + String(matrix_modulation_time[idx][0]) + "] " +
    "[T1 "  + String(matrix_modulation_time[idx][1]) + "] "
  );
}

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

void receiveEvent(int) {
  // ------------------------------------------------------------
  // Read incoming data
  // ------------------------------------------------------------
  memset(I2CLink.INPUT_BUFFER, 0, sizeof(I2CLink.INPUT_BUFFER));
  Wire.readBytesUntil('\n', I2CLink.INPUT_BUFFER, sizeof(I2CLink.INPUT_BUFFER));
  Serial.println("[RCV] " + String(I2CLink.INPUT_BUFFER));
  // ------------------------------------------------------------
  // Tokenize data tag
  // ------------------------------------------------------------
  I2CLink.token = strtok(I2CLink.INPUT_BUFFER, ",");
  // ------------------------------------------------------------
  // Mode 0: Clear existing
  // ------------------------------------------------------------
  if (strcmp(I2CLink.INPUT_BUFFER, "M0")==0) {clearMatrixSwitch();}
  // ------------------------------------------------------------
  // Mode 1: Set new
  // ------------------------------------------------------------
  if (strncmp(I2CLink.INPUT_BUFFER, "M1", strlen("M1"))==0) {
    // Serial.println("[size]" + String(sizeof(I2CLink.INPUT_BUFFER)));
    // -----------------------
    // index
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[IDX] " + String(I2CLink.token));
    port_index = atoi(I2CLink.token);
    // -----------------------
    // port
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[PIN] " + String(I2CLink.token));
    matrix_port_map[port_index] = atoi(I2CLink.token);
    // -----------------------
    // state
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[V] " + String(I2CLink.token));
    output_value[port_index] = atoi(I2CLink.token);
    // -----------------------
    // off time micros
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[T0] " + String(I2CLink.token));
    matrix_modulation_time[port_index][0]=atol(I2CLink.token);
    // -----------------------
    // on time micros
    // -----------------------
    I2CLink.token = strtok(NULL, ",");
    // Serial.println("[T1] " + String(I2CLink.token));
    matrix_modulation_time[port_index][1]=atol(I2CLink.token);
    // ----------------------------------------------------------
    // reset previous modulation time (experiment with behaviour)
    // ----------------------------------------------------------
    // matrix_modulation_time[port_index][2]=0;
    // ----------------------------------------------------------
    // set output value (experiment with behaviour)
    // ----------------------------------------------------------
    // try to avoid high/low spam (to be modulator friendly)
    // ----------------------------------------------------------
    
    if (matrix_port_map[port_index]<54) {
      int current_reading=digitalRead(matrix_port_map[port_index]);
      if ( (current_reading==0) && (output_value[port_index]==1) ) {
        digitalWrite(matrix_port_map[port_index], HIGH);
        matrix_modulation_time[port_index][2]=0;
      }
      else if ( (current_reading==1) && (output_value[port_index]==0) ) {
        digitalWrite(matrix_port_map[port_index], LOW);
        matrix_modulation_time[port_index][2]=0;
      }
      else {
        // Serial.println("digital: ignoring spam");
      }
    }
    else {
      // int current_reading=analogRead(matrix_port_map[port_index]);
      // Serial.println(current_reading);
      // Serial.println(output_value[port_index]);
      analogWrite(matrix_port_map[port_index], output_value[port_index]);
      // Serial.println(current_reading);
    }
  }
}

void requestEvent() {
  // ------------------------------------------------------------
  // Write Bytes of Chars
  // ------------------------------------------------------------
  memset(I2CLink.OUTPUT_BUFFER, 0, sizeof(I2CLink.OUTPUT_BUFFER));
  for (byte i=0;i<sizeof(I2CLink.OUTPUT_BUFFER);i++) {I2CLink.OUTPUT_BUFFER[i] = (byte)I2CLink.TMP_BUFFER[i];}
  Wire.write(I2CLink.OUTPUT_BUFFER, sizeof(I2CLink.OUTPUT_BUFFER));
}

void PrintCheckRead(int i) {
  if (matrix_port_map[i]<54) {Serial.println("digital read: " + String(digitalRead(matrix_port_map[i])));}
  else {Serial.println("analog read: " + String(analogRead(matrix_port_map[i])));}
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
            // PrintCheckRead(i);
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
              // PrintCheckRead(i);
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
              // PrintCheckRead(i);
              matrix_modulation_time[i][2]=micros();
              matrix_modulation_switch_state[i]=false;
            }
          }
        }
      }
    }
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
  // Matrix switches: avoid portmap on startup
  // ------------------------------------------------------------
  for (int i=PIN_MIN; i<PIN_MAX; i++) {
    pinMode(i, OUTPUT);
    if (i<54) {digitalWrite(matrix_port_map[i], LOW);}
    else {analogWrite(matrix_port_map[i], 0);}
  }
  
  // ----------------------------------------------------------------------------------------------------------------------------
  // I2C
  // ----------------------------------------------------------------------------------------------------------------------------
  Wire.begin(SLAVE_ADDR);
  Serial.println("[IIC] starting IIC as slave address: " + String(SLAVE_ADDR));
  // Serial.println("[IIC] setting IIC clock: 400kHz (400000L)");
  Wire.setClock(400000L);
  delay(1000);
  // Wire.setClock(100000L);

  // ------------------------------------------------------------
  // Function to run when data requested from master
  // ------------------------------------------------------------
  Wire.onRequest(requestEvent);

  // ------------------------------------------------------------
  // Function to run when data received from master
  // ------------------------------------------------------------
  Wire.onReceive(receiveEvent);

  Serial.println("[READY] waiting for instructions");
}

// ------------------------------------------------------------------------------------------------------------------
//                                                                                                          MAIN LOOP
// ------------------------------------------------------------------------------------------------------------------

void loop() {
  // ------------------------------------------------------------
  // uncomment to debug
  // ------------------------------------------------------------
  // Serial.println("---------------------------------------");
  // Serial.println("[loop] ");
  // timeData.mainLoopTimeStart = millis();  // store current time to measure this loop time

  modulator();
  // delayMicroseconds(1000);
  // delay(1);

  // ------------------------------------------------------------
  // uncomment to debug
  // ------------------------------------------------------------
  // timeData.mainLoopTimeTaken = millis() - timeData.mainLoopTimeStart;  // store time taken to complete
  // Serial.print("[looptime] "); Serial.println(timeData.mainLoopTimeTaken);
  // delayMicroseconds(250);
}