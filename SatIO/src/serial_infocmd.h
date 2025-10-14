/*
  Serial Information Command Library.

  Returns information over serial.
  Commands system over serial. 
*/

#ifndef SERIAL_INFOCMD_H
#define SERIAL_INFOCMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

// ----------------------------------------------------------------------------------------
// Serial0Struct.
// ----------------------------------------------------------------------------------------
struct Serial0Struct {
  unsigned long nbytes; // number of bytes read by serial.
  unsigned long iter_token; // count token iterations.
  char BUFFER[MAX_GLOBAL_SERIAL_BUFFER_SIZE]; // serial buffer.
  char * token; // token pointer.
  int collected; // counts how many unique sentences have been collected.
  char checksum[MAX_CHECKSUM_SIZE];
  uint8_t checksum_of_buffer;
  uint8_t checksum_in_buffer;
  char gotSum[MAX_CHEKSUM_SUM_SIZE];
  int i_XOR;
  int XOR;
  int c_XOR;
};
extern struct Serial0Struct serial0Data;

// ----------------------------------------------------------------------------------------
// Function Prototypes.
// ----------------------------------------------------------------------------------------
void outputSentences(void);
void CmdProcess(void);
void outputStat(void);

#ifdef __cplusplus
}
#endif

#endif