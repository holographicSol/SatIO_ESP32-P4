/*
  Serial Information Command - Written By Benjamin Jack Cullen
*/

#ifndef SERIAL_INFOCMD_H
#define SERIAL_INFOCMD_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/**
 * @struct Serial0Struct 
 */
struct Serial0Struct {
  unsigned long nbytes; // number of bytes read by serial.
  unsigned long iter_token; // count token iterations.
  char BUFFER[MAX_GLOBAL_SERIAL_BUFFER_SIZE]; // serial buffer.
  char * token; // token pointer.
  char checksum[MAX_CHECKSUM_SIZE];
  uint8_t checksum_of_buffer;
  uint8_t checksum_in_buffer;
  char gotSum[MAX_CHEKSUM_SUM_SIZE];
  int i_XOR;
  int XOR;
  int c_XOR;
};
extern struct Serial0Struct serial0Data;

/**
 * Serial Output.
 * 
 * @brief Checks flags to see if data should be output over a serial port. 
 */
void outputSentences(void);

/**
 * Command Process.
 * 
 * @brief Checks a serial port to see if commands have been recieved. 
 */
void CmdProcess(void);

/**
 * Output Stat.
 * 
 * @brief Prints various infomation for debugging purposes only. 
 */
void outputStat(void);

#ifdef __cplusplus
}
#endif

#endif