/*
  I2C Helper Functions - Written By Benjamin Jack Cullen.

  Intends to standardize I2C communication functions across
  multiple I2C buses, devices, and across multiple projects.
*/

#include <Arduino.h>
#include <Wire.h>
#include "i2c_helper.h"

TwoWire iic_0(0);
TwoWire iic_1(1);
TwoWire iic_2(2);

IICLink I2CLinkBus0;
IICLink I2CLinkBus1;
IICLink I2CLinkBus2;

/**
 * Prints a human-readable description of Wire.endTransmission() error codes
 * 
 * @param code The return value from Wire.endTransmission()
 * @param debugTag Optional tag to identify the source of the error
 */
void printI2CError(uint8_t code, String debugTag) {    
    switch (code) {
        case 0:
            // Serial.println("[I2C] Code 0: Success");
            break;
            
        case 1:
            Serial.printf("[I2C] Code %d: Data too long to fit in transmit buffer. sender: %s\n", code, debugTag.c_str());
            break;
            
        case 2:
            Serial.printf("[I2C] Code %d: Received NACK (device not found/not responding). Sender: %s\n", code, debugTag.c_str());
            break;
            
        case 3:
            Serial.printf("[I2C] Code %d: Received NACK on data. Sender: %s\n", code, debugTag.c_str());
            break;
            
        case 4:
            Serial.printf("[I2C] Code %d: Other/TWI error (e.g. lost arbitration, timeout, etc.). Sender: %s\n", code, debugTag.c_str());
            break;
            
        case 5:  // ESP32 / ESP8266 specific
            Serial.printf("[I2C] Code %d: Timeout (ESP32/ESP8266). Sender: %s\n", code, debugTag.c_str());
            break;
            
        default:
            Serial.printf("[I2C] Code %d: Unknown error code. Sender: %s\n", code, debugTag.c_str());
            break;
    }
}

/* Clear output buffer chars */
void clearI2CLinkOutputChars(IICLink &iic_link) {
  memset(iic_link.OUTPUT_BUFFER_CHARS, 0, sizeof(iic_link.OUTPUT_BUFFER_CHARS));
}

/* Clear output buffer bytes */
void clearI2CLinkOutputBytes(IICLink &iic_link) {
  memset(iic_link.OUTPUT_BUFFER_BYTES, 0, sizeof(iic_link.OUTPUT_BUFFER_BYTES));
}

/* Clear intput buffer chars */
void clearI2CLinkInputChars(IICLink &iic_link) {
  memset(iic_link.INPUT_BUFFER, 0, sizeof(iic_link.INPUT_BUFFER));
}

/* Transmit to slave */
void writeI2CToSlave(TwoWire &wire,
                     IICLink &iic_link,
                     int address,
                     long delayMs,
                     String debugTag) {
  clearI2CLinkOutputBytes(iic_link);
  int len = strlen(iic_link.OUTPUT_BUFFER_CHARS);
  for (int i=0;i<len;i++)
  {iic_link.OUTPUT_BUFFER_BYTES[i]=(byte)iic_link.OUTPUT_BUFFER_CHARS[i];}
  wire.beginTransmission(address);
  wire.write(iic_link.OUTPUT_BUFFER_BYTES, len);
  printI2CError(wire.endTransmission(), debugTag);
  delay(delayMs); // Allow time for receiving device to process data.
}

/* Transmit to master */
void writeI2CToMaster(TwoWire &wire,
                      IICLink &iic_link,
                      long delayMs) {
  clearI2CLinkOutputBytes(iic_link);
  int len = strlen(iic_link.OUTPUT_BUFFER_CHARS);
  for (int i=0;i<len;i++)
  {iic_link.OUTPUT_BUFFER_BYTES[i]=(byte)iic_link.OUTPUT_BUFFER_CHARS[i];}
  wire.write(iic_link.OUTPUT_BUFFER_BYTES, len);
  delay(delayMs); // Allow time for receiving device to process data.
}

/* Request from slave */
void requestFromSlave(TwoWire &wire,
                     IICLink &iic_link,
                     int address,
                     long request_id,
                     size_t len_expected,
                     long delayMs,
                     String debugTag) {
  // Send request ID
  clearI2CLinkOutputChars(iic_link);
  strcpy(iic_link.OUTPUT_BUFFER_CHARS, String(request_id).c_str());
  writeI2CToSlave(wire, iic_link, address, delayMs, debugTag);
  
  // Uncomment to read potentially old response before requesting new data
  clearI2CLinkInputChars(iic_link);
  int len_req = wire.requestFrom(address, len_expected);
  int len_read = wire.readBytes((char *)iic_link.INPUT_BUFFER, len_expected);
  // Serial.printf("[requestFromSlave] %s (%d bytes). Sender: %s\n", iic_link.INPUT_BUFFER, len, debugTag.c_str());

  delay(1);

  // Read response
  clearI2CLinkInputChars(iic_link);
  len_req = wire.requestFrom(address, len_expected);
  len_read = wire.readBytes((char *)iic_link.INPUT_BUFFER, len_expected);
  Serial.printf("[requestFromSlave] %s (%d bytes / %d bytes). Sender: %s\n", iic_link.INPUT_BUFFER, len_read, len_expected, debugTag.c_str());
  if (len_read < 1) {return;}
  if (len_read != len_expected) {
    Serial.printf("[I2C] Warning: Expected %d bytes but received %d bytes from slave. Sender: %s\n", len_expected, len_read, debugTag.c_str());
    return;
  }

  // Clear all tokens
  for (int i=0;i<MAX_TOKENS;i++) {
    memset(iic_link.TOKENS[i], 0, sizeof(iic_link.TOKENS[i]));
  }

  // Tokenize response
  iic_link.i_token=0;
  iic_link.token = strtok(iic_link.INPUT_BUFFER, ",");
  while (iic_link.token != NULL) {
    strcpy(iic_link.TOKENS[iic_link.i_token], iic_link.token);
    iic_link.i_token++;
    iic_link.token = strtok(NULL, ",");
  }
  delay(delayMs);
}

/* Receive event handler for Bus 0 (INCOMPLETE, PENDING CONSIDERATION) */
void receiveEventBus0(size_t n_bytes_received) {
  int len = iic_0.readBytes((char *)I2CLinkBus0.INPUT_BUFFER, n_bytes_received);
  if (len < 1) return;
  I2CLinkBus0.INPUT_BUFFER[len] = '\0';
  I2CLinkBus0.i_token=0;
  I2CLinkBus0.token = strtok(I2CLinkBus0.INPUT_BUFFER, ",");
  while (I2CLinkBus0.token != NULL) {
    if (I2CLinkBus0.i_token == 0) {I2CLinkBus0.REQUEST_ID = atol(I2CLinkBus0.token);}
    // customize...
    I2CLinkBus0.i_token++;
    I2CLinkBus0.token = strtok(NULL, ",");
  }
}

/* Request event handler for Bus 0 (INCOMPLETE, PENDING CONSIDERATION) */
void requestEventBus0() {
  if (I2CLinkBus0.REQUEST_ID==0) {
    memset(I2CLinkBus0.OUTPUT_BUFFER_CHARS, 0, sizeof(I2CLinkBus0.OUTPUT_BUFFER_CHARS));
    strcpy(I2CLinkBus0.OUTPUT_BUFFER_CHARS, "foobar");
    writeI2CToMaster(iic_0, I2CLinkBus0, 0);
  }
}