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

/** ----------------------------------------------------------------------------
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
            Serial.printf("[I2C] Code %d: Data too long to fit in transmit buffer. Function: %s\n",
                          code,
                          debugTag.c_str());
            break;
            
        case 2:
            Serial.printf("[I2C] Code %d: Received NACK (device not found/not responding). Function: %s\n",
                          code,
                          debugTag.c_str());
            break;
            
        case 3:
            Serial.printf("[I2C] Code %d: Received NACK on data. Function: %s\n",
                          code,
                          debugTag.c_str());
            break;
            
        case 4:
            Serial.printf("[I2C] Code %d: Other/TWI error (e.g. lost arbitration, timeout, etc.). Function: %s\n",
                          code,
                          debugTag.c_str());
            break;
            
        case 5:  // ESP32 / ESP8266 specific
            Serial.printf("[I2C] Code %d: Timeout (ESP32/ESP8266). Function: %s\n",
                          code,
                          debugTag.c_str());
            break;
            
        default:
            Serial.printf("[I2C] Code %d: Unknown error code. Function: %s\n",
                          code,
                          debugTag.c_str());
            break;
    }
}

/** ----------------------------------------------------------------------------
 * @brief Clears the output buffer chars of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputChars(IICLink &iic_link) {
  memset(iic_link.OUTPUT_BUFFER_CHARS, 0, sizeof(iic_link.OUTPUT_BUFFER_CHARS));
}

/** ----------------------------------------------------------------------------
 * @brief Clears the output buffer bytes of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputBytes(IICLink &iic_link) {
  memset(iic_link.OUTPUT_BUFFER_BYTES, 0, sizeof(iic_link.OUTPUT_BUFFER_BYTES));
}

/** ----------------------------------------------------------------------------
 * @brief Clears the input buffer chars of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkInputChars(IICLink &iic_link) {
  memset(iic_link.INPUT_BUFFER, 0, sizeof(iic_link.INPUT_BUFFER));
}

/** ----------------------------------------------------------------------------
 * @brief Writes data to an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void writeI2CToSlaveChars(TwoWire &wire,
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

/** ----------------------------------------------------------------------------
 * @brief Writes data to an I2C master device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param delayMs Delay in milliseconds after writing.
 */
void writeI2CToMasterChars(TwoWire &wire,
                           IICLink &iic_link,
                           long delayMs) {
  clearI2CLinkOutputBytes(iic_link);
  int len = strlen(iic_link.OUTPUT_BUFFER_CHARS);
  for (int i=0;i<len;i++)
  {iic_link.OUTPUT_BUFFER_BYTES[i]=(byte)iic_link.OUTPUT_BUFFER_CHARS[i];}
  wire.write(iic_link.OUTPUT_BUFFER_BYTES, len);
  delay(delayMs); // Allow time for receiving device to process data.
}

/** ----------------------------------------------------------------------------
 * @brief Requests data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param request_id Request ID to send to the slave so that slave knows what is being requested.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void requestFromSlaveChars(TwoWire &wire,
                           IICLink &iic_link,
                           int address,
                           long request_id,
                           size_t len_expected,
                           long delayMs,
                           String debugTag) {
  // Send request ID
  clearI2CLinkOutputChars(iic_link);
  strcpy(iic_link.OUTPUT_BUFFER_CHARS, String(request_id).c_str());
  writeI2CToSlaveChars(wire, iic_link, address, delayMs, debugTag);
  // Uncomment to read potentially old response before requesting new data (may be required)
  clearI2CLinkInputChars(iic_link);
  int len_req = wire.requestFrom(address, len_expected);
  int len_read = wire.readBytes((char *)iic_link.INPUT_BUFFER, len_expected);
  // Serial.printf("[requestFromSlave] %s (%d bytes). Function: %s\n", iic_link.INPUT_BUFFER, len, debugTag.c_str());
  delay(1);
  // Read response
  clearI2CLinkInputChars(iic_link);
  len_req = wire.requestFrom(address, len_expected);
  len_read = wire.readBytes((char *)iic_link.INPUT_BUFFER, len_expected);
  Serial.printf("[requestFromSlave] %s (%d bytes / %d bytes). Function: %s\n",
                iic_link.INPUT_BUFFER,
                len_read,
                len_expected,
                debugTag.c_str());
  if (len_read < 1) {return;}
  if (len_read != len_expected) {
    Serial.printf("[I2C] Warning: Expected %d bytes but received %d bytes from slave. Function: %s\n",
                  len_expected,
                  len_read,
                  debugTag.c_str());
    return;
  }
  // Clear all tokens
  for (int i=0;i<I2C_MAX_TOKENS;i++) {
    memset(iic_link.TOKENS[i], 0, sizeof(iic_link.TOKENS[i]));
  }
  // Tokenize response
  iic_link.i_token=0;
  iic_link.token = strtok(iic_link.INPUT_BUFFER, ",");
  while (iic_link.token != NULL && iic_link.i_token<I2C_MAX_TOKENS) {
    strcpy(iic_link.TOKENS[iic_link.i_token], iic_link.token);
    iic_link.i_token++;
    iic_link.token = strtok(NULL, ",");
  }
  delay(delayMs);
}

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 0.
 * @warning Uncomment and customize to use locally (backup first) or copy into project!
*/
void requestEventBus0Chars() {
  Serial.printf("[requestEventBus0Chars] %d\n", I2CLinkBus0.REQUEST_ID);
  if (I2CLinkBus0.REQUEST_ID==0) {
    memset(I2CLinkBus0.OUTPUT_BUFFER_CHARS, 0, sizeof(I2CLinkBus0.OUTPUT_BUFFER_CHARS));
    strcpy(I2CLinkBus0.OUTPUT_BUFFER_CHARS, "0,value_from_slave");
    writeI2CToMasterChars(iic_0, I2CLinkBus0, 0);
  }
}

/** ----------------------------------------------------------------------------
 * @brief Receive event handler for Bus 0
 * @warning Uncomment and customize to use locally (backup first) or copy into project!
*/
void receiveEventBus0Chars(size_t n_bytes_received) {
  int len = iic_0.readBytes((char *)I2CLinkBus0.INPUT_BUFFER, n_bytes_received);
  if (len < 1) return;
  I2CLinkBus0.INPUT_BUFFER[len] = '\0';
  // Serial.printf("[receiveEventBus0Chars] %s (%d bytes)\n", I2CLinkBus0.INPUT_BUFFER, len);
  // -----------------------------------------------------
  // Check for request ID's (no operation)
  // -----------------------------------------------------
  if (strcmp(I2CLinkBus0.INPUT_BUFFER, "0") == 0) {I2CLinkBus0.REQUEST_ID = 0; return;}
  // -----------------------------------------------------
  // Tokenize input
  // -----------------------------------------------------
  I2CLinkBus0.i_token=0;
  I2CLinkBus0.token = strtok(I2CLinkBus0.INPUT_BUFFER, ",");
  while (I2CLinkBus0.token != NULL) {
    if (I2CLinkBus0.i_token == 0) {/* customize.. */}
    I2CLinkBus0.i_token++;
    I2CLinkBus0.token = strtok(NULL, ",");
  }
}

// now write bytes versions (faster but less human readable)

/** ----------------------------------------------------------------------------
 * @brief Writes binary data to an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param len_packet Length of the packet to write in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void writeI2CToSlaveBin(TwoWire &wire,
                        IICLink &iic_link,
                        int address,
                        long len_packet,
                        long delayMs,
                        String debugTag) {
  
  wire.beginTransmission(address);
  wire.write(iic_link.OUTPUT_PACKET, len_packet);
  printI2CError(wire.endTransmission(), debugTag);
  delay(delayMs); // Allow time for receiving device to process data.
}

/** ----------------------------------------------------------------------------
 * @brief Writes binary data to an I2C master device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param len_packet Length of the packet to write in bytes.
 * @param delayMs Delay in milliseconds after writing.
 */
void writeI2CToMasterBin(TwoWire &wire,
                         IICLink &iic_link,
                         long len_packet,
                         long delayMs) {
  wire.write(iic_link.OUTPUT_PACKET, len_packet);
  delay(delayMs); // Allow time for receiving device to process data.
}

/** ----------------------------------------------------------------------------
 * @brief Requests binary data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param request_id Request ID to send to the slave so that slave knows what is being requested.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
void requestFromSlaveBin(TwoWire &wire,
                         IICLink &iic_link,
                         int address,
                         long len_packet,
                         long request_id,
                         size_t len_expected,
                         long delayMs,
                         String debugTag) {
  // Send request ID
  memset(iic_link.OUTPUT_PACKET, 0, sizeof(iic_link.OUTPUT_PACKET));
  iic_link.OUTPUT_PACKET[0] = (uint8_t)(request_id & 0xFF);
  iic_link.OUTPUT_PACKET[1] = (uint8_t)((request_id >> 8) & 0xFF);
  iic_link.OUTPUT_PACKET[2] = (uint8_t)((request_id >> 16) & 0xFF);
  iic_link.OUTPUT_PACKET[3] = (uint8_t)((request_id >> 24) & 0xFF);
  writeI2CToSlaveBin(wire, iic_link, address, 4, delayMs, debugTag);

  // Send request
  int len_req = wire.requestFrom(address, len_expected);
//   Serial.printf("[requestFromSlaveBin] received %d bytes. Function: %s\n",
//                 len_req,
//                 debugTag.c_str());
  
  // Check response length
  if (len_req != len_expected) {
    Serial.printf("[requestFromSlaveBin] Warning: Expected %d bytes. Function: %s\n",
                  len_expected,
                  debugTag.c_str());
    return;
  }

  // Read response
  memset(iic_link.INPUT_PACKET, 0, sizeof(iic_link.INPUT_PACKET));
  for (int i=0;i<len_req;i++) {iic_link.INPUT_PACKET[i] = wire.read();}
  
  // Debug
//   Serial.printf("[requestFromSlaveBin] bin: ");
//   for (int i = 0; i < len_req; i++) {Serial.printf("%02X ", iic_link.INPUT_PACKET[i]);}
//   Serial.printf("Function: %s\n", debugTag.c_str());

  delay(delayMs);
}

/** ----------------------------------------------------------------------------
 * @brief Requests binary data from an I2C slave device.
 * @param wire Specify TwoWire instance.
 * @param iic_link Specify IICLink instance.
 * @param address I2C address of the slave device.
 * @param len_expected Expected length of the response in bytes.
 * @param delayMs Delay in milliseconds after writing.
 * @param debugTag Tag to identify the source of the error (recommend using caller function name).
 */
bool requestFromSlaveBinNoID(TwoWire &wire,
                         IICLink &iic_link,
                         int address,
                         size_t len_expected,
                         long delayMs,
                         String debugTag) {
  // Send request
  int len_req = wire.requestFrom(address, len_expected);
//   Serial.printf("[requestFromSlaveBin] received %d bytes. Function: %s\n",
//                 len_req,
//                 debugTag.c_str());
  
  // Check response length
  if (len_req != len_expected) {
    Serial.printf("[requestFromSlaveBin] Warning: Expected %d bytes. Function: %s\n",
                  len_expected,
                  debugTag.c_str());
    return false;
  }
  delay(delayMs);
  return true;
}

/**
 * @brief Read uint8_t from I2C wire into specified value.
 * @warning Specified value is expected to be uint8_t.
 */
void read_uint8_FromWire(TwoWire &wire, uint8_t &value) {
  value = wire.read(); 
}

/**
 * @brief Read int8_t from I2C wire into specified value.
 * @warning Specified value is expected to be int8_t.
 */
void read_int8_FromWire(TwoWire &wire, int8_t &value) {
  value = (int8_t)wire.read();
}

/**
 * @brief Read uint16_t from I2C wire into specified value (2 bytes, little-endian).
 * @warning Specified value is expected to be uint16_t.
 */
void read_uint16_FromWire(TwoWire &wire, uint16_t &value) {
  union { uint16_t u; uint8_t bytes[2]; } un;
  un.bytes[0] = wire.read();
  un.bytes[1] = wire.read();
  value = un.u;
}

/**
 * @brief Read int16_t from I2C wire into specified value (2 bytes, little-endian).
 * @warning Specified value is expected to be int16_t.
 */
void read_int16_FromWire(TwoWire &wire, int16_t &value) {
  union { int16_t i; uint8_t bytes[2]; } un;
  un.bytes[0] = wire.read();
  un.bytes[1] = wire.read();
  value = un.i;
}

/**
 * @brief Read uint32_t from I2C wire into specified value (4 bytes, little-endian).
 * @warning Specified value is expected to be uint32_t.
 */
void read_uint32_FromWire(TwoWire &wire, uint32_t &value) {
  union { uint32_t u; uint8_t bytes[4]; } un;
  un.bytes[0] = wire.read();
  un.bytes[1] = wire.read();
  un.bytes[2] = wire.read();
  un.bytes[3] = wire.read();
  value = un.u;
}

/**
 * @brief Read int32_t from I2C wire into specified value (4 bytes, little-endian).
 * @warning Specified value is expected to be int32_t.
 */
void read_int32_FromWire(TwoWire &wire, int32_t &value) {
  union { int32_t i; uint8_t bytes[4]; } un;
  un.bytes[0] = wire.read();
  un.bytes[1] = wire.read();
  un.bytes[2] = wire.read();
  un.bytes[3] = wire.read();
  value = un.i;
}

/**
 * @brief Read uint64_t from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be uint64_t.
 */
void read_uint64_FromWire(TwoWire &wire, uint64_t &value) {
  union { uint64_t u; uint8_t bytes[8]; } un;
  for (int i = 0; i < 8; i++) un.bytes[i] = wire.read();
  value = un.u;
}

/**
 * @brief Read int64_t from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be int64_t.
 */
void read_int64_FromWire(TwoWire &wire, int64_t &value) {
  union { int64_t i; uint8_t bytes[8]; } un;
  for (int i = 0; i < 8; i++) un.bytes[i] = wire.read();
  value = un.i;
}

/**
 * @brief Read float from I2C wire into specified value (4 bytes, little-endian).
 * @warning Specified value is expected to be float.
 */
void read_float_FromWire(TwoWire &wire, float &value) {
  union { float f; uint8_t bytes[4]; } u;
  u.bytes[0] = wire.read();
  u.bytes[1] = wire.read();
  u.bytes[2] = wire.read();
  u.bytes[3] = wire.read();
  value = u.f;
}

/**
 * @brief Read double from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be double.
 */
void read_double_FromWire(TwoWire &wire, double &value) {
  union { double d; uint8_t bytes[8]; } u;
  for (int i = 0; i < 8; i++) u.bytes[i] = wire.read();
  value = u.d;
}

/**
 * @brief Read long from I2C wire into specified value (little-endian).
 * @warning Specified value is expected to be long.
 */
void read_long_FromWire(TwoWire &wire, long &value) {
  union { long l; uint8_t bytes[sizeof(long)]; } un;
  for (int i = 0; i < sizeof(long); i++) un.bytes[i] = wire.read();
  value = un.l;
}

/**
 * @brief Read long long from I2C wire into specified value (8 bytes, little-endian).
 * @warning Specified value is expected to be long long.
 */
void read_longlong_FromWire(TwoWire &wire, long long &value) {
  union { long long ll; uint8_t bytes[8]; } un;
  for (int i = 0; i < 8; i++) un.bytes[i] = wire.read();
  value = un.ll;
}

/**
 * @brief Read char from I2C wire into specified value.
 * @warning Specified value is expected to be char.
 */
void read_char_FromWire(TwoWire &wire, char &value) {
  value = (char)wire.read();
}

/**
 * @brief Read N chars from I2C wire into specified char array.
 * @param wire Specify TwoWire instance.
 * @param value Pointer to char array to store the read values.
 * @param n_chars Number of chars to read.
 * @warning Specified value is expected to be a char array with at least n_chars size.
 * @warning Ensure the char array is large enough to hold n_chars values.
 */
void read_nchars_FromWire(TwoWire &wire, char *value, size_t n_chars) {
  for (size_t i = 0; i < n_chars; i++) {
    value[i] = (char)wire.read();
  }
}

/**
 * @brief Read bool from I2C wire into specified value.
 * @warning Specified value is expected to be bool.
 */
void read_bool_FromWire(TwoWire &wire, bool &value) {
  value = (bool)wire.read();
}

/**
 * @brief Read byte from I2C wire into specified value.
 * @warning Specified value is expected to be byte.
 */
void read_byte_FromWire(TwoWire &wire, byte &value) {
  value = wire.read();
}