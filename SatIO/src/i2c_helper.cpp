/*
  I2C Helper Functions - Written By Benjamin Jack Cullen.

  Intends to standardize I2C communication functions across
  multiple I2C buses, devices, and across multiple projects.

  Reading & writing in binary over I2C is faster than sending bytes of char
  arrays but less human readable, therefore this library also intends to make
  r/w binary data over I2C both fast and human readable from a high/project level.

  Includes binary packet building functions for:
    int8 to int64.
    uint8 to uint64.
    long, long long.
    float, double.
    bool.
    char, nchars.
    byte, nbytes.

  Includes binary packet reading functions for all of the above mentioned types.
*/

#include <Arduino.h>
#include <Wire.h>
#include "driver/i2c_slave.h"
#include "esp_err.h"
#include "i2c_helper.h"

TwoWire iic_0(0);
TwoWire iic_1(1);
TwoWire iic_2(2);

IICLink I2CLinkBus0;
IICLink I2CLinkBus1;
IICLink I2CLinkBus2;

// #####################################################################################################################
// ## BEGIN WIRE I2C SLAVE REQUEST CHAR BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 0
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void requestEventBus0Chars() {
//   Serial.printf("[requestEventBus0Chars] %d\n", I2CLinkBus0.REQUEST_ID);
//   if (I2CLinkBus0.REQUEST_ID==0) {
//     memset(I2CLinkBus0.OUTPUT_BUFFER_CHARS, 0, sizeof(I2CLinkBus0.OUTPUT_BUFFER_CHARS));
//     strcpy(I2CLinkBus0.OUTPUT_BUFFER_CHARS, "0,value_from_slave");
//     writeI2CToMasterChars(iic_0, I2CLinkBus0, 0);
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 1
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void requestEventBus1Chars() {
//   Serial.printf("[requestEventBus1Chars] %d\n", I2CLinkBus1.REQUEST_ID);
//   if (I2CLinkBus1.REQUEST_ID==0) {
//     memset(I2CLinkBus1.OUTPUT_BUFFER_CHARS, 0, sizeof(I2CLinkBus1.OUTPUT_BUFFER_CHARS));
//     strcpy(I2CLinkBus1.OUTPUT_BUFFER_CHARS, "0,value_from_slave");
//     writeI2CToMasterChars(iic_1, I2CLinkBus1, 0);
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 2
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void requestEventBus2Chars() {
//   Serial.printf("[requestEventBus1Chars] %d\n", I2CLinkBus2.REQUEST_ID);
//   if (I2CLinkBus2.REQUEST_ID==0) {
//     memset(I2CLinkBus2.OUTPUT_BUFFER_CHARS, 0, sizeof(I2CLinkBus2.OUTPUT_BUFFER_CHARS));
//     strcpy(I2CLinkBus2.OUTPUT_BUFFER_CHARS, "0,value_from_slave");
//     writeI2CToMasterChars(iic_2, I2CLinkBus2, 0);
//   }
// }

// #####################################################################################################################
// ## BEGIN WIRE I2C SLAVE RECEIVE CHAR BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief Receive event handler for Bus 0
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void receiveEventBus0Chars(size_t n_bytes_received) {
//   int len = iic_0.readBytes((char *)I2CLinkBus0.INPUT_BUFFER, n_bytes_received);
//   if (len < 1) return;
//   I2CLinkBus0.INPUT_BUFFER[len] = '\0';
//   Serial.printf("[receiveEventBus0Chars] %s (%d bytes)\n", I2CLinkBus0.INPUT_BUFFER, len);
//   // -----------------------------------------------------
//   // Check for request ID's (no operation)
//   // -----------------------------------------------------
//   if (strcmp(I2CLinkBus0.INPUT_BUFFER, "0") == 0) {I2CLinkBus0.REQUEST_ID = 0; return;}
//   // -----------------------------------------------------
//   // Tokenize input
//   // -----------------------------------------------------
//   I2CLinkBus0.i_token=0;
//   I2CLinkBus0.token = strtok(I2CLinkBus0.INPUT_BUFFER, ",");
//   while (I2CLinkBus0.token != NULL) {
//     if (I2CLinkBus0.i_token == 0) {/* customize.. */}
//     I2CLinkBus0.i_token++;
//     I2CLinkBus0.token = strtok(NULL, ",");
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Receive event handler for Bus 1
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void receiveEventBus1Chars(size_t n_bytes_received) {
//   int len = iic_1.readBytes((char *)I2CLinkBus1.INPUT_BUFFER, n_bytes_received);
//   if (len < 1) return;
//   I2CLinkBus1.INPUT_BUFFER[len] = '\0';
//   Serial.printf("[receiveEventBus1Chars] %s (%d bytes)\n", I2CLinkBus1.INPUT_BUFFER, len);
//   // -----------------------------------------------------
//   // Check for request ID's (no operation)
//   // -----------------------------------------------------
//   if (strcmp(I2CLinkBus1.INPUT_BUFFER, "0") == 0) {I2CLinkBus1.REQUEST_ID = 0; return;}
//   // -----------------------------------------------------
//   // Tokenize input
//   // -----------------------------------------------------
//   I2CLinkBus1.i_token=0;
//   I2CLinkBus1.token = strtok(I2CLinkBus1.INPUT_BUFFER, ",");
//   while (I2CLinkBus1.token != NULL) {
//     if (I2CLinkBus1.i_token == 0) {/* customize.. */}
//     I2CLinkBus1.i_token++;
//     I2CLinkBus1.token = strtok(NULL, ",");
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Receive event handler for Bus 2
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void receiveEventBus2Chars(size_t n_bytes_received) {
//   int len = iic_2.readBytes((char *)I2CLinkBus2.INPUT_BUFFER, n_bytes_received);
//   if (len < 1) return;
//   I2CLinkBus2.INPUT_BUFFER[len] = '\0';
//   Serial.printf("[receiveEventBus2Chars] %s (%d bytes)\n", I2CLinkBus2.INPUT_BUFFER, len);
//   // -----------------------------------------------------
//   // Check for request ID's (no operation)
//   // -----------------------------------------------------
//   if (strcmp(I2CLinkBus2.INPUT_BUFFER, "0") == 0) {I2CLinkBus2.REQUEST_ID = 0; return;}
//   // -----------------------------------------------------
//   // Tokenize input
//   // -----------------------------------------------------
//   I2CLinkBus2.i_token=0;
//   I2CLinkBus2.token = strtok(I2CLinkBus2.INPUT_BUFFER, ",");
//   while (I2CLinkBus2.token != NULL) {
//     if (I2CLinkBus2.i_token == 0) {/* customize.. */}
//     I2CLinkBus2.i_token++;
//     I2CLinkBus2.token = strtok(NULL, ",");
//   }
// }

// #####################################################################################################################
// ## BEGIN WIRE I2C SLAVE REQUEST BINARY BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief Request binary event handler for Bus 0
 * @warning Customize to use locally (backup first) or copy into project!
 */
// void requestEventBus0Bin() {
//   Serial.println("[requestEventBus0Bin] id: " + String(I2CLinkBus0.REQUEST_ID));
//   switch (I2CLinkBus0.REQUEST_ID) {
//     case 0x00: {
//         Serial.println("[requestEventBus0Bin] preparing to send requested data: " + String());
//         break;
//     }
//     default: {
//         Serial.println("[requestEventBus0Bin] event id is not defined: " + String(I2CLinkBus0.REQUEST_ID));
//         break;
//     }
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Request binary event handler for Bus 1
 * @warning Customize to use locally (backup first) or copy into project!
 */
// void requestEventBus1Bin() {
//   Serial.println("[requestEventBus1Bin] id: " + String(I2CLinkBus1.REQUEST_ID));
//   switch (I2CLinkBus1.REQUEST_ID) {
//     case 0x00: {
//         Serial.println("[requestEventBus1Bin] preparing to send requested data: " + String());
//         break;
//     }
//     default: {
//         Serial.println("[requestEventBus1Bin] event id is not defined: " + String(I2CLinkBus1.REQUEST_ID));
//         break;
//     }
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Request binary event handler for Bus 2
 * @warning Customize to use locally (backup first) or copy into project!
 */
// void requestEventBus2Bin() {
//   Serial.println("[requestEventBus2Bin] id: " + String(I2CLinkBus2.REQUEST_ID));
//   switch (I2CLinkBus2.REQUEST_ID) {
//     case 0x00: {
//         Serial.println("[requestEventBus2Bin] preparing to send requested data: " + String());
//         break;
//     }
//     default: {
//         Serial.println("[requestEventBus2Bin] event id is not defined: " + String(I2CLinkBus2.REQUEST_ID));
//         break;
//     }
//   }
// }

// #####################################################################################################################
// ## BEGIN WIRE I2C SLAVE RECEIVE BINARY BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief Receive binary event handler for Bus 0
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void receiveEventBus0Bin(size_t n_bytes_received) {
//   if (n_bytes_received < 1) return;
//   uint8_t cmd = Wire.read(); // expects uint8 command byte (up to 255 unique commands can be accepted). 
//   Serial.println("[receiveEventBus0Bin] " + String(cmd) + " (" + String(n_bytes_received) + " bytes)");
//   switch (cmd) {
//     case 0x00: {
//         Serial.println("[requestEvent] preparing to process command: " + String(cmd));
//         break;
//     }
//     default: {
//         Serial.println("[receiveEventBus0Bin] command is not defined: " + String(cmd));
//         break;
//     }
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Receive binary event handler for Bus 1
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void receiveEventBus1Bin(size_t n_bytes_received) {
//   if (n_bytes_received < 1) return;
//   uint8_t cmd = Wire.read(); // expects uint8 command byte (up to 255 unique commands can be accepted). 
//   Serial.println("[receiveEventBus1Bin] " + String(cmd) + " (" + String(n_bytes_received) + " bytes)");
//   switch (cmd) {
//     case 0x00: {
//         Serial.println("[receiveEventBus1Bin] preparing to process command: " + String(cmd));
//         break;
//     }
//     default: {
//         Serial.println("[receiveEventBus1Bin] command is not defined: " + String(cmd));
//         break;
//     }
//   }
// }

/** ----------------------------------------------------------------------------
 * @brief Receive binary event handler for Bus 2
 * @warning Customize to use locally (backup first) or copy into project!
*/
// void receiveEventBus2Bin(size_t n_bytes_received) {
//   if (n_bytes_received < 1) return;
//   uint8_t cmd = Wire.read(); // expects uint8 command byte (up to 255 unique commands can be accepted). 
//   Serial.println("[receiveEventBus2Bin] " + String(cmd) + " (" + String(n_bytes_received) + " bytes)");
//   switch (cmd) {
//     case 0x00: {
//         Serial.println("[receiveEventBus2Bin] preparing to process command: " + String(cmd));
//         break;
//     }
//     default: {
//         Serial.println("[receiveEventBus2Bin] command is not defined: " + String(cmd));
//         break;
//     }
//   }
// }

// #####################################################################################################################
// ## BEGIN ESPIDF I2C SLAVE REQUEST BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 0 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// bool onRequestESPIDFBus0(i2c_slave_dev_handle_t i2c_slave,
//                          const i2c_slave_stretch_event_data_t *event_data,
//                          void *user_data)
// {
//   Serial.println("[onRequestESPIDFBus0Bin] ID: " + String(I2CLinkBus0.REQUEST_ID));
//   switch (I2CLinkBus0.REQUEST_ID) {
//     case 0x01: {
//     // -----------------------------------------------------------------------
//     // example response data for request ID 0x01
//     // -----------------------------------------------------------------------
//       memset(I2CLinkBus0.OUTPUT_PACKET, 0, sizeof(I2CLinkBus0.OUTPUT_PACKET));
//       write_uint8_ToPacket(I2CLinkBus0.OUTPUT_PACKET, 0, 0x00); // include a junk byte...
//       write_uint8_ToPacket(I2CLinkBus0.OUTPUT_PACKET, 1, 0xAB); // actual data 171
//       write_uint8_ToPacket(I2CLinkBus0.OUTPUT_PACKET, 2, 0xCD); // actual data 205
//       i2c_slave_transmit(i2c_slave, I2CLinkBus0.OUTPUT_PACKET, 3, I2C_TIMEOUT_MS_BUS0);
//       break;
//     }
//     // -----------------------------------------------------------------------
//     // example default: undefined request ID
//     // -----------------------------------------------------------------------
//     default: {
//       Serial.println("[onRequestESPIDFBus0Bin] undefine request ID: " + String(I2CLinkBus0.REQUEST_ID));
//       break;
//     }
//   }
//   return true;
// }

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 1 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// bool onRequestESPIDFBus1(i2c_slave_dev_handle_t i2c_slave,
//                          const i2c_slave_stretch_event_data_t *event_data,
//                          void *user_data)
// {
//   Serial.println("[onRequestESPIDFBus1Bin] ID: " + String(I2CLinkBus1.REQUEST_ID));
//   switch (I2CLinkBus1.REQUEST_ID) {
//     // -----------------------------------------------------------------------
//     // example response data for request ID 0x01
//     // -----------------------------------------------------------------------
//     case 0x01: {
//       // example response data for request ID 0x01
//       memset(I2CLinkBus1.OUTPUT_PACKET, 0, sizeof(I2CLinkBus1.OUTPUT_PACKET));
//       write_uint8_ToPacket(I2CLinkBus1.OUTPUT_PACKET, 0, 0x00); // include a junk byte...
//       write_uint8_ToPacket(I2CLinkBus1.OUTPUT_PACKET, 1, 0xAB); // actual data 171
//       write_uint8_ToPacket(I2CLinkBus1.OUTPUT_PACKET, 2, 0xCD); // actual data 205
//       i2c_slave_transmit(i2c_slave, I2CLinkBus1.OUTPUT_PACKET, 3, I2C_TIMEOUT_MS_BUS1);
//       break;
//     }
//     // -----------------------------------------------------------------------
//     // example default: undefined request ID
//     // -----------------------------------------------------------------------
//     default: {
//       Serial.println("[onRequestESPIDFBus1Bin] undefine request ID: " + String(I2CLinkBus1.REQUEST_ID));
//       break;
//     }
//   }
//   return true;
// }

/** ----------------------------------------------------------------------------
 * @brief Request event handler for Bus 2 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// bool onRequestESPIDFBus2(i2c_slave_dev_handle_t i2c_slave,
//                          const i2c_slave_stretch_event_data_t *event_data,
//                          void *user_data)
// {
//   Serial.println("[onRequestESPIDFBus2Bin] ID: " + String(I2CLinkBus2.REQUEST_ID));
//   switch (I2CLinkBus2.REQUEST_ID) {
//     // -----------------------------------------------------------------------
//     // example response data for request ID 0x01
//     // -----------------------------------------------------------------------
//     case 0x01: {
//       // example response data for request ID 0x01
//       memset(I2CLinkBus2.OUTPUT_PACKET, 0, sizeof(I2CLinkBus2.OUTPUT_PACKET));
//       write_uint8_ToPacket(I2CLinkBus2.OUTPUT_PACKET, 0, 0x00); // include a junk byte...
//       write_uint8_ToPacket(I2CLinkBus2.OUTPUT_PACKET, 1, 0xAB); // actual data 171
//       write_uint8_ToPacket(I2CLinkBus2.OUTPUT_PACKET, 2, 0xCD); // actual data 205
//       i2c_slave_transmit(i2c_slave, I2CLinkBus2.OUTPUT_PACKET, 3, I2C_TIMEOUT_MS_BUS2);
//       break;
//     }
//     // -----------------------------------------------------------------------
//     // example default: undefined request ID
//     // -----------------------------------------------------------------------
//     default: {
//       Serial.println("[onRequestESPIDFBus2Bin] undefine request ID: " + String(I2CLinkBus2.REQUEST_ID));
//       break;
//     }
//   }
//   return true;
// }

// #####################################################################################################################
// ## BEGIN ESPIDF I2C SLAVE RECEIVE BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief I2C Slave receive event callback: Bus 0
 * @warning Customize to use locally (backup first) or copy into project!
 */
// bool onReceiveESPIDFBus0(i2c_slave_dev_handle_t i2c_slave,
//                const i2c_slave_rx_done_event_data_t  *evt_data,
//                void *arg)
// {
//   // Serial.println("[slave] onReceiveESPIDFBus0 event triggered");
//   I2CLinkBus0.n_bytes_received = (uint8_t)evt_data->buffer[0];
//   I2CLinkBus0.on_receive_flag = true;
//   return true;
// }

/** ----------------------------------------------------------------------------
 * @brief I2C Slave process received data: Bus 0
 * @note Call this function regularly in task to process received data.
 * @note i2c_slave_receive() is called again after processing to re-arm the receiver.
 */
// void processReceivedDataBus0() {
//   if (!I2CLinkBus0.on_receive_flag) {return;}
//   // ---------------------------------------------------------------------------
//   // uncomment for debug (performance cost)
//   // ---------------------------------------------------------------------------
//   // Serial.printf("[slave] Received %d bytes\n", I2CLinkBus0.n_bytes_received);
//   // Serial.print("[slave] Data: ");
//   // for (int i = 0; i < I2CLinkBus0.n_bytes_received; i++) {
//   //   Serial.printf("0x%02X ", I2CLinkBus0.INPUT_PACKET[i]);
//   // }
//   // Serial.println();
//   // ---------------------------------------------------------------------------
//   // example command processing
//   // ---------------------------------------------------------------------------
//   uint8_t cmd = I2CLinkBus0.INPUT_PACKET[0];
//   switch (cmd) {
//     // -------------------------------------------------------------------------
//     // example command 0x01: set request ID for next master read
//     // -------------------------------------------------------------------------
//     case 0x01: {
//       Serial.println("[slave] Processing command 0x01");
//       I2CLinkBus0.REQUEST_ID = 0x01;
//       break;
//     }
//     // -------------------------------------------------------------------------
//     // example default: unknown command
//     // -------------------------------------------------------------------------
//     default: {Serial.printf("[slave] Unknown command: 0x%02X\n", cmd); break;}
//   }
//   // ---------------------------------------------------------------------------
//   // re-arm receiver for next data
//   // ---------------------------------------------------------------------------
//   I2CLinkBus0.on_receive_flag = false;
//   i2c_slave_receive(slave_handle_bus0, I2CLinkBus0.INPUT_PACKET, MAX_IIC_BUFFER_SIZE);
// }

/** ----------------------------------------------------------------------------
 * @brief I2C Slave receive event callback: Bus 1
 * @warning Customize to use locally (backup first) or copy into project!
 */
// bool onReceiveESPIDFBus1(i2c_slave_dev_handle_t i2c_slave,
//                const i2c_slave_rx_done_event_data_t  *evt_data,
//                void *arg)
// {
//   // Serial.println("[slave] onReceiveESPIDFBus1 event triggered");
//   I2CLinkBus1.n_bytes_received = (uint8_t)evt_data->buffer[0];
//   I2CLinkBus1.on_receive_flag = true;
//   return true;
// }

/** ----------------------------------------------------------------------------
 * @brief I2C Slave process received data: Bus 1
 * @note Call this function regularly in task to process received data.
 * @note i2c_slave_receive() is called again after processing to re-arm the receiver.
 */
// void processReceivedDataBus1() {
//   if (!I2CLinkBus1.on_receive_flag) {return;}
//   // ---------------------------------------------------------------------------
//   // uncomment for debug (performance cost)
//   // ---------------------------------------------------------------------------
//   // Serial.printf("[slave] Received %d bytes\n", I2CLinkBus1.n_bytes_received);
//   // Serial.print("[slave] Data: ");
//   // for (int i = 0; i < I2CLinkBus1.n_bytes_received; i++) {
//   //   Serial.printf("0x%02X ", I2CLinkBus1.INPUT_PACKET[i]);
//   // }
//   // Serial.println();
//   // ---------------------------------------------------------------------------
//   // example command processing
//   // ---------------------------------------------------------------------------
//   uint8_t cmd = I2CLinkBus1.INPUT_PACKET[0];
//   switch (cmd) {
//     // -------------------------------------------------------------------------
//     // example command 0x01: set request ID for next master read
//     // -------------------------------------------------------------------------
//     case 0x01: {
//       Serial.println("[slave] Processing command 0x01");
//       I2CLinkBus1.REQUEST_ID = 0x01;
//       break;
//     }
//     // -------------------------------------------------------------------------
//     // example default: unknown command
//     // -------------------------------------------------------------------------
//     default: {Serial.printf("[slave] Unknown command: 0x%02X\n", cmd); break;}
//   }
//   // ---------------------------------------------------------------------------
//   // re-arm receiver for next data
//   // ---------------------------------------------------------------------------
//   I2CLinkBus1.on_receive_flag = false;
//   i2c_slave_receive(slave_handle_bus1, I2CLinkBus1.INPUT_PACKET, MAX_IIC_BUFFER_SIZE);
// }

/** ----------------------------------------------------------------------------
 * @brief I2C Slave receive event callback: Bus 2
 * @warning Customize to use locally (backup first) or copy into project!
 */
// bool onReceiveESPIDFBus2(i2c_slave_dev_handle_t i2c_slave,
//                const i2c_slave_rx_done_event_data_t  *evt_data,
//                void *arg)
// {
//   // Serial.println("[slave] onReceiveESPIDFBus0 event triggered");
//   I2CLinkBus2.n_bytes_received = (uint8_t)evt_data->buffer[0];
//   I2CLinkBus2.on_receive_flag = true;
//   return true;
// }

/** ----------------------------------------------------------------------------
 * @brief I2C Slave process received data: Bus 2
 * @note Call this function regularly in task to process received data.
 * @note i2c_slave_receive() is called again after processing to re-arm the receiver.
 */
// void processReceivedDataBus2() {
//   if (!I2CLinkBus2.on_receive_flag) {return;}
//   // ---------------------------------------------------------------------------
//   // uncomment for debug (performance cost)
//   // ---------------------------------------------------------------------------
//   // Serial.printf("[slave] Received %d bytes\n", I2CLinkBus2.n_bytes_received);
//   // Serial.print("[slave] Data: ");
//   // for (int i = 0; i < I2CLinkBus2.n_bytes_received; i++) {
//   //   Serial.printf("0x%02X ", I2CLinkBus2.INPUT_PACKET[i]);
//   // }
//   // Serial.println();
//   // ---------------------------------------------------------------------------
//   // example command processing
//   // ---------------------------------------------------------------------------
//   uint8_t cmd = I2CLinkBus2.INPUT_PACKET[0];
//   switch (cmd) {
//     // -------------------------------------------------------------------------
//     // example command 0x01: set request ID for next master read
//     // -------------------------------------------------------------------------
//     case 0x01: {
//       Serial.println("[slave] Processing command 0x01");
//       I2CLinkBus2.REQUEST_ID = 0x01;
//       break;
//     }
//     // -------------------------------------------------------------------------
//     // example default: unknown command
//     // -------------------------------------------------------------------------
//     default: {Serial.printf("[slave] Unknown command: 0x%02X\n", cmd); break;}
//   }
//   // ---------------------------------------------------------------------------
//   // re-arm receiver for next data
//   // ---------------------------------------------------------------------------
//   I2CLinkBus2.on_receive_flag = false;
//   i2c_slave_receive(slave_handle_bus2, I2CLinkBus2.INPUT_PACKET, MAX_IIC_BUFFER_SIZE);
// }

// #####################################################################################################################
// ## BEGIN ESPIDF I2C SLAVE CONFIGFURATION BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief I2C Slave Configuration for Bus 0 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// i2c_slave_config_t slv_config_bus0 = {
//     .i2c_port = 0,
//     .sda_io_num = (gpio_num_t)IIC_BUS0_SDA,
//     .scl_io_num = (gpio_num_t)IIC_BUS0_SCL,
//     .clk_source = I2C_CLK_SRC_DEFAULT,
//     .send_buf_depth = MAX_IIC_BUFFER_SIZE,
//     .slave_addr = SLAVE_ADDR_BUS0, // 7-bit address
// };
// i2c_slave_dev_handle_t slave_handle_bus0;

/** ----------------------------------------------------------------------------
 * @brief I2C Slave Configuration for Bus 1 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// i2c_slave_config_t slv_config_bus1 = {
//     .i2c_port = 1,
//     .sda_io_num = (gpio_num_t)IIC_BUS1_SDA,
//     .scl_io_num = (gpio_num_t)IIC_BUS1_SCL,
//     .clk_source = I2C_CLK_SRC_DEFAULT,
//     .send_buf_depth = MAX_IIC_BUFFER_SIZE,
//     .slave_addr = SLAVE_ADDR_BUS1, // 7-bit address
// };
// i2c_slave_dev_handle_t slave_handle_bus1;

/** ----------------------------------------------------------------------------
 * @brief I2C Slave Configuration for Bus 2 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// i2c_slave_config_t slv_config_bus2 = {
//     .i2c_port = 2,
//     .sda_io_num = (gpio_num_t)IIC_BUS2_SDA,
//     .scl_io_num = (gpio_num_t)IIC_BUS2_SCL,
//     .clk_source = I2C_CLK_SRC_DEFAULT,
//     .send_buf_depth = MAX_IIC_BUFFER_SIZE,
//     .slave_addr = SLAVE_ADDR_BUS2, // 7-bit address
// };
// i2c_slave_dev_handle_t slave_handle_bus2;

// #####################################################################################################################
// ## BEGIN ESPIDF I2C SLAVE CALLBACK BLOCKS
// #####################################################################################################################

/** ----------------------------------------------------------------------------
 * @brief Callbacks structure for Bus 0 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// i2c_slave_event_callbacks_t cbs_bus0 = {
//   .on_stretch_occur = onRequestESPIDFBus0Bin,
//   .on_recv_done = onReceiveESPIDFBus0Bin,
// };

/** ----------------------------------------------------------------------------
 * @brief Callbacks structure for Bus 1 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// i2c_slave_event_callbacks_t cbs_bus1 = {
//   .on_stretch_occur = onRequestESPIDFBus1Bin,
//   .on_recv_done = onReceiveESPIDFBus1Bin,
// };

/** ----------------------------------------------------------------------------
 * @brief Callbacks structure for Bus 2 using ESPIDF I2C Slave Driver
 * @warning Customize to use locally (backup first) or copy into project!
 */
// i2c_slave_event_callbacks_t cbs_bus2 = {
//   .on_stretch_occur = onRequestESPIDFBus2Bin,
//   .on_recv_done = onReceiveESPIDFBus2Bin,
// };

// #####################################################################################################################
// ## END ESPIDF I2C SLAVE CALLBACK BLOCKS
// #####################################################################################################################

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
 * @brief Clears the input packet bytes of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkInputPacket(IICLink &iic_link) {
  memset(iic_link.INPUT_PACKET, 0, sizeof(iic_link.INPUT_PACKET));
}

/** ----------------------------------------------------------------------------
 * @brief Clears the output packet bytes of the given IICLink structure.
 * @param iic_link Specify IICLink instance.
 */
void clearI2CLinkOutputPacket(IICLink &iic_link) {
  memset(iic_link.OUTPUT_PACKET, 0, sizeof(iic_link.OUTPUT_PACKET));
}

/** ----------------------------------------------------------------------------
 * @brief Drains all available bytes from the I2C wire.
 * @param wire Specify TwoWire instance.
 */
void drainBus(TwoWire &wire) {
  while (wire.available()) {wire.read();}
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
                        size_t len_packet,
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
                         size_t len_packet,
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
                         size_t len_packet,
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
 * @brief Read bool from I2C wire into specified value.
 * @warning Specified value is expected to be bool.
 */
void read_bool_FromWire(TwoWire &wire, bool &value) {
  value = (bool)wire.read();
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
  memset(value, 0, n_chars);
  for (size_t i = 0; i < n_chars; i++) {
    value[i] = (char)wire.read();
  }
}

/**
 * @brief Read byte from I2C wire into specified value.
 * @warning Specified value is expected to be byte.
 */
void read_byte_FromWire(TwoWire &wire, byte &value) {
  value = wire.read();
}

/**
 * @brief Read N bytes from I2C wire into specified byte array.
 * @param wire Specify TwoWire instance.
 * @param value Pointer to byte array to store the read values.
 * @param n_bytes Number of bytes to read.
 * @warning Specified value is expected to be a byte array with at least n_bytes size.
 * @warning Ensure the byte array is large enough to hold n_bytes values.
 */
void read_nbytes_FromWire(TwoWire &wire, byte *value, size_t n_bytes) {
    for (size_t i = 0; i < n_bytes; i++) {
        value[i] = wire.read();
    }
}

/**
 * @brief Write uint8_t to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_uint8_ToPacket(uint8_t *buffer, size_t offset, uint8_t value) {
  buffer[offset] = value;
}

/**
 * @brief Write int8_t to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_int8_ToPacket(uint8_t *buffer, size_t offset, int8_t value) {
  buffer[offset] = (uint8_t)value;
}

/**
 * @brief Write uint16_t to packet buffer at specified offset (2 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_uint16_ToPacket(uint8_t *buffer, size_t offset, uint16_t value) {
  buffer[offset]     = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

/**
 * @brief Write int16_t to packet buffer at specified offset (2 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_int16_ToPacket(uint8_t *buffer, size_t offset, int16_t value) {
  buffer[offset]     = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
}

/**
 * @brief Write uint32_t to packet buffer at specified offset (4 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_uint32_ToPacket(uint8_t *buffer, size_t offset, uint32_t value) {
  buffer[offset]     = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
  buffer[offset + 3] = (uint8_t)((value >> 24) & 0xFF);
}

/**
 * @brief Write int32_t to packet buffer at specified offset (4 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_int32_ToPacket(uint8_t *buffer, size_t offset, int32_t value) {
  buffer[offset]     = (uint8_t)(value & 0xFF);
  buffer[offset + 1] = (uint8_t)((value >> 8) & 0xFF);
  buffer[offset + 2] = (uint8_t)((value >> 16) & 0xFF);
  buffer[offset + 3] = (uint8_t)((value >> 24) & 0xFF);
}

/**
 * @brief Write uint64_t to packet buffer at specified offset (8 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_uint64_ToPacket(uint8_t *buffer, size_t offset, uint64_t value) {
  for (int i = 0; i < 8; i++) {
    buffer[offset + i] = (uint8_t)((value >> (i * 8)) & 0xFF);
  }
}

/**
 * @brief Write int64_t to packet buffer at specified offset (8 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_int64_ToPacket(uint8_t *buffer, size_t offset, int64_t value) {
  for (int i = 0; i < 8; i++) {
    buffer[offset + i] = (uint8_t)((value >> (i * 8)) & 0xFF);
  }
}

/**
 * @brief Write long to packet buffer at specified offset (little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_long_ToPacket(uint8_t *buffer, size_t offset, long value) {
  for (int i = 0; i < sizeof(long); i++) {
    buffer[offset + i] = (uint8_t)((value >> (i * 8)) & 0xFF);
  }
}

/**
 * @brief Write long long to packet buffer at specified offset (8 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_longlong_ToPacket(uint8_t *buffer, size_t offset, long long value) {
  for (int i = 0; i < 8; i++) {
    buffer[offset + i] = (uint8_t)((value >> (i * 8)) & 0xFF);
  }
}

/**
 * @brief Write float to packet buffer at specified offset (4 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_float_ToPacket(uint8_t *buffer, size_t offset, float value) {
  union { float f; uint8_t bytes[4]; } u;
  u.f = value;
  for (int i = 0; i < 4; i++) {
    buffer[offset + i] = u.bytes[i];
  }
}

/**
 * @brief Write double to packet buffer at specified offset (8 bytes, little-endian).
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_double_ToPacket(uint8_t *buffer, size_t offset, double value) {
  union { double d; uint8_t bytes[8]; } u;
  u.d = value;
  for (int i = 0; i < 8; i++) {
    buffer[offset + i] = u.bytes[i];
  }
}

/**
 * @brief Write bool to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_bool_ToPacket(uint8_t *buffer, size_t offset, bool value) {
  buffer[offset] = (uint8_t)value;
}

/**
 * @brief Write char to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_char_ToPacket(uint8_t *buffer, size_t offset, char value) {
  buffer[offset] = (uint8_t)value;
}

/**
 * @brief Write N chars to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Pointer to char array to write.
 * @param n_chars Number of chars to write.
 * @warning Ensure source char array is at least n_chars in size.
 */
void write_nchars_ToPacket(uint8_t *buffer, size_t offset, const char *value, size_t n_chars) {
  for (size_t i = 0; i < n_chars; i++) {
    buffer[offset + i] = (uint8_t)value[i];
  }
}

/**
 * @brief Write byte to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Value to write.
 */
void write_byte_ToPacket(uint8_t *buffer, size_t offset, byte value) {
    buffer[offset] = value;
}

/**
 * @brief Write N bytes to packet buffer at specified offset.
 * @param buffer Pointer to packet buffer.
 * @param offset Byte offset in buffer.
 * @param value Pointer to byte array to write.
 * @param n_bytes Number of bytes to write.
 * @warning Ensure source array is at least n_bytes in size.
 */
void write_nbytes_ToPacket(uint8_t *buffer, size_t offset, const uint8_t *value, size_t n_bytes) {
    for (size_t i = 0; i < n_bytes; i++) {
        buffer[offset + i] = value[i];
    }
}