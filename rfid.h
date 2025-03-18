#ifndef RFID_H
#define RFID

#include "stdlib.h"

uint8_t txData[25];

// Registers on the MFRC522 peripheral
typedef enum MFRC522_Register : uint8_t {
  CommandReg   = 0x01,
  FIFODataReg  = 0x09,
  FIFOLevelReg = 0x0A,
  AutoTestReg  = 0x36
};

// Reg = valid register from above, write = 0 (read) 1 (write)
inline uint8_t addr_format(MFRC522_Register reg, uint8_t write) {
  return (reg << 1) | (write << 8);
} // addr_format()

// Commands on the MFRC522 (written to CommandReg)
typedef enum MFRC522_Command : uint8_t {
  Idle             = 0x00, // No action, cancels current command action
  Mem              = 0x01, // stores 25 bytes into the internal buffer
  GenerateRandomID = 0x02, // Generates a 10-byte random ID number
  CalcCRC          = 0x03, // activates the CRC coprocessor or performs a self-test
  Transmit         = 0x04, // Transmits data from the FIFO buffer
  NoCMDChange      = 0x07, // No command change
  Receive          = 0x08, // Receives data into the FIFO buffer
  Transceive       = 0x0C, // Allows it to act as a receiver and transmitter
  MFAuthent        = 0x0E, // Performs the MIFARE standard authentication as a reader
  SoftReset        = 0x0F  // Resets the peripheral
};

// Performs a soft reset
void MFRC522_Reset() {
  
} // MFRC522_Reset()

#endif
