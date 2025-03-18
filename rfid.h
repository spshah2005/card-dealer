#ifndef RFID_H
#define RFID

#include "stdlib.h"

uint8_t txData[25];

// Registers on the MFRC522 peripheral
typedef enum MFRC522_Register : uint8_t {
  CommandReg    = 0x01 << 1,
  ComIEnReg     = 0x02 << 1,
  DivIEnReg     = 0x03 << 1,
  ComIrqReg     = 0x04 << 1,
  DivIrqReg     = 0x05 << 1,
  ErrorReg      = 0x06 << 1,
  Status1Reg    = 0x07 << 1,
  Status2Reg    = 0x08 << 1,
  FIFODataReg   = 0x09 << 1,
  FIFOLevelReg  = 0x0A << 1,
  AutoTestReg   = 0x36 << 1
}; // MFRC522_Register{}

// These two functions officially work in main.c but not in this library yet. Still have to decide how I'm going to handle that
void MFRC522_writeRegister(MFRC522_Register reg, uint8_t value) {
  uint8_t buf[2] = {reg, value};
  HAL_SPI_Transmit(&hspi1, buf, 2, HAL_MAX_DELAY);
} // MFRC522_writeRegister()

void MFRC522_readRegister(MFRC522_Register reg, uint8_t value) {
  uint8_t buf[2] = {(reg | 0x80), value};
  HAL_SPI_Transmit(&hspi, buf, 2, HAL_MAX_DELAY);
} // MFRC522_readRegister()

// TODO: Multi-byte versions of these two functions

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
}; // MFRC522_Command{}

// Performs a soft reset
void MFRC522_Reset() {
  MFRC522_writeRegister(CommandReg, SoftReset);
  // TODO: 150ms always works, but might be too long
  HAL_Delay(150);
} // MFRC522_Reset()

#endif
