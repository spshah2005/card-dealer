#ifndef RFID_H
#define RFID

#include "stdlib.h"

uint8_t txData[25];

// Registers on the MFRC522 peripheral
typedef enum MFRC522_Register {
  // Page 0:
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
  ControlReg    = 0x0C << 1,
  BitFramingReg = 0x0D << 1,
  CollReg			  = 0x0E << 1,
  // Page 1:
  ModeReg				= 0x11 << 1,
  TxModeReg     = 0x12 << 1,
  RxModeReg     = 0x13 << 1,
  TxControlReg  = 0x14 << 1,
  TxASKReg			= 0x15 << 1,
  // Page 2:
  ModWidthReg   = 0x24 << 1,
  TModeReg			= 0x2A << 1,
  TPrescalerReg = 0x2B << 1,
  TReloadRegH		= 0x2C << 1,
  TReloadRegL 	= 0x2D << 1,
  // Page 3:
  AutoTestReg   = 0x36 << 1,
  VersionReg    = 0x37 << 1

} MFRC522_Register; // MFRC522_Register{}

// Commands on the MFRC522 (written to CommandReg)
typedef enum MFRC522_Command  {
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
} MFRC522_Command; // MFRC522_Command{}

// Commands sent to the PICC (card)
typedef enum PICC_Command {
	PICC_CMD_REQA    = 0x26, // REQuest (Type A card)
  PICC_CMD_WUPA    = 0x52, // WakeUP  (Type A card)
 	PICC_CMD_CT      = 0x88, // Cascade Tag
	PICC_CMD_SEL_CL1 = 0x93, // Used in anti-collision
	PICC_CMD_SEL_CL2 = 0x95, // "
	PICC_CMD_SEL_CL3 = 0x97, // "
	PICC_CMD_HALTA   = 0x50  // Puts PICC in HALT mode
} PICC_Command;

// Return codes
typedef enum Status {
	STATUS_OK,
	STATUS_ERROR,
	STATUS_COLLISION,
	STATUS_TIMEOUT,
	STATUS_NO_ROOM,
	STATUS_INTERNAL_ERROR,
	STATUS_INVALID,
	STATUS_CRC_WRONG
} Status;

// This is where I've gotten to as of now, there's some bug in the process of identifying a new card that I cannot find. It's 4:16 AM right now.

void MFRC522_WriteRegister(MFRC522_Register reg, uint8_t value) {
	uint8_t txBuffer[2];
	txBuffer[0] = reg;
	txBuffer[1] = value;
	HAL_SPI_Transmit(&hspi1, txBuffer, 2, HAL_MAX_DELAY);
} // MFRC522_WriteRegister()

// Write n bytes of data, stored in byte array value, to register reg
void MFRC522_WriteNtoRegister(MFRC522_Register reg, int n, uint8_t* value) {
	uint8_t txBuffer[n+1];
	txBuffer[0] = reg;
	// Fill the buffer with values (crazy slow, but it won't let me do anything else! yay!)
	for (int i = 1; i < n+1; i++) {
		txBuffer[i] = value[i-1];
	}
	HAL_SPI_Transmit(&hspi1, txBuffer, n+1, HAL_MAX_DELAY);
} // MFRC522_WriteNtoRegister()

uint8_t MFRC522_ReadRegister(MFRC522_Register reg) {
	uint8_t txBuffer[2];
	txBuffer[0] = (reg | 0x80);
	txBuffer[1] = 0x00;
	uint8_t rxBuffer[2];
	HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 2, HAL_MAX_DELAY);
	return rxBuffer[1];
} // MFRC522_ReadRegister()

void MFRC522_ReadNfromRegister(MFRC522_Register reg, int n, uint8_t* rxBuffer) {
	uint8_t txBuffer[n+1];
	for (int i = 0; i < n; i++) {
		txBuffer[i] = (reg | 0x80);
	}
	HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, n+1, HAL_MAX_DELAY);
} // MFRC522_ReadNfromRegister()

// Performs a soft reset
void MFRC522_Reset() {
	// Write the reset command to CommandReg
	MFRC522_WriteRegister(CommandReg, SoftReset);
	// TODO: if using soft power-down mode, have to delay here
} // MFRC522_Reset()

void MFRC522_Init() {
	// Perform a soft reset
	MFRC522_Reset();

	// Reset baud rates
	MFRC522_WriteRegister(TxModeReg, 0x00);
	MFRC522_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	MFRC522_WriteRegister(ModWidthReg, 0x26);

	// Setup timeout
	MFRC522_WriteRegister(TModeReg, 0x80);
	MFRC522_WriteRegister(TPrescalerReg, 0xA9);
	MFRC522_WriteRegister(TReloadRegH, 0x03);
	MFRC522_WriteRegister(TReloadRegL, 0xE8);

	MFRC522_WriteRegister(TxASKReg, 0x40);
	MFRC522_WriteRegister(ModeReg, 0x3D);
	// Turn on the antenna
	uint8_t value = MFRC522_ReadRegister(TxControlReg);
	if ((value & 0x03) != 0x03) {
		MFRC522_WriteRegister(TxControlReg, value | 0x03);
	} // if
} // MFRC522_Init()

void MFRC522_printVersion() {
	uint8_t version = MFRC522_ReadRegister(VersionReg);
	printf("version: %X\n\r", version);
} // MFRC522_printVersion()

Status MFRC522_CommunicateWithPICC(MFRC522_Command command, uint8_t waitIrq, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t *backLen, uint8_t* validBits) {
	uint8_t bitFraming = 0x3;

	MFRC522_WriteRegister(CommandReg, Idle);
	MFRC522_WriteRegister(ComIrqReg, 0x7F);
	MFRC522_WriteRegister(FIFOLevelReg, 0x80);
	MFRC522_WriteNtoRegister(FIFODataReg, sendLen, sendData);
	MFRC522_WriteRegister(BitFramingReg, bitFraming);
	MFRC522_WriteRegister(CommandReg, command);
	if (command == Transceive) {
		uint8_t value = MFRC522_ReadRegister(BitFramingReg);
		value |= 0x80;
		// TODO: optimize by just doing one write here
		MFRC522_WriteRegister(BitFramingReg, value);
	} // if

	// Wait 36 milliseconds maximum (really?)
	// TODO: do this better
	HAL_Delay(36);

	// If nothing was detected, call a timeout
	if (!(MFRC522_ReadRegister(ComIrqReg) & waitIrq)) {
		return STATUS_TIMEOUT;
	}

	// If there was any error except collisions
	uint8_t errorRegValue = MFRC522_ReadRegister(ErrorReg);
	if (errorRegValue & 0x13) {
		return STATUS_ERROR;
	}

	uint8_t _validBits = 0;

	// If caller requested data back, provide it
	if (backData && backLen) {
		uint8_t level = MFRC522_ReadRegister(FIFOLevelReg);
		if (level > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = level;
		// Get received data from FIFO
		MFRC522_ReadNfromRegister(FIFODataReg, level, backData);
		_validBits = MFRC522_ReadRegister(ControlReg) & 0x07;
		if (validBits) {
			*validBits = _validBits;
		}
	} // if

	if (errorRegValue & 0x08) {
		return STATUS_COLLISION;
	}

	// (we're not doing crc validation)

	return STATUS_OK;
} // MFRC522_CommunicateWithPICC()

Status MFRC522_TransceiveData(uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t* backLen, uint8_t* validBits) {
	uint8_t waitIrq = 0x30;
	return MFRC522_CommunicateWithPICC(Transceive, waitIrq, sendData, sendLen, backData, &backLen, validBits);
} // MFRC522_Transceive()

Status MFRC522_PICC_REQA_OR_WUPA(PICC_Command command, uint8_t* bufferATQA, uint8_t* bufferSize) {
	// Check that buffer is good
	if (bufferATQA == 0 || *bufferSize < 2) {
		return STATUS_NO_ROOM;
	}
	// Clear MSB of CollReg
	uint8_t value = MFRC522_ReadRegister(CollReg);
	value &= ~(0x80);
	MFRC522_WriteRegister(CollReg, value);
	//
	uint8_t validBits = 7;

	Status status = MFRC522_TransceiveData(&command, 1, bufferATQA, &bufferSize, (uint8_t*)(&validBits));
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) { // ATQA must be 16 bits
		return STATUS_ERROR;
	}
	return STATUS_OK;
} // MFRC522_PICC_REQA_OR_WUPA

int MFRC522_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	MFRC522_WriteRegister(TxModeReg, 0x00);
	MFRC522_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	MFRC522_WriteRegister(ModWidthReg, 0x26);

	Status result = MFRC522_PICC_REQA_OR_WUPA(PICC_CMD_REQA, bufferATQA, &bufferSize);
	for (int i = 0; i < bufferSize; i++) {
		printf("[%d]: bufferATQA[i]\r\n", bufferATQA[i]);
	}
	return (result == STATUS_OK || result == STATUS_COLLISION);
} // MFRD522_IsNewCardPresent()

#endif
