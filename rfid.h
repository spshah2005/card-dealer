#ifndef RFID_H
#define RFID

#include "stdlib.h"

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
  CollReg	= 0x0E << 1,
  // Page 1:
  ModeReg	= 0x11 << 1,
  TxModeReg     = 0x12 << 1,
  RxModeReg     = 0x13 << 1,
  TxControlReg  = 0x14 << 1,
  TxASKReg	= 0x15 << 1,
  // Page 2:
  ModWidthReg   = 0x24 << 1,
  TModeReg	= 0x2A << 1,
  TPrescalerReg = 0x2B << 1,
  TReloadRegH	= 0x2C << 1,
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

void MFRC522_WriteRegister(MFRC522_Register reg, uint8_t value) {
	uint8_t txBuffer[2];
	txBuffer[0] = reg;
	txBuffer[1] = value;
	// Select peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	// Perform interaction
	HAL_SPI_Transmit(&hspi1, txBuffer, 2, HAL_MAX_DELAY);
	// Free peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
} // MFRC522_WriteRegister()

void MFRC522_WriteRegisterMulti(MFRC522_Register reg, int count, uint8_t* values) {
	// uint8_t txBuffer[2] = {reg, 0x00};
	// Select peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, ((uint8_t*)&reg), 1, HAL_MAX_DELAY);
	HAL_SPI_Transmit(&hspi1, values, count, HAL_MAX_DELAY);
	// Free peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

uint8_t MFRC522_ReadRegister(MFRC522_Register reg) {
	uint8_t txBuffer[2];
	txBuffer[0] = (reg|0x80);
	txBuffer[1] = 0x00;
	uint8_t rxBuffer[2];
	// Select peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	// Perform read
	HAL_SPI_TransmitReceive(&hspi1, txBuffer, rxBuffer, 2, HAL_MAX_DELAY);
	// Free peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
	// Return value read from register
	return rxBuffer[1];
} // MFRC522_ReadRegister()

void MFRC522_ReadRegisterMulti(MFRC522_Register reg, int count, uint8_t* values) {
	uint8_t read_addr = reg | 0x80;
	// Select peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_RESET);
	HAL_SPI_Transmit(&hspi1, ((uint8_t*)&read_addr), 1, HAL_MAX_DELAY);
	// Loop requesting
	for (int i = 0; i < count; i++) {
		HAL_SPI_TransmitReceive(&hspi1, ((uint8_t*)&read_addr), &values[i], 1, HAL_MAX_DELAY);
	}
	// Get final read
	HAL_SPI_TransmitReceive(&hspi1, 0x00, &values[count], 1, HAL_MAX_DELAY);
	// Free peripheral
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_4, GPIO_PIN_SET);
}

void MFRC522_Reset() {
	// Write the reset command to CommandReg
	MFRC522_WriteRegister(CommandReg, SoftReset);
} // MFRC522_Reset()

void MFRC522_Init() {
	// Perform a garbage write first (if this works, it doesn't matter)
	MFRC522_WriteRegister(CommandReg, Idle);
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

void MFRC522_performSelfTest() {

	// 1. Soft Reset
	MFRC522_Reset();

	// 2. Clear internal buffer by writing 25 bytes of 0x00
	MFRC522_WriteRegister(FIFOLevelReg, 0x80); // flush FIFO buffer
	uint8_t zeroes[25] = {0x00};
	MFRC522_WriteRegisterMulti(FIFODataReg, 25, zeroes);
	MFRC522_WriteRegister(CommandReg, Mem); // transfer to internal buffer

	// 3. Enable self-test
	MFRC522_WriteRegister(AutoTestReg, 0x09);

	// 4. Write 0x00 to the FIFO buffer
	MFRC522_WriteRegister(FIFODataReg, 0x00);

	// 5. Start self-test by issuing the CalcCRC command
	MFRC522_WriteRegister(CommandReg, CalcCRC);

	// 6. Wait for self-test to complete
	HAL_Delay(500);

	// 7. Read out resulting 64 bytes from the FIFO buffer
	uint8_t result[64];
	MFRC522_ReadRegisterMulti(FIFODataReg, 64, result);

	// Reset AutoTestReg
	MFRC522_WriteRegister(AutoTestReg, 0x00);
	// Re-init:
	MFRC522_Init();

} // MFRC522_performSelfTest()

Status MFRC522_CommunicateWithPICC(MFRC522_Command command, uint8_t waitIRq, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t* backLen, uint8_t* validBits) {
	uint8_t bitFraming = *validBits;
	MFRC522_WriteRegister(CommandReg, Idle);
	MFRC522_WriteRegister(ComIrqReg, 0x7F);
	MFRC522_WriteRegister(FIFOLevelReg, 0x80);
	MFRC522_WriteRegisterMulti(FIFODataReg, sendLen, sendData);
	MFRC522_WriteRegister(BitFramingReg, bitFraming);
	MFRC522_WriteRegister(CommandReg, command);
	if (command == Transceive) {
		uint8_t value = 0x80 | MFRC522_ReadRegister(BitFramingReg);
		MFRC522_WriteRegister(BitFramingReg, value);
	}

	// 36ms, worst-case scenario
	uint32_t start = 0;
	uint32_t deadline = 36;
	int completed = 0;
	while(!completed && start < deadline) {
		uint8_t n = MFRC522_ReadRegister(ComIrqReg);
		if (n & waitIRq) {
			completed = 1;
			break;
		}
		/* if (n & 0x01) {
			return STATUS_TIMEOUT;
		}*/
		HAL_Delay(1);
		start++;
	} // while

	if (!completed) {
		return STATUS_TIMEOUT;
	}

	uint8_t errorRegValue = MFRC522_ReadRegister(ErrorReg);
	if (errorRegValue & 0x13) {
		return STATUS_ERROR;
	}

	uint8_t _validBits = 0;
	if (backData != NULL && backLen != NULL) {
		uint8_t n = MFRC522_ReadRegister(FIFOLevelReg);
		if (n > *backLen) {
			return STATUS_NO_ROOM;
		}
		*backLen = n;
		MFRC522_ReadRegisterMulti(FIFODataReg, n, backData);
		_validBits = MFRC522_ReadRegister(ControlReg) & 0x07;
		if (validBits != NULL) {
			*validBits = _validBits;
		}
	}

	// Tell about collisions
	if (errorRegValue & 0x08) {
		return STATUS_COLLISION;
	}

	return STATUS_OK;

} // MFRC522_CommunicateWithPICC()

Status MFRC522_TransceiveData(uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t* backLen, uint8_t* validBits) {
	uint8_t waitIrq = 0x30;
	return MFRC522_CommunicateWithPICC(Transceive, waitIrq, sendData, sendLen, backData, backLen, validBits);
}

Status MFRC522_REQA_or_WUPA(PICC_Command command, uint8_t* bufferATQA, uint8_t* bufferSize) {
	uint8_t validBits;
	Status status;

	if (bufferATQA == NULL || *bufferSize < 2) {
		return STATUS_NO_ROOM;
	}
	// Clear CollReg MSB
	uint8_t value = MFRC522_ReadRegister(CollReg);
	value &= ~(0x80);
	MFRC522_WriteRegister(CollReg, value);

	validBits = 7;
	status = MFRC522_TransceiveData(&command, 1, bufferATQA, bufferSize, &validBits);
	if (status != STATUS_OK) {
		return status;
	}
	if (*bufferSize != 2 || validBits != 0) {
		return STATUS_ERROR;
	}
	return STATUS_OK;
}

Status MFRC522_RequestA(uint8_t* bufferATQA, uint8_t *bufferSize) {
	return MFRC522_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
}

int MFRC522_IsNewCardPresent() {
	uint8_t bufferATQA[2];
	uint8_t bufferSize = sizeof(bufferATQA);

	// Reset baud rates
	MFRC522_WriteRegister(TxModeReg, 0x00);
	MFRC522_WriteRegister(RxModeReg, 0x00);
	// Reset ModWidthReg
	MFRC522_WriteRegister(ModWidthReg, 0x26);

	Status result = MFRC522_RequestA(bufferATQA, &bufferSize);
	printf("Status: %d\r\n",  (uint8_t)result);
	return (result == STATUS_OK || result == STATUS_COLLISION);
}

#endif
