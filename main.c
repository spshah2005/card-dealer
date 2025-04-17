/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "rfid.h"
#include "stdlib.h"
#include "stdio.h"
#include "stm32l4xx_hal.h"
#include "ps2.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAX_PLAYERS 5 // The number of players refers to the human players, a dealer is always assumed
#define HX711_TIMEOUT 1000000  // Prevent infinite loop for HX711 Load Cell Sensor
#define HX711_DATA_GPIO   GPIOB
#define HX711_DATA_PIN    GPIO_PIN_0   // HX711 DT → PA1

#define HX711_CLK_GPIO    GPIOA
#define HX711_CLK_PIN     GPIO_PIN_0   // HX711 SCK → PA2
static int32_t OFFSET = 26700;  // Calibration Offset
static float SCALE_FACTOR = -500;  // Calibration Scale Factor
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
DAC_HandleTypeDef hdac1;
DMA_HandleTypeDef hdma_dac1_ch2;

UART_HandleTypeDef hlpuart1;

SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi3;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;

/* USER CODE BEGIN PV */
uint8_t num_player;
//load cell timer
TIM_HandleTypeDef *HX711_Timer;
//ps2 contoller
PS2Buttons *PS2_PS2;
TIM_HandleTypeDef *PS2_TIMER;

#define NSS_BANK GPIOG
#define NSS_PIN GPIO_PIN_12

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_DAC1_Init(void);
static void MX_LPUART1_UART_Init(void);
static void MX_SPI2_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM4_Init(void);
static void MX_TIM5_Init(void);
static void MX_SPI3_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

// Creates a delay of us microseconds
void delay(uint16_t us) {
	__HAL_TIM_SET_COUNTER(&htim4, 0);
	while (__HAL_TIM_GET_COUNTER(&htim4) < us);
}

// Offsets the Stepper Motor by the given angle, in the dir (direction) (0 : counter-clockwise; 1 : clockwise), at the set rpm
void stepperOffsetAngle(float angle, uint8_t dir, uint32_t rpm) { 
	float anglePerUpdate = 0.1125;	// Assuming an update is a 16th microstep
	uint32_t updates = angle/anglePerUpdate;
	if (dir) {	// clockwise
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 1);
	}
	else {	// counter-clockwise
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_1, 0);
	}
	for (uint32_t i = 0; i < updates; ++i) {
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 1);
		uint16_t us = (60000000 / (200*16)) / rpm; // (60000000 / (stepsPerRev*microStepSize)) / rpm
  delay(us);
		HAL_GPIO_WritePin(GPIOC, GPIO_PIN_4, 0);
	}
}

void MFRC522_WriteRegister(MFRC522_Register reg, uint8_t value) {
  uint8_t txBuffer[2];
  txBuffer[0] = reg;
  txBuffer[1] = value;
  // Select peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_RESET);
  // Perform interaction
  HAL_SPI_Transmit(&hspi3, txBuffer, 2, HAL_MAX_DELAY);
  // Free peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_SET);
} // MFRC522_WriteRegister()

void MFRC522_WriteRegisterMulti(MFRC522_Register reg, int count, uint8_t* values) {
  // uint8_t txBuffer[2] = {reg, 0x00};
  // Select peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, ((uint8_t*)&reg), 1, HAL_MAX_DELAY);
  HAL_SPI_Transmit(&hspi3, values, count, HAL_MAX_DELAY);
  // Free peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_SET);
}

uint8_t MFRC522_ReadRegister(MFRC522_Register reg) {
  uint8_t txBuffer[2];
  txBuffer[0] = (reg|0x80);
  txBuffer[1] = 0x00;
  uint8_t rxBuffer[2];
  // Select peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_RESET);
  // Perform read
  HAL_SPI_TransmitReceive(&hspi3, txBuffer, rxBuffer, 2, HAL_MAX_DELAY);
  // Free peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_SET);
  // Return value read from register
  return rxBuffer[1];
} // MFRC522_ReadRegister()

void MFRC522_ReadRegisterMulti(MFRC522_Register reg, int count, uint8_t* values) {
  uint8_t read_addr = reg | 0x80;
  // Select peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_RESET);
  HAL_SPI_Transmit(&hspi3, ((uint8_t*)&read_addr), 1, HAL_MAX_DELAY);
  // Loop requesting
  for (int i = 0; i < count; i++) {
  HAL_SPI_TransmitReceive(&hspi3, ((uint8_t*)&read_addr), &values[i], 1, HAL_MAX_DELAY);
  }
  // Get final read
  HAL_SPI_TransmitReceive(&hspi3, 0x00, &values[count], 1, HAL_MAX_DELAY);
  // Free peripheral
  HAL_GPIO_WritePin(NSS_BANK, NSS_PIN, GPIO_PIN_SET);
}

void MFRC522_Reset() {
  // Write the reset command to CommandReg
  MFRC522_WriteRegister(CommandReg, SoftReset);
} // MFRC522_Reset()

void MFRC522_ResetBaudAndModWidth() {
 	// Reset baud rates
 	MFRC522_WriteRegister(TxModeReg, 0x00); // 0b 0[010] 0000
 	MFRC522_WriteRegister(RxModeReg, 0x00);
 	// Reset modulation width
 	MFRC522_WriteRegister(ModWidthReg, 0x26);
}

void MFRC522_Init() {
  // Perform a garbage write first (if this works, it doesn't matter)
  MFRC522_WriteRegister(CommandReg, Idle);
  // Perform a soft reset
  MFRC522_Reset();

  MFRC522_ResetBaudAndModWidth();

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

// Returns 0 if the self test performed correctly, -1 otherwise
int MFRC522_performSelfTest() {

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
  // Re-init: (otherwise everything breaks)
  MFRC522_Init();

  // Compare result to the desired value
  for (int i = 0; i < 64; i++) {
  	if (result[i] != correct_self_test[i]) {
  		return -1;
  	}
  } // for
  // All good :)
  return 0;

} // MFRC522_performSelfTest()

MFRC522_Status MFRC522_CommunicateWithPICC(MFRC522_Command command, uint8_t waitIRq, uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t* backLen, uint8_t* validBits) {
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
  } // if

  // 36ms, worst-case scenario
  uint32_t start = 0;
  uint32_t deadline = 36; // 32ms
  int completed = 0;
  while(!completed && start < deadline) {
  	uint8_t n = MFRC522_ReadRegister(ComIrqReg);
  	if (n & waitIRq) {
  		completed = 1;
  		break;
  	} // if
  /* if (n & 0x01) {
  return STATUS_TIMEOUT;
  }*/
  	HAL_Delay(1);
  	start++;
  } // while

  if (!completed) {
  	// printf("| timing out from the TRANSCEIVE command!\r\n");
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
  } // if

  // Tell about collisions
  if (errorRegValue & 0x08) {
  	return STATUS_COLLISION;
  }

  return STATUS_OK;

} // MFRC522_CommunicateWithPICC()

MFRC522_Status MFRC522_TransceiveData(uint8_t* sendData, uint8_t sendLen, uint8_t* backData, uint8_t* backLen, uint8_t* validBits) {
  uint8_t waitIrq = 0x30;
  	return MFRC522_CommunicateWithPICC(Transceive, waitIrq, sendData, sendLen, backData, backLen, validBits);
  }

MFRC522_Status MFRC522_REQA_or_WUPA(PICC_Command command, uint8_t* bufferATQA, uint8_t* bufferSize) {
  uint8_t validBits;
  MFRC522_Status status;

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

MFRC522_Status MFRC522_RequestA(uint8_t* bufferATQA, uint8_t *bufferSize) {
  return MFRC522_REQA_or_WUPA(PICC_CMD_REQA, bufferATQA, bufferSize);
} // MFRC522_RequestA()

MFRC522_Status MFRC522_WakeUpA(uint8_t* bufferATQA, uint8_t *bufferSize) {
	return MFRC522_REQA_or_WUPA(PICC_CMD_WUPA, bufferATQA, bufferSize);
} // MFRC522_WakeUpA

int MFRC522_IsNewCardPresent() {
  uint8_t bufferATQA[2];
  uint8_t bufferSize = sizeof(bufferATQA);

  MFRC522_ResetBaudAndModWidth();

  MFRC522_Status result = MFRC522_RequestA(bufferATQA, &bufferSize);
  // printf("Status: %d\r\n",  (uint8_t)result);
  return (result == STATUS_OK || result == STATUS_COLLISION);
} // MFRC522_IsNewCardPresent()

MFRC522_Status MFRC522_CalculateCRC(uint8_t* data, uint8_t length, uint8_t* result) {
 MFRC522_WriteRegister(CommandReg, Idle);
 MFRC522_WriteRegister(DivIrqReg, 0x04);
 MFRC522_WriteRegister(FIFOLevelReg, 0x80);
 MFRC522_WriteRegisterMulti(FIFODataReg, length, data);
 MFRC522_WriteRegister(CommandReg, CalcCRC);

 // TODO: fix
 HAL_Delay(89);

 // Check about timeout
 uint8_t value = MFRC522_ReadRegister(DivIrqReg);
 if (!(value & 0x04)) {
 // DEBUG:
 printf("timed out of calc CRC command\r\n");
 return STATUS_TIMEOUT;
 }
 // Get the CRC value
 MFRC522_WriteRegister(CommandReg, Idle);
// result[0] = MFRC522_ReadRegister(CRCResultRegL); FIX NO REG W THIS NAME FOUND
// result[1] = MFRC522_ReadRegister(CRCResultRegH);
 return STATUS_OK;
  } // MFRC522_CalculateCRC()

MFRC522_Status MFRC522_Select(uint8_t uid[7]) {
	int8_t validBits = 0;
  int uidComplete;
  int selectDone;
  int useCascadeTag;
  uint8_t cascadeLevel = 1;
  MFRC522_Status result;
  uint8_t count;
  uint8_t checkBit;
  uint8_t index;
  uint8_t uidIndex; // The first index in uid->uidByte[] that is used in the current Cascade Level.
  int8_t currentLevelKnownBits; // The number of known UID bits in the current Cascade Level.
  uint8_t buffer[9]; // The SELECT/ANTICOLLISION commands uses a 7 byte standard frame + 2 bytes CRC_A
  uint8_t bufferUsed; // The number of bytes used in the buffer, ie the number of bytes to transfer to the FIFO.
  uint8_t rxAlign; // Used in BitFramingReg. Defines the bit position for the first bit received.
  uint8_t txLastBits; // Used in BitFramingReg. The number of valid bits in the last transmitted byte.
  uint8_t *responseBuffer;
  uint8_t responseLength;

  // Description of buffer structure:
  // Byte 0: SEL Indicates the Cascade Level: PICC_CMD_SEL_CL1, PICC_CMD_SEL_CL2 or PICC_CMD_SEL_CL3
  // Byte 1: NVB Number of Valid Bits (in complete command, not just the UID): High nibble: complete bytes, Low nibble: Extra bits.
  // Byte 2: UID-data or CT See explanation below. CT means Cascade Tag.
  // Byte 3: UID-data
  // Byte 4: UID-data
  // Byte 5: UID-data
  // Byte 6: BCC Block Check Character - XOR of bytes 2-5
  // Byte 7: CRC_A
  // Byte 8: CRC_A
  // The BCC and CRC_A are only transmitted if we know all the UID bits of the current Cascade Level.
  //
  // Description of bytes 2-5: (Section 6.5.4 of the ISO/IEC 14443-3 draft: UID contents and cascade levels)
  // UID size Cascade level Byte2 Byte3 Byte4 Byte5
  // ======== ============= ===== ===== ===== =====
  // 7 bytes 1 CT uid0 uid1 uid2
  // 2 uid3 uid4 uid5 uid6

  // Sanity checks
  if (validBits > 80) {
  	return STATUS_INVALID;
  }

  // Prepare MFRC522
  uint8_t value = MFRC522_ReadRegister(CollReg);
  value &= ~(0x80);
  MFRC522_WriteRegister(CollReg, value);

  // Repeat Cascade Level loop until we have a complete UID.
  uidComplete = 0;
  while (!uidComplete) {
  // Set the Cascade Level in the SEL byte, find out if we need to use the Cascade Tag in byte 2.
  	switch (cascadeLevel) {
  		case 1:
  		buffer[0] = PICC_CMD_SEL_CL1;
  		uidIndex = 0;
  		useCascadeTag = validBits; // When we know that the UID has more than 4 bytes
  		break;

 		case 2:
  		buffer[0] = PICC_CMD_SEL_CL2;
  		uidIndex = 3;
  		useCascadeTag = 0; // When we know that the UID has more than 7 bytes
  		break;

  	case 3:
  		buffer[0] = PICC_CMD_SEL_CL3;
  		uidIndex = 6;
  		useCascadeTag = 0; // Never used in CL3.
  		break;

  	default:
  		return STATUS_INTERNAL_ERROR;
  		break; // Redundant break statement go brrr
  	} // switch

  // How many UID bits are known in this Cascade Level?
  currentLevelKnownBits = validBits - (8 * uidIndex);
  	if (currentLevelKnownBits < 0) {
  	currentLevelKnownBits = 0;
  }
  // Copy the known bits from uid->uidByte[] to buffer[]
  index = 2; // destination index in buffer[]
  if (useCascadeTag) {
  	buffer[index] = PICC_CMD_CT;
  	index++;
  }
  uint8_t bytesToCopy = currentLevelKnownBits / 8 + (currentLevelKnownBits % 8 ? 1 : 0); // The number of bytes needed to represent the known bits for this level.
  if (bytesToCopy) {
  	uint8_t maxBytes = useCascadeTag ? 3 : 4; // Max 4 bytes in each Cascade Level. Only 3 left if we use the Cascade Tag
  	if (bytesToCopy > maxBytes) {
  		bytesToCopy = maxBytes;
  	}
  	for (count = 0; count < bytesToCopy; count++) {
  		buffer[index++] = uid[uidIndex + count];
  	} // for
  } // if
  // Now that the data has been copied we need to include the 8 bits in CT in currentLevelKnownBits
  if (useCascadeTag) {
  	currentLevelKnownBits += 8;
  }

  // Repeat anti collision loop until we can transmit all UID bits + BCC and receive a SAK - max 32 iterations.
  selectDone = 0;
  while (!selectDone) {
  // Find out how many bits and bytes to send and receive.
  if (currentLevelKnownBits >= 32) { // All UID bits in this Cascade Level are known. This is a SELECT.
  	buffer[1] = 0x70; // NVB - Number of Valid Bits: Seven whole bytes
  	// Calculate BCC - Block Check Character
  	buffer[6] = buffer[2] ^ buffer[3] ^ buffer[4] ^ buffer[5];
  	// Calculate CRC_A
  	result = MFRC522_CalculateCRC(buffer, 7, &buffer[7]);
  	if (result != STATUS_OK) {
  		return result;
  	}
  	txLastBits = 0; // 0 => All 8 bits are valid.
  	bufferUsed = 9;
  	// Store response in the last 3 bytes of buffer (BCC and CRC_A - not needed after tx)
  	responseBuffer = &buffer[6];
  	responseLength = 3;
  } else { // This is an ANTICOLLISION.
  	txLastBits = currentLevelKnownBits % 8;
  	count = currentLevelKnownBits / 8; // Number of whole bytes in the UID part.
  	index = 2 + count; // Number of whole bytes: SEL + NVB + UIDs
  	buffer[1] = (index << 4) + txLastBits; // NVB - Number of Valid Bits
  	bufferUsed = index + (txLastBits ? 1 : 0);
  	// Store response in the unused part of buffer
  	responseBuffer = &buffer[index];
  	responseLength = sizeof(buffer) - index;
  } // else

  // Set bit adjustments
  rxAlign = txLastBits; // Having a separate variable is overkill. But it makes the next line easier to read.
  MFRC522_WriteRegister(BitFramingReg, (rxAlign << 4) + txLastBits); // RxAlign = BitFramingReg[6..4]. TxLastBits = BitFramingReg[2..0]

  // Transmit the buffer and receive the response.
  result = MFRC522_TransceiveData(buffer, bufferUsed, responseBuffer, &responseLength, &txLastBits);
  if (result == STATUS_COLLISION) { // More than one PICC in the field => collision.
  	uint8_t valueOfCollReg = MFRC522_ReadRegister(CollReg); // CollReg[7..0] bits are: ValuesAfterColl reserved CollPosNotValid CollPos[4:0]
  	if (valueOfCollReg & 0x20) { // CollPosNotValid
  		return STATUS_COLLISION; // Without a valid collision position we cannot continue
  	}
  	uint8_t collisionPos = valueOfCollReg & 0x1F; // Values 0-31, 0 means bit 32.
  	if (collisionPos == 0) {
  		collisionPos = 32;
  	}
  	if (collisionPos <= currentLevelKnownBits) { // No progress - should not happen
  		return STATUS_INTERNAL_ERROR;
  	}
  	// Choose the PICC with the bit set.
  	currentLevelKnownBits = collisionPos;
  	count = currentLevelKnownBits % 8; // The bit to modify
  	checkBit = (currentLevelKnownBits - 1) % 8;
  	index = 1 + (currentLevelKnownBits / 8) + (count ? 1 : 0); // First byte is index 0.
  	buffer[index] |= (1 << checkBit);
  } else if (result != STATUS_OK) {
  	return result;
  } else { // STATUS_OK
  	if (currentLevelKnownBits >= 32) { // This was a SELECT.
  	selectDone = 1; // No more anticollision
  	// We continue below outside the while.
  } else { // This was an ANTICOLLISION.
  // We now have all 32 bits of the UID in this Cascade Level
  currentLevelKnownBits = 32;
  // Run loop again to do the SELECT.
  }
  }
  } // End of while (!selectDone)

  // We do not check the CBB - it was constructed by us above.

  // Copy the found UID bytes from buffer[] to uid->uidByte[]
  index = (buffer[2] == PICC_CMD_CT) ? 3 : 2; // source index in buffer[]
  bytesToCopy = (buffer[2] == PICC_CMD_CT) ? 3 : 4;
  for (count = 0; count < bytesToCopy; count++) {
  uid[uidIndex + count] = buffer[index];
  index++;
  }

  // Check response SAK (Select Acknowledge)
  if (responseLength != 3 || txLastBits != 0) { // SAK must be exactly 24 bits (1 byte + CRC_A).
  return STATUS_ERROR;
  }
  // Verify CRC_A - do our own calculation and store the control in buffer[2..3] - those bytes are not needed anymore.
  result = MFRC522_CalculateCRC(responseBuffer, 1, &buffer[2]);
  if (result != STATUS_OK) {
  return result;
  }
  if ((buffer[2] != responseBuffer[1]) || (buffer[3] != responseBuffer[2])) {
  return STATUS_CRC_WRONG;
  }
  if (responseBuffer[0] & 0x04) { // Cascade bit set - UID not complete yes
  cascadeLevel++;
  }
  else {
  uidComplete = 1;
  }
  } // End of while (!uidComplete)

  // Set correct uid->size

  return STATUS_OK;
  } // End PICC_Select()

// Returns the number of a card, 0 if no change, -1 if no card is present
int MFRC522_ReadCard() {
	static int last_card = -1;
	// Check if a card is present
	int card_present = MFRC522_IsNewCardPresent();
	if (!card_present) {
		return -1;
	} // if

	// Read the uid on the card
 	uint8_t uid[7];
 	int error_reading = MFRC522_Select(uid);
 	if (error_reading) {
 		return -1;
 	} // if
	
 	// find the corresponding card
 	for (int i = 0; i < 104; i++) {
 		int j = 1;
 		for (; j < 3; j++) {
 			if (uid[j] != card_uids[i][j-1]) break;
 		} // for
 		if (j == 3) { // if this card matches
 			if (i % 52 == last_card) {
 				return 0;
 			} else {
 				last_card = i % 52;
 				return (i % 52) + 1;
 			}
 		} // if
 	} // for

 	// If there's an error reading the uid
 	return -1;

} // MFRC522_ReadCard()

/*******************************
Load Cell Sensor
*******************************/
void delay_ten_ns(uint16_t us) { //10 ns delay function
	__HAL_TIM_SET_COUNTER(HX711_Timer, 0);  // set the counter value a 0
	while (__HAL_TIM_GET_COUNTER(HX711_Timer) < us){
		continue;
	}
}
void HX711_Init(TIM_HandleTypeDef *timer) {
 // Ensure SCK starts LOW
HX711_Timer = timer;
HAL_TIM_Base_Start(HX711_Timer);
 HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);
}


int32_t HX711_Read(void) {
 int32_t data = 0;

 // 1. Wait until HX711 is ready (DATA goes LOW)
 uint32_t timeout = HX711_TIMEOUT;
 //Data pin goes low when data is ready
 while (HAL_GPIO_ReadPin(HX711_DATA_GPIO, HX711_DATA_PIN)) {
     if (--timeout == 0) {
         // printf("HX711 Timeout! Check wiring.\n");
         return -1;
     }
 }
 delay_ten_ns(12);

 // 2. Read 24 bits (MSB first)
 for (uint8_t i = 0; i < 24; i++) {
     HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_SET);
     delay_ten_ns(5);
     data = (data << 1) | HAL_GPIO_ReadPin(HX711_DATA_GPIO, HX711_DATA_PIN);
     delay_ten_ns(20);
     HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);
     delay_ten_ns(25);
 }

 // 3. Set Gain (128 by default → 1 extra clock pulse)
 HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_SET);
 delay_ten_ns(25);
 HAL_GPIO_WritePin(HX711_CLK_GPIO, HX711_CLK_PIN, GPIO_PIN_RESET);

 // 4. Convert signed 24-bit value
 if (data & 0x800000) {  // If MSB is 1 (negative number)
     data |= 0xFF000000;  // Sign extend to 32-bit
 }

 return data;
}
int HX711_GetWeight(void) {
 int32_t raw_data = HX711_Read();
 if (raw_data == -1) return -1;  // Error case
//    printf("rawww: %ld\r\n", raw_data);
 return (raw_data + OFFSET) / SCALE_FACTOR;
}

/*******************************
PS2 Controller Code
*******************************/
uint8_t PS2_SendByte(uint8_t cmd) {
	uint8_t data = 0x00;
	uint16_t buffer = 0x01;
	for (buffer = 0x01; buffer < 0x0100; buffer <<= 1) {
		if (buffer & cmd) {
			HAL_GPIO_WritePin(PS2_MOSI_GPIO_Port, PS2_MOSI_Pin, GPIO_PIN_SET);
		} else {
			HAL_GPIO_WritePin(PS2_MOSI_GPIO_Port, PS2_MOSI_Pin, GPIO_PIN_RESET);
		}
		HAL_GPIO_WritePin(PS2_SCK_GPIO_Port, PS2_SCK_Pin, GPIO_PIN_SET);
		delay(8);
		HAL_GPIO_WritePin(PS2_SCK_GPIO_Port, PS2_SCK_Pin, GPIO_PIN_RESET);
		delay(8);
		HAL_GPIO_WritePin(PS2_SCK_GPIO_Port, PS2_SCK_Pin, GPIO_PIN_SET);
		if (HAL_GPIO_ReadPin(PS2_MISO_GPIO_Port, PS2_MISO_Pin)) {
			data |= buffer;
		}
	}
	delay(8);
	return data;
}
void PS2_Cmd(uint8_t *transmit, uint8_t *receive, uint8_t length) {
	HAL_GPIO_WritePin(PS2_SS_GPIO_Port, PS2_SS_Pin, GPIO_PIN_RESET);
	delay(8);
	for (uint8_t i = 0; i < length; i++) {
		receive[i] = PS2_SendByte(transmit[i]);
	}
	HAL_GPIO_WritePin(PS2_SS_GPIO_Port, PS2_SS_Pin, GPIO_PIN_SET);
	delay(8);
}

void PS2_Poll() {
	uint8_t transmit[5] = { 0x01, 0x42 };
	uint8_t receive[5] = { 0x00 };
	PS2_Cmd(transmit, receive, 5);
}

void PS2_EnterConfig() {
	uint8_t transmit[5] = { 0x01, 0x43, 0x00, 0x01 };
	uint8_t receive[5] = { 0x00 };
	PS2_Cmd(transmit, receive, 5);
}

void PS2_AnalogMode() {
	uint8_t transmit[9] = { 0x01, 0x44, 0x00, 0x01, 0x03 };
	uint8_t receive[9] = { 0x00 };
	PS2_Cmd(transmit, receive, 9);
}

void PS2_ExitConfig() {
	uint8_t transmit[9] = { 0x01, 0x43, 0x00, 0x00, 0x5A, 0x5A, 0x5A, 0x5A, 0x5A };
	uint8_t receive[9] = { 0x00 };
	PS2_Cmd(transmit, receive, 9);
}

void PS2_Init(TIM_HandleTypeDef *timer, PS2Buttons *ps2) {
	// Set hardware
	PS2_TIMER = timer;
	PS2_PS2 = ps2;
	// Enable timer
	HAL_TIM_Base_Start(PS2_TIMER);
	// Enable analog mode
	PS2_Poll();
	PS2_EnterConfig();
	PS2_AnalogMode();
	PS2_Poll();
	PS2_ExitConfig();
}

void PS2_Update() {
	uint8_t transmit[9] = { 0x01, 0x42 };
	uint8_t receive[9] = { 0x00 };
	PS2_Cmd(transmit, receive, 9);
	if (receive[1] != 0x41 && receive[1] != 0x73) return;
	uint16_t buttonStatus = receive[3] | (receive[4] << 8);
	PS2_PS2->SELECT = !((buttonStatus >> 0) & 0x01);
	PS2_PS2->L3 = !((buttonStatus >> 1) & 0x01);
	PS2_PS2->R3 = !((buttonStatus >> 2) & 0x01);
	PS2_PS2->START = !((buttonStatus >> 3) & 0x01);
	PS2_PS2->UP = !((buttonStatus >> 4) & 0x01);
	PS2_PS2->RIGHT = !((buttonStatus >> 5) & 0x01);
	PS2_PS2->DOWN = !((buttonStatus >> 6) & 0x01);
	PS2_PS2->LEFT = !((buttonStatus >> 7) & 0x01);
	PS2_PS2->L2 = !((buttonStatus >> 8) & 0x01);
	PS2_PS2->R2 = !((buttonStatus >> 9) & 0x01);
	PS2_PS2->L1 = !((buttonStatus >> 10) & 0x01);
	PS2_PS2->R1 = !((buttonStatus >> 11) & 0x01);
	PS2_PS2->TRIANGLE = !((buttonStatus >> 12) & 0x01);
	PS2_PS2->CIRCLE = !((buttonStatus >> 13) & 0x01);
	PS2_PS2->CROSS = !((buttonStatus >> 14) & 0x01);
	PS2_PS2->SQUARE = !((buttonStatus >> 15) & 0x01);
	if (receive[1] != 0x73) return;
	PS2_PS2->RX = receive[5];
	PS2_PS2->RY = receive[6];
	PS2_PS2->LX = receive[7];
	PS2_PS2->LY = receive[8];
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */


	
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_DAC1_Init();
  MX_LPUART1_UART_Init();
  MX_SPI2_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM4_Init();
  MX_TIM5_Init();
  MX_SPI3_Init();
  /* USER CODE BEGIN 2 */
	
// Initialize MFRC522 RFID module
MFRC522_Init();
//Initialize the Load Cell Module
  HX711_Init(&htim1);
//Initialize the PS2 Controller
 PS2Buttons PS2;
 PS2_Init(&htim4, &PS2);
 // Initializes timer 4, channel 2 used for delay()
HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
// Initializes timer 2, channel 3 used for the Servo PWM
HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1499); // Toggles Servo speed ( =1499 : stop;  <1499 : clockwise; >1499 : counter-clockwise)
// Initializes the DC Motor to off
HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 0);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	__HAL_TIM_SET_COMPARE(&htim2, TIM_CHANNEL_3, 1299);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_15, 1);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1_BOOST) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 25;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = RCC_PLLQ_DIV2;
  RCC_OscInitStruct.PLL.PLLR = RCC_PLLR_DIV2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_4) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief DAC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_DAC1_Init(void)
{

  /* USER CODE BEGIN DAC1_Init 0 */

  /* USER CODE END DAC1_Init 0 */

  DAC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN DAC1_Init 1 */

  /* USER CODE END DAC1_Init 1 */

  /** DAC Initialization
  */
  hdac1.Instance = DAC1;
  if (HAL_DAC_Init(&hdac1) != HAL_OK)
  {
    Error_Handler();
  }

  /** DAC channel OUT2 config
  */
  sConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;
  sConfig.DAC_Trigger = DAC_TRIGGER_T5_TRGO;
  sConfig.DAC_HighFrequency = DAC_HIGH_FREQUENCY_INTERFACE_MODE_DISABLE;
  sConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;
  sConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  sConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  if (HAL_DAC_ConfigChannel(&hdac1, &sConfig, DAC_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN DAC1_Init 2 */

  /* USER CODE END DAC1_Init 2 */

}

/**
  * @brief LPUART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_LPUART1_UART_Init(void)
{

  /* USER CODE BEGIN LPUART1_Init 0 */

  /* USER CODE END LPUART1_Init 0 */

  /* USER CODE BEGIN LPUART1_Init 1 */

  /* USER CODE END LPUART1_Init 1 */
  hlpuart1.Instance = LPUART1;
  hlpuart1.Init.BaudRate = 209700;
  hlpuart1.Init.WordLength = UART_WORDLENGTH_7B;
  hlpuart1.Init.StopBits = UART_STOPBITS_1;
  hlpuart1.Init.Parity = UART_PARITY_NONE;
  hlpuart1.Init.Mode = UART_MODE_TX_RX;
  hlpuart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  hlpuart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  hlpuart1.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  hlpuart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  hlpuart1.FifoMode = UART_FIFOMODE_DISABLE;
  if (HAL_UART_Init(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&hlpuart1, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&hlpuart1, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&hlpuart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN LPUART1_Init 2 */

  /* USER CODE END LPUART1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_HARD_OUTPUT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 7;
  hspi2.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI3_Init(void)
{

  /* USER CODE BEGIN SPI3_Init 0 */

  /* USER CODE END SPI3_Init 0 */

  /* USER CODE BEGIN SPI3_Init 1 */

  /* USER CODE END SPI3_Init 1 */
  /* SPI3 parameter configuration*/
  hspi3.Instance = SPI3;
  hspi3.Init.Mode = SPI_MODE_MASTER;
  hspi3.Init.Direction = SPI_DIRECTION_2LINES;
  hspi3.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi3.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi3.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi3.Init.NSS = SPI_NSS_SOFT;
  hspi3.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_256;
  hspi3.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi3.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi3.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi3.Init.CRCPolynomial = 7;
  hspi3.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi3.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI3_Init 2 */

  /* USER CODE END SPI3_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 65535;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 99;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 19999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */
  HAL_TIM_MspPostInit(&htim2);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 99;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */

}

/**
  * @brief TIM5 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM5_Init(void)
{

  /* USER CODE BEGIN TIM5_Init 0 */

  /* USER CODE END TIM5_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM5_Init 1 */

  /* USER CODE END TIM5_Init 1 */
  htim5.Instance = TIM5;
  htim5.Init.Prescaler = 0;
  htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim5.Init.Period = 22676;
  htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim5) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim5, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM5_Init 2 */

  /* USER CODE END TIM5_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMAMUX1_CLK_ENABLE();
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  HAL_PWREx_EnableVddIO2();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_4|RST_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, LOADCELL_SCK_Pin|D_C_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(PS2_SS_GPIO_Port, PS2_SS_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, PS2_SCK_Pin|PS2_MOSI_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(RFID_NSS_GPIO_Port, RFID_NSS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : PC3 PC4 RST_Pin */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_4|RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : LOADCELL_SCK_Pin D_C_Pin */
  GPIO_InitStruct.Pin = LOADCELL_SCK_Pin|D_C_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /*Configure GPIO pin : LOADCELL_DATA_Pin */
  GPIO_InitStruct.Pin = LOADCELL_DATA_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(LOADCELL_DATA_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_MISO_Pin */
  GPIO_InitStruct.Pin = PS2_MISO_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(PS2_MISO_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PS2_SS_Pin */
  GPIO_InitStruct.Pin = PS2_SS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(PS2_SS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PS2_SCK_Pin PS2_MOSI_Pin */
  GPIO_InitStruct.Pin = PS2_SCK_Pin|PS2_MOSI_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pin : SD_DET_Pin */
  GPIO_InitStruct.Pin = SD_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SD_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : RFID_NSS_Pin */
  GPIO_InitStruct.Pin = RFID_NSS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(RFID_NSS_GPIO_Port, &GPIO_InitStruct);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
