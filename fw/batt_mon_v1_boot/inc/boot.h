#ifndef INC_BOOT_H_
#define INC_BOOT_H_

#include "SI_EFM8BB1_Defs.inc"
#include "SI_EFM8BB1_Register_Enums.inc"

#define EFM8BB1_DEVICE EFM8BB10F8I_SOIC16
#include "SI_EFM8BB1_Devices.h"

// Parameters that describe the flash memory geometry
#define BL_FLASH0_LIMIT DEVICE_FLASH_SIZE
#define BL_FLASH0_PSIZE 512

// Define the size of the bootloader in flash pages
#define BL_PAGE_COUNT 1

// Define the starting address for the bootloader's code segments
#define BL_LIMIT_ADDRESS (BL_FLASH0_LIMIT - (BL_FLASH0_PSIZE * BL_PAGE_COUNT))
#define BL_START_ADDRESS (BL_FLASH0_LIMIT - BL_FLASH0_PSIZE)
#define BL_LOCK_ADDRESS  (BL_FLASH0_LIMIT - 1)

#define APP_MAX_PAGE ((BL_FLASH0_LIMIT / BL_FLASH0_PSIZE) - BL_PAGE_COUNT - 1)
#define APP_CRC_ADDR (((APP_MAX_PAGE+1) * BL_FLASH0_PSIZE) - 2)

#define CAL_START_ADDRESS (BL_LIMIT_ADDRESS - BL_FLASH0_PSIZE)

#define PKT_MINLEN 6
#define PKT_MAXLEN 73

#define PKT_INDEX_COBS	0
#define PKT_INDEX_LEN	1
#define PKT_INDEX_DST	2
#define PKT_INDEX_SEQ	3
#define PKT_INDEX_CMD	4
#define PKT_INDEX_DATA	5

#define BL_CMD_IDENT		0x30	// Send back info
#define BL_CMD_ERASE		0x31	// Erase flash page
#define BL_CMD_READ			0x32	// Read flash  (64 bytes)
#define BL_CMD_WRITE		0x33	// Write flash (64 bytes)
#define BL_CMD_RESET		0x34	// Soft reset

#define KEY_1 	0xA5
#define KEY_2 	0xF1

#define BL_START_PIN P2_B0
#define BL_SIGNATURE 0xA5
#define BL_DEFAULT_ADDR 0xFE

// Bootloader firmware revision number
#define BL_REVISION 0x01

#endif /* INC_BOOT_H_ */
