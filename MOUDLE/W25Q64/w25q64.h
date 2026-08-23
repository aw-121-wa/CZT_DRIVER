#ifndef W25Q64_H
#define W25Q64_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define W25Q64_TOTAL_SIZE_BYTES        (8UL * 1024UL * 1024UL)
#define W25Q64_PAGE_SIZE               256U
#define W25Q64_SECTOR_SIZE             4096UL
#define W25Q64_MAX_ADDRESS             (W25Q64_TOTAL_SIZE_BYTES - 1UL)

#define W25Q64_CMD_WRITE_ENABLE        0x06U
#define W25Q64_CMD_READ_SR1            0x05U
#define W25Q64_CMD_READ_DATA           0x03U
#define W25Q64_CMD_PAGE_PROGRAM        0x02U
#define W25Q64_CMD_SECTOR_ERASE_4K     0x20U
#define W25Q64_CMD_CHIP_ERASE          0xC7U
#define W25Q64_CMD_JEDEC_ID            0x9FU

#define W25Q64_SR1_BUSY_MASK           0x01U
#define W25Q64_SR1_WEL_MASK            0x02U

#define W25Q64_MANUFACTURER_ID_WINBOND 0xEFU
#define W25Q64_MEMORY_TYPE_EXPECTED    0x40U
#define W25Q64_CAPACITY_ID_64MBIT      0x17U

#ifndef W25Q64_SPI_TIMEOUT_MS
#define W25Q64_SPI_TIMEOUT_MS          100U
#endif

#ifndef W25Q64_PROGRAM_TIMEOUT_MS
#define W25Q64_PROGRAM_TIMEOUT_MS      1000U
#endif

#ifndef W25Q64_SECTOR_ERASE_TIMEOUT_MS
#define W25Q64_SECTOR_ERASE_TIMEOUT_MS 5000U
#endif

#ifndef W25Q64_CHIP_ERASE_TIMEOUT_MS
#define W25Q64_CHIP_ERASE_TIMEOUT_MS   300000U
#endif

typedef struct
{
    SPI_HandleTypeDef *hspi;
    GPIO_TypeDef *cs_port;
    uint16_t cs_pin;

    uint8_t manufacturer_id;
    uint8_t memory_type;
    uint8_t capacity_id;

} W25Q64_Handle_t;

HAL_StatusTypeDef W25Q64_Init(
    W25Q64_Handle_t *dev,
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin
);

HAL_StatusTypeDef W25Q64_ReadJEDEC_ID(
    W25Q64_Handle_t *dev
);

uint8_t W25Q64_IsDetected(
    const W25Q64_Handle_t *dev
);

HAL_StatusTypeDef W25Q64_ReadStatusRegister1(
    W25Q64_Handle_t *dev,
    uint8_t *status_reg
);

HAL_StatusTypeDef W25Q64_WaitBusy(
    W25Q64_Handle_t *dev,
    uint32_t timeout_ms
);

HAL_StatusTypeDef W25Q64_WriteEnable(
    W25Q64_Handle_t *dev
);

uint8_t W25Q64_IsWriteEnabled(
    W25Q64_Handle_t *dev
);

HAL_StatusTypeDef W25Q64_Read(
    W25Q64_Handle_t *dev,
    uint32_t address,
    uint8_t *data,
    uint32_t length
);

HAL_StatusTypeDef W25Q64_PageProgram(
    W25Q64_Handle_t *dev,
    uint32_t address,
    const uint8_t *data,
    uint16_t length
);

HAL_StatusTypeDef W25Q64_Write(
    W25Q64_Handle_t *dev,
    uint32_t address,
    const uint8_t *data,
    uint32_t length
);

HAL_StatusTypeDef W25Q64_EraseSector(
    W25Q64_Handle_t *dev,
    uint32_t address
);

HAL_StatusTypeDef W25Q64_EraseChip(
    W25Q64_Handle_t *dev
);

#ifdef __cplusplus
}
#endif

#endif /* W25Q64_H */