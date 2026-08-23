#include "w25q64.h"

static void W25Q64_CS_Low(W25Q64_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_RESET);
}

static void W25Q64_CS_High(W25Q64_Handle_t *dev)
{
    HAL_GPIO_WritePin(dev->cs_port, dev->cs_pin, GPIO_PIN_SET);
}

static uint8_t W25Q64_IsRangeValid(uint32_t address, uint32_t length)
{
    if (address >= W25Q64_TOTAL_SIZE_BYTES)
    {
        return 0U;
    }

    if (length == 0U)
    {
        return 1U;
    }

    if (length > (W25Q64_TOTAL_SIZE_BYTES - address))
    {
        return 0U;
    }

    return 1U;
}

static HAL_StatusTypeDef W25Q64_SendCommandAddress(
    W25Q64_Handle_t *dev,
    uint8_t command,
    uint32_t address
)
{
    uint8_t header[4];

    header[0] = command;
    header[1] = (uint8_t)(address >> 16);
    header[2] = (uint8_t)(address >> 8);
    header[3] = (uint8_t)address;

    return HAL_SPI_Transmit(
        dev->hspi,
        header,
        4U,
        W25Q64_SPI_TIMEOUT_MS
    );
}

HAL_StatusTypeDef W25Q64_Init(
    W25Q64_Handle_t *dev,
    SPI_HandleTypeDef *hspi,
    GPIO_TypeDef *cs_port,
    uint16_t cs_pin
)
{
    if (dev == NULL || hspi == NULL || cs_port == NULL)
    {
        return HAL_ERROR;
    }

    dev->hspi = hspi;
    dev->cs_port = cs_port;
    dev->cs_pin = cs_pin;

    dev->manufacturer_id = 0U;
    dev->memory_type = 0U;
    dev->capacity_id = 0U;

    W25Q64_CS_High(dev);

    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_ReadJEDEC_ID(W25Q64_Handle_t *dev)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t command = W25Q64_CMD_JEDEC_ID;
    uint8_t id[3] = {0U};
    HAL_StatusTypeDef status;

    W25Q64_CS_Low(dev);

    status = HAL_SPI_Transmit(
        dev->hspi,
        &command,
        1U,
        W25Q64_SPI_TIMEOUT_MS
    );

    if (status != HAL_OK)
    {
        W25Q64_CS_High(dev);
        return status;
    }

    status = HAL_SPI_Receive(
        dev->hspi,
        id,
        3U,
        W25Q64_SPI_TIMEOUT_MS
    );

    W25Q64_CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    dev->manufacturer_id = id[0];
    dev->memory_type = id[1];
    dev->capacity_id = id[2];

    return HAL_OK;
}

uint8_t W25Q64_IsDetected(const W25Q64_Handle_t *dev)
{
    if (dev == NULL)
    {
        return 0U;
    }

    return (
        dev->manufacturer_id == W25Q64_MANUFACTURER_ID_WINBOND &&
        dev->memory_type == W25Q64_MEMORY_TYPE_EXPECTED &&
        dev->capacity_id == W25Q64_CAPACITY_ID_64MBIT
    ) ? 1U : 0U;
}

HAL_StatusTypeDef W25Q64_ReadStatusRegister1(
    W25Q64_Handle_t *dev,
    uint8_t *status_reg
)
{
    if (dev == NULL ||
        dev->hspi == NULL ||
        dev->cs_port == NULL ||
        status_reg == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t command = W25Q64_CMD_READ_SR1;
    HAL_StatusTypeDef status;

    W25Q64_CS_Low(dev);

    status = HAL_SPI_Transmit(
        dev->hspi,
        &command,
        1U,
        W25Q64_SPI_TIMEOUT_MS
    );

    if (status != HAL_OK)
    {
        W25Q64_CS_High(dev);
        return status;
    }

    status = HAL_SPI_Receive(
        dev->hspi,
        status_reg,
        1U,
        W25Q64_SPI_TIMEOUT_MS
    );

    W25Q64_CS_High(dev);

    return status;
}

HAL_StatusTypeDef W25Q64_WaitBusy(
    W25Q64_Handle_t *dev,
    uint32_t timeout_ms
)
{
    if (dev == NULL || dev->hspi == NULL)
    {
        return HAL_ERROR;
    }

    uint32_t start_tick = HAL_GetTick();
    uint8_t status_reg = 0U;

    while (1)
    {
        HAL_StatusTypeDef status =
            W25Q64_ReadStatusRegister1(dev, &status_reg);

        if (status != HAL_OK)
        {
            return status;
        }

        if ((status_reg & W25Q64_SR1_BUSY_MASK) == 0U)
        {
            return HAL_OK;
        }

        if (timeout_ms != HAL_MAX_DELAY &&
            (HAL_GetTick() - start_tick) >= timeout_ms)
        {
            return HAL_TIMEOUT;
        }

        HAL_Delay(1U);
    }
}

HAL_StatusTypeDef W25Q64_WriteEnable(W25Q64_Handle_t *dev)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL)
    {
        return HAL_ERROR;
    }

    uint8_t command = W25Q64_CMD_WRITE_ENABLE;
    HAL_StatusTypeDef status;

    W25Q64_CS_Low(dev);

    status = HAL_SPI_Transmit(
        dev->hspi,
        &command,
        1U,
        W25Q64_SPI_TIMEOUT_MS
    );

    W25Q64_CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    uint8_t status_reg = 0U;

    status = W25Q64_ReadStatusRegister1(dev, &status_reg);

    if (status != HAL_OK)
    {
        return status;
    }

    if ((status_reg & W25Q64_SR1_WEL_MASK) == 0U)
    {
        return HAL_ERROR;
    }

    return HAL_OK;
}

uint8_t W25Q64_IsWriteEnabled(W25Q64_Handle_t *dev)
{
    uint8_t status_reg = 0U;

    if (dev == NULL)
    {
        return 0U;
    }

    if (W25Q64_ReadStatusRegister1(dev, &status_reg) != HAL_OK)
    {
        return 0U;
    }

    return ((status_reg & W25Q64_SR1_WEL_MASK) != 0U) ? 1U : 0U;
}

HAL_StatusTypeDef W25Q64_Read(
    W25Q64_Handle_t *dev,
    uint32_t address,
    uint8_t *data,
    uint32_t length
)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL)
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    if (data == NULL || W25Q64_IsRangeValid(address, length) == 0U)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
        W25Q64_WaitBusy(dev, W25Q64_PROGRAM_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        return status;
    }

    W25Q64_CS_Low(dev);

    status = W25Q64_SendCommandAddress(
        dev,
        W25Q64_CMD_READ_DATA,
        address
    );

    if (status != HAL_OK)
    {
        W25Q64_CS_High(dev);
        return status;
    }

    uint32_t remaining = length;
    uint8_t *write_ptr = data;

    while (remaining > 0U)
    {
        uint16_t chunk =
            (remaining > 65535UL) ?
            65535U :
            (uint16_t)remaining;

        status = HAL_SPI_Receive(
            dev->hspi,
            write_ptr,
            chunk,
            W25Q64_SPI_TIMEOUT_MS
        );

        if (status != HAL_OK)
        {
            W25Q64_CS_High(dev);
            return status;
        }

        write_ptr += chunk;
        remaining -= chunk;
    }

    W25Q64_CS_High(dev);

    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_PageProgram(
    W25Q64_Handle_t *dev,
    uint32_t address,
    const uint8_t *data,
    uint16_t length
)
{
    if (dev == NULL ||
        dev->hspi == NULL ||
        dev->cs_port == NULL ||
        data == NULL)
    {
        return HAL_ERROR;
    }

    if (length == 0U || length > W25Q64_PAGE_SIZE)
    {
        return HAL_ERROR;
    }

    if (W25Q64_IsRangeValid(address, (uint32_t)length) == 0U)
    {
        return HAL_ERROR;
    }

    uint16_t page_offset =
        (uint16_t)(address % W25Q64_PAGE_SIZE);

    if (((uint32_t)page_offset + (uint32_t)length) >
        W25Q64_PAGE_SIZE)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
        W25Q64_WaitBusy(dev, W25Q64_PROGRAM_TIMEOUT_MS);

    if (status != HAL_OK)
    {
        return status;
    }

    status = W25Q64_WriteEnable(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    W25Q64_CS_Low(dev);

    status = W25Q64_SendCommandAddress(
        dev,
        W25Q64_CMD_PAGE_PROGRAM,
        address
    );

    if (status != HAL_OK)
    {
        W25Q64_CS_High(dev);
        return status;
    }

    status = HAL_SPI_Transmit(
        dev->hspi,
        (uint8_t *)data,
        length,
        W25Q64_SPI_TIMEOUT_MS
    );

    W25Q64_CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    return W25Q64_WaitBusy(
        dev,
        W25Q64_PROGRAM_TIMEOUT_MS
    );
}

HAL_StatusTypeDef W25Q64_Write(
    W25Q64_Handle_t *dev,
    uint32_t address,
    const uint8_t *data,
    uint32_t length
)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL)
    {
        return HAL_ERROR;
    }

    if (length == 0U)
    {
        return HAL_OK;
    }

    if (data == NULL || W25Q64_IsRangeValid(address, length) == 0U)
    {
        return HAL_ERROR;
    }

    uint32_t current_address = address;
    const uint8_t *read_ptr = data;
    uint32_t remaining = length;

    while (remaining > 0U)
    {
        uint32_t page_offset =
            current_address % W25Q64_PAGE_SIZE;

        uint32_t page_remaining =
            W25Q64_PAGE_SIZE - page_offset;

        uint16_t chunk =
            (remaining > page_remaining) ?
            (uint16_t)page_remaining :
            (uint16_t)remaining;

        HAL_StatusTypeDef status =
            W25Q64_PageProgram(
                dev,
                current_address,
                read_ptr,
                chunk
            );

        if (status != HAL_OK)
        {
            return status;
        }

        current_address += chunk;
        read_ptr += chunk;
        remaining -= chunk;
    }

    return HAL_OK;
}

HAL_StatusTypeDef W25Q64_EraseSector(
    W25Q64_Handle_t *dev,
    uint32_t address
)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL)
    {
        return HAL_ERROR;
    }

    if (address >= W25Q64_TOTAL_SIZE_BYTES)
    {
        return HAL_ERROR;
    }

    uint32_t sector_address =
        address & ~(W25Q64_SECTOR_SIZE - 1UL);

    HAL_StatusTypeDef status =
        W25Q64_WaitBusy(
            dev,
            W25Q64_SECTOR_ERASE_TIMEOUT_MS
        );

    if (status != HAL_OK)
    {
        return status;
    }

    status = W25Q64_WriteEnable(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    W25Q64_CS_Low(dev);

    status = W25Q64_SendCommandAddress(
        dev,
        W25Q64_CMD_SECTOR_ERASE_4K,
        sector_address
    );

    W25Q64_CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    return W25Q64_WaitBusy(
        dev,
        W25Q64_SECTOR_ERASE_TIMEOUT_MS
    );
}

HAL_StatusTypeDef W25Q64_EraseChip(
    W25Q64_Handle_t *dev
)
{
    if (dev == NULL || dev->hspi == NULL || dev->cs_port == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
        W25Q64_WaitBusy(
            dev,
            W25Q64_CHIP_ERASE_TIMEOUT_MS
        );

    if (status != HAL_OK)
    {
        return status;
    }

    status = W25Q64_WriteEnable(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    uint8_t command = W25Q64_CMD_CHIP_ERASE;

    W25Q64_CS_Low(dev);

    status = HAL_SPI_Transmit(
        dev->hspi,
        &command,
        1U,
        W25Q64_SPI_TIMEOUT_MS
    );

    W25Q64_CS_High(dev);

    if (status != HAL_OK)
    {
        return status;
    }

    return W25Q64_WaitBusy(
        dev,
        W25Q64_CHIP_ERASE_TIMEOUT_MS
    );
}