#ifndef QR_SCANNER_H
#define QR_SCANNER_H

#include "main.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define QR_SCANNER_RX_DMA_SIZE       128U
#define QR_SCANNER_DATA_MAX_LEN       64U

typedef enum
{
    QR_SCANNER_FRAME_CRLF = 0,
    QR_SCANNER_FRAME_IDLE,
    QR_SCANNER_FRAME_FIXED_LENGTH
} QRScanner_FrameMode_t;

typedef struct
{
    UART_HandleTypeDef *huart;

    uint8_t rx_dma[QR_SCANNER_RX_DMA_SIZE];

    uint8_t work_buf[QR_SCANNER_DATA_MAX_LEN];
    uint16_t work_len;

    uint8_t data[QR_SCANNER_DATA_MAX_LEN];
    volatile uint16_t data_len;
    volatile uint8_t available;

    QRScanner_FrameMode_t frame_mode;
    uint16_t fixed_length;

    volatile uint32_t frame_count;
    volatile uint32_t overflow_count;
    volatile uint32_t dropped_count;

} QRScanner_Handle_t;

HAL_StatusTypeDef QRScanner_Init(
    QRScanner_Handle_t *dev,
    UART_HandleTypeDef *huart
);

HAL_StatusTypeDef QRScanner_StartReceive(
    QRScanner_Handle_t *dev
);

HAL_StatusTypeDef QRScanner_SetFrameMode(
    QRScanner_Handle_t *dev,
    QRScanner_FrameMode_t mode,
    uint16_t fixed_length
);

void QRScanner_RxEventCallback(
    QRScanner_Handle_t *dev,
    UART_HandleTypeDef *huart,
    uint16_t size
);

void QRScanner_ErrorCallback(
    QRScanner_Handle_t *dev,
    UART_HandleTypeDef *huart
);

void QRScanner_ProcessBytes(
    QRScanner_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
);

uint8_t QRScanner_Available(
    const QRScanner_Handle_t *dev
);

uint16_t QRScanner_Read(
    QRScanner_Handle_t *dev,
    char *out,
    uint16_t out_size
);

void QRScanner_Clear(
    QRScanner_Handle_t *dev
);

uint8_t QRScanner_ParseDual3Digit(
    const char *text,
    uint16_t *first,
    uint16_t *second
);

#ifdef __cplusplus
}
#endif

#endif /* QR_SCANNER_H */