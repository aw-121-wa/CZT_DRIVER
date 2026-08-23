#include "QR_SCANNER.h"

#include <string.h>

static void qr_scanner_reset_work(QRScanner_Handle_t *dev)
{
    dev->work_len = 0U;
}

static void qr_scanner_publish(QRScanner_Handle_t *dev)
{
    if (dev->work_len == 0U)
        return;

    if (dev->available)
    {
        dev->dropped_count++;
        qr_scanner_reset_work(dev);
        return;
    }

    uint16_t len = dev->work_len;

    if (len >= QR_SCANNER_DATA_MAX_LEN)
        len = QR_SCANNER_DATA_MAX_LEN - 1U;

    memcpy(dev->data, dev->work_buf, len);
    dev->data[len] = '\0';

    dev->data_len = len;
    dev->available = 1U;
    dev->frame_count++;

    qr_scanner_reset_work(dev);
}

static void qr_scanner_push_byte(
    QRScanner_Handle_t *dev,
    uint8_t byte
)
{
    /*
     * 固定长度模式下常见扫描器仍可能附带 CR/LF。
     * 空闲状态直接忽略，避免它们污染下一帧。
     */
    if (dev->frame_mode == QR_SCANNER_FRAME_FIXED_LENGTH &&
        dev->work_len == 0U &&
        (byte == '\r' || byte == '\n'))
    {
        return;
    }

    if (dev->frame_mode == QR_SCANNER_FRAME_CRLF)
    {
        if (byte == '\r' || byte == '\n')
        {
            qr_scanner_publish(dev);
            return;
        }
    }

    if (dev->work_len >= QR_SCANNER_DATA_MAX_LEN - 1U)
    {
        dev->overflow_count++;
        qr_scanner_reset_work(dev);
        return;
    }

    dev->work_buf[dev->work_len++] = byte;

    if (dev->frame_mode == QR_SCANNER_FRAME_FIXED_LENGTH &&
        dev->fixed_length > 0U &&
        dev->work_len >= dev->fixed_length)
    {
        qr_scanner_publish(dev);
    }
}

HAL_StatusTypeDef QRScanner_Init(
    QRScanner_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL || huart == NULL)
        return HAL_ERROR;

    memset(dev, 0, sizeof(*dev));

    dev->huart = huart;
    dev->frame_mode = QR_SCANNER_FRAME_CRLF;

    if (huart->hdmarx == NULL)
        return HAL_ERROR;

    return QRScanner_StartReceive(dev);
}

HAL_StatusTypeDef QRScanner_StartReceive(
    QRScanner_Handle_t *dev
)
{
    if (dev == NULL ||
        dev->huart == NULL ||
        dev->huart->hdmarx == NULL)
    {
        return HAL_ERROR;
    }

    HAL_StatusTypeDef status =
        HAL_UARTEx_ReceiveToIdle_DMA(
            dev->huart,
            dev->rx_dma,
            QR_SCANNER_RX_DMA_SIZE
        );

    if (status == HAL_OK)
    {
        __HAL_DMA_DISABLE_IT(
            dev->huart->hdmarx,
            DMA_IT_HT
        );
    }

    return status;
}

HAL_StatusTypeDef QRScanner_SetFrameMode(
    QRScanner_Handle_t *dev,
    QRScanner_FrameMode_t mode,
    uint16_t fixed_length
)
{
    if (dev == NULL)
        return HAL_ERROR;

    if (mode > QR_SCANNER_FRAME_FIXED_LENGTH)
        return HAL_ERROR;

    if (mode == QR_SCANNER_FRAME_FIXED_LENGTH)
    {
        if (fixed_length == 0U ||
            fixed_length >= QR_SCANNER_DATA_MAX_LEN)
        {
            return HAL_ERROR;
        }
    }

    dev->frame_mode = mode;
    dev->fixed_length =
        (mode == QR_SCANNER_FRAME_FIXED_LENGTH)
        ? fixed_length
        : 0U;

    qr_scanner_reset_work(dev);

    return HAL_OK;
}

void QRScanner_ProcessBytes(
    QRScanner_Handle_t *dev,
    const uint8_t *data,
    uint16_t len
)
{
    if (dev == NULL || data == NULL)
        return;

    for (uint16_t i = 0U; i < len; i++)
    {
        qr_scanner_push_byte(dev, data[i]);
    }
}

void QRScanner_RxEventCallback(
    QRScanner_Handle_t *dev,
    UART_HandleTypeDef *huart,
    uint16_t size
)
{
    if (dev == NULL ||
        huart == NULL ||
        huart != dev->huart)
    {
        return;
    }

    if (size > QR_SCANNER_RX_DMA_SIZE)
        size = QR_SCANNER_RX_DMA_SIZE;

    QRScanner_ProcessBytes(
        dev,
        dev->rx_dma,
        size
    );

    if (dev->frame_mode == QR_SCANNER_FRAME_IDLE)
    {
        qr_scanner_publish(dev);
    }

    (void)QRScanner_StartReceive(dev);
}

void QRScanner_ErrorCallback(
    QRScanner_Handle_t *dev,
    UART_HandleTypeDef *huart
)
{
    if (dev == NULL ||
        huart == NULL ||
        huart != dev->huart)
    {
        return;
    }

    (void)HAL_UART_AbortReceive(dev->huart);

    qr_scanner_reset_work(dev);

    (void)QRScanner_StartReceive(dev);
}

uint8_t QRScanner_Available(
    const QRScanner_Handle_t *dev
)
{
    if (dev == NULL)
        return 0U;

    return dev->available;
}

uint16_t QRScanner_Read(
    QRScanner_Handle_t *dev,
    char *out,
    uint16_t out_size
)
{
    if (dev == NULL ||
        out == NULL ||
        out_size == 0U ||
        !dev->available)
    {
        return 0U;
    }

    uint16_t len = dev->data_len;

    if (len >= out_size)
        len = out_size - 1U;

    memcpy(out, dev->data, len);
    out[len] = '\0';

    dev->available = 0U;
    dev->data_len = 0U;

    return len;
}

void QRScanner_Clear(
    QRScanner_Handle_t *dev
)
{
    if (dev == NULL)
        return;

    dev->available = 0U;
    dev->data_len = 0U;
}

static uint8_t qr_is_digit(char c)
{
    return (c >= '0' && c <= '9') ? 1U : 0U;
}

uint8_t QRScanner_ParseDual3Digit(
    const char *text,
    uint16_t *first,
    uint16_t *second
)
{
    if (text == NULL ||
        first == NULL ||
        second == NULL)
    {
        return 0U;
    }

    if (strlen(text) < 7U)
        return 0U;

    for (uint8_t i = 0U; i < 3U; i++)
    {
        if (!qr_is_digit(text[i]))
            return 0U;
    }

    for (uint8_t i = 4U; i < 7U; i++)
    {
        if (!qr_is_digit(text[i]))
            return 0U;
    }

    *first =
        (uint16_t)(text[0] - '0') * 100U +
        (uint16_t)(text[1] - '0') * 10U  +
        (uint16_t)(text[2] - '0');

    *second =
        (uint16_t)(text[4] - '0') * 100U +
        (uint16_t)(text[5] - '0') * 10U  +
        (uint16_t)(text[6] - '0');

    return 1U;
}