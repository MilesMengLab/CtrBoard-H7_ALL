#include "status_led.h"

#include "spi.h"

#define STATUS_LED_SPI              hspi6
#define WS2812_ZERO_SYMBOL          0xC0U
#define WS2812_ONE_SYMBOL           0xF0U
#define WS2812_RESET_BYTES          240U
#define STATUS_LED_BRIGHTNESS       10U
#define STATUS_LED_SPI_TIMEOUT_MS   10U

static uint8_t ws2812_reset_buffer[WS2812_RESET_BYTES];

static void ws2812_encode_byte(uint8_t value, uint8_t output[8])
{
    for (uint32_t bit = 0U; bit < 8U; ++bit) {
        output[bit] = ((value & (0x80U >> bit)) != 0U)
                          ? WS2812_ONE_SYMBOL
                          : WS2812_ZERO_SYMBOL;
    }
}

HAL_StatusTypeDef StatusLed_SetRgb(uint8_t red, uint8_t green, uint8_t blue)
{
    uint8_t frame[24];
    HAL_StatusTypeDef status;

    /* WS2812 uses GRB byte order. */
    ws2812_encode_byte(green, &frame[0]);
    ws2812_encode_byte(red, &frame[8]);
    ws2812_encode_byte(blue, &frame[16]);

    status = HAL_SPI_Transmit(&STATUS_LED_SPI,
                              ws2812_reset_buffer,
                              sizeof(ws2812_reset_buffer),
                              STATUS_LED_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_SPI_Transmit(&STATUS_LED_SPI,
                              frame,
                              sizeof(frame),
                              STATUS_LED_SPI_TIMEOUT_MS);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_SPI_Transmit(&STATUS_LED_SPI,
                            ws2812_reset_buffer,
                            sizeof(ws2812_reset_buffer),
                            STATUS_LED_SPI_TIMEOUT_MS);
}

HAL_StatusTypeDef StatusLed_Set(StatusLedState state)
{
    switch (state) {
    case STATUS_LED_STARTING:
        return StatusLed_SetRgb(STATUS_LED_BRIGHTNESS,
                                STATUS_LED_BRIGHTNESS,
                                0U);

    case STATUS_LED_RUNNING:
        return StatusLed_SetRgb(0U, STATUS_LED_BRIGHTNESS, 0U);

    case STATUS_LED_ERROR:
        return StatusLed_SetRgb(0U, 0U, STATUS_LED_BRIGHTNESS);

    case STATUS_LED_STOPPED:
    default:
        return StatusLed_SetRgb(STATUS_LED_BRIGHTNESS, 0U, 0U);
    }
}
