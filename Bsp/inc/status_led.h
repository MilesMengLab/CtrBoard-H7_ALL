#ifndef STATUS_LED_H
#define STATUS_LED_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

typedef enum {
    STATUS_LED_STOPPED = 0,
    STATUS_LED_STARTING,
    STATUS_LED_RUNNING,
    STATUS_LED_ERROR
} StatusLedState;

HAL_StatusTypeDef StatusLed_Set(StatusLedState state);
HAL_StatusTypeDef StatusLed_SetRgb(uint8_t red, uint8_t green, uint8_t blue);

#ifdef __cplusplus
}
#endif

#endif /* STATUS_LED_H */
