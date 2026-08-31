#include "Algorithm_Example.h"

#include "AK70_Moter.h"
#include "fdcan.h"
#include "main.h"
#include "status_led.h"

#include <stdbool.h>
#include <math.h>

#define ALGORITHM_MOTOR_ID               1U
#define ALGORITHM_SWING_PERIOD_MS        2000U
#define ALGORITHM_CONTROL_PERIOD_MS      10U
#define ALGORITHM_KEY_DEBOUNCE_MS        20U
#define ALGORITHM_STARTUP_RAMP_MS        2000U
#define ALGORITHM_FEEDBACK_STARTUP_MS    100U
#define ALGORITHM_FEEDBACK_TIMEOUT_MS    50U
#define ALGORITHM_SWING_AMPLITUDE_RAD    1.5707963f
#define ALGORITHM_POSITION_LIMIT_RAD     1.5707963f
#define ALGORITHM_POSITION_MARGIN_RAD    0.1745329f
#define ALGORITHM_POSITION_KP            25.0f
#define ALGORITHM_POSITION_KD            1.0f
#define ALGORITHM_PI                     3.1415926f

#define ALGORITHM_KEY_PRESSED_STATE      GPIO_PIN_RESET
#define ALGORITHM_KEY_GPIO_PORT          GPIOA
#define ALGORITHM_KEY_PIN                GPIO_PIN_15

static uint32_t algorithm_start_tick;
static uint32_t algorithm_last_control_tick;
static uint32_t algorithm_key_press_tick;
static bool algorithm_initialized;
static bool algorithm_stopped = true;
static bool algorithm_stopping;
static bool algorithm_zero_command_sent;
static bool algorithm_key_debouncing;
static bool algorithm_key_press_latched;
static bool algorithm_feedback_confirmed;
static AK70_MIT_Feedback algorithm_feedback;
static volatile uint32_t algorithm_feedback_tick;
static volatile bool algorithm_feedback_valid;

static void algorithm_set_status(StatusLedState state)
{
    (void)StatusLed_Set(state);
}

static void algorithm_limit_motion(float *position, float *velocity)
{
    if (*position >= ALGORITHM_POSITION_LIMIT_RAD) {
        *position = ALGORITHM_POSITION_LIMIT_RAD;
        if (*velocity > 0.0f) {
            *velocity = 0.0f;
        }
    } else if (*position <= -ALGORITHM_POSITION_LIMIT_RAD) {
        *position = -ALGORITHM_POSITION_LIMIT_RAD;
        if (*velocity < 0.0f) {
            *velocity = 0.0f;
        }
    }
}

static bool algorithm_key_press_event(uint32_t now)
{
    if (HAL_GPIO_ReadPin(ALGORITHM_KEY_GPIO_PORT,
                         ALGORITHM_KEY_PIN) != ALGORITHM_KEY_PRESSED_STATE) {
        algorithm_key_debouncing = false;
        algorithm_key_press_latched = false;
        return false;
    }

    if (!algorithm_key_debouncing) {
        algorithm_key_debouncing = true;
        algorithm_key_press_tick = now;
        return false;
    }

    if (!algorithm_key_press_latched &&
        ((now - algorithm_key_press_tick) >= ALGORITHM_KEY_DEBOUNCE_MS)) {
        algorithm_key_press_latched = true;
        return true;
    }

    return false;
}

static void algorithm_process_stop(void)
{
    if (!algorithm_zero_command_sent) {
        if (AK70_MIT_Control(&hfdcan1,
                             ALGORITHM_MOTOR_ID,
                             &AK70_10_MIT_PARAM,
                             0.0f,
                             0.0f,
                             0.0f,
                             0.0f,
                             0.0f) != HAL_OK) {
            return;
        }

        algorithm_zero_command_sent = true;
    }

    if (AK70_MIT_Exit(&hfdcan1, ALGORITHM_MOTOR_ID) == HAL_OK) {
        algorithm_stopping = false;
        algorithm_stopped = true;
    }
}

static void algorithm_request_stop(StatusLedState status)
{
    if (algorithm_initialized && !algorithm_stopped) {
        algorithm_stopping = true;
        algorithm_zero_command_sent = false;
    }

    algorithm_set_status(status);
}

static void algorithm_enter_fault(void)
{
    algorithm_request_stop(STATUS_LED_ERROR);
    algorithm_process_stop();
}

static HAL_StatusTypeDef algorithm_start_motor(void)
{
    HAL_StatusTypeDef status;

    algorithm_feedback_valid = false;
    algorithm_feedback_confirmed = false;
    algorithm_set_status(STATUS_LED_STARTING);

    HAL_Delay(100);
    status = AK70_MIT_Enter(&hfdcan1, ALGORITHM_MOTOR_ID);
    if (status != HAL_OK) {
        return status;
    }

    HAL_Delay(100);
    /* Ignore any isolated response received before periodic control starts. */
    algorithm_feedback_valid = false;
    /*
     * Do not reset the motor zero position on every start.
     * Set the fixed mechanical zero in a separate calibration procedure.
     *
     * status = AK70_MIT_SetZero(&hfdcan1, ALGORITHM_MOTOR_ID);
     * if (status != HAL_OK) {
     *     (void)AK70_MIT_Exit(&hfdcan1, ALGORITHM_MOTOR_ID);
     *     return status;
     * }
     * HAL_Delay(100);
     */

    algorithm_start_tick = HAL_GetTick();
    algorithm_last_control_tick = algorithm_start_tick;
    algorithm_stopping = false;
    algorithm_stopped = false;

    return HAL_OK;
}

HAL_StatusTypeDef Algorithm_Example_Init(void)
{
    HAL_StatusTypeDef status;

    algorithm_set_status(STATUS_LED_STOPPED);
    algorithm_initialized = false;
    algorithm_stopped = true;
    algorithm_stopping = false;
    algorithm_zero_command_sent = false;
    algorithm_key_debouncing = false;
    algorithm_key_press_latched = false;
    algorithm_feedback_confirmed = false;
    algorithm_feedback_valid = false;

    status = AK70_CAN_Start(&hfdcan1, ALGORITHM_MOTOR_ID);
    if (status != HAL_OK) {
        algorithm_set_status(STATUS_LED_ERROR);
        return status;
    }

    algorithm_initialized = true;

    return HAL_OK;
}

bool Algorithm_Example_GetFeedback(AK70_MIT_Feedback *feedback,
                                   uint32_t *feedback_age_ms)
{
    uint32_t primask;
    uint32_t feedback_tick;
    bool valid;

    if (feedback == NULL) {
        return false;
    }

    primask = __get_PRIMASK();
    __disable_irq();
    valid = algorithm_feedback_valid;
    if (valid) {
        *feedback = algorithm_feedback;
    }
    feedback_tick = algorithm_feedback_tick;
    if (primask == 0U) {
        __enable_irq();
    }

    if (valid && (feedback_age_ms != NULL)) {
        *feedback_age_ms = HAL_GetTick() - feedback_tick;
    }

    return valid;
}

void Algorithm_Example_Stop(void)
{
    if (!algorithm_initialized || algorithm_stopped || algorithm_stopping) {
        return;
    }

    algorithm_request_stop(STATUS_LED_STOPPED);
}

void Algorithm_Example_Run(void)
{
    uint32_t now;
    uint32_t period_tick;
    float time_s;
    float omega;
    float position;
    float velocity;
    float ramp;
    float ramp_rate;
    AK70_MIT_Feedback feedback;
    uint32_t feedback_age_ms;

    if (!algorithm_initialized) {
        return;
    }

    now = HAL_GetTick();
    if (algorithm_key_press_event(now)) {
        if (algorithm_stopped) {
            if (algorithm_start_motor() != HAL_OK) {
                algorithm_set_status(STATUS_LED_ERROR);
            }
        } else if (!algorithm_stopping) {
            Algorithm_Example_Stop();
            algorithm_process_stop();
        }
        return;
    }

    if (algorithm_stopping) {
        algorithm_process_stop();
        return;
    }

    if (algorithm_stopped) {
        return;
    }

    if (Algorithm_Example_GetFeedback(&feedback, &feedback_age_ms)) {
        if ((feedback_age_ms > ALGORITHM_FEEDBACK_TIMEOUT_MS) ||
            (feedback.error != 0U) ||
            (fabsf(feedback.position_rad) >
             (ALGORITHM_POSITION_LIMIT_RAD + ALGORITHM_POSITION_MARGIN_RAD))) {
            algorithm_enter_fault();
            return;
        }

        if (!algorithm_feedback_confirmed) {
            algorithm_feedback_confirmed = true;
            algorithm_set_status(STATUS_LED_RUNNING);
        }
    } else if ((now - algorithm_start_tick) >= ALGORITHM_FEEDBACK_STARTUP_MS) {
        algorithm_enter_fault();
        return;
    }

    if ((now - algorithm_last_control_tick) < ALGORITHM_CONTROL_PERIOD_MS) {
        return;
    }

    period_tick = (now - algorithm_start_tick) % ALGORITHM_SWING_PERIOD_MS;
    time_s = (float)period_tick * 0.001f;
    omega = 2.0f * ALGORITHM_PI / ((float)ALGORITHM_SWING_PERIOD_MS * 0.001f);
    if ((now - algorithm_start_tick) < ALGORITHM_STARTUP_RAMP_MS) {
        ramp = (float)(now - algorithm_start_tick) /
               (float)ALGORITHM_STARTUP_RAMP_MS;
        ramp_rate = 1000.0f / (float)ALGORITHM_STARTUP_RAMP_MS;
    } else {
        ramp = 1.0f;
        ramp_rate = 0.0f;
    }

    position = ALGORITHM_SWING_AMPLITUDE_RAD * ramp * sinf(omega * time_s);
    velocity = ALGORITHM_SWING_AMPLITUDE_RAD *
               ((ramp * omega * cosf(omega * time_s)) +
                (ramp_rate * sinf(omega * time_s)));
    algorithm_limit_motion(&position, &velocity);

    if (AK70_MIT_Control(&hfdcan1,
                         ALGORITHM_MOTOR_ID,
                         &AK70_10_MIT_PARAM,
                         position,
                         velocity,
                         ALGORITHM_POSITION_KP,
                         ALGORITHM_POSITION_KD,
                         0.0f) != HAL_OK) {
        algorithm_enter_fault();
        return;
    }

    algorithm_last_control_tick = now;
}

void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan,
                               uint32_t rx_fifo0_its)
{
    FDCAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    AK70_MIT_Feedback feedback;

    if ((hfdcan == NULL) ||
        (hfdcan->Instance != FDCAN1) ||
        ((rx_fifo0_its & FDCAN_IT_RX_FIFO0_NEW_MESSAGE) == 0U)) {
        return;
    }

    while (HAL_FDCAN_GetRxFifoFillLevel(hfdcan, FDCAN_RX_FIFO0) > 0U) {
        if (HAL_FDCAN_GetRxMessage(hfdcan,
                                  FDCAN_RX_FIFO0,
                                  &rx_header,
                                  rx_data) != HAL_OK) {
            break;
        }

        if ((rx_header.IdType != FDCAN_STANDARD_ID) ||
            (rx_header.Identifier != ALGORITHM_MOTOR_ID) ||
            (rx_header.RxFrameType != FDCAN_DATA_FRAME) ||
            (rx_header.DataLength != FDCAN_DLC_BYTES_8)) {
            continue;
        }

        AK70_MIT_ParseFeedback(rx_data, &AK70_10_MIT_PARAM, &feedback);
        if (feedback.id != ALGORITHM_MOTOR_ID) {
            continue;
        }

        algorithm_feedback = feedback;
        algorithm_feedback_tick = HAL_GetTick();
        algorithm_feedback_valid = true;
    }
}
