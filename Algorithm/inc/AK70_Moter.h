#ifndef __AK70_MOTER_H__
#define __AK70_MOTER_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "fdcan.h"
#include <stdint.h>

typedef enum {
    AK70_CAN_PACKET_SET_DUTY = 0,
    AK70_CAN_PACKET_SET_CURRENT,
    AK70_CAN_PACKET_SET_CURRENT_BRAKE,
    AK70_CAN_PACKET_SET_RPM,
    AK70_CAN_PACKET_SET_POS,
    AK70_CAN_PACKET_SET_ORIGIN_HERE,
    AK70_CAN_PACKET_SET_POS_SPD,
    AK70_CAN_PACKET_SET_MIT = 8
} AK70_CAN_PacketId;

typedef enum {
    AK70_ORIGIN_TEMPORARY = 0,
    AK70_ORIGIN_PERMANENT = 1
} AK70_OriginMode;

typedef struct {
    float p_min;
    float p_max;
    float v_min;
    float v_max;
    float kp_min;
    float kp_max;
    float kd_min;
    float kd_max;
    float t_min;
    float t_max;
} AK70_MIT_Param;

typedef struct {
    uint8_t id;
    float position_rad;
    float velocity_rad_s;
    float torque_nm;
    int16_t temperature_c;
    uint8_t error;
} AK70_MIT_Feedback;

typedef struct {
    float position_deg;
    float speed_erpm;
    float current_a;
    int8_t temperature_c;
    uint8_t error;
} AK70_Servo_Feedback;

extern const AK70_MIT_Param AK70_10_MIT_PARAM;

HAL_StatusTypeDef AK70_CAN_Start(FDCAN_HandleTypeDef *hfdcan, uint16_t feedback_id);
HAL_StatusTypeDef AK70_CAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, uint16_t feedback_id);

HAL_StatusTypeDef AK70_MIT_Enter(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
HAL_StatusTypeDef AK70_MIT_Exit(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
HAL_StatusTypeDef AK70_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id);
HAL_StatusTypeDef AK70_MIT_Control(FDCAN_HandleTypeDef *hfdcan,
                                   uint16_t motor_id,
                                   const AK70_MIT_Param *param,
                                   float position_rad,
                                   float velocity_rad_s,
                                   float kp,
                                   float kd,
                                   float torque_nm);
void AK70_MIT_ParseFeedback(const uint8_t data[8],
                            const AK70_MIT_Param *param,
                            AK70_MIT_Feedback *feedback);

HAL_StatusTypeDef AK70_Servo_SetDuty(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float duty);
HAL_StatusTypeDef AK70_Servo_SetCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a);
HAL_StatusTypeDef AK70_Servo_SetBrakeCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a);
HAL_StatusTypeDef AK70_Servo_SetRpm(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t rpm);
HAL_StatusTypeDef AK70_Servo_SetPosition(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float position_deg);
HAL_StatusTypeDef AK70_Servo_SetOrigin(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, AK70_OriginMode mode);
HAL_StatusTypeDef AK70_Servo_SetPositionSpeed(FDCAN_HandleTypeDef *hfdcan,
                                              uint8_t motor_id,
                                              float position_deg,
                                              int16_t speed_erpm,
                                              int16_t accel_erpm_s2);
void AK70_Servo_ParseFeedback(const uint8_t data[8], AK70_Servo_Feedback *feedback);

uint32_t AK70_FloatToUint(float value, float min, float max, uint8_t bits);
float AK70_UintToFloat(uint32_t value, float min, float max, uint8_t bits);

#ifdef __cplusplus
}
#endif

#endif
