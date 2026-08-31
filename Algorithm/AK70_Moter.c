#include "AK70_Moter.h"
#include "fdcan.h"

const AK70_MIT_Param AK70_10_MIT_PARAM = {
    .p_min = -12.5f,
    .p_max = 12.5f,
    .v_min = -50.0f,
    .v_max = 50.0f,
    .kp_min = 0.0f,
    .kp_max = 500.0f,
    .kd_min = 0.0f,
    .kd_max = 5.0f,
    .t_min = -25.0f,
    .t_max = 25.0f,
};

static float ak70_limit(float value, float min, float max)
{
    if (value < min) {
        return min;
    }

    if (value > max) {
        return max;
    }

    return value;
}

uint32_t AK70_FloatToUint(float value, float min, float max, uint8_t bits)
{
    float span = max - min;
    uint32_t max_int = (1UL << bits) - 1UL;

    value = ak70_limit(value, min, max);
    return (uint32_t)((value - min) * (float)max_int / span);
}

float AK70_UintToFloat(uint32_t value, float min, float max, uint8_t bits)
{
    float span = max - min;
    uint32_t max_int = (1UL << bits) - 1UL;

    return ((float)value * span / (float)max_int) + min;
}

static void ak70_append_int16(uint8_t *buffer, int16_t value, uint8_t *index)
{
    buffer[(*index)++] = (uint8_t)(value >> 8);
    buffer[(*index)++] = (uint8_t)value;
}

static void ak70_append_int32(uint8_t *buffer, int32_t value, uint8_t *index)
{
    buffer[(*index)++] = (uint8_t)(value >> 24);
    buffer[(*index)++] = (uint8_t)(value >> 16);
    buffer[(*index)++] = (uint8_t)(value >> 8);
    buffer[(*index)++] = (uint8_t)value;
}

static int16_t ak70_get_int16(const uint8_t *buffer, uint8_t index)
{
    return (int16_t)(((uint16_t)buffer[index] << 8) | buffer[index + 1]);
}

static HAL_StatusTypeDef ak70_can_send_std(FDCAN_HandleTypeDef *hfdcan,
                                           uint16_t id,
                                           const uint8_t *data,
                                           uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};

    if (len > 8U) {
        len = 8U;
    }

    for (uint8_t i = 0; i < len; i++) {
        tx_data[i] = data[i];
    }

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_STANDARD_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = (uint32_t)len << 16;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
}

static HAL_StatusTypeDef ak70_can_send_ext(FDCAN_HandleTypeDef *hfdcan,
                                           uint32_t id,
                                           const uint8_t *data,
                                           uint8_t len)
{
    FDCAN_TxHeaderTypeDef tx_header = {0};
    uint8_t tx_data[8] = {0};

    if (len > 8U) {
        len = 8U;
    }

    for (uint8_t i = 0; i < len; i++) {
        tx_data[i] = data[i];
    }

    tx_header.Identifier = id;
    tx_header.IdType = FDCAN_EXTENDED_ID;
    tx_header.TxFrameType = FDCAN_DATA_FRAME;
    tx_header.DataLength = (uint32_t)len << 16;
    tx_header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    tx_header.BitRateSwitch = FDCAN_BRS_OFF;
    tx_header.FDFormat = FDCAN_CLASSIC_CAN;
    tx_header.TxEventFifoControl = FDCAN_NO_TX_EVENTS;

    return HAL_FDCAN_AddMessageToTxFifoQ(hfdcan, &tx_header, tx_data);
}

HAL_StatusTypeDef AK70_CAN_ConfigFilter(FDCAN_HandleTypeDef *hfdcan, uint16_t feedback_id)
{
    FDCAN_FilterTypeDef filter = {0};
    HAL_StatusTypeDef status;

    filter.IdType = FDCAN_STANDARD_ID;
    filter.FilterIndex = 0;
    filter.FilterType = FDCAN_FILTER_MASK;
    filter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
    filter.FilterID1 = feedback_id;
    filter.FilterID2 = 0x07FFU;

    status = HAL_FDCAN_ConfigFilter(hfdcan, &filter);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_FDCAN_ConfigGlobalFilter(hfdcan,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT,
                                         FDCAN_REJECT_REMOTE,
                                         FDCAN_REJECT_REMOTE);
}

HAL_StatusTypeDef AK70_CAN_Start(FDCAN_HandleTypeDef *hfdcan, uint16_t feedback_id)
{
    HAL_StatusTypeDef status;

    status = AK70_CAN_ConfigFilter(hfdcan, feedback_id);
    if (status != HAL_OK) {
        return status;
    }

    status = HAL_FDCAN_Start(hfdcan);
    if (status != HAL_OK) {
        return status;
    }

    return HAL_FDCAN_ActivateNotification(hfdcan,
                                           FDCAN_IT_RX_FIFO0_NEW_MESSAGE,
                                           0U);
}

HAL_StatusTypeDef AK70_MIT_Enter(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfc};
    return ak70_can_send_std(hfdcan, motor_id, data, 8);
}

HAL_StatusTypeDef AK70_MIT_Exit(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfd};
    return ak70_can_send_std(hfdcan, motor_id, data, 8);
}

HAL_StatusTypeDef AK70_MIT_SetZero(FDCAN_HandleTypeDef *hfdcan, uint16_t motor_id)
{
    uint8_t data[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xfe};
    return ak70_can_send_std(hfdcan, motor_id, data, 8);
}

HAL_StatusTypeDef AK70_MIT_Control(FDCAN_HandleTypeDef *hfdcan,
                                   uint16_t motor_id,
                                   const AK70_MIT_Param *param,
                                   float position_rad,
                                   float velocity_rad_s,
                                   float kp,
                                   float kd,
                                   float torque_nm)
{
    uint8_t data[8];
    uint32_t p_int;
    uint32_t v_int;
    uint32_t kp_int;
    uint32_t kd_int;
    uint32_t t_int;

    if (param == NULL) {
        param = &AK70_10_MIT_PARAM;
    }

    p_int = AK70_FloatToUint(position_rad, param->p_min, param->p_max, 16);
    v_int = AK70_FloatToUint(velocity_rad_s, param->v_min, param->v_max, 12);
    kp_int = AK70_FloatToUint(kp, param->kp_min, param->kp_max, 12);
    kd_int = AK70_FloatToUint(kd, param->kd_min, param->kd_max, 12);
    t_int = AK70_FloatToUint(torque_nm, param->t_min, param->t_max, 12);

    data[0] = (uint8_t)(p_int >> 8);
    data[1] = (uint8_t)p_int;
    data[2] = (uint8_t)(v_int >> 4);
    data[3] = (uint8_t)(((v_int & 0x0fU) << 4) | (kp_int >> 8));
    data[4] = (uint8_t)kp_int;
    data[5] = (uint8_t)(kd_int >> 4);
    data[6] = (uint8_t)(((kd_int & 0x0fU) << 4) | (t_int >> 8));
    data[7] = (uint8_t)t_int;

    return ak70_can_send_std(hfdcan, motor_id, data, 8);
}

void AK70_MIT_ParseFeedback(const uint8_t data[8],
                            const AK70_MIT_Param *param,
                            AK70_MIT_Feedback *feedback)
{
    uint16_t p_int;
    uint16_t v_int;
    uint16_t t_int;

    if ((data == NULL) || (feedback == NULL)) {
        return;
    }

    if (param == NULL) {
        param = &AK70_10_MIT_PARAM;
    }

    feedback->id = data[0];
    p_int = ((uint16_t)data[1] << 8) | data[2];
    v_int = ((uint16_t)data[3] << 4) | (data[4] >> 4);
    t_int = ((uint16_t)(data[4] & 0x0fU) << 8) | data[5];

    feedback->position_rad = AK70_UintToFloat(p_int, param->p_min, param->p_max, 16);
    feedback->velocity_rad_s = AK70_UintToFloat(v_int, param->v_min, param->v_max, 12);
    feedback->torque_nm = AK70_UintToFloat(t_int, param->t_min, param->t_max, 12);
    feedback->temperature_c = (int16_t)data[6] - 40;
    feedback->error = data[7];
}

static HAL_StatusTypeDef ak70_servo_send(FDCAN_HandleTypeDef *hfdcan,
                                         uint8_t motor_id,
                                         AK70_CAN_PacketId packet_id,
                                         const uint8_t *data,
                                         uint8_t len)
{
    uint32_t ext_id = ((uint32_t)packet_id << 8) | motor_id;
    return ak70_can_send_ext(hfdcan, ext_id, data, len);
}

HAL_StatusTypeDef AK70_Servo_SetDuty(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float duty)
{
    uint8_t data[4];
    uint8_t index = 0;

    ak70_append_int32(data, (int32_t)(duty * 100000.0f), &index);
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_DUTY, data, index);
}

HAL_StatusTypeDef AK70_Servo_SetCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a)
{
    uint8_t data[4];
    uint8_t index = 0;

    ak70_append_int32(data, (int32_t)(current_a * 1000.0f), &index);
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_CURRENT, data, index);
}

HAL_StatusTypeDef AK70_Servo_SetBrakeCurrent(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float current_a)
{
    uint8_t data[4];
    uint8_t index = 0;

    ak70_append_int32(data, (int32_t)(current_a * 1000.0f), &index);
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_CURRENT_BRAKE, data, index);
}

HAL_StatusTypeDef AK70_Servo_SetRpm(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, int32_t rpm)
{
    uint8_t data[4];
    uint8_t index = 0;

    ak70_append_int32(data, rpm, &index);
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_RPM, data, index);
}

HAL_StatusTypeDef AK70_Servo_SetPosition(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, float position_deg)
{
    uint8_t data[4];
    uint8_t index = 0;

    ak70_append_int32(data, (int32_t)(position_deg * 10000.0f), &index);
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_POS, data, index);
}

HAL_StatusTypeDef AK70_Servo_SetOrigin(FDCAN_HandleTypeDef *hfdcan, uint8_t motor_id, AK70_OriginMode mode)
{
    uint8_t data = (uint8_t)mode;
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_ORIGIN_HERE, &data, 1);
}

HAL_StatusTypeDef AK70_Servo_SetPositionSpeed(FDCAN_HandleTypeDef *hfdcan,
                                              uint8_t motor_id,
                                              float position_deg,
                                              int16_t speed_erpm,
                                              int16_t accel_erpm_s2)
{
    uint8_t data[8];
    uint8_t index = 0;

    ak70_append_int32(data, (int32_t)(position_deg * 10000.0f), &index);
    ak70_append_int16(data, (int16_t)(speed_erpm / 10), &index);
    ak70_append_int16(data, (int16_t)(accel_erpm_s2 / 10), &index);
    return ak70_servo_send(hfdcan, motor_id, AK70_CAN_PACKET_SET_POS_SPD, data, index);
}

void AK70_Servo_ParseFeedback(const uint8_t data[8], AK70_Servo_Feedback *feedback)
{
    int16_t position;
    int16_t speed;
    int16_t current;

    if ((data == NULL) || (feedback == NULL)) {
        return;
    }

    position = ak70_get_int16(data, 0);
    speed = ak70_get_int16(data, 2);
    current = ak70_get_int16(data, 4);

    feedback->position_deg = (float)position * 0.1f;
    feedback->speed_erpm = (float)speed * 10.0f;
    feedback->current_a = (float)current * 0.01f;
    feedback->temperature_c = (int8_t)data[6];
    feedback->error = data[7];
}
