#ifndef R1_SUPERSTRUCTURE_BLAZER_FOC_MOTOR_H
#define R1_SUPERSTRUCTURE_BLAZER_FOC_MOTOR_H

#include <stdint.h>
#include "main.h"

/*
 * Number of Blazer FOC nodes controlled by this firmware.
 * To add more self-made ESCs, increase this count and add entries in
 * g_blazer_foc_motor_registry[] in blazer_foc_motor.c.
 */
#define BLAZER_FOC_MOTOR_COUNT 1U

typedef enum {
    BLAZER_FOC_MOTOR1 = 0,
} Blazer_FOC_MotorID_e;

/*
 * Blazer FOC CAN parameter IDs from the user manual.
 * Even IDs are write IDs. Read requests and read replies use ID + 1.
 * Example: write speed setpoint -> 0x04; read speed feedback -> 0x39.
 */
typedef enum {
    BLAZER_FOC_PARAM_MODE       = 0x00,
    BLAZER_FOC_PARAM_I_SET      = 0x02,
    BLAZER_FOC_PARAM_SPD_SET    = 0x04,
    BLAZER_FOC_PARAM_POS_SET    = 0x06,
    BLAZER_FOC_PARAM_CAN_ID     = 0x08,
    BLAZER_FOC_PARAM_POL        = 0x0A,
    BLAZER_FOC_PARAM_ENC_TYPE   = 0x0C,
    BLAZER_FOC_PARAM_I_CAL      = 0x0E,
    BLAZER_FOC_PARAM_I_LIM      = 0x10,
    BLAZER_FOC_PARAM_SPD_LIM    = 0x12,
    BLAZER_FOC_PARAM_SPD_ACC    = 0x14,
    BLAZER_FOC_PARAM_SPD_DEC    = 0x16,
    BLAZER_FOC_PARAM_SPD_KP     = 0x18,
    BLAZER_FOC_PARAM_SPD_KI     = 0x1A,
    BLAZER_FOC_PARAM_POS_ACC    = 0x1C,
    BLAZER_FOC_PARAM_POS_DEC    = 0x1E,
    BLAZER_FOC_PARAM_POS_MAXSPD = 0x20,
    BLAZER_FOC_PARAM_POS_KP     = 0x22,
    BLAZER_FOC_PARAM_POS_KI     = 0x24,
    BLAZER_FOC_PARAM_COGGING    = 0x26,
    BLAZER_FOC_PARAM_CAN_HB     = 0x28,
    BLAZER_FOC_PARAM_VBUS       = 0x2A,
    BLAZER_FOC_PARAM_IBUS       = 0x2C,
    BLAZER_FOC_PARAM_IA         = 0x2E,
    BLAZER_FOC_PARAM_IB         = 0x30,
    BLAZER_FOC_PARAM_IC         = 0x32,
    BLAZER_FOC_PARAM_ID         = 0x34,
    BLAZER_FOC_PARAM_IQ         = 0x36,
    BLAZER_FOC_PARAM_SPD_FILT   = 0x38,
    BLAZER_FOC_PARAM_ENC_RAW    = 0x3A,
    BLAZER_FOC_PARAM_TEMP       = 0x3C,
    BLAZER_FOC_PARAM_RS         = 0x3E,
    BLAZER_FOC_PARAM_LD         = 0x40,
    BLAZER_FOC_PARAM_LQ         = 0x42,
    BLAZER_FOC_PARAM_FLUX       = 0x44,
    BLAZER_FOC_PARAM_ERROR      = 0x46,
} Blazer_FOC_ParamID_e;

/*
 * Runtime modes from the manual. In normal robot control you will usually use:
 *   DISABLE: motor released
 *   CURRENT: current loop
 *   SPEED: speed loop, speed unit is mechanical revolutions per second
 *   POSITION: position loop, position unit is mechanical revolutions
 */
typedef enum {
    BLAZER_FOC_MODE_DISABLE       = 0,
    BLAZER_FOC_MODE_CURRENT       = 1,
    BLAZER_FOC_MODE_SPEED         = 2,
    BLAZER_FOC_MODE_POSITION      = 3,
    BLAZER_FOC_MODE_IDENTIFY_RL   = 4,
    BLAZER_FOC_MODE_ENCODER_CALIB = 5,
    BLAZER_FOC_MODE_COGGING_CALIB = 6,
    BLAZER_FOC_MODE_SET_ZERO      = 7,
    BLAZER_FOC_MODE_RESTORE       = 8,
    BLAZER_FOC_MODE_SAVE          = 9,
    BLAZER_FOC_MODE_CLEAR_ERROR   = 10,
} Blazer_FOC_Mode_e;

typedef struct {
    float vbus;
    float ibus;
    float iq;
    float speed;
    float enc_raw;
    float temp;
    float error;
    uint32_t last_rx_tick_ms;
    uint8_t online;
} Blazer_FOC_Feedback_t;

typedef struct {
    Blazer_FOC_Mode_e mode;
    float current_set;
    float speed_set;
    float position_set;
    uint8_t mode_pending;
    uint8_t setpoint_pending;
    uint8_t enabled;
} Blazer_FOC_Control_t;

typedef struct {
    FDCAN_HandleTypeDef *hcan;       /* CAN bus used by this ESC. */
    uint8_t node_id;                 /* Blazer FOC node ID, valid range 0..7. */
    Blazer_FOC_Control_t ctrl;
    Blazer_FOC_Feedback_t feedback;
    uint8_t read_cursor;             /* Round-robin state feedback polling index. */
} Blazer_FOC_Motor_t;

extern Blazer_FOC_Motor_t g_blazer_foc_motor_registry[BLAZER_FOC_MOTOR_COUNT];

void Blazer_FOC_Motor_Init(void);
void Blazer_FOC_SetMode(Blazer_FOC_Motor_t *motor, Blazer_FOC_Mode_e mode);
void Blazer_FOC_SetSpeed(Blazer_FOC_Motor_t *motor, float speed_rps);
void Blazer_FOC_SetCurrent(Blazer_FOC_Motor_t *motor, float current_a);
void Blazer_FOC_SetPosition(Blazer_FOC_Motor_t *motor, float position_rev);
void Blazer_FOC_Stop(Blazer_FOC_Motor_t *motor);
void Blazer_FOC_Control_Send(Blazer_FOC_Motor_t *motor);
void Blazer_FOC_Update_Feedback(Blazer_FOC_Motor_t *motor, uint32_t identifier, const uint8_t data[4]);
uint8_t Blazer_FOC_Match_Feedback(const Blazer_FOC_Motor_t *motor, FDCAN_HandleTypeDef *hfdcan, uint32_t identifier);
uint32_t Blazer_FOC_MakeID(uint8_t node_id, Blazer_FOC_ParamID_e param_id);

#endif /* R1_SUPERSTRUCTURE_BLAZER_FOC_MOTOR_H */
