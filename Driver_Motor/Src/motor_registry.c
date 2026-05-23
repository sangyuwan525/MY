#include "motor_registry.h"

#include <math.h>
#include <stdbool.h>
#include <string.h>
#include "SEGGER_RTT.h"

extern Dji_Motor_t g_dji_motor_registry[DJI_MOTOR_COUNT];
extern Damiao_Motor_t g_dm_motor_registry[DM_MOTOR_COUNT];
extern Xiaomi_Motor_t g_xiaomi_motor_registry[XIAOMI_MOTOR_COUNT];
extern Unitree_GO_M8010_6_Motor_t g_unitree_go_m8010_6_motor_registry[UNITREE_GO_M8010_6_MOTOR_COUNT];
extern Blazer_FOC_Motor_t g_blazer_foc_motor_registry[BLAZER_FOC_MOTOR_COUNT];

Motor_Class_t g_motor_list[MOTOR_TOTAL_NUM];

#define MOTOR_TRAJ_TWO_PI 6.28318530717958647692f
#define MOTOR_TRAJ_PI     3.14159265358979323846f
#define DM_CMODE_SAVE_DISABLE 0U
#define DM_CMODE_SAVE_ENABLE  1U

static void DJI_Adapter_Init(Motor_Class_t *self) {
    (void)self;
}

static void DJI_Adapter_Enable(Motor_Class_t *self) {
    Dji_Motor_t *dji;

    if (self == NULL || self->instance == NULL) {
        return;
    }

    dji = (Dji_Motor_t *)self->instance;
    dji->is_enabled = true;
}

static void DJI_Adapter_Stop(Motor_Class_t *self, uint8_t clear_error) {
    Dji_Motor_t *dji;
    (void)clear_error;

    if (self == NULL || self->instance == NULL) {
        return;
    }

    dji = (Dji_Motor_t *)self->instance;
    dji->is_enabled = false;
    dji->target_spd = 0.0f;
    dji->target_loc = (float)dji->feedback.total_angle;
}

static void DJI_Adapter_SetZero(Motor_Class_t *self) {
    (void)self;
}

static void DJI_Adapter_SetSpeed(Motor_Class_t *self, float speed) {
    Dji_Motor_t *dji;
    int index;

    if (self == NULL || self->instance == NULL) {
        return;
    }

    dji = (Dji_Motor_t *)self->instance;
    index = (int)(dji - g_dji_motor_registry);
    Dji_Motor_SetSpeed((Dji_MotorID_e)index, speed);
}

static void DJI_Adapter_SetPosition(Motor_Class_t *self, float position, float vel_limit) {
    Dji_Motor_t *dji;
    int index;

    if (self == NULL || self->instance == NULL) {
        return;
    }

    dji = (Dji_Motor_t *)self->instance;
    index = (int)(dji - g_dji_motor_registry);

    if (vel_limit > 0.0f) {
        dji->pid_params.loc.out_limit_up = vel_limit;
        dji->pid_params.loc.out_limit_down = -vel_limit;
    }

    if (dji->sync_group_id != -1) {
        Dji_Motor_SetGroupLoc(dji->sync_group_id, position);
    } else {
        Dji_Motor_SetLoc((Dji_MotorID_e)index, position);
    }
}

static void DJI_Adapter_SetMIT(Motor_Class_t *self, float position, float speed, float kp, float kd, float torque) {
    (void)self;
    (void)position;
    (void)speed;
    (void)kp;
    (void)kd;
    (void)torque;
}

static void DJI_Adapter_SetPSI(Motor_Class_t *self, float position, float speed, float current) {
    (void)self;
    (void)position;
    (void)speed;
    (void)current;
}

static void DJI_Adapter_Update(Motor_Class_t *self, uint8_t *rx_data, uint32_t identifier) {
    (void)identifier;
    Dji_Motor_Update((Dji_Motor_t *)self->instance, rx_data);
}

static Motor_State_t DJI_Adapter_GetState(Motor_Class_t *self) {
    Motor_State_t state = {0};
    Dji_Motor_t *dji;

    if (self == NULL || self->instance == NULL) {
        return state;
    }

    dji = (Dji_Motor_t *)self->instance;
    state.angle = (float)dji->feedback.total_angle;
    state.speed = (float)dji->feedback.speed_rpm;
    state.torque = (float)dji->feedback.given_current;
    state.temp = (float)dji->feedback.temperate;
    return state;
}

static uint16_t DM_Adapter_GetModeID(const Damiao_Motor_t *dm);

static void DM_Adapter_SetRuntimeControlMode(Damiao_Motor_t *dm, mode_e mode) {
    if (dm == NULL) {
        return;
    }

    if (mode != mit_mode && mode != pos_mode && mode != spd_mode && mode != psi_mode) {
        return;
    }

    if ((mode_e)dm->ctrl.mode == mode && dm->tmp.cmode == (uint32_t)mode) {
        return;
    }

    dm_motor_clear_para(dm);
    dm_motor_set_control_mode(dm, mode, DM_CMODE_SAVE_DISABLE);
}

static void DM_Adapter_Init(Motor_Class_t *self) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL) {
        return;
    }

    dm->feedback_online = 0U;
    dm_motor_clear_para(dm);

    if (dm->hcan != NULL) {
        for (int i = 0; i < 3; ++i) {
            dm_motor_clear_err(dm);
            osDelay(10);
        }

        for (int i = 0; i < 5; ++i) {
            dm_motor_enable(dm);
            osDelay(100);
        }
    }
}

static uint16_t DM_Adapter_GetModeID(const Damiao_Motor_t *dm) {
    if (dm == NULL) {
        return MIT_MODE;
    }

    switch (dm->ctrl.mode) {
        case pos_mode:
            return POS_MODE;
        case spd_mode:
            return SPD_MODE;
        case psi_mode:
            return PSI_MODE;
        case mit_mode:
        default:
            return MIT_MODE;
    }
}

static void DM_Adapter_Enable(Motor_Class_t *self) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL) {
        return;
    }

    dm_motor_enable(dm);
}

static void DM_Adapter_Stop(Motor_Class_t *self, uint8_t clear_error) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;
    (void)clear_error;

    if (dm == NULL) {
        return;
    }

    dm_motor_disable(dm);
}

static void DM_Adapter_SetZero(Motor_Class_t *self) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL || dm->hcan == NULL) {
        return;
    }

    save_pos_zero(dm->hcan, dm->id, DM_Adapter_GetModeID(dm));
}

static void DM_Adapter_SetSpeed(Motor_Class_t *self, float speed) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL) {
        return;
    }

    DM_Adapter_SetRuntimeControlMode(dm, spd_mode);
    dm->ctrl.vel_set = speed;
    dm->ctrl.pos_set = 0.0f;
}

static void DM_Adapter_SetPosition(Motor_Class_t *self, float position, float vel_limit) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL) {
        return;
    }

    DM_Adapter_SetRuntimeControlMode(dm, pos_mode);
    dm->ctrl.pos_set = position;
    dm->ctrl.vel_set = vel_limit;
}

static void DM_Adapter_SetMIT(Motor_Class_t *self, float position, float speed, float kp, float kd, float torque) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL) {
        return;
    }

    DM_Adapter_SetRuntimeControlMode(dm, mit_mode);
    dm->ctrl.pos_set = position;
    dm->ctrl.vel_set = speed;
    dm->ctrl.kp_set = kp;
    dm->ctrl.kd_set = kd;
    dm->ctrl.tor_set = torque;
}

static void DM_Adapter_SetPSI(Motor_Class_t *self, float position, float speed, float current) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;

    if (dm == NULL) {
        return;
    }

    DM_Adapter_SetRuntimeControlMode(dm, psi_mode);
    dm->ctrl.pos_set = position;
    dm->ctrl.vel_set = speed;
    dm->ctrl.cur_set = current;
}

static void DM_Adapter_Update(Motor_Class_t *self, uint8_t *rx_data, uint32_t identifier) {
    Damiao_Motor_t *dm = (Damiao_Motor_t *)self->instance;
    (void)identifier;

    if (dm == NULL) {
        return;
    }

    dm_motor_fbdata(dm, rx_data);
    receive_motor_data(dm, rx_data);
}

static Motor_State_t DM_Adapter_GetState(Motor_Class_t *self) {
    Motor_State_t state = {0};
    Damiao_Motor_t *dm;

    if (self == NULL || self->instance == NULL) {
        return state;
    }

    dm = (Damiao_Motor_t *)self->instance;
    state.angle = dm->para.pos;
    state.speed = dm->para.vel;
    state.torque = dm->para.tor;
    state.temp = dm->para.Tmos;
    return state;
}

static void XIAOMI_Adapter_Init(Motor_Class_t *self) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL || xiaomi->hcan == NULL) {
        return;
    }

    xiaomi_motor_enable(xiaomi);
    osDelay(20);
}

static void XIAOMI_Adapter_Enable(Motor_Class_t *self) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi_motor_enable(xiaomi);
}

static void XIAOMI_Adapter_Stop(Motor_Class_t *self, uint8_t clear_error) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi_motor_stop(xiaomi, clear_error);
}

static void XIAOMI_Adapter_SetZero(Motor_Class_t *self) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi_motor_set_zero(xiaomi);
}

static void XIAOMI_Adapter_SetSpeed(Motor_Class_t *self, float speed) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi->ctrl.run_mode = XIAOMI_MODE_SPEED;
    xiaomi->ctrl.mode_configured = 1U;
    xiaomi->ctrl.speed_set = speed;
    xiaomi->ctrl.pending_cycles = 3;
}

static void XIAOMI_Adapter_SetPosition(Motor_Class_t *self, float position, float vel_limit) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi->ctrl.run_mode = XIAOMI_MODE_POSITION;
    xiaomi->ctrl.mode_configured = 1U;
    xiaomi->ctrl.pos_set = position;
    xiaomi->ctrl.speed_limit = vel_limit;
    xiaomi->ctrl.pending_cycles = 3;
}

static void XIAOMI_Adapter_SetMIT(Motor_Class_t *self, float position, float speed, float kp, float kd, float torque) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi->ctrl.run_mode = XIAOMI_MODE_MOTION;
    xiaomi->ctrl.mode_configured = 1U;
    xiaomi->ctrl.pos_set = position;
    xiaomi->ctrl.speed_set = speed;
    xiaomi->ctrl.kp_set = kp;
    xiaomi->ctrl.kd_set = kd;
    xiaomi->ctrl.torque_set = torque;
}

static void XIAOMI_Adapter_SetPSI(Motor_Class_t *self, float position, float speed, float current) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;
    (void)position;
    (void)speed;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi->ctrl.run_mode = XIAOMI_MODE_CURRENT;
    xiaomi->ctrl.mode_configured = 1U;
    xiaomi->ctrl.current_set = current;
    xiaomi->ctrl.pending_cycles = 3;
}

static void XIAOMI_Adapter_Update(Motor_Class_t *self, uint8_t *rx_data, uint32_t identifier) {
    Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)self->instance;

    if (xiaomi == NULL) {
        return;
    }

    xiaomi_motor_update_feedback(xiaomi, rx_data, identifier);
}

static Motor_State_t XIAOMI_Adapter_GetState(Motor_Class_t *self) {
    Motor_State_t state = {0};
    Xiaomi_Motor_t *xiaomi;

    if (self == NULL || self->instance == NULL) {
        return state;
    }

    xiaomi = (Xiaomi_Motor_t *)self->instance;
    state.angle = xiaomi->feedback.angle;
    state.speed = xiaomi->feedback.speed;
    state.torque = xiaomi->feedback.torque;
    state.temp = xiaomi->feedback.temp;
    return state;
}

static void UNITREE_GO_Adapter_Init(Motor_Class_t *self) {
    (void)self;
}

static void UNITREE_GO_Adapter_Enable(Motor_Class_t *self) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;

    if (unitree == NULL) {
        return;
    }

    unitree->ctrl.mode_configured = 1U;
}

static void UNITREE_GO_Adapter_Stop(Motor_Class_t *self, uint8_t clear_error) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;
    (void)clear_error;

    if (unitree == NULL) {
        return;
    }

    unitree_go_m8010_6_motor_stop(unitree);
}

static void UNITREE_GO_Adapter_SetZero(Motor_Class_t *self) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;

    if (unitree == NULL) {
        return;
    }

    unitree_go_m8010_6_motor_set_zero(unitree);
}

static void UNITREE_GO_Adapter_SetSpeed(Motor_Class_t *self, float speed) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;

    if (unitree == NULL) {
        return;
    }

    unitree->ctrl.mode = UNITREE_GO_M8010_6_MODE_FOC;
    unitree->ctrl.mode_configured = 1U;
    unitree->ctrl.pos_set = 0.0f;
    unitree->ctrl.speed_set = speed;
    unitree->ctrl.torque_set = 0.0f;
    unitree->ctrl.kp_set = 0.0f;
    unitree->ctrl.kd_set = 0.01f;
}

static void UNITREE_GO_Adapter_SetPosition(Motor_Class_t *self, float position, float vel_limit) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;
    (void)vel_limit;

    if (unitree == NULL) {
        return;
    }

    unitree->ctrl.mode = UNITREE_GO_M8010_6_MODE_FOC;
    unitree->ctrl.mode_configured = 1U;
    unitree->ctrl.pos_set = position;
    unitree->ctrl.speed_set = 0.0f;
    unitree->ctrl.torque_set = 0.0f;
    unitree->ctrl.kp_set = 0.05f;
    unitree->ctrl.kd_set = 0.01f;
}

static void UNITREE_GO_Adapter_SetMIT(Motor_Class_t *self, float position, float speed, float kp, float kd, float torque) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;

    if (unitree == NULL) {
        return;
    }

    unitree->ctrl.mode = UNITREE_GO_M8010_6_MODE_FOC;
    unitree->ctrl.mode_configured = 1U;
    unitree->ctrl.pos_set = position;
    unitree->ctrl.speed_set = speed;
    unitree->ctrl.kp_set = kp;
    unitree->ctrl.kd_set = kd;
    unitree->ctrl.torque_set = torque;
}

static void UNITREE_GO_Adapter_SetPSI(Motor_Class_t *self, float position, float speed, float current) {
    Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;
    (void)position;
    (void)speed;

    if (unitree == NULL) {
        return;
    }

    unitree->ctrl.mode = UNITREE_GO_M8010_6_MODE_FOC;
    unitree->ctrl.mode_configured = 1U;
    unitree->ctrl.pos_set = 0.0f;
    unitree->ctrl.speed_set = 0.0f;
    unitree->ctrl.kp_set = 0.0f;
    unitree->ctrl.kd_set = 0.0f;
    unitree->ctrl.torque_set = current;
}

static void UNITREE_GO_Adapter_Update(Motor_Class_t *self, uint8_t *rx_data, uint32_t identifier) {
    (void)identifier;
    unitree_go_m8010_6_update_feedback((Unitree_GO_M8010_6_Motor_t *)self->instance, rx_data);
}

static Motor_State_t UNITREE_GO_Adapter_GetState(Motor_Class_t *self) {
    Motor_State_t state = {0};
    Unitree_GO_M8010_6_Motor_t *unitree;

    if (self == NULL || self->instance == NULL) {
        return state;
    }

    unitree = (Unitree_GO_M8010_6_Motor_t *)self->instance;
    state.angle = unitree->feedback.angle;
    state.speed = unitree->feedback.speed;
    state.torque = unitree->feedback.torque;
    state.temp = unitree->feedback.temp;
    return state;
}

static void BLAZER_FOC_Adapter_Init(Motor_Class_t *self) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_SetMode(blazer, BLAZER_FOC_MODE_DISABLE);
}

static void BLAZER_FOC_Adapter_Enable(Motor_Class_t *self) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;

    if (blazer == NULL) {
        return;
    }

    blazer->ctrl.enabled = 1U;
}

static void BLAZER_FOC_Adapter_Stop(Motor_Class_t *self, uint8_t clear_error) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;
    (void)clear_error;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_Stop(blazer);
}

static void BLAZER_FOC_Adapter_SetZero(Motor_Class_t *self) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_SetMode(blazer, BLAZER_FOC_MODE_SET_ZERO);
}

/* Unified registry speed API -> Blazer FOC speed mode.
 * Unit: RPM, matching DJI's registry-facing speed API.
 */
static void BLAZER_FOC_Adapter_SetSpeed(Motor_Class_t *self, float speed) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_SetSpeed(blazer, speed);
}

/* Unified registry position API -> Blazer FOC position mode.
 * Unit: mechanical revolutions. vel_limit is ignored because Blazer uses its
 * own pos_maxspd/pos_acc/pos_dec parameters configured inside the ESC.
 */
static void BLAZER_FOC_Adapter_SetPosition(Motor_Class_t *self, float position, float vel_limit) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;
    (void)vel_limit;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_SetPosition(blazer, position);
}

/* Blazer FOC has no MIT mode; map the torque/current-like argument to i_set. */
static void BLAZER_FOC_Adapter_SetMIT(Motor_Class_t *self, float position, float speed, float kp, float kd, float torque) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;
    (void)position;
    (void)speed;
    (void)kp;
    (void)kd;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_SetCurrent(blazer, torque);
}

/* PSI current command maps directly to Blazer current mode. */
static void BLAZER_FOC_Adapter_SetPSI(Motor_Class_t *self, float position, float speed, float current) {
    Blazer_FOC_Motor_t *blazer = (Blazer_FOC_Motor_t *)self->instance;
    (void)position;
    (void)speed;

    if (blazer == NULL) {
        return;
    }

    Blazer_FOC_SetCurrent(blazer, current);
}

static void BLAZER_FOC_Adapter_Update(Motor_Class_t *self, uint8_t *rx_data, uint32_t identifier) {
    Blazer_FOC_Update_Feedback((Blazer_FOC_Motor_t *)self->instance, identifier, rx_data);
}

static Motor_State_t BLAZER_FOC_Adapter_GetState(Motor_Class_t *self) {
    Motor_State_t state = {0};
    Blazer_FOC_Motor_t *blazer;

    if (self == NULL || self->instance == NULL) {
        return state;
    }

    blazer = (Blazer_FOC_Motor_t *)self->instance;
    state.angle = blazer->feedback.enc_raw;
    state.speed = blazer->feedback.speed;
    state.torque = blazer->feedback.iq;
    state.temp = blazer->feedback.temp;
    return state;
}

static uint8_t Motor_IsFeedbackOnline(const Motor_Class_t *motor) {
    if (motor == NULL || motor->instance == NULL) {
        return 0U;
    }

    switch (motor->type) {
    case MOTOR_TYPE_DAMIAO:
        return ((Damiao_Motor_t *)motor->instance)->feedback_online ? 1U : 0U;
    case MOTOR_TYPE_XIAOMI:
        return ((Xiaomi_Motor_t *)motor->instance)->feedback.online ? 1U : 0U;
    case MOTOR_TYPE_UNITREE_GO_M8010_6:
        return ((Unitree_GO_M8010_6_Motor_t *)motor->instance)->feedback.online ? 1U : 0U;
    default:
        return 1U;
    }
}

static void Motor_SmoothGoto_StartFromPosition(Motor_Smooth_Goto_Profile_t *profile,
                                               float start_position,
                                               float target_position,
                                               float max_speed) {
    float delta_pos;

    if (profile == NULL) {
        return;
    }

    memset(profile, 0, sizeof(*profile));
    profile->start_position = start_position;
    profile->target_position = target_position;
    profile->max_speed = fabsf(max_speed);
    profile->start_tick_ms = HAL_GetTick();

    delta_pos = profile->target_position - profile->start_position;
    if (fabsf(delta_pos) < 1e-6f || profile->max_speed < 1e-6f) {
        profile->duration_s = 0.0f;
        profile->active = 0U;
        return;
    }

    profile->duration_s = fabsf(delta_pos) * MOTOR_TRAJ_PI / (2.0f * profile->max_speed);
    if (profile->duration_s < 0.001f) {
        profile->duration_s = 0.001f;
    }
    profile->active = 1U;
}

static void Motor_UpdateSmoothGotoMIT(int motor_index, Motor_Class_t *motor) {
    if (motor == NULL || motor->smooth_pending == 0U) {
        return;
    }

    if (motor->smooth_started == 0U) {
        if (motor->get_state == NULL) {
            return;
        }

        if (Motor_IsFeedbackOnline(motor) != 0U) {
            Motor_SmoothGoto_Start(&motor->smooth_goto, motor_index, motor->smooth_target, motor->smooth_max_speed);
        } else if (motor->type == MOTOR_TYPE_UNITREE_GO_M8010_6) {
            Unitree_GO_M8010_6_Motor_t *unitree = (Unitree_GO_M8010_6_Motor_t *)motor->instance;
            Motor_SmoothGoto_StartFromPosition(&motor->smooth_goto,
                                               unitree->ctrl.pos_set,
                                               motor->smooth_target,
                                               motor->smooth_max_speed);
        } else {
            return;
        }
        motor->smooth_started = 1U;
    }

    if (Motor_RunSmoothGotoMIT(motor_index,
                               &motor->smooth_goto,
                               motor->smooth_kp,
                               motor->smooth_kd,
                               motor->smooth_torque_ff) == 0U) {
        motor->smooth_pending = 0U;
        motor->smooth_started = 0U;
    }
}

void Motor_Registry_Init(void) {
    memset(g_motor_list, 0, sizeof(g_motor_list));

    Dji_Motor_Registry_Init();
    SEGGER_RTT_printf(0, "finish dji init\r\n");

    dm_motor_init();
    SEGGER_RTT_printf(0, "finish dm init\r\n");

    xiaomi_motor_init();
    SEGGER_RTT_printf(0, "finish xiaomi init\r\n");

    unitree_go_m8010_6_motor_init();
    SEGGER_RTT_printf(0, "finish unitree go m8010-6 init\r\n");

    Blazer_FOC_Motor_Init();
    SEGGER_RTT_printf(0, "finish blazer foc init\r\n");

    for (int i = 0; i < DJI_MOTOR_COUNT; ++i) {
        g_motor_list[i].type = MOTOR_TYPE_DJI;
        g_motor_list[i].instance = &g_dji_motor_registry[i];
        g_motor_list[i].init = DJI_Adapter_Init;
        g_motor_list[i].enable = DJI_Adapter_Enable;
        g_motor_list[i].stop = DJI_Adapter_Stop;
        g_motor_list[i].set_zero = DJI_Adapter_SetZero;
        g_motor_list[i].set_speed = DJI_Adapter_SetSpeed;
        g_motor_list[i].set_position = DJI_Adapter_SetPosition;
        g_motor_list[i].set_mit = DJI_Adapter_SetMIT;
        g_motor_list[i].set_psi = DJI_Adapter_SetPSI;
        g_motor_list[i].update_feedback = DJI_Adapter_Update;
        g_motor_list[i].get_state = DJI_Adapter_GetState;
    }

    for (int i = 0; i < DM_MOTOR_COUNT; ++i) {
        int global_idx = DJI_MOTOR_COUNT + i;

        g_motor_list[global_idx].type = MOTOR_TYPE_DAMIAO;
        g_motor_list[global_idx].instance = &g_dm_motor_registry[i];
        g_motor_list[global_idx].init = DM_Adapter_Init;
        g_motor_list[global_idx].enable = DM_Adapter_Enable;
        g_motor_list[global_idx].stop = DM_Adapter_Stop;
        g_motor_list[global_idx].set_zero = DM_Adapter_SetZero;
        g_motor_list[global_idx].set_speed = DM_Adapter_SetSpeed;
        g_motor_list[global_idx].set_position = DM_Adapter_SetPosition;
        g_motor_list[global_idx].set_mit = DM_Adapter_SetMIT;
        g_motor_list[global_idx].set_psi = DM_Adapter_SetPSI;
        g_motor_list[global_idx].update_feedback = DM_Adapter_Update;
        g_motor_list[global_idx].get_state = DM_Adapter_GetState;
    }

    for (int i = 0; i < XIAOMI_MOTOR_COUNT; ++i) {
        int global_idx = DJI_MOTOR_COUNT + DM_MOTOR_COUNT + i;

        g_motor_list[global_idx].type = MOTOR_TYPE_XIAOMI;
        g_motor_list[global_idx].instance = &g_xiaomi_motor_registry[i];
        g_motor_list[global_idx].init = XIAOMI_Adapter_Init;
        g_motor_list[global_idx].enable = XIAOMI_Adapter_Enable;
        g_motor_list[global_idx].stop = XIAOMI_Adapter_Stop;
        g_motor_list[global_idx].set_zero = XIAOMI_Adapter_SetZero;
        g_motor_list[global_idx].set_speed = XIAOMI_Adapter_SetSpeed;
        g_motor_list[global_idx].set_position = XIAOMI_Adapter_SetPosition;
        g_motor_list[global_idx].set_mit = XIAOMI_Adapter_SetMIT;
        g_motor_list[global_idx].set_psi = XIAOMI_Adapter_SetPSI;
        g_motor_list[global_idx].update_feedback = XIAOMI_Adapter_Update;
        g_motor_list[global_idx].get_state = XIAOMI_Adapter_GetState;
    }

    for (int i = 0; i < UNITREE_GO_M8010_6_MOTOR_COUNT; ++i) {
        int global_idx = DJI_MOTOR_COUNT + DM_MOTOR_COUNT + XIAOMI_MOTOR_COUNT + i;

        g_motor_list[global_idx].type = MOTOR_TYPE_UNITREE_GO_M8010_6;
        g_motor_list[global_idx].instance = &g_unitree_go_m8010_6_motor_registry[i];
        g_motor_list[global_idx].init = UNITREE_GO_Adapter_Init;
        g_motor_list[global_idx].enable = UNITREE_GO_Adapter_Enable;
        g_motor_list[global_idx].stop = UNITREE_GO_Adapter_Stop;
        g_motor_list[global_idx].set_zero = UNITREE_GO_Adapter_SetZero;
        g_motor_list[global_idx].set_speed = UNITREE_GO_Adapter_SetSpeed;
        g_motor_list[global_idx].set_position = UNITREE_GO_Adapter_SetPosition;
        g_motor_list[global_idx].set_mit = UNITREE_GO_Adapter_SetMIT;
        g_motor_list[global_idx].set_psi = UNITREE_GO_Adapter_SetPSI;
        g_motor_list[global_idx].update_feedback = UNITREE_GO_Adapter_Update;
        g_motor_list[global_idx].get_state = UNITREE_GO_Adapter_GetState;
    }

    for (int i = 0; i < BLAZER_FOC_MOTOR_COUNT; ++i) {
        int global_idx = DJI_MOTOR_COUNT + DM_MOTOR_COUNT + XIAOMI_MOTOR_COUNT + UNITREE_GO_M8010_6_MOTOR_COUNT + i;

        g_motor_list[global_idx].type = MOTOR_TYPE_BLAZER_FOC;
        g_motor_list[global_idx].instance = &g_blazer_foc_motor_registry[i];
        g_motor_list[global_idx].init = BLAZER_FOC_Adapter_Init;
        g_motor_list[global_idx].enable = BLAZER_FOC_Adapter_Enable;
        g_motor_list[global_idx].stop = BLAZER_FOC_Adapter_Stop;
        g_motor_list[global_idx].set_zero = BLAZER_FOC_Adapter_SetZero;
        g_motor_list[global_idx].set_speed = BLAZER_FOC_Adapter_SetSpeed;
        g_motor_list[global_idx].set_position = BLAZER_FOC_Adapter_SetPosition;
        g_motor_list[global_idx].set_mit = BLAZER_FOC_Adapter_SetMIT;
        g_motor_list[global_idx].set_psi = BLAZER_FOC_Adapter_SetPSI;
        g_motor_list[global_idx].update_feedback = BLAZER_FOC_Adapter_Update;
        g_motor_list[global_idx].get_state = BLAZER_FOC_Adapter_GetState;
    }

    for (int i = 0; i < MOTOR_TOTAL_NUM; ++i) {
        if (g_motor_list[i].init != NULL) {
            g_motor_list[i].init(&g_motor_list[i]);
        }
    }
}

void Motor_Feedback_Dispatch(FDCAN_HandleTypeDef *hfdcan, uint32_t identifier, uint8_t *data) {
    for (int i = 0; i < MOTOR_TOTAL_NUM; ++i) {
        Motor_Class_t *motor_obj = &g_motor_list[i];
        bool is_match = false;

        if (motor_obj->instance == NULL) {
            continue;
        }

        if (motor_obj->type == MOTOR_TYPE_DJI) {
            Dji_Motor_t *dji = (Dji_Motor_t *)motor_obj->instance;
            if (dji->hcan_tx == hfdcan && dji->can_rx_id == identifier) {
                is_match = true;
            }
        } else if (motor_obj->type == MOTOR_TYPE_DAMIAO) {
            Damiao_Motor_t *dm = (Damiao_Motor_t *)motor_obj->instance;
            if (dm->hcan == hfdcan && dm->mst_id == identifier) {
                is_match = true;
            }
        } else if (motor_obj->type == MOTOR_TYPE_XIAOMI) {
            Xiaomi_Motor_t *xiaomi = (Xiaomi_Motor_t *)motor_obj->instance;
            uint8_t comm_type = xiaomi_motor_extract_comm_type(identifier);
            uint8_t feedback_id = xiaomi_motor_extract_feedback_id(identifier);
            uint8_t target_id = xiaomi_motor_extract_target_id(identifier);

            if (xiaomi->hcan == hfdcan &&
                ((comm_type == 0x02U && feedback_id == xiaomi->feedback_id) ||
                 (comm_type == 0x15U && target_id == xiaomi->can_id))) {
                is_match = true;
            }
        } else if (motor_obj->type == MOTOR_TYPE_BLAZER_FOC) {
            if (Blazer_FOC_Match_Feedback((Blazer_FOC_Motor_t *)motor_obj->instance, hfdcan, identifier) != 0U) {
                is_match = true;
            }
        }

        if (is_match && motor_obj->update_feedback != NULL) {
            motor_obj->update_feedback(motor_obj, data, identifier);
            return;
        }
    }
}

void Motor_All_Control_Loop(void) {
    static uint8_t unitree_send_slot = 0U;
    Dji_3508_all_motor_control();

    for (int i = DJI_MOTOR_COUNT; i < MOTOR_TOTAL_NUM; ++i) {
        Motor_Class_t *cls = &g_motor_list[i];

        if (cls->instance == NULL) {
            continue;
        }

        Motor_UpdateSmoothGotoMIT(i, cls);

        if (cls->type == MOTOR_TYPE_DAMIAO) {
            dm_motor_ctrl_send((Damiao_Motor_t *)cls->instance);
        } else if (cls->type == MOTOR_TYPE_XIAOMI) {
            xiaomi_motor_ctrl_send((Xiaomi_Motor_t *)cls->instance);
        } else if (cls->type == MOTOR_TYPE_UNITREE_GO_M8010_6) {
            continue;
        } else if (cls->type == MOTOR_TYPE_BLAZER_FOC) {
            continue;
        }
    }

    for (uint8_t n = 0U; n < UNITREE_GO_M8010_6_MOTOR_COUNT; ++n) {
        uint8_t motor_idx = (uint8_t)((unitree_send_slot + n) % UNITREE_GO_M8010_6_MOTOR_COUNT);
        int global_idx = DJI_MOTOR_COUNT + DM_MOTOR_COUNT + XIAOMI_MOTOR_COUNT + motor_idx;
        Motor_Class_t *cls = &g_motor_list[global_idx];

        if (cls->instance != NULL && cls->type == MOTOR_TYPE_UNITREE_GO_M8010_6) {
            unitree_send_slot = (uint8_t)((motor_idx + 1U) % UNITREE_GO_M8010_6_MOTOR_COUNT);
            unitree_go_m8010_6_motor_ctrl_send((Unitree_GO_M8010_6_Motor_t *)cls->instance);
            break;
        }
    }

    Blazer_FOC_Control_Dispatch();
}

void Motor_Enable(int motor_index) {
    Motor_Class_t *motor;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    motor = &g_motor_list[motor_index];
    if (motor->enable != NULL) {
        motor->enable(motor);
    }
}

void Motor_Stop(int motor_index, uint8_t clear_error) {
    Motor_Class_t *motor;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    motor = &g_motor_list[motor_index];
    if (motor->stop != NULL) {
        motor->stop(motor, clear_error);
    }
}

void Motor_SetZero(int motor_index) {
    Motor_Class_t *motor;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    motor = &g_motor_list[motor_index];
    if (motor->set_zero != NULL) {
        motor->set_zero(motor);
    }
}

void Motor_SetMIT(int motor_index, float position, float speed, float kp, float kd, float torque) {
    Motor_Class_t *motor;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    motor = &g_motor_list[motor_index];
    if (motor->set_mit != NULL) {
        motor->set_mit(motor, position, speed, kp, kd, torque);
    }
}

void Motor_SetPSI(int motor_index, float position, float speed, float current) {
    Motor_Class_t *motor;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    motor = &g_motor_list[motor_index];
    if (motor->set_psi != NULL) {
        motor->set_psi(motor, position, speed, current);
    }
}

void Motor_SineProfile_Init(Motor_Sine_Profile_t *profile,
                            float position_offset,
                            float position_amplitude,
                            float period_s,
                            float phase_rad) {
    if (profile == NULL) {
        return;
    }

    profile->position_offset = position_offset;
    profile->position_amplitude = position_amplitude;
    profile->period_s = period_s;
    profile->phase_rad = phase_rad;
    profile->start_tick_ms = HAL_GetTick();
}

void Motor_SineProfile_Reset(Motor_Sine_Profile_t *profile) {
    if (profile == NULL) {
        return;
    }

    profile->start_tick_ms = HAL_GetTick();
}

void Motor_SineProfile_Eval(const Motor_Sine_Profile_t *profile, float *position, float *speed) {
    float elapsed_s;
    float omega;
    float theta;

    if (position != NULL) {
        *position = 0.0f;
    }
    if (speed != NULL) {
        *speed = 0.0f;
    }

    if (profile == NULL || profile->period_s <= 0.0f) {
        return;
    }

    elapsed_s = ((float)(HAL_GetTick() - profile->start_tick_ms)) * 0.001f;
    omega = MOTOR_TRAJ_TWO_PI / profile->period_s;
    theta = omega * elapsed_s + profile->phase_rad;

    if (position != NULL) {
        *position = profile->position_offset + profile->position_amplitude * sinf(theta);
    }

    if (speed != NULL) {
        *speed = profile->position_amplitude * omega * cosf(theta);
    }
}

void Motor_RunSineMIT(int motor_index,
                      const Motor_Sine_Profile_t *profile,
                      float kp,
                      float kd,
                      float torque_ff) {
    float position = 0.0f;
    float speed = 0.0f;

    Motor_SineProfile_Eval(profile, &position, &speed);
    Motor_SetMIT(motor_index, position, speed, kp, kd, torque_ff);
}

void Motor_SmoothGoto_Start(Motor_Smooth_Goto_Profile_t *profile,
                            int motor_index,
                            float target_position,
                            float max_speed) {
    Motor_State_t state;
    float delta_pos;

    if (profile == NULL || motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    memset(profile, 0, sizeof(*profile));

    if (g_motor_list[motor_index].get_state == NULL) {
        return;
    }

    state = g_motor_list[motor_index].get_state(&g_motor_list[motor_index]);
    profile->start_position = state.angle;
    profile->target_position = target_position;
    profile->max_speed = fabsf(max_speed);
    profile->start_tick_ms = HAL_GetTick();

    delta_pos = profile->target_position - profile->start_position;

    if (fabsf(delta_pos) < 1e-6f || profile->max_speed < 1e-6f) {
        profile->duration_s = 0.0f;
        profile->active = 0U;
        return;
    }

    profile->duration_s = fabsf(delta_pos) * MOTOR_TRAJ_PI / (2.0f * profile->max_speed);
    if (profile->duration_s < 0.001f) {
        profile->duration_s = 0.001f;
    }
    profile->active = 1U;
}

void Motor_SmoothGoto_Reset(Motor_Smooth_Goto_Profile_t *profile) {
    if (profile == NULL) {
        return;
    }

    memset(profile, 0, sizeof(*profile));
}

uint8_t Motor_SmoothGoto_Eval(const Motor_Smooth_Goto_Profile_t *profile, float *position, float *speed) {
    float elapsed_s;
    float ratio;
    float delta_pos;
    float theta;

    if (position != NULL) {
        *position = 0.0f;
    }
    if (speed != NULL) {
        *speed = 0.0f;
    }

    if (profile == NULL) {
        return 0U;
    }

    if (!profile->active || profile->duration_s <= 0.0f) {
        if (position != NULL) {
            *position = profile->target_position;
        }
        return 0U;
    }

    elapsed_s = ((float)(HAL_GetTick() - profile->start_tick_ms)) * 0.001f;
    if (elapsed_s >= profile->duration_s) {
        if (position != NULL) {
            *position = profile->target_position;
        }
        if (speed != NULL) {
            *speed = 0.0f;
        }
        return 0U;
    }

    ratio = elapsed_s / profile->duration_s;
    delta_pos = profile->target_position - profile->start_position;
    theta = MOTOR_TRAJ_PI * ratio;

    if (position != NULL) {
        *position = profile->start_position + 0.5f * delta_pos * (1.0f - cosf(theta));
    }

    if (speed != NULL) {
        *speed = 0.5f * delta_pos * (MOTOR_TRAJ_PI / profile->duration_s) * sinf(theta);
    }

    return 1U;
}

uint8_t Motor_RunSmoothGotoMIT(int motor_index,
                               Motor_Smooth_Goto_Profile_t *profile,
                               float kp,
                               float kd,
                               float torque_ff) {
    float position = 0.0f;
    float speed = 0.0f;
    uint8_t active;

    active = Motor_SmoothGoto_Eval(profile, &position, &speed);
    Motor_SetMIT(motor_index, position, speed, kp, kd, torque_ff);

    if (profile != NULL && active == 0U) {
        profile->active = 0U;
    }

    return active;
}

void Motor_StartSmoothGotoMIT(int motor_index,
                              float target_position,
                              float max_speed,
                              float kp,
                              float kd,
                              float torque_ff) {
    Motor_Class_t *motor;

    if (motor_index < 0 || motor_index >= MOTOR_TOTAL_NUM) {
        return;
    }

    motor = &g_motor_list[motor_index];
    if (motor->set_mit == NULL || motor->get_state == NULL) {
        return;
    }

    Motor_SmoothGoto_Reset(&motor->smooth_goto);
    motor->smooth_target = target_position;
    motor->smooth_max_speed = max_speed;
    motor->smooth_kp = kp;
    motor->smooth_kd = kd;
    motor->smooth_torque_ff = torque_ff;
    motor->smooth_started = 0U;
    motor->smooth_pending = 1U;
}
