#ifndef _LOCATOR_IMU_MANAGER_LOG_H_
#define _LOCATOR_IMU_MANAGER_LOG_H_

#include "datatype_basic.h"

namespace LOCATOR {

/* Packages of log to be recorded. */
#pragma pack(1)
struct ImuManagerLogMeasures {
    float time_stamp_s = 0.0f;
    // Accel unit : m/s2
    float accel_x_mps2 = 0.0f;
    float accel_y_mps2 = 0.0f;
    float accel_z_mps2 = 0.0f;
    // Gyro unit : rad/s
    float gyro_x_rps = 0.0f;
    float gyro_y_rps = 0.0f;
    float gyro_z_rps = 0.0f;
};

struct ImuManagerLogStates {
    float time_stamp_s = 0.0f;
    float p_wi_x = 0.0f;
    float p_wi_y = 0.0f;
    float p_wi_z = 0.0f;
    float q_wi_w = 0.0f;
    float q_wi_x = 0.0f;
    float q_wi_y = 0.0f;
    float q_wi_z = 0.0f;
    float q_wi_pitch = 0.0f;
    float q_wi_roll = 0.0f;
    float q_wi_yaw = 0.0f;
    float v_wi_x = 0.0f;
    float v_wi_y = 0.0f;
    float v_wi_z = 0.0f;
    float bias_a_x = 0.0f;
    float bias_a_y = 0.0f;
    float bias_a_z = 0.0f;
    float bias_g_x = 0.0f;
    float bias_g_y = 0.0f;
    float bias_g_z = 0.0f;
    float acc_wi_x = 0.0f;
    float acc_wi_y = 0.0f;
    float acc_wi_z = 0.0f;
    float gyro_wi_x = 0.0f;
    float gyro_wi_y = 0.0f;
    float gyro_wi_z = 0.0f;

    float v_ii_x = 0.0f;
    float v_ii_y = 0.0f;
    float v_ii_z = 0.0f;

};

struct ImuManagerLogMonitor {
    float time_stamp_s = 0.0f;
    uint32_t measure_buffer_size = 0;

    uint8_t is_attitude_valid = 0;
    uint8_t is_pos_vel_valid = 0;
    float current_updated_timestamp_s = 0.0f;
};

struct ImuManagerLogObserveResidual {
    float time_stamp_s = 0.0f;

    float gravity_b_x = 0.0f;
    float gravity_b_y = 0.0f;
    float gravity_b_z = 0.0f;
    float gyro_bias_w_z = 0.0f;
};
#pragma pack()

}

#endif // end of _LOCATOR_IMU_MANAGER_LOG_H_
