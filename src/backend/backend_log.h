#ifndef _VIO_STEREO_VINS_BACKEND_LOG_H_
#define _VIO_STEREO_VINS_BACKEND_LOG_H_

#include "datatype_basic.h"

namespace VIO {

/* Packages of log to be recorded. */
#pragma pack(1)
struct BackendLogStates {
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
};
#pragma pack()

}

#endif // end of _VIO_STEREO_VINS_BACKEND_LOG_H_