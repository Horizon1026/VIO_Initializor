#ifndef _VIO_STEREO_SCHUR_VINS_CONFIG_H_
#define _VIO_STEREO_SCHUR_VINS_CONFIG_H_

#include "datatype_basic.h"
#include "string"

namespace VIO {

struct VioOptionsOfCamera {
    float fx = 0.0f;
    float fy = 0.0f;
    float cx = 0.0f;
    float cy = 0.0f;

    float k1 = 0.0f;
    float k2 = 0.0f;
    float k3 = 0.0f;
    float p1 = 0.0f;
    float p2 = 0.0f;
};

struct VioOptionsOfImu {
    float noise_accel = std::sqrt(2.0000e-3f);
    float noise_gyro = std::sqrt(1.6968e-04f);
    float random_walk_accel = std::sqrt(3.0000e-3f);
    float random_walk_gyro = std::sqrt(1.9393e-05f);
};

struct VioOptionsOfFeatureDetector {
    int32_t min_valid_feature_distance = 25;
    int32_t grid_filter_rows = 11;
    int32_t grid_filter_cols = 11;
};

struct VioOptionsOfFeatureTracker {
    int32_t half_row_size_of_patch = 6;
    int32_t half_col_size_of_patch = 6;
    uint32_t max_iterations = 15;
};

struct VioOptionsOfFrontend {
    uint32_t image_rows = 0;
    uint32_t image_cols = 0;

    uint32_t max_feature_number = 121;
    uint32_t min_feature_number = 40;

    VioOptionsOfFeatureDetector feature_detector;
    VioOptionsOfFeatureTracker feature_tracker;

    bool select_keyframe = false;

    bool enable_drawing_track_result = false;
    bool enable_recording_curve_binlog = true;
    bool enable_recording_image_binlog = false;
    std::string log_file_name = "frontend.binlog";
};

struct VioOptionsOfDataLoader {
    uint32_t max_size_of_imu_buffer = 200;
    uint32_t max_size_of_image_buffer = 20;

    bool enable_recording_curve_binlog = true;
    bool enable_recording_raw_data_binlog = true;
    std::string log_file_name = "data_loader.binlog";
};

/* Options for vio. */
struct VioOptions {
    float max_tolerence_time_s_for_no_data = 2.0f;
    float heart_beat_period_time_s = 1.0f;

    std::vector<VioOptionsOfCamera> cameras;
    VioOptionsOfImu imu;
    VioOptionsOfFrontend frontend;
    VioOptionsOfDataLoader data_loader;

    std::string log_file_root_name = "../../Slam_Workspace/output/";
};

}

#endif // end of _VIO_STEREO_SCHUR_VINS_CONFIG_H_
