#include "vio.h"
#include "slam_operations.h"
#include "log_report.h"

namespace VIO {

bool Vio::RunOnce() {
    // Try to load packed measurements.
    std::unique_ptr<SingleMeasurement> measure = std::make_unique<SingleMeasurement>();
    const bool res = data_loader_->PopSingleMeasurement(*measure);
    if (!res) {
        const float time_s_for_no_data = measure_invalid_timer_.TockInSecond();
        if (time_s_for_no_data > options_.max_tolerence_time_s_for_no_data) {
            ReportWarn("[Vio] Failed to load packed measures for " << time_s_for_no_data << " s. Stop vio thread.");
        }
        return false;
    }
    measure_invalid_timer_.TockTickInSecond();

    // Check image measurements validation.
    const bool is_imu_valid = measure->imu != nullptr;
    const bool is_left_image_valid = measure->left_image != nullptr;
    const bool is_right_image_valid = measure->right_image != nullptr;
    ReportInfo("[Vio] Single measure : imu[" << is_imu_valid << "], left image[" << is_left_image_valid <<
        "], right image[" << is_right_image_valid << "].");

    // Transform image measurement to be features measurement.
    // TODO:

    // Heart Beat.
    HeartBeat();

    return true;
}

void Vio::HeartBeat() {
    if (vio_heart_beat_timer_.TockInSecond() > options_.heart_beat_period_time_s) {
        ReportInfo("[Vio] Heart beat for " << vio_heart_beat_timer_.TockTickInSecond() << " s. Vio has running for " <<
            vio_sys_timer_.TockInSecond() << " s.");
    }
}

}
