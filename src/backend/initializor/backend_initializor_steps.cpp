#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::ComputeRotationBasedOnFirstImuFrame(std::vector<Quat> &all_q_i0ii) {
    RETURN_FALSE_IF(data_manager_->visual_local_map()->frames().empty());
    all_q_i0ii.clear();
    all_q_i0ii.reserve(data_manager_->imu_based_frames().size());

    // Compute relative rotation between each frame.
    for (auto &imu_based_frame : data_manager_->imu_based_frames()) {
        RecomputeImuPreintegrationBlock(Vec3::Zero(), Vec3::Zero(), imu_based_frame);
    }

    // Compute relative rotation based on first imu frame.
    for (auto &imu_based_frame : data_manager_->imu_based_frames()) {
        if (all_q_i0ii.empty()) {
            all_q_i0ii.emplace_back(Quat::Identity());
        } else {
            const Quat q_i0ii = all_q_i0ii.back() * imu_based_frame.imu_preint_block.q_ij();
            all_q_i0ii.emplace_back(q_i0ii);
        }
    }

    return true;
}

}
