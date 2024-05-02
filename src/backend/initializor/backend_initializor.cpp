#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::TryToInitialize() {
    if (data_manager_->imu_based_frames().size() < data_manager_->options().kMaxStoredKeyFrames) {
        ReportWarn("[Backend] Backend cannot initialize for lack of frames.");
        return false;
    }

    // Convert all frames into a covisible graph.
    if (!data_manager_->ConvertAllImuBasedFramesToLocalMap()) {
        ReportError("[Backend] Backend failed to convert frames to covisible graph.");
        return false;
    }
    data_manager_->ShowLocalMapFramesAndFeatures(-1, 0, 0);

    // Compute R_i0i1, R_i0i2, R_i0i3 and so on.
    std::vector<Quat> all_q_i0ii;
    std::vector<Vec3> all_p_i0ii;
    std::vector<float> all_dt_i0ii;
    if (!PreintegrateBasedOnFirstImuFrame(all_q_i0ii, all_p_i0ii, all_dt_i0ii)) {
        ReportError("[Backend] Backend failed to compute q_i0i1, q_i0i2, q_i0i3 and so on.");
        return false;
    }

    // Compute v_i0i0, g_i0 and p_c of each feature.
    Vec3 v_i0i0 = Vec3::Zero();
    Vec3 gravity_i0 = Vec3::Zero();
    if (!ComputeVelocityGravityAndFeaturePosition(all_q_i0ii, all_p_i0ii, all_dt_i0ii, v_i0i0, gravity_i0)) {
        ReportError("[Backend] Backend failed to compute v_i0i0, g_i0 and p_c of each feature.");
        return false;
    }

    return true;
}

}
