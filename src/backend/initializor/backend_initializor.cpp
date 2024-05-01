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

    // Compute R_i0i1, R_i0i2, R_i0i3 and so on.
    std::vector<Quat> all_q_i0ii;
    if (!ComputeRotationBasedOnFirstImuFrame(all_q_i0ii)) {
        ReportError("[Backend] Backend failed to compute q_i0i1, q_i0i2, q_i0i3 and so on.");
        return false;
    }

    return true;
}

}
