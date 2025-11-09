#include "backend.h"
#include "slam_log_reporter.h"

namespace VIO {

bool Backend::TryToInitialize() {
    if (data_manager_->imu_based_frames().size() < data_manager_->options().kMaxStoredKeyFrames) {
        ReportWarn("[Backend] Backend cannot initialize for lack of new frames.");
        return false;
    }

    // Convert all frames into a covisible graph.
    if (!data_manager_->ConvertAllImuBasedFramesToLocalMap()) {
        ReportError("[Backend] Backend failed to convert frames to covisible graph.");
        return false;
    }

    // Estiamte bias_g (one-for-all), q_wc (define c0 frame as w frame) of each frame.
    if (!EstimateGyroBiasAndRotationForInitialization()) {
        ReportError("[Backend] Backend failed to estimate gyro bias.");
        return false;
    }

    // Estimate velocity of each frame, and gravity vector based on frame c0(camera).
    Vec3 gravity_i0 = Vec3::Zero();
    if (!EstimateVelocityAndGravityForInitialization(gravity_i0)) {
        ReportError("[Backend] Backend failed to estimate velocity or gravity.");
        return false;
    }

    // Transform all states from frame c0(camera) to frame w(world). The z-axis of frame w must be upward.
    if (!SyncInitializedResult(gravity_i0)) {
        ReportError("[Backend] Backend failed to transform from i0 to w frame.");
        return false;
    }

    return true;
}

bool Backend::SyncInitializedResult(const Vec3 &gravity_i0) {
    // Compute the gravity vector based on frame c0.
    const Quat q_ic = data_manager_->camera_extrinsics().front().q_ic;
    const Vec3 gravity_c0 = q_ic.inverse() * gravity_i0;

    // Compute the rotation from c0 to w.
    const Vec3 gravity_w = options_.kGravityInWordFrame;
    const Vec3 cross_vec = gravity_c0.cross(gravity_w);
    const float norm = cross_vec.norm();
    const Vec3 u = cross_vec / norm;
    const float theta = std::atan2(norm, gravity_c0.dot(gravity_w));
    const Vec3 angle_axis = u * theta;
    const Quat q_wc0 = Utility::Exponent(angle_axis);
    ReportInfo(GREEN "[Backend] Estimated q_wc0 is " << LogQuat(q_wc0) << "." << RESET_COLOR);

    // Iterate all frames, transform all states of them from i0 to w.
    auto it = data_manager_->imu_based_frames().begin();
    for (auto &frame: data_manager_->visual_local_map()->frames()) {
        const Quat q_c0c = frame.q_wc();
        const Vec3 p_c0c = frame.p_wc();
        const Vec3 v_c0c = it->v_wi;

        frame.q_wc() = q_wc0 * q_c0c;
        frame.p_wc() = q_wc0 * p_c0c;
        it->v_wi = q_wc0 * v_c0c;
        ++it;
    }

    // Try to triangulize all features of vision.
    if (!TriangulizeAllVisualFeatures()) {
        ReportError("[Backend] Backend failed to triangulize features.");
        return false;
    }
    return true;
}

}  // namespace VIO
