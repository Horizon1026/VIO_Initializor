#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::PreintegrateBasedOnFirstImuFrame(std::vector<Quat> &all_q_i0ii, std::vector<Vec3> &all_p_i0ii, std::vector<float> &all_dt_i0ii) {
    RETURN_FALSE_IF(data_manager_->visual_local_map()->frames().empty());

    // Do some preparation.
    all_q_i0ii.clear();
    all_p_i0ii.clear();
    all_dt_i0ii.clear();
    all_q_i0ii.reserve(data_manager_->imu_based_frames().size());
    all_p_i0ii.reserve(data_manager_->imu_based_frames().size());
    all_dt_i0ii.reserve(data_manager_->imu_based_frames().size());

    // Create a new imu preintegraion block.
    auto imu_preint_block = data_manager_->imu_based_frames().front().imu_preint_block;
    imu_preint_block.Reset();
    imu_preint_block.bias_accel() = Vec3::Zero();
    imu_preint_block.bias_gyro() = Vec3::Zero();
    imu_preint_block.SetImuNoiseSigma(imu_model_->options().kAccelNoiseSigma,
                                      imu_model_->options().kGyroNoiseSigma,
                                      imu_model_->options().kAccelRandomWalkSigma,
                                      imu_model_->options().kGyroRandomWalkSigma);

    // Compute relative rotation based on first imu frame.
    for (auto &imu_based_frame : data_manager_->imu_based_frames()) {
        if (imu_based_frame.time_stamp_s == data_manager_->imu_based_frames().front().time_stamp_s) {
            all_q_i0ii.emplace_back(Quat::Identity());
            all_p_i0ii.emplace_back(Vec3::Zero());
            all_dt_i0ii.emplace_back(0.0f);
        } else {
            const uint32_t max_idx = imu_based_frame.packed_measure->imus.size();
            for (uint32_t i = 1; i < max_idx; ++i) {
                imu_preint_block.Propagate(*imu_based_frame.packed_measure->imus[i - 1], *imu_based_frame.packed_measure->imus[i]);
            }

            all_q_i0ii.emplace_back(imu_preint_block.q_ij());
            all_p_i0ii.emplace_back(imu_preint_block.p_ij());
            all_dt_i0ii.emplace_back(imu_preint_block.integrate_time_s());
        }
    }

    // Debug.
    for (uint32_t i = 0; i < all_q_i0ii.size(); ++i) {
        ReportInfo("q_i0ii " << LogQuat(all_q_i0ii[i]) << ", p_i0ii " << LogVec(all_p_i0ii[i]) << ", dt_i0ii " << all_dt_i0ii[i] << "s.");
    }

    return true;
}

bool Backend::ComputeVelocityGravityAndFeaturePosition(const std::vector<Quat> &all_q_i0ii, const std::vector<Vec3> &all_p_i0ii,
    const std::vector<float> &all_dt_i0ii, Vec3 &v_i0i0, Vec3 &g_i0) {
    RETURN_FALSE_IF(data_manager_->visual_local_map()->features().empty());
    RETURN_FALSE_IF(data_manager_->camera_extrinsics().empty());

    // Select valid feaures.
    std::vector<FeatureType *> selected_features;
    int32_t num_of_all_obv = 0;
    for (auto &pair : data_manager_->visual_local_map()->features()) {
        auto &feature = pair.second;
        const int32_t num_of_obv = feature.observes().size();
        CONTINUE_IF(num_of_obv < 5);
        selected_features.emplace_back(&feature);
        num_of_all_obv += num_of_obv;
    }

    // Extract camera extrinsics.
    const Quat &q_ic = data_manager_->camera_extrinsics().front().q_ic;
    const Vec3 &p_ic = data_manager_->camera_extrinsics().front().p_ic;

    // Construct linear function.
    // x = [gravity_i0, v_i0i0, p_i0_f1, p_i0_f2, p_i0_f3, ...]
    Mat A = Mat::Zero(2 * num_of_all_obv, 6 + selected_features.size() * 3);
    Vec b = Vec::Zero(2 * num_of_all_obv);
    int32_t row_index = 0;
    for (uint32_t feature_index = 0; feature_index < selected_features.size(); ++feature_index) {
        auto &feature_ptr = selected_features[feature_index];
        int32_t vector_index = feature_ptr->first_frame_id() - data_manager_->visual_local_map()->frames().front().id();
        for (const auto &observe : feature_ptr->observes()) {
            /* gama = [1  0  -u]
                      [0  1  -v] */
            const Vec2 norm_xy = observe[0].rectified_norm_xy;
            Mat2x3 gama = Mat2x3::Identity();
            gama.col(2) = - norm_xy;
            /* upsilon = gama * q_ic.inv * q_i0ii.inv */
            const Quat &q_i0ii = all_q_i0ii[vector_index];
            const Mat2x3 upsilon = gama * q_ic.toRotationMatrix().transpose() * q_i0ii.toRotationMatrix().transpose();
            // Fill linear function.
            const Vec3 &p_i0ii = all_p_i0ii[vector_index];
            const float &dt = all_dt_i0ii[vector_index];
            A.block<2, 3>(row_index, 0) = upsilon * dt * dt * 0.5f;
            A.block<2, 3>(row_index, 3) = - upsilon * dt;
            A.block<2, 3>(row_index, 6 + feature_index * 3) = upsilon;
            b.segment<2>(row_index) = upsilon * p_i0ii + gama * q_ic.toRotationMatrix().transpose() * p_ic;
            // Prepare for next observation.
            ++vector_index;
            row_index += 2;
        }
    }

    // Solve linear function.
    const Vec x = A.householderQr().solve(b);
    g_i0 = x.segment<3>(0);
    v_i0i0 = x.segment<3>(3);
    for (uint32_t feature_index = 0; feature_index < selected_features.size(); ++feature_index) {
        auto &feature_ptr = selected_features[feature_index];
        feature_ptr->status() = FeatureSolvedStatus::kSolved;
        feature_ptr->param() = x.segment<3>(6 + feature_index * 3);
    }

    // Debug.
    const Vec3 gravity_c0 = q_ic.inverse() * g_i0;
    ReportInfo("estimate : gravity_c0 " << LogVec(gravity_c0) << ", gravity_i0 " << LogVec(g_i0));
    ReportInfo("groud truth : gravity_c0 [vec][ 3.6141 -8.3455 -3.0455], gravity_i0 is [vec][ 8.38406 -1.21679 -4.92615]");

    return true;
}

}
