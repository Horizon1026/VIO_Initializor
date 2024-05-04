#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"
#include "visualizor.h"

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
    // x = [p_i0_f1, p_i0_f2, p_i0_f3, ..., v_i0i0, gravity_i0]
    Mat A = Mat::Zero(2 * num_of_all_obv, selected_features.size() * 3 + 6);
    Vec b = Vec::Zero(2 * num_of_all_obv);
    const int32_t col_index_of_v_i0i0 = selected_features.size() * 3;
    int32_t row_index = 0;
    for (uint32_t feature_index = 0; feature_index < selected_features.size(); ++feature_index) {
        auto &feature_ptr = selected_features[feature_index];
        int32_t vector_index = feature_ptr->first_frame_id() - data_manager_->visual_local_map()->frames().front().id();
        for (const auto &observe : feature_ptr->observes()) {
            /* gama = [1  0  -u]
                      [0  1  -v] */
            Mat2x3 gama = Mat2x3::Identity();
            gama.col(2) = - observe[0].rectified_norm_xy;
            /* upsilon = gama * q_ic.inv * q_i0ii.inv */
            const Quat &q_i0ii = all_q_i0ii[vector_index];
            const Mat2x3 upsilon = gama * q_ic.toRotationMatrix().transpose() * q_i0ii.toRotationMatrix().transpose();
            // Fill linear function.
            const Vec3 &p_i0ii = all_p_i0ii[vector_index];
            const float &dt = all_dt_i0ii[vector_index];
            // x = [p_i0_f1, p_i0_f2, p_i0_f3, ..., v_i0i0, gravity_i0]
            A.block<2, 3>(row_index, feature_index * 3) = upsilon;
            A.block<2, 3>(row_index, col_index_of_v_i0i0) = - upsilon * dt;
            A.block<2, 3>(row_index, col_index_of_v_i0i0 + 3) = 0.5f * upsilon * dt * dt;
            b.segment<2>(row_index) = upsilon * p_i0ii + gama * q_ic.toRotationMatrix().transpose() * p_ic;
            // Prepare for next observation.
            ++vector_index;
            row_index += 2;
        }
    }

    Vec x = Vec::Zero(A.cols());
    if (!TryToEstimageGravityWithConstraintOfNorm(A, b, x)) {
        // Solve linear function Ax = b without constraint.
        x = (A.transpose() * A).colPivHouseholderQr().solve(A.transpose() * b);
        ReportWarn("[Backend] Initializor failed to esgimate gravity with constraint. Directly estimate them.");
    }
    g_i0 = x.segment<3>(col_index_of_v_i0i0 + 3);
    v_i0i0 = x.segment<3>(col_index_of_v_i0i0);
    for (uint32_t feature_index = 0; feature_index < selected_features.size(); ++feature_index) {
        auto &feature_ptr = selected_features[feature_index];
        feature_ptr->status() = FeatureSolvedStatus::kSolved;
        feature_ptr->param() = x.segment<3>(feature_index * 3);
    }
    ReportColorInfo("[Backend] Initializor succeed with g_i0 " << LogVec(g_i0) << ", v_i0i0 " << LogVec(v_i0i0) << ".");

    // Debug.
    const Vec3 gravity_c0 = q_ic.inverse() * g_i0;
    ReportInfo("estimate : gravity_c0 " << LogVec(gravity_c0) << ", gravity_i0 " << LogVec(g_i0));
    ReportInfo("groud truth : gravity_c0 [vec][ 3.6141 -8.3455 -3.0455], gravity_i0 is [vec][ 8.38406 -1.21679 -4.92615]");

    return true;
}

bool Backend::TryToEstimageGravityWithConstraintOfNorm(const Mat &A, const Vec &b, Vec &x) {
    // Solve linear function Ax = b subject to g_i0.norm() == 9.81
    const float gravity_norm = options_.kGravityInWordFrame.norm();
    const Mat &&A1 = A.block(0, 0, A.rows(), A.cols() - 3);
    const Mat A1_t = A1.transpose();
    const Mat &&A2 = A.block(0, A.cols() - 3, A.rows(), 3);
    const Mat A2_t = A2.transpose();
    const Mat A1tA1_invA1t = (A1_t * A1).llt().solve(Mat::Identity(A1.cols(), A1.cols())) * A1_t;  // (A1.t * A1).inv * A1.t
    const Mat temp = A2_t * (Mat::Identity(A1.rows(), A1.rows()) - A1 * A1tA1_invA1t);
    const Mat3 D = temp * A2;
    const Vec3 d = temp * b;
    const Vec7 ceoff = ComputeDongsiCoeff(D, d, gravity_norm);
    if (ceoff(0) != 1) {
        ReportWarn("[Backend] Initializor failed to compute dongsi coeff.");
        return false;
    }

    // Create companion matrix of polynomial.
    // Reference : https://en.wikipedia.org/wiki/Companion_matrix
    Mat6 companion_mat = Mat6::Zero();
    companion_mat.diagonal(-1).setOnes();
    companion_mat.col(companion_mat.cols() - 1) = - ceoff.reverse().head(ceoff.rows() - 1);
    ReportInfo("companion matrix\n" << companion_mat);
    Eigen::JacobiSVD<Mat6> svd(companion_mat);
    const Mat singular_values = svd.singularValues();
    const float cond = singular_values(0) / singular_values(singular_values.rows() - 1);
    ReportInfo("[Backend] Decompose companion matrix, cond [" << cond << "], rank [" <<
        svd.rank() << "], threshold [" << svd.threshold() << "].");
    if (svd.rank() != companion_mat.rows()) {
        ReportWarn("[Backend] Eigen value decomposition is not full rank.");
        return false;
    }

    // Compute eigen values of companion matrix.
    Eigen::EigenSolver<Mat6> solver(companion_mat, false);
    if (solver.info() != Eigen::Success) {
        ReportWarn("[Backend] Initializor failed to compute eigen values of companion matrix.");
        return false;
    }

    // Find the minimun real eigen value.
    float min_lambda = -1.0f;
    float min_cost = INFINITY;
    for (int32_t i = 0; i < solver.eigenvalues().size(); ++i) {
        const auto eigen_value = solver.eigenvalues()(i);
        CONTINUE_IF(eigen_value.imag() != 0);

        const float lambda = eigen_value.real();
        const Mat3 D_lambda_I_inv = (D - lambda * Mat3::Identity()).llt().solve(Mat3::Identity());
        const Vec3 state_gravity = D_lambda_I_inv * d;
        const float cost = std::fabs(state_gravity.norm() - gravity_norm);
        if (min_cost > cost) {
            min_cost = cost;
            min_lambda = lambda;
        }
    }
    if (std::isinf(min_cost)) {
        ReportWarn("[Backend] Initializor failed to find the minimun real eigen value.");
        return false;
    }

    // Recover gravity_i0 with constraint, velocity_i0 and each p_i0 of features.
    const Mat3 D_lambda_I_inv = (D - min_lambda * Mat3::Identity()).llt().solve(Mat3::Identity());
    const Vec3 g_i0 = D_lambda_I_inv * d;
    if (std::fabs(g_i0.norm() - gravity_norm) > 1e-3f) {
        ReportWarn("[Backend] Initializor failed to recovery gravity_i0.");
        return false;
    }
    x = Vec::Zero(A.cols());
    x.head(A.cols() - 3) = - A1tA1_invA1t * A2 * g_i0 + A1tA1_invA1t * b;
    x.tail(3) = g_i0;

    return true;
}

Vec7 Backend::ComputeDongsiCoeff(const Mat3 &D, const Vec3 &d, float gravity_norm) {
    // Reference[OpenVINS] : https://gist.github.com/goldbattle/3791cbb11bbf4f5feb3f049dad72bfdd
    float D1_1 = D(0, 0);
    float D1_2 = D(0, 1);
    float D1_3 = D(0, 2);
    float D2_1 = D(1, 0);
    float D2_2 = D(1, 1);
    float D2_3 = D(1, 2);
    float D3_1 = D(2, 0);
    float D3_2 = D(2, 1);
    float D3_3 = D(2, 2);
    float d1 = d(0, 0);
    float d2 = d(1, 0);
    float d3 = d(2, 0);
    float g = gravity_norm;

    // squared version we subbed for x^2.
    float D1_1_sq = D1_1 * D1_1, D1_2_sq = D1_2 * D1_2, D1_3_sq = D1_3 * D1_3;
    float D2_1_sq = D2_1 * D2_1, D2_2_sq = D2_2 * D2_2, D2_3_sq = D2_3 * D2_3;
    float D3_1_sq = D3_1 * D3_1, D3_2_sq = D3_2 * D3_2, D3_3_sq = D3_3 * D3_3;
    float d1_sq = d1 * d1, d2_sq = d2 * d2, d3_sq = d3 * d3;
    float g_sq = g * g;

    // Compute the coefficients.
    Vec7 coeff = Vec7::Zero();
    coeff(6) =
        -(-D1_1_sq * D2_2_sq * D3_3_sq * g_sq + D1_1_sq * D2_2_sq * d3_sq + 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * D3_3 * g_sq -
          D1_1_sq * D2_2 * D2_3 * d2 * d3 - D1_1_sq * D2_2 * D3_2 * d2 * d3 - D1_1_sq * D2_3_sq * D3_2_sq * g_sq +
          D1_1_sq * D2_3 * D3_2 * d2_sq + D1_1_sq * D2_3 * D3_2 * d3_sq - D1_1_sq * D2_3 * D3_3 * d2 * d3 -
          D1_1_sq * D3_2 * D3_3 * d2 * d3 + D1_1_sq * D3_3_sq * d2_sq + 2 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq -
          2 * D1_1 * D1_2 * D2_1 * D2_2 * d3_sq - 2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq + D1_1 * D1_2 * D2_1 * D2_3 * d2 * d3 +
          D1_1 * D1_2 * D2_1 * D3_2 * d2 * d3 - 2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_2 * D2_2 * D2_3 * d1 * d3 +
          D1_1 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_1 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq - D1_1 * D1_2 * D2_3 * D3_1 * d2_sq -
          D1_1 * D1_2 * D2_3 * D3_1 * d3_sq - D1_1 * D1_2 * D2_3 * D3_2 * d1 * d2 + D1_1 * D1_2 * D2_3 * D3_3 * d1 * d3 +
          D1_1 * D1_2 * D3_1 * D3_3 * d2 * d3 - D1_1 * D1_2 * D3_3_sq * d1 * d2 - 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq +
          D1_1 * D1_3 * D2_1 * D2_2 * d2 * d3 + 2 * D1_1 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq - D1_1 * D1_3 * D2_1 * D3_2 * d2_sq -
          D1_1 * D1_3 * D2_1 * D3_2 * d3_sq + D1_1 * D1_3 * D2_1 * D3_3 * d2 * d3 + 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq -
          D1_1 * D1_3 * D2_2_sq * d1 * d3 - 2 * D1_1 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq + D1_1 * D1_3 * D2_2 * D3_2 * d1 * d2 +
          D1_1 * D1_3 * D2_3 * D3_1 * d2 * d3 - D1_1 * D1_3 * D2_3 * D3_2 * d1 * d3 + D1_1 * D1_3 * D3_1 * D3_2 * d2 * d3 -
          2 * D1_1 * D1_3 * D3_1 * D3_3 * d2_sq + D1_1 * D1_3 * D3_2 * D3_3 * d1 * d2 + D1_1 * D2_1 * D2_2 * D3_2 * d1 * d3 -
          D1_1 * D2_1 * D2_3 * D3_2 * d1 * d2 + D1_1 * D2_1 * D3_2 * D3_3 * d1 * d3 - D1_1 * D2_1 * D3_3_sq * d1 * d2 -
          D1_1 * D2_2_sq * D3_1 * d1 * d3 + D1_1 * D2_2 * D2_3 * D3_1 * d1 * d2 - D1_1 * D2_3 * D3_1 * D3_2 * d1 * d3 +
          D1_1 * D2_3 * D3_1 * D3_3 * d1 * d2 - D1_2_sq * D2_1_sq * D3_3_sq * g_sq + D1_2_sq * D2_1_sq * d3_sq +
          2 * D1_2_sq * D2_1 * D2_3 * D3_1 * D3_3 * g_sq - D1_2_sq * D2_1 * D2_3 * d1 * d3 - D1_2_sq * D2_1 * D3_1 * d2 * d3 -
          D1_2_sq * D2_3_sq * D3_1_sq * g_sq + D1_2_sq * D2_3 * D3_1 * d1 * d2 + 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * D3_3 * g_sq -
          D1_2 * D1_3 * D2_1_sq * d2 * d3 - 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * D3_3 * g_sq + D1_2 * D1_3 * D2_1 * D2_2 * d1 * d3 -
          2 * D1_2 * D1_3 * D2_1 * D2_3 * D3_1 * D3_2 * g_sq + D1_2 * D1_3 * D2_1 * D3_1 * d2_sq + D1_2 * D1_3 * D2_1 * D3_1 * d3_sq -
          D1_2 * D1_3 * D2_1 * D3_3 * d1 * d3 + 2 * D1_2 * D1_3 * D2_2 * D2_3 * D3_1_sq * g_sq - D1_2 * D1_3 * D2_2 * D3_1 * d1 * d2 -
          D1_2 * D1_3 * D3_1_sq * d2 * d3 + D1_2 * D1_3 * D3_1 * D3_3 * d1 * d2 - D1_2 * D2_1_sq * D3_2 * d1 * d3 +
          D1_2 * D2_1 * D2_2 * D3_1 * d1 * d3 + D1_2 * D2_1 * D2_3 * D3_2 * d1_sq + D1_2 * D2_1 * D2_3 * D3_2 * d3_sq -
          D1_2 * D2_1 * D2_3 * D3_3 * d2 * d3 - D1_2 * D2_1 * D3_1 * D3_3 * d1 * d3 - D1_2 * D2_1 * D3_2 * D3_3 * d2 * d3 +
          D1_2 * D2_1 * D3_3_sq * d1_sq + D1_2 * D2_1 * D3_3_sq * d2_sq - D1_2 * D2_2 * D2_3 * D3_1 * d1_sq -
          D1_2 * D2_2 * D2_3 * D3_1 * d3_sq + D1_2 * D2_2 * D2_3 * D3_3 * d1 * d3 + D1_2 * D2_2 * D3_1 * D3_3 * d2 * d3 -
          D1_2 * D2_2 * D3_3_sq * d1 * d2 + D1_2 * D2_3_sq * D3_1 * d2 * d3 - D1_2 * D2_3_sq * D3_2 * d1 * d3 +
          D1_2 * D2_3 * D3_1_sq * d1 * d3 - D1_2 * D2_3 * D3_1 * D3_3 * d1_sq - D1_2 * D2_3 * D3_1 * D3_3 * d2_sq +
          D1_2 * D2_3 * D3_2 * D3_3 * d1 * d2 - D1_3_sq * D2_1_sq * D3_2_sq * g_sq + 2 * D1_3_sq * D2_1 * D2_2 * D3_1 * D3_2 * g_sq -
          D1_3_sq * D2_1 * D3_1 * d2 * d3 + D1_3_sq * D2_1 * D3_2 * d1 * d3 - D1_3_sq * D2_2_sq * D3_1_sq * g_sq +
          D1_3_sq * D3_1_sq * d2_sq - D1_3_sq * D3_1 * D3_2 * d1 * d2 + D1_3 * D2_1_sq * D3_2 * d1 * d2 -
          D1_3 * D2_1 * D2_2 * D3_1 * d1 * d2 - D1_3 * D2_1 * D2_2 * D3_2 * d1_sq - D1_3 * D2_1 * D2_2 * D3_2 * d3_sq +
          D1_3 * D2_1 * D2_2 * D3_3 * d2 * d3 + D1_3 * D2_1 * D3_1 * D3_3 * d1 * d2 + D1_3 * D2_1 * D3_2_sq * d2 * d3 -
          D1_3 * D2_1 * D3_2 * D3_3 * d1_sq - D1_3 * D2_1 * D3_2 * D3_3 * d2_sq + D1_3 * D2_2_sq * D3_1 * d1_sq +
          D1_3 * D2_2_sq * D3_1 * d3_sq - D1_3 * D2_2_sq * D3_3 * d1 * d3 - D1_3 * D2_2 * D2_3 * D3_1 * d2 * d3 +
          D1_3 * D2_2 * D2_3 * D3_2 * d1 * d3 - D1_3 * D2_2 * D3_1 * D3_2 * d2 * d3 + D1_3 * D2_2 * D3_2 * D3_3 * d1 * d2 -
          D1_3 * D2_3 * D3_1_sq * d1 * d2 + D1_3 * D2_3 * D3_1 * D3_2 * d1_sq + D1_3 * D2_3 * D3_1 * D3_2 * d2_sq -
          D1_3 * D2_3 * D3_2_sq * d1 * d2 + D2_1 * D2_2 * D3_2 * D3_3 * d1 * d3 - D2_1 * D2_2 * D3_3_sq * d1 * d2 -
          D2_1 * D2_3 * D3_2_sq * d1 * d3 + D2_1 * D2_3 * D3_2 * D3_3 * d1 * d2 - D2_2_sq * D3_1 * D3_3 * d1 * d3 +
          D2_2_sq * D3_3_sq * d1_sq + D2_2 * D2_3 * D3_1 * D3_2 * d1 * d3 + D2_2 * D2_3 * D3_1 * D3_3 * d1 * d2 -
          2 * D2_2 * D2_3 * D3_2 * D3_3 * d1_sq - D2_3_sq * D3_1 * D3_2 * d1 * d2 + D2_3_sq * D3_2_sq * d1_sq) /
        g_sq;
    coeff(5) =
        (-(2 * D1_1_sq * D2_2_sq * D3_3 * g_sq - 2 * D1_1_sq * D2_2 * D2_3 * D3_2 * g_sq + 2 * D1_1_sq * D2_2 * D3_3_sq * g_sq -
           2 * D1_1_sq * D2_2 * d3_sq - 2 * D1_1_sq * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1_sq * D2_3 * d2 * d3 +
           2 * D1_1_sq * D3_2 * d2 * d3 - 2 * D1_1_sq * D3_3 * d2_sq - 4 * D1_1 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq +
           2 * D1_1 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_1 * D1_2 * D2_1 * D3_3_sq * g_sq + 2 * D1_1 * D1_2 * D2_1 * d3_sq +
           2 * D1_1 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq + 2 * D1_1 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_1 * D1_2 * D2_3 * d1 * d3 -
           2 * D1_1 * D1_2 * D3_1 * d2 * d3 + 2 * D1_1 * D1_2 * D3_3 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq +
           2 * D1_1 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 2 * D1_1 * D1_3 * D2_1 * d2 * d3 - 2 * D1_1 * D1_3 * D2_2_sq * D3_1 * g_sq -
           4 * D1_1 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 2 * D1_1 * D1_3 * D2_2 * d1 * d3 + 2 * D1_1 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq +
           2 * D1_1 * D1_3 * D3_1 * d2_sq - 2 * D1_1 * D1_3 * D3_2 * d1 * d2 - 2 * D1_1 * D2_1 * D3_2 * d1 * d3 +
           2 * D1_1 * D2_1 * D3_3 * d1 * d2 + 2 * D1_1 * D2_2_sq * D3_3_sq * g_sq - 2 * D1_1 * D2_2_sq * d3_sq -
           4 * D1_1 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + 2 * D1_1 * D2_2 * D2_3 * d2 * d3 + 2 * D1_1 * D2_2 * D3_1 * d1 * d3 +
           2 * D1_1 * D2_2 * D3_2 * d2 * d3 + 2 * D1_1 * D2_3_sq * D3_2_sq * g_sq - 2 * D1_1 * D2_3 * D3_1 * d1 * d2 -
           2 * D1_1 * D2_3 * D3_2 * d2_sq - 2 * D1_1 * D2_3 * D3_2 * d3_sq + 2 * D1_1 * D2_3 * D3_3 * d2 * d3 +
           2 * D1_1 * D3_2 * D3_3 * d2 * d3 - 2 * D1_1 * D3_3_sq * d2_sq + 2 * D1_2_sq * D2_1_sq * D3_3 * g_sq -
           2 * D1_2_sq * D2_1 * D2_3 * D3_1 * g_sq - 2 * D1_2 * D1_3 * D2_1_sq * D3_2 * g_sq + 2 * D1_2 * D1_3 * D2_1 * D2_2 * D3_1 * g_sq +
           2 * D1_2 * D1_3 * D2_1 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D1_3 * D2_3 * D3_1_sq * g_sq - 2 * D1_2 * D2_1 * D2_2 * D3_3_sq * g_sq +
           2 * D1_2 * D2_1 * D2_2 * d3_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * D3_3 * g_sq - 2 * D1_2 * D2_1 * D3_3 * d1_sq -
           2 * D1_2 * D2_1 * D3_3 * d2_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * D3_3 * g_sq - 2 * D1_2 * D2_2 * D2_3 * d1 * d3 -
           2 * D1_2 * D2_2 * D3_1 * d2 * d3 + 2 * D1_2 * D2_2 * D3_3 * d1 * d2 - 2 * D1_2 * D2_3_sq * D3_1 * D3_2 * g_sq +
           2 * D1_2 * D2_3 * D3_1 * d1_sq + 2 * D1_2 * D2_3 * D3_1 * d2_sq + 2 * D1_2 * D2_3 * D3_1 * d3_sq -
           2 * D1_2 * D2_3 * D3_3 * d1 * d3 - 2 * D1_2 * D3_1 * D3_3 * d2 * d3 + 2 * D1_2 * D3_3_sq * d1 * d2 -
           2 * D1_3_sq * D2_1 * D3_1 * D3_2 * g_sq + 2 * D1_3_sq * D2_2 * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * D3_3 * g_sq -
           2 * D1_3 * D2_1 * D2_2 * d2 * d3 - 2 * D1_3 * D2_1 * D2_3 * D3_2_sq * g_sq + 2 * D1_3 * D2_1 * D3_2 * d1_sq +
           2 * D1_3 * D2_1 * D3_2 * d2_sq + 2 * D1_3 * D2_1 * D3_2 * d3_sq - 2 * D1_3 * D2_1 * D3_3 * d2 * d3 -
           2 * D1_3 * D2_2_sq * D3_1 * D3_3 * g_sq + 2 * D1_3 * D2_2_sq * d1 * d3 + 2 * D1_3 * D2_2 * D2_3 * D3_1 * D3_2 * g_sq -
           2 * D1_3 * D2_2 * D3_1 * d1_sq - 2 * D1_3 * D2_2 * D3_1 * d3_sq - 2 * D1_3 * D2_2 * D3_2 * d1 * d2 +
           2 * D1_3 * D2_2 * D3_3 * d1 * d3 + 2 * D1_3 * D3_1 * D3_3 * d2_sq - 2 * D1_3 * D3_2 * D3_3 * d1 * d2 -
           2 * D2_1 * D2_2 * D3_2 * d1 * d3 + 2 * D2_1 * D2_2 * D3_3 * d1 * d2 - 2 * D2_1 * D3_2 * D3_3 * d1 * d3 +
           2 * D2_1 * D3_3_sq * d1 * d2 + 2 * D2_2_sq * D3_1 * d1 * d3 - 2 * D2_2_sq * D3_3 * d1_sq - 2 * D2_2 * D2_3 * D3_1 * d1 * d2 +
           2 * D2_2 * D2_3 * D3_2 * d1_sq + 2 * D2_2 * D3_1 * D3_3 * d1 * d3 - 2 * D2_2 * D3_3_sq * d1_sq -
           2 * D2_3 * D3_1 * D3_3 * d1 * d2 + 2 * D2_3 * D3_2 * D3_3 * d1_sq) /
         g_sq);
    coeff(4) =
        ((D1_1_sq * D2_2_sq * g_sq + 4 * D1_1_sq * D2_2 * D3_3 * g_sq - 2 * D1_1_sq * D2_3 * D3_2 * g_sq + D1_1_sq * D3_3_sq * g_sq -
          D1_1_sq * d2_sq - D1_1_sq * d3_sq - 2 * D1_1 * D1_2 * D2_1 * D2_2 * g_sq - 4 * D1_1 * D1_2 * D2_1 * D3_3 * g_sq +
          2 * D1_1 * D1_2 * D2_3 * D3_1 * g_sq + D1_1 * D1_2 * d1 * d2 + 2 * D1_1 * D1_3 * D2_1 * D3_2 * g_sq -
          4 * D1_1 * D1_3 * D2_2 * D3_1 * g_sq - 2 * D1_1 * D1_3 * D3_1 * D3_3 * g_sq + D1_1 * D1_3 * d1 * d3 + D1_1 * D2_1 * d1 * d2 +
          4 * D1_1 * D2_2_sq * D3_3 * g_sq - 4 * D1_1 * D2_2 * D2_3 * D3_2 * g_sq + 4 * D1_1 * D2_2 * D3_3_sq * g_sq -
          4 * D1_1 * D2_2 * d3_sq - 4 * D1_1 * D2_3 * D3_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * d2 * d3 + D1_1 * D3_1 * d1 * d3 +
          4 * D1_1 * D3_2 * d2 * d3 - 4 * D1_1 * D3_3 * d2_sq + D1_2_sq * D2_1_sq * g_sq + 2 * D1_2 * D1_3 * D2_1 * D3_1 * g_sq -
          4 * D1_2 * D2_1 * D2_2 * D3_3 * g_sq + 2 * D1_2 * D2_1 * D2_3 * D3_2 * g_sq - 2 * D1_2 * D2_1 * D3_3_sq * g_sq -
          D1_2 * D2_1 * d1_sq - D1_2 * D2_1 * d2_sq + 2 * D1_2 * D2_1 * d3_sq + 2 * D1_2 * D2_2 * D2_3 * D3_1 * g_sq +
          D1_2 * D2_2 * d1 * d2 + 2 * D1_2 * D2_3 * D3_1 * D3_3 * g_sq - 3 * D1_2 * D2_3 * d1 * d3 - 3 * D1_2 * D3_1 * d2 * d3 +
          4 * D1_2 * D3_3 * d1 * d2 + D1_3_sq * D3_1_sq * g_sq + 2 * D1_3 * D2_1 * D2_2 * D3_2 * g_sq +
          2 * D1_3 * D2_1 * D3_2 * D3_3 * g_sq - 3 * D1_3 * D2_1 * d2 * d3 - 2 * D1_3 * D2_2_sq * D3_1 * g_sq -
          4 * D1_3 * D2_2 * D3_1 * D3_3 * g_sq + 4 * D1_3 * D2_2 * d1 * d3 + 2 * D1_3 * D2_3 * D3_1 * D3_2 * g_sq - D1_3 * D3_1 * d1_sq +
          2 * D1_3 * D3_1 * d2_sq - D1_3 * D3_1 * d3_sq - 3 * D1_3 * D3_2 * d1 * d2 + D1_3 * D3_3 * d1 * d3 + D2_1 * D2_2 * d1 * d2 -
          3 * D2_1 * D3_2 * d1 * d3 + 4 * D2_1 * D3_3 * d1 * d2 + D2_2_sq * D3_3_sq * g_sq - D2_2_sq * d1_sq - D2_2_sq * d3_sq -
          2 * D2_2 * D2_3 * D3_2 * D3_3 * g_sq + D2_2 * D2_3 * d2 * d3 + 4 * D2_2 * D3_1 * d1 * d3 + D2_2 * D3_2 * d2 * d3 -
          4 * D2_2 * D3_3 * d1_sq + D2_3_sq * D3_2_sq * g_sq - 3 * D2_3 * D3_1 * d1 * d2 + 2 * D2_3 * D3_2 * d1_sq - D2_3 * D3_2 * d2_sq -
          D2_3 * D3_2 * d3_sq + D2_3 * D3_3 * d2 * d3 + D3_1 * D3_3 * d1 * d3 + D3_2 * D3_3 * d2 * d3 - D3_3_sq * d1_sq - D3_3_sq * d2_sq) /
         g_sq);
    coeff(3) =
        ((2 * D1_1 * d2_sq + 2 * D1_1 * d3_sq + 2 * D2_2 * d1_sq + 2 * D2_2 * d3_sq + 2 * D3_3 * d1_sq + 2 * D3_3 * d2_sq -
          2 * D1_1 * D2_2_sq * g_sq - 2 * D1_1_sq * D2_2 * g_sq - 2 * D1_1 * D3_3_sq * g_sq - 2 * D1_1_sq * D3_3 * g_sq -
          2 * D2_2 * D3_3_sq * g_sq - 2 * D2_2_sq * D3_3 * g_sq - 2 * D1_2 * d1 * d2 - 2 * D1_3 * d1 * d3 - 2 * D2_1 * d1 * d2 -
          2 * D2_3 * d2 * d3 - 2 * D3_1 * d1 * d3 - 2 * D3_2 * d2 * d3 + 2 * D1_1 * D1_2 * D2_1 * g_sq + 2 * D1_1 * D1_3 * D3_1 * g_sq +
          2 * D1_2 * D2_1 * D2_2 * g_sq - 8 * D1_1 * D2_2 * D3_3 * g_sq + 4 * D1_1 * D2_3 * D3_2 * g_sq + 4 * D1_2 * D2_1 * D3_3 * g_sq -
          2 * D1_2 * D2_3 * D3_1 * g_sq - 2 * D1_3 * D2_1 * D3_2 * g_sq + 4 * D1_3 * D2_2 * D3_1 * g_sq + 2 * D1_3 * D3_1 * D3_3 * g_sq +
          2 * D2_2 * D2_3 * D3_2 * g_sq + 2 * D2_3 * D3_2 * D3_3 * g_sq) /
         g_sq);
    coeff(2) =
        (-(d1_sq + d2_sq + d3_sq - D1_1_sq * g_sq - D2_2_sq * g_sq - D3_3_sq * g_sq - 4 * D1_1 * D2_2 * g_sq + 2 * D1_2 * D2_1 * g_sq -
           4 * D1_1 * D3_3 * g_sq + 2 * D1_3 * D3_1 * g_sq - 4 * D2_2 * D3_3 * g_sq + 2 * D2_3 * D3_2 * g_sq) /
         g_sq);
    coeff(1) = (-(2 * D1_1 * g_sq + 2 * D2_2 * g_sq + 2 * D3_3 * g_sq) / g_sq);
    coeff(0) = 1;

    return coeff;

}

}
