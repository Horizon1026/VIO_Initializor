#include "imu_manager.h"
#include "slam_operations.h"
#include "log_report.h"

namespace LOCATOR {

float ImuManager::GetOldestMeasurementTimestamp() {
    if (measures().empty()) {
        return 0.0f;
    }
    return measures().front()->time_stamp_s;
}

float ImuManager::GetNewestMeasurementTimestamp() {
    if (measures().empty()) {
        return 0.0f;
    }
    return measures().back()->time_stamp_s;
}

void ImuManager::UpdateStates(const Vec &dx) {
    auto &imu_state = states().oldest_states;

    imu_state.p_wi() += dx.segment(StateIndex::kPosition, 3);
    imu_state.q_wi() = imu_state.q_wi() * Utility::DeltaQ(dx.segment(StateIndex::kRotation, 3));
    imu_state.q_wi().normalize();
    imu_state.v_wi() += dx.segment(StateIndex::kVelocity, 3);
    imu_state.ba() += dx.segment(StateIndex::kBiasAccel, 3);
    imu_state.bg() += dx.segment(StateIndex::kBiasGyro, 3);
}

void ImuManager::ResetStates() {
    states().oldest_states.Clear();
    states().is_attitude_valid = false;
    states().is_pos_vel_valid = false;
}

void ImuManager::RefreshCurrentUpdatedTimeStamp(const float time_stamp_s) {
    if (!states().is_pos_vel_valid) {
        return;
    }

    states().current_updated_time_stamp_s = time_stamp_s;
}

void ImuManager::CheckPositionVelocityStateValidation() {
    if (!states().is_pos_vel_valid) {
        return;
    }

    // If pure imu propagated too long, set position and velocity to be invalid.
    if (states().oldest_states.time_stamp_s() - states().current_updated_time_stamp_s >
        options_.kMaxTolerancePureImuPropagatePositionInSecond) {

        ReportWarn("[ImuManager] Position, velocity, bias_accel become invalid at " <<
            states().oldest_states.time_stamp_s() << "s for lack of observation.");
        states().is_pos_vel_valid = false;
        states().oldest_states.p_wi().setZero();
        states().oldest_states.v_wi().setZero();
        states().oldest_states.ba().setZero();
    }
}

bool ImuManager::TryToInitializeAttitude(Mat &sqrt_cov) {
    RETURN_FALSE_IF(measures().empty());
    const auto &measure = measures().front();

    ResetStates();
    states().oldest_states.g_w() = options_.kGravityInWorldFrame;

    // Get attitude by gravity vector.
    const Vec3 g_i = measure->accel - states().oldest_states.ba();
    const Vec3 &g_w = options_.kGravityInWorldFrame;
    RETURN_FALSE_IF(std::fabs(g_w.norm() - g_i.norm()) > options_.kMaxToleranceResidualFromAccelNormToGravityNorm);
    const Vec3 g_cross = g_i.cross(g_w);
    const float norm = g_cross.norm();
    const Vec3 u = g_cross / norm;
    const float theta = std::atan2(norm, g_i.dot(g_w));

    // Initialize states.
    states().oldest_states.q_wi() = Utility::DeltaQ(u * theta).normalized();
    states().oldest_states.time_stamp_s() = measure->time_stamp_s;

    // Initialize covariance of states.
    sqrt_cov.setZero();
    sqrt_cov.block<3, 3>(StateIndex::kRotation, StateIndex::kRotation) = Mat3::Identity() * imu_model_.options().kGyroNoiseSigma;
    sqrt_cov.block<3, 3>(StateIndex::kBiasGyro, StateIndex::kBiasGyro) = Mat3::Identity() * imu_model_.options().kGyroRandomWalkSigma;

    // Update validation flag.
    states().is_attitude_valid = true;

    ReportColorInfo("[ImuManager] Succeed to initialize attitude with imu measurement at " << states().oldest_states.time_stamp_s() <<
        "s, q_wi " << LogQuat(states().oldest_states.q_wi()));

    return true;
}

bool ImuManager::PropagateStatesAndCovariance(Mat &sqrt_cov) {
    RETURN_FALSE_IF(measures().size() < 2);

    if (states().oldest_states.time_stamp_s() != measures().front()->time_stamp_s) {
        ReportError("[ImuManager] Time stamp error. [Oldest state | Oldest imu state] [" <<
            states().oldest_states.time_stamp_s() << " | " << measures().front()->time_stamp_s << "].");
        return false;
    }

    // [ x(i) | x(j) | x | ... | x | x ].
    const auto imu_state_i = states().oldest_states;
    auto &imu_state_j = states().oldest_states;
    const auto imu_measure_i = std::move(measures().front());
    measures().pop_front();
    const auto &imu_measure_j = measures().front();

    // Check validation of time delay.
    const float dt = imu_measure_j->time_stamp_s - imu_measure_i->time_stamp_s;
    if (dt < 0) {
        ReportWarn("[ImuManager] Detecting timestamp of imu is rolling back. Discard this tick.");
        return true;
    }
    if (dt > options_.kMaxToleranceDelayInSecond) {
        ReportWarn("[ImuManager] Imu measurements delay too long. Reinitialize attitude.");
        states().is_attitude_valid = TryToInitializeAttitude(sqrt_cov);
        return true;
    }

    // Propagate states and covariance.
    if (states().is_pos_vel_valid) {
        // If position is valid, propagate full states.
        Vec3 mid_gyro = Vec3::Zero();
        Vec3 mid_accel = Vec3::Zero();
        imu_model_.PropagateNominalState(*imu_measure_i, *imu_measure_j, imu_state_i, imu_state_j, mid_accel, mid_gyro);

        // Compute some template values.
        const float sqrt_dt = std::sqrt(dt);
        const Mat3 dt_I3 = dt * Mat3::Identity();
        const Mat3 sqrt_dt_I3 = sqrt_dt * Mat3::Identity();
        const Mat3 R_wi_i = imu_state_i.q_wi().matrix();
        const Mat3 dt_R_wi_i = dt * R_wi_i;

        // Propagate square-root covariance of states.
        const int32_t index = StatesColIndex();
        const int32_t size = StatesSize();
        filter_.S_t() = sqrt_cov.block(index, index, size, size);
        filter_.F().setIdentity(StatesSize(), StatesSize());
        filter_.F().block<3, 3>(StateIndex::kPosition, StateIndex::kVelocity) = dt_I3;
        filter_.F().block<3, 3>(StateIndex::kVelocity, StateIndex::kRotation) = - dt_R_wi_i * Utility::SkewSymmetricMatrix(mid_accel);
        filter_.F().block<3, 3>(StateIndex::kVelocity, StateIndex::kBiasAccel) = - dt_R_wi_i;
        filter_.F().block<3, 3>(StateIndex::kRotation, StateIndex::kRotation) = Mat3::Identity() - dt * Utility::SkewSymmetricMatrix(mid_gyro);
        filter_.F().block<3, 3>(StateIndex::kRotation, StateIndex::kBiasGyro) = - dt_I3;
        filter_.square_Q_t().setZero(StatesSize(), StatesSize());
        filter_.square_Q_t().block<3, 3>(StateIndex::kVelocity, StateIndex::kVelocity) = dt_R_wi_i * imu_model_.options().kAccelNoiseSigma;
        filter_.square_Q_t().block<3, 3>(StateIndex::kRotation, StateIndex::kRotation) = dt_I3 * imu_model_.options().kGyroNoiseSigma;
        filter_.square_Q_t().block<3, 3>(StateIndex::kBiasAccel, StateIndex::kBiasAccel) = sqrt_dt_I3 * imu_model_.options().kAccelRandomWalkSigma;
        filter_.square_Q_t().block<3, 3>(StateIndex::kBiasGyro, StateIndex::kBiasGyro) = sqrt_dt_I3 * imu_model_.options().kGyroRandomWalkSigma;
        filter_.PropagateCovariance();

        // Sync covariance and timestamp.
        sqrt_cov.block(index, index, size, size).noalias() = filter_.predict_S_t();

    } else {
        // If position is not valid, only propagate attitude.
        // Propagate nominal states.
        const Vec3 mid_gyro = 0.5f * (imu_measure_i->gyro + imu_measure_j->gyro) - imu_state_j.bg();
        imu_state_j.q_wi() = imu_state_i.q_wi() * Utility::DeltaQ(mid_gyro * dt);
        imu_state_j.q_wi().normalize();

        // Propagate covariance of states.
        const int32_t index = StatesColIndex();
        const int32_t size = StatesSize();
        filter_.S_t() = sqrt_cov.block(index, index, size, size);
        filter_.F().setIdentity(StatesSize(), StatesSize());
        filter_.F().block<3, 3>(StateIndex::kRotation, StateIndex::kRotation) = Mat3::Identity() - Utility::SkewSymmetricMatrix(mid_gyro) * dt;
        filter_.F().block<3, 3>(StateIndex::kRotation, StateIndex::kBiasGyro) = - dt * Mat3::Identity();
        filter_.square_Q_t().setZero(StatesSize(), StatesSize());
        filter_.square_Q_t().block<3, 3>(StateIndex::kRotation, StateIndex::kRotation) = Mat3::Identity() * imu_model_.options().kGyroNoiseSigma * dt;
        filter_.square_Q_t().block<3, 3>(StateIndex::kBiasGyro, StateIndex::kBiasGyro) = Mat3::Identity() * imu_model_.options().kGyroRandomWalkSigma * dt;
        filter_.PropagateCovariance();
        sqrt_cov.block(index, index, size, size).noalias() = filter_.predict_S_t();
    }

    // Sync timestamp.
    imu_state_j.time_stamp_s() = imu_measure_j->time_stamp_s;

    return true;
}

bool ImuManager::UpdateImuStatesByGravity(Mat &sqrt_cov) {
    RETURN_FALSE_IF(measures().empty());
    RETURN_FALSE_IF(!states().is_attitude_valid);

    const auto &measure = measures().front();
    const auto &imu_state = states().oldest_states;

    // Compute observation and prediction.
    const Vec3 gravity_b = measure->accel - imu_state.ba();
    Vec3 obv = gravity_b / gravity_b.norm();
    Vec3 pred = imu_state.q_wi().matrix().transpose().col(2);
    residual() = obv - pred;

    // Compute dynamic weight.
    const float weight = std::max(options_.kMinGravityNormSigma, std::fabs(gravity_b.norm() - options_.kGravityInWorldFrame.norm()));

    // Construct measurement function to update states.
    const int32_t index = StatesColIndex();
    const int32_t size = StatesSize();
    filter_.predict_S_t() = sqrt_cov.block(index, index, size, size);
    filter_.H().setZero(obv.rows(), StatesSize());
    filter_.H().block<3, 3>(ObserveIndex::kGb, StateIndex::kRotation) = Utility::SkewSymmetricMatrix(obv.head<3>()) * Utility::SkewSymmetricMatrix(pred.head<3>());
    filter_.square_R_t().setZero(obv.rows(), obv.rows());
    filter_.square_R_t().block<3, 3>(ObserveIndex::kGb, ObserveIndex::kGb) = Mat3::Identity() * std::sqrt(weight);
    filter_.UpdateStateAndCovariance(residual());

    // Update states and covariance.
    UpdateStates(filter_.dx());
    sqrt_cov.block(index, index, size, size).noalias() = filter_.S_t();

    return true;
}

bool ImuManager::UpdateImuStatesByGravityAndFixGyroBiasZ(Mat &sqrt_cov) {
    RETURN_FALSE_IF(measures().empty());
    RETURN_FALSE_IF(!states().is_attitude_valid);

    const auto &measure = measures().front();
    const auto &imu_state = states().oldest_states;

    // Compute observation and prediction.
    const Vec3 gravity_b = measure->accel - imu_state.ba();
    Vec4 obv = Vec4::Zero();
    obv.segment<3>(ObserveIndex::kGb) = gravity_b / gravity_b.norm();
    obv.segment<1>(ObserveIndex::kBgz) = (imu_state.q_wi() * imu_state.bg()).tail<1>();
    Vec4 pred = Vec4::Zero();
    pred.segment<3>(ObserveIndex::kGb) = imu_state.q_wi().matrix().transpose().col(2);
    pred.segment<1>(ObserveIndex::kBgz) = obv.segment<1>(ObserveIndex::kBgz);
    residual() = Vec4::Zero();
    residual().segment<3>(ObserveIndex::kGb) = Utility::SkewSymmetricMatrix(pred.segment<3>(ObserveIndex::kGb)) * obv.segment<3>(ObserveIndex::kGb);
    residual().segment<1>(ObserveIndex::kBgz) = obv.segment<1>(ObserveIndex::kBgz) - pred.segment<1>(ObserveIndex::kBgz);

    // Compute dynamic weight.
    const float weight = std::max(options_.kMinGravityNormSigma, std::fabs(gravity_b.norm() - options_.kGravityInWorldFrame.norm()));

    // Construct measurement function to update states.
    const int32_t index = StatesColIndex();
    const int32_t size = StatesSize();
    filter_.predict_S_t() = sqrt_cov.block(index, index, size, size);
    filter_.H().setZero(obv.rows(), StatesSize());
    filter_.H().block<3, 3>(ObserveIndex::kGb, StateIndex::kRotation) = Utility::SkewSymmetricMatrix(obv.head<3>()) * Utility::SkewSymmetricMatrix(pred.head<3>());
    filter_.H().block<1, 3>(ObserveIndex::kBgz, StateIndex::kRotation) = - (imu_state.q_wi() * Utility::SkewSymmetricMatrix(imu_state.bg())).row(2);
    filter_.H().block<1, 3>(ObserveIndex::kBgz, StateIndex::kBiasGyro) = - (imu_state.q_wi().matrix()).row(2);
    filter_.square_R_t().setZero(obv.rows(), obv.rows());
    filter_.square_R_t().block<3, 3>(ObserveIndex::kGb, ObserveIndex::kGb) = Mat3::Identity() * std::sqrt(weight);
    filter_.square_R_t().block<1, 1>(ObserveIndex::kBgz, ObserveIndex::kBgz) = Mat1::Identity() * options_.kGyroBiasInWorldFrameSigma;
    filter_.UpdateStateAndCovariance(residual());

    // Update states and covariance.
    UpdateStates(filter_.dx());
    sqrt_cov.block(index, index, size, size).noalias() = filter_.S_t();

    return true;
}

void ImuManager::SynchroniseBasicStates() {
    const auto &imu_measure = measures().front();
    auto &imu_state = states().oldest_states;
    if (states().is_attitude_valid) {
        imu_state.gyro() = imu_measure->gyro - imu_state.bg();
        imu_state.accel() = imu_state.q_wi() * (imu_measure->accel - imu_state.ba()) - imu_state.g_w();
    } else {
        imu_state.gyro().setZero();
        imu_state.accel().setZero();
    }
}

}
