#include "ins.h"
#include "slam_operations.h"
#include "log_report.h"

namespace INS {

bool InsFusion::Initialize() {
    RETURN_FALSE_IF(imu_manager_ == nullptr);

    int32_t full_size_of_states = 0;
    imu_manager_->SetStatesColIndex(full_size_of_states);
    full_size_of_states += imu_manager_->StatesSize();

    states_.motion.Clear();
    states_.sqrt_cov.setZero(full_size_of_states, full_size_of_states);
    states_.time_stamp_s = 0.0f;
    ReportColorInfo("[InsFusion] Full size of states is " << full_size_of_states << ".");

    return true;
}

bool InsFusion::RunOnce() {
    if (ProcessImuMeasurementOnce()) {
        imu_manager_->RecordLog();
        states_.motion = imu_manager_->states().oldest_states;
        states_.time_stamp_s = imu_manager_->states().oldest_states.time_stamp_s();
    }

    RecordLog();

    return true;
}

bool InsFusion::ProcessImuMeasurementOnce() {
    // Check imu state validation.
    if (!imu_manager_->states().is_attitude_valid) {
        if (!imu_manager_->TryToInitializeAttitude(states_.sqrt_cov)) {
            ReportWarn("[InsFusion] Failed to initialize attitude.");
            return true;
        }
    }
    imu_manager_->CheckPositionVelocityStateValidation();

    // Delay process imu measurements.
    if (imu_manager_->GetNewestMeasurementTimestamp() - imu_manager_->GetOldestMeasurementTimestamp() <= options_.kMaxBufferSizeInSecond) {
        return true;
    }

    // Propagate oldest imu states and covariance.
    // After propagateion, oldest imu measurements will be discarded.
    imu_manager_->PropagateStatesAndCovariance(states_.sqrt_cov);
    // Update attitude with gravity observation.
    if (imu_manager_->states().is_attitude_valid && !imu_manager_->measures().empty()) {
        imu_manager_->UpdateImuStatesByGravityAndFixGyroBiasZ(states_.sqrt_cov);
    }
    // Sync basic states by imu measurements.
    imu_manager_->SynchroniseBasicStates();

    return true;
}

}
