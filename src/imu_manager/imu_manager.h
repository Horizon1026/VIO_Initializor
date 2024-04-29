#ifndef _LOCATOR_IMU_MANAGER_H_
#define _LOCATOR_IMU_MANAGER_H_

#include "datatype_basic.h"
#include "sensor_manager.h"

#include "imu_manager_log.h"
#include "imu_measurement.h"
#include "imu_state.h"
#include "imu.h"

#include "square_root_kalman_filter.h"

namespace LOCATOR {

using namespace SLAM_UTILITY;
using namespace SLAM_SOLVER;
using namespace SENSOR_MODEL;

/* States of ImuManager. */
struct ImuManagerStates {
    ImuState oldest_states;
    float current_updated_time_stamp_s = 0.0f;
    bool is_attitude_valid = false;
    bool is_pos_vel_valid = false;
};

/* Options of ImuManager. */
struct ImuManagerOptions {
    Vec3 kGravityInWorldFrame = Vec3(0, 0, 9.81f);
    float kMaxToleranceResidualFromAccelNormToGravityNorm = 0.05f;

    float kMinGravityNormSigma = 0.1f;
    float kGyroBiasInWorldFrameSigma = 0.2f;

    float kMaxToleranceDelayInSecond = 0.3f;
    float kMaxTolerancePureImuPropagatePositionInSecond = 2.0f;

    bool kEnableRecordLog = true;
};

/* Class ImuManager Declaration. */
class ImuManager : public SensorManager<ImuManagerStates, ImuMeasurement> {

public:
    /* Indice of gnss manager state. */
    using StateIndex = ImuIndex;
    /* Indice of gnss manager observe. */
    enum ObserveIndex : uint8_t {
        kGb = 0,
        kBgz = 3,
    };

public:
    ImuManager() = default;
    virtual ~ImuManager() = default;

    // Record log.
    virtual bool ConfigurationLog(const std::string &log_file_name) override;
    virtual bool RecordLog() override;

    // States propagation and update.
    bool TryToInitializeAttitude(Mat &sqrt_cov);
    bool PropagateStatesAndCovariance(Mat &sqrt_cov);
    bool UpdateImuStatesByGravity(Mat &sqrt_cov);
    bool UpdateImuStatesByGravityAndFixGyroBiasZ(Mat &sqrt_cov);
    void SynchroniseBasicStates();

    // Process measurement.
    virtual float GetOldestMeasurementTimestamp() override;
    virtual float GetNewestMeasurementTimestamp() override;

    // Process states.
    virtual int32_t StatesSize() override { return 15; }
    virtual void UpdateStates(const Vec &dx) override;
    virtual void ResetStates() override;
    void RefreshCurrentUpdatedTimeStamp(const float time_stamp_s);
    void CheckPositionVelocityStateValidation();

    // Reference for Member Variables.
    ImuManagerOptions &options() { return options_; }
    Imu &imu_model() { return imu_model_; }
    SquareRootKalmanFilterDynamic<float> &filter() { return filter_; }

    // Const Reference for Member Variables.
    const ImuManagerOptions &options() const { return options_; }
    const Imu &imu_model() const { return imu_model_; }
    const SquareRootKalmanFilterDynamic<float> &filter() const { return filter_; }

private:
    // Options of imu manager.
    ImuManagerOptions options_;
    // Sensor model.
    Imu imu_model_;
    /* Kalman filter. */
    SquareRootKalmanFilterDynamic<float> filter_;
    // States : [ p_wi | v_wi | q_wi(3-dof) | ba | bg ].
    // Measures : [ gravity_i.x | gravity_i.y | gravity_i.z | bias_gyro_w.z ].
};

}

#endif // end of _LOCATOR_IMU_MANAGER_H_
