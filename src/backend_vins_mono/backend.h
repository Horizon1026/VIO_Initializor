#ifndef _VIO_INITIALIZOR_BACKEND_H_
#define _VIO_INITIALIZOR_BACKEND_H_

#include "basic_type.h"
#include "data_manager.h"
#include "general_graph_optimizor.h"
#include "imu.h"
#include "visual_frontend.h"

namespace vio {

using namespace slam_utility;
using namespace slam_data_log;
using namespace slam_solver;
using namespace sensor_model;
using DorF = float;

/* Options for Backend. */
struct BackendOptions {
    bool kEnableRecordBinaryCurveLog = true;

    Vec3 kGravityInWordFrame = Vec3(0.0f, 0.0f, 0.0f);

    float kMaxValidFeatureDepthInMeter = 0.0f;
    float kMinValidFeatureDepthInMeter = 0.0f;
    float kDefaultFeatureDepthInMeter = 0.0f;
};

struct BackendStatus {
    struct {
        uint32_t is_initialized : 1;
        uint32_t reserved : 31;
    };
};

struct BackendSignals {
    struct {
        uint32_t should_quit : 1;
        uint32_t reserved : 31;
    };
};

struct BackendStates {
    // Motion states.
    struct {
        Vec3 p_wi = Vec3::Zero();
        Quat q_wi = Quat::Identity();
        Vec3 v_wi = Vec3::Zero();
        Vec3 ba = Vec3::Zero();
        Vec3 bg = Vec3::Zero();
        float time_stamp_s = 0.0f;
    } motion;
};

/* Class Backend Declaration. */
class Backend final {

public:
    Backend() = default;
    ~Backend() = default;

    // Backend operations.
    bool RunOnce();
    void Reset();
    void ResetToReintialize();

    // Reference for member variables.
    BackendOptions &options() { return options_; }
    BackendSignals &signals() { return signals_; }
    VisualFrontend *&visual_frontend() { return visual_frontend_; }
    DataManager *&data_manager() { return data_manager_; }
    std::unique_ptr<Imu> &imu_model() { return imu_model_; }

    // Const reference for member variables.
    const BackendOptions &options() const { return options_; }
    const BackendStatus &status() const { return status_; }
    const BackendStates &states() const { return states_; }
    const BackendSignals &signals() const { return signals_; }

private:
    // Backend initializor.
    bool TryToInitialize();
    bool PrepareForPureVisualSfmByMonoView();
    bool PrepareForPureVisualSfmByMultiView();
    bool PerformPureVisualBundleAdjustment();
    bool EstimateGyroBias();
    bool EstimateVelocityGravityScaleIn3Dof(Vec3 &gravity_c0, float &scale);
    bool EstimateVelocityGravityScaleIn2Dof(Vec3 &gravity_c0, Vec &all_v_ii);
    bool SyncInitializedResult(const Vec3 &gravity_c0, const Vec &all_v_ii, const float &scale);

    // Backend data processor.
    bool TryToSolveFramePoseByFeaturesObservedByItself(const int32_t frame_id, const Vec3 &init_p_wc = Vec3::Zero(), const Quat &init_q_wc = Quat::Identity());
    bool TryToSolveFeaturePositionByFramesObservingIt(const int32_t feature_id, const int32_t min_frame_id = -1, const int32_t max_frame_id = kMaxInt32,
                                                      const bool use_multi_view = false);
    void RecomputeImuPreintegrationBlock(const Vec3 &bias_accel, const Vec3 &bias_gyro, ImuBasedFrame &imu_based_frame);
    TMat2<DorF> GetVisualObserveInformationMatrix();

private:
    // Options of backend.
    BackendOptions options_;
    // Flags of status of backend.
    BackendStatus status_;
    // Motion and prior states of backend.
    BackendStates states_;
    // Signals of backend.
    BackendSignals signals_;

    // Register some relative components.
    VisualFrontend *visual_frontend_ = nullptr;
    DataManager *data_manager_ = nullptr;
    std::unique_ptr<Imu> imu_model_ = nullptr;
};

}  // namespace vio

#endif  // end of _VIO_INITIALIZOR_BACKEND_H_
