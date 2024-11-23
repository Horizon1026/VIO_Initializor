#ifndef _VIO_INITIALIZOR_BACKEND_H_
#define _VIO_INITIALIZOR_BACKEND_H_

#include "basic_type.h"
#include "imu.h"
#include "data_manager.h"
#include "visual_frontend.h"
#include "general_graph_optimizor.h"

namespace VIO {

using namespace SLAM_UTILITY;
using namespace SLAM_DATA_LOG;
using namespace SLAM_SOLVER;
using namespace SENSOR_MODEL;
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
    bool EstimatePureRotationOfCameraFrame(const uint32_t ref_frame_id,
                                           const uint32_t cur_frame_id,
                                           const uint32_t min_frame_id,
                                           std::vector<Vec2> &ref_norm_xy,
                                           std::vector<Vec2> &cur_norm_xy,
                                           Quat &q_cr);
    // Estimate gyro bias for initialization.
    bool EstimateGyroBiasAndRotationForInitialization();
    bool EstimateGyroBiasByMethodOneForInitialization();
    bool EstimateGyroBiasByMethodTwoForInitialization();
    bool EstimateGyroBiasByMethodThreeForInitialization();
    // Estimate velocity and gravity for initialization.
    bool EstimateVelocityAndGravityForInitialization(Vec3 &gravity_i0);
    bool SelectTwoFramesWithMaxParallax(CovisibleGraphType *local_map, const FeatureType &feature, int32_t &frame_id_l, int32_t &frame_id_r);
    bool ComputeImuPreintegrationBasedOnFirstFrameForInitialization(std::vector<ImuPreintegrateBlock<>> &imu_blocks);
    bool ConstructLigtFunction(const std::vector<ImuPreintegrateBlock<>> &imu_blocks, Mat6 &A, Vec6 &b, float &Q);
    bool RefineGravityForInitialization(const Mat &M, const Vec &m, const float Q, const float gravity_mag, Vec &rhs);
    bool PropagateAllBasedOnFirstCameraFrameForInitializaion(const std::vector<ImuPreintegrateBlock<>> &imu_blocks, const Vec3 &v_i0i0, const Vec3 &gravity_i0);
    bool SyncInitializedResult(const Vec3 &gravity_i0);

    // Backend data processor.
    bool TryToSolveFramePoseByFeaturesObservedByItself(const int32_t frame_id,
                                                       const Vec3 &init_p_wc = Vec3::Zero(),
                                                       const Quat &init_q_wc = Quat::Identity());
    bool TryToSolveFeaturePositionByFramesObservingIt(const int32_t feature_id,
                                                      const int32_t min_frame_id = -1,
                                                      const int32_t max_frame_id = kMaxInt32,
                                                      const bool use_multi_view = false);
    void RecomputeImuPreintegrationBlock(const Vec3 &bias_accel,
                                         const Vec3 &bias_gyro,
                                         ImuBasedFrame &imu_based_frame);
    TMat2<DorF> GetVisualObserveInformationMatrix();
    bool TriangulizeAllVisualFeatures();

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

}

#endif // end of _VIO_INITIALIZOR_BACKEND_H_
