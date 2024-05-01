#ifndef _VIO_STEREO_SCHUR_VINS_BACKEND_H_
#define _VIO_STEREO_SCHUR_VINS_BACKEND_H_

#include "datatype_basic.h"

#include "imu.h"
#include "data_manager.h"

#include "visual_frontend.h"
#include "general_graph_optimizor.h"

#include "backend_log.h"
#include "binary_data_log.h"

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

    // Backend log recorder.
    bool Configuration(const std::string &log_file_name);

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
    // Backend log recorder.
    void RegisterLogPackages();
    void RecordBackendLogStates();

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

    // Record log.
    BinaryDataLog logger_;
};

}

#endif // end of _VIO_STEREO_SCHUR_VINS_BACKEND_H_
