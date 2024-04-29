#ifndef _INS_FUSION_H_
#define _INS_FUSION_H_

#include "datatype_basic.h"
#include "ins_log.h"
#include "binary_data_log.h"

#include "imu_measurement.h"
#include "imu_state.h"
#include "imu_manager.h"

namespace INS {

using namespace SLAM_SOLVER;

/* Frames Definition. */
// w. World frame.
// i. Imu frame.

/* Options of InsFusion. */
struct InsFusionOptions {
    float kMaxBufferSizeInSecond = 0.3f;
    float kMaxValidTimestampDiffOfMeasurementInSecond = 0.05f;

    bool kEnableRecordLog = true;
};

/* States of InsFusion. */
struct InsFusionStates {
    // Newest motion states.
    ImuState motion;

    // Covariance of full size for all sensor states in oldest timestamp.
    Mat sqrt_cov = Mat15::Zero();
    float time_stamp_s = 0.0;
};

/* Class InsFusion Declaration. */
class InsFusion {

public:
    InsFusion() = default;
    virtual ~InsFusion() = default;

    bool Initialize();
    bool RunOnce();

    // Record log.
    bool ConfigurationLog(const std::string &log_file_name);
    bool RecordLog();

    // Process all measurements.
    bool ProcessImuMeasurementOnce();

    // Reference for Member Variables.
    InsFusionOptions &options() { return options_; }
    std::unique_ptr<ImuManager> &imu_manager() { return imu_manager_; }

    // Const Reference for Member Variables.
    const InsFusionOptions &options() const { return options_; }
    const std::unique_ptr<ImuManager> &imu_manager() const { return imu_manager_; }

private:
    // Managers of each type of observation.
    std::unique_ptr<ImuManager> imu_manager_ = std::make_unique<ImuManager>();

    // States of InsFusion.
    InsFusionStates states_;
    // Options of InsFusion.
    InsFusionOptions options_;
    // Record log.
    BinaryDataLog logger_;
};

}

#endif // end of _INS_FUSION_H_
