#include "ins.h"
#include "log_report.h"
#include "slam_operations.h"

namespace LOCATOR {

namespace {
    constexpr uint32_t kInsFusionCovarianceLogIndex = 1;
}

bool InsFusion::ConfigurationLog(const std::string &log_file_name) {
    if (!logger_.CreateLogFile(log_file_name)) {
        ReportError("[InsFusion] Failed to create log file.");
        options_.kEnableRecordLog = false;
        return false;
    }
    using namespace SLAM_DATA_LOG;

    std::unique_ptr<PackageInfo> package_oldest_cov_ptr = std::make_unique<PackageInfo>();
    package_oldest_cov_ptr->id = kInsFusionCovarianceLogIndex;
    package_oldest_cov_ptr->name = "oldest states covariance";
    package_oldest_cov_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kMatrix, .name = "sqrt_covariance"});
    if (!logger_.RegisterPackage(package_oldest_cov_ptr)) {
        ReportError("[ImuManager] Failed to register package for oldest state covariance log.");
    }

    logger_.PrepareForRecording();
    options_.kEnableRecordLog = true;

    return true;
}

bool InsFusion::RecordLog() {
    RETURN_TRUE_IF(!options_.kEnableRecordLog);
    const float log_time_stamp_s = states_.time_stamp_s;

    // Record subold states covariance log.
    logger_.RecordPackage(kInsFusionCovarianceLogIndex, states_.sqrt_cov, log_time_stamp_s);

    return true;
}

}
