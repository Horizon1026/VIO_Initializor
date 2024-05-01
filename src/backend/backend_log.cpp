#include "backend.h"
#include "log_report.h"

namespace VIO {

namespace {
    constexpr uint32_t kBackendStatesLogIndex = 1;
}

bool Backend::Configuration(const std::string &log_file_name) {
    if (options_.kEnableRecordBinaryCurveLog) {
        if (!logger_.CreateLogFile(log_file_name)) {
            ReportError("[Backend] Failed to create log file.");
            options_.kEnableRecordBinaryCurveLog = false;
            return false;
        }

        RegisterLogPackages();
        logger_.PrepareForRecording();
    }

    return true;
}

void Backend::RegisterLogPackages() {
    using namespace SLAM_DATA_LOG;

    std::unique_ptr<PackageInfo> package_states_ptr = std::make_unique<PackageInfo>();
    package_states_ptr->id = kBackendStatesLogIndex;
    package_states_ptr->name = "backend states";
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "time_stamp_s"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_x"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_y"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_z"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_w"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_x"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_y"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_z"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_pitch"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_roll"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_yaw"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_x"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_y"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_z"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_x"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_y"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_z"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_x"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_y"});
    package_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_z"});
    if (!logger_.RegisterPackage(package_states_ptr)) {
        ReportError("[Backend] Failed to register package for backend states log.");
    }
}

void Backend::RecordBackendLogStates() {
    RETURN_IF(!options().kEnableRecordBinaryCurveLog);

    BackendLogStates log_package;
    log_package.time_stamp_s = states_.motion.time_stamp_s;

    log_package.p_wi_x = states_.motion.p_wi.x();
    log_package.p_wi_y = states_.motion.p_wi.y();
    log_package.p_wi_z = states_.motion.p_wi.z();

    const Vec3 euler = Utility::QuaternionToEuler(states_.motion.q_wi);
    log_package.q_wi_pitch = euler.x();
    log_package.q_wi_roll = euler.y();
    log_package.q_wi_yaw = euler.z();

    log_package.q_wi_w = states_.motion.q_wi.w();
    log_package.q_wi_x = states_.motion.q_wi.x();
    log_package.q_wi_y = states_.motion.q_wi.y();
    log_package.q_wi_z = states_.motion.q_wi.z();

    log_package.v_wi_x = states_.motion.v_wi.x();
    log_package.v_wi_y = states_.motion.v_wi.y();
    log_package.v_wi_z = states_.motion.v_wi.z();

    log_package.bias_a_x = states_.motion.ba.x();
    log_package.bias_a_y = states_.motion.ba.y();
    log_package.bias_a_z = states_.motion.ba.z();

    log_package.bias_g_x = states_.motion.bg.x();
    log_package.bias_g_y = states_.motion.bg.y();
    log_package.bias_g_z = states_.motion.bg.z();

    // Record log.
    logger_.RecordPackage(kBackendStatesLogIndex, reinterpret_cast<const char *>(&log_package), states_.motion.time_stamp_s);
}

}
