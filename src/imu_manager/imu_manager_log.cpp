#include "imu_manager.h"
#include "log_report.h"
#include "slam_operations.h"

namespace LOCATOR {

namespace {
    constexpr uint32_t kImuManagerMeasuresLogIndex = 1;
    constexpr uint32_t kImuManagerSuboldStatesLogIndex = 2;
    constexpr uint32_t kImuManagerMonitorLogIndex = 3;
    constexpr uint32_t kImuManagerObserveResidualLogIndex = 4;
}

bool ImuManager::ConfigurationLog(const std::string &log_file_name) {
    if (!logger().CreateLogFile(log_file_name)) {
        ReportError("[ImuManager] Failed to create log file.");
        options_.kEnableRecordLog = false;
        return false;
    }
    using namespace SLAM_DATA_LOG;

    std::unique_ptr<PackageInfo> package_measure_ptr = std::make_unique<PackageInfo>();
    package_measure_ptr->id = kImuManagerMeasuresLogIndex;
    package_measure_ptr->name = "measurements";
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "time_stamp_s"});
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "accel_x_mps2"});
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "accel_y_mps2"});
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "accel_z_mps2"});
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_x_rps"});
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_y_rps"});
    package_measure_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_z_rps"});
    if (!logger().RegisterPackage(package_measure_ptr)) {
        ReportError("[ImuManager] Failed to register package for measurements log.");
    }

    std::unique_ptr<PackageInfo> package_oldest_states_ptr = std::make_unique<PackageInfo>();
    package_oldest_states_ptr->id = kImuManagerSuboldStatesLogIndex;
    package_oldest_states_ptr->name = "oldest states";
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "time_stamp_s"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "p_wi_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_w"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_pitch"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_roll"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "q_wi_yaw"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_wi_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_a_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "bias_g_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "acc_wi_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "acc_wi_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "acc_wi_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_wi_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_wi_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_wi_z"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_ii_x"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_ii_y"});
    package_oldest_states_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "v_ii_z"});
    if (!logger().RegisterPackage(package_oldest_states_ptr)) {
        ReportError("[ImuManager] Failed to register package for oldest state log.");
    }

    std::unique_ptr<PackageInfo> package_monitor_ptr = std::make_unique<PackageInfo>();
    package_monitor_ptr->id = kImuManagerMonitorLogIndex;
    package_monitor_ptr->name = "manager monitor";
    package_monitor_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "time_stamp_s"});
    package_monitor_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint32, .name = "measure_buffer_size"});
    package_monitor_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint8, .name = "is_attitude_valid"});
    package_monitor_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kUint8, .name = "is_pos_vel_valid"});
    package_monitor_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "current_updated_timestamp_s"});
    if (!logger().RegisterPackage(package_monitor_ptr)) {
        ReportError("[ImuManager] Failed to register package for imu manager monitor log.");
    }

    std::unique_ptr<PackageInfo> package_residual_ptr = std::make_unique<PackageInfo>();
    package_residual_ptr->id = kImuManagerObserveResidualLogIndex;
    package_residual_ptr->name = "observe residual";
    package_residual_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "time_stamp_s"});
    package_residual_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gravity_b_x"});
    package_residual_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gravity_b_y"});
    package_residual_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gravity_b_z"});
    package_residual_ptr->items.emplace_back(PackageItemInfo{.type = ItemType::kFloat, .name = "gyro_bias_w_z"});
    if (!logger().RegisterPackage(package_residual_ptr)) {
        ReportError("[ImuManager] Failed to register package for imu manager observe residual log.");
    }

    logger().PrepareForRecording();
    options_.kEnableRecordLog = true;

    return true;
}

bool ImuManager::RecordLog() {
    RETURN_TRUE_IF(!options_.kEnableRecordLog);
    RETURN_FALSE_IF(measures().empty());
    RETURN_FALSE_IF(states().oldest_states.time_stamp_s() <= logger().current_recorded_time_stamp_s());
    logger().current_recorded_time_stamp_s() = states().oldest_states.time_stamp_s();

    // Record measurements.
    ImuManagerLogMeasures measure_log;
    const auto &oldest_measure = measures().front();
    measure_log.time_stamp_s = oldest_measure->time_stamp_s;
    measure_log.accel_x_mps2 = oldest_measure->accel.x();
    measure_log.accel_y_mps2 = oldest_measure->accel.y();
    measure_log.accel_z_mps2 = oldest_measure->accel.z();
    measure_log.gyro_x_rps = oldest_measure->gyro.x();
    measure_log.gyro_y_rps = oldest_measure->gyro.y();
    measure_log.gyro_z_rps = oldest_measure->gyro.z();
    logger().RecordPackage(kImuManagerMeasuresLogIndex, reinterpret_cast<const char *>(&measure_log), logger().current_recorded_time_stamp_s());

    // Record subold states log.
    ImuManagerLogStates states_log;
    states_log.time_stamp_s = logger().current_recorded_time_stamp_s();
    states_log.p_wi_x = states().oldest_states.p_wi().x();
    states_log.p_wi_y = states().oldest_states.p_wi().y();
    states_log.p_wi_z = states().oldest_states.p_wi().z();
    const Vec3 oldest_euler = Utility::QuaternionToEuler(states().oldest_states.q_wi());
    states_log.q_wi_pitch = oldest_euler.x();
    states_log.q_wi_roll = oldest_euler.y();
    states_log.q_wi_yaw = oldest_euler.z();
    states_log.q_wi_w = states().oldest_states.q_wi().w();
    states_log.q_wi_x = states().oldest_states.q_wi().x();
    states_log.q_wi_y = states().oldest_states.q_wi().y();
    states_log.q_wi_z = states().oldest_states.q_wi().z();
    states_log.v_wi_x = states().oldest_states.v_wi().x();
    states_log.v_wi_y = states().oldest_states.v_wi().y();
    states_log.v_wi_z = states().oldest_states.v_wi().z();
    states_log.bias_a_x = states().oldest_states.ba().x();
    states_log.bias_a_y = states().oldest_states.ba().y();
    states_log.bias_a_z = states().oldest_states.ba().z();
    states_log.bias_g_x = states().oldest_states.bg().x();
    states_log.bias_g_y = states().oldest_states.bg().y();
    states_log.bias_g_z = states().oldest_states.bg().z();
    states_log.acc_wi_x = states().oldest_states.accel().x();
    states_log.acc_wi_y = states().oldest_states.accel().y();
    states_log.acc_wi_z = states().oldest_states.accel().z();
    states_log.gyro_wi_x = states().oldest_states.gyro().x();
    states_log.gyro_wi_y = states().oldest_states.gyro().y();
    states_log.gyro_wi_z = states().oldest_states.gyro().z();
    const Vec3 v_ii = states().oldest_states.q_wi().inverse() * states().oldest_states.v_wi();
    states_log.v_ii_x = v_ii.x();
    states_log.v_ii_y = v_ii.y();
    states_log.v_ii_z = v_ii.z();
    logger().RecordPackage(kImuManagerSuboldStatesLogIndex, reinterpret_cast<const char *>(&states_log), logger().current_recorded_time_stamp_s());

    // Record monitor.
    ImuManagerLogMonitor monitor_log;
    monitor_log.time_stamp_s = logger().current_recorded_time_stamp_s();
    monitor_log.measure_buffer_size = measures().size();
    monitor_log.is_attitude_valid = states().is_attitude_valid;
    monitor_log.is_pos_vel_valid = states().is_pos_vel_valid;
    monitor_log.current_updated_timestamp_s = states().current_updated_time_stamp_s;
    logger().RecordPackage(kImuManagerMonitorLogIndex, reinterpret_cast<const char *>(&monitor_log), logger().current_recorded_time_stamp_s());

    // Record residual.
    ImuManagerLogObserveResidual residual_log;
    residual_log.time_stamp_s = logger().current_recorded_time_stamp_s();
    float *ptr = &residual_log.time_stamp_s + 1;
    for (uint32_t i = 0; i < residual().rows(); ++i) {
        *ptr = residual()(i);
        ++ptr;
    }
    logger().RecordPackage(kImuManagerObserveResidualLogIndex, reinterpret_cast<const char *>(&residual_log), logger().current_recorded_time_stamp_s());
    residual().setZero();

    return true;
}

}
