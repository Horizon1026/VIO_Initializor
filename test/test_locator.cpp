#include <fstream>
#include <iostream>

#include "datatype_basic.h"
#include "log_report.h"

#include "imu_measurement.h"
#include "ins.h"

using namespace INS;

void LoadSimDataAndPublish(const std::string &file_name, InsFusion &ins) {
    std::ifstream file(file_name.c_str());
    if (!file.is_open()) {
        ReportError("Failed to load data file " << file_name);
        return;
    }

    // Publish each line of data file.
    std::string one_line;
    uint32_t cnt = 10000;
    while (std::getline(file, one_line) && !one_line.empty()) {
        if (!cnt--) {
            break;
        }

        std::istringstream data(one_line);

        // Extract first item to check type.
        std::string type;
        data >> type;

        // Process different type of measurements.
        double time_stamp_s = 0.0f;
        if (type == "IMU") {
            ImuMeasurement imu;
            data >> time_stamp_s >> imu.gyro.x() >> imu.gyro.y() >> imu.gyro.z() >>
                imu.accel.x() >> imu.accel.y() >> imu.accel.z();
            imu.time_stamp_s = time_stamp_s - 1624426287.19101906;

            ins.imu_manager()->AddMeasurement(imu);
        }

        ins.RunOnce();
    }

    file.close();
}

int main(int argc, char **argv) {
    ReportInfo(YELLOW ">> Test INS." RESET_COLOR);
    LogFixPercision(3);

    InsFusion ins;

    // Config imu manager.
    ins.imu_manager()->imu_model().options().kAccelNoiseSigma = 1e-2f;
    ins.imu_manager()->imu_model().options().kGyroNoiseSigma = 1e-2f;
    ins.imu_manager()->imu_model().options().kAccelRandomWalkSigma = 1e-3f;
    ins.imu_manager()->imu_model().options().kGyroRandomWalkSigma = 1e-4f;
    ins.imu_manager()->ConfigurationLog("../../Slam_Workspace/output/imu_manager.binlog");
    ins.imu_manager()->options().kGravityInWorldFrame = Vec3(0, 0, 9.81);
    ins.imu_manager()->options().kMaxToleranceResidualFromAccelNormToGravityNorm = 0.05f;
    ins.imu_manager()->options().kMinGravityNormSigma = 0.05f;
    ins.imu_manager()->options().kGyroBiasInWorldFrameSigma = 1.0f;
    ins.imu_manager()->options().kMaxToleranceDelayInSecond = 0.3f;
    ins.imu_manager()->options().kMaxTolerancePureImuPropagatePositionInSecond = 3.0f;
    ins.imu_manager()->options().kEnableRecordLog = true;

    // Config inertial navigation system.
    ins.Initialize();
    ins.ConfigurationLog("../../Slam_Workspace/output/ins_fusion.binlog");

    LoadSimDataAndPublish("../examples/dataset.txt", ins);

    return 0;
}
