#ifndef _LOCATOR_SENSOR_MANAGER_H_
#define _LOCATOR_SENSOR_MANAGER_H_

#include "datatype_basic.h"
#include "binary_data_log.h"
#include "deque"
#include "memory"

namespace LOCATOR {

using namespace SLAM_UTILITY;
using namespace SLAM_DATA_LOG;

/* Class SensorManager Declaration. */
template <typename StateType, typename MeasureType>
class SensorManager {

public:
    SensorManager() = default;
    virtual ~SensorManager() = default;

    // Record log.
    virtual bool ConfigurationLog(const std::string &log_file_name) { return true; }
    virtual bool RecordLog() { return true; }

    // Process measurement.
    virtual bool AddMeasurement(const MeasureType &new_measure);
    virtual float GetOldestMeasurementTimestamp() = 0;
    virtual float GetNewestMeasurementTimestamp() = 0;
    std::unique_ptr<MeasureType> GetOldestMeasurement();

    // Process states.
    virtual int32_t StatesSize() = 0;
    int32_t StatesColIndex() { return col_index_; }
    void SetStatesColIndex(const int32_t col_index) { col_index_ = col_index; }
    virtual void UpdateStates(const Vec &dx) = 0;
    virtual void ResetStates() = 0;

    // Reference for Member Variables.
    std::deque<std::unique_ptr<MeasureType>> &measures() { return measures_; }
    StateType &states() { return states_; }
    BinaryDataLog &logger() { return logger_; }
    Vec &residual() { return residual_; }

    // Const Reference for Member Variables.
    const std::deque<std::unique_ptr<MeasureType>> &measures() const { return measures_; }
    const StateType &states() const { return states_; }
    const BinaryDataLog &logger() const { return logger_; }
    const Vec &residual() const { return residual_; }

private:
    // Sequence of imu raw measurements.
    std::deque<std::unique_ptr<MeasureType>> measures_;
    // Sensor states. Covariance is not managed here.
    StateType states_;
    // Covariance index.
    int32_t col_index_;
    // Record log.
    BinaryDataLog logger_;
    // Residual of observation;
    Vec residual_;
};

/* Class SensorManager Definition. */
template <typename StateType, typename MeasureType>
bool SensorManager<StateType, MeasureType>::AddMeasurement(const MeasureType &new_measure) {
    if (!measures_.empty() && GetNewestMeasurementTimestamp() > new_measure.time_stamp_s) {
        return false;
    }

    measures_.emplace_back(std::make_unique<MeasureType>(new_measure));
    return false;
}

template <typename StateType, typename MeasureType>
std::unique_ptr<MeasureType> SensorManager<StateType, MeasureType>::GetOldestMeasurement() {
    if (measures_.empty()) {
        return nullptr;
    }

    std::unique_ptr<MeasureType> oldest_measure = std::move(measures_.front());
    measures_.pop_front();
    return std::move(oldest_measure);
}

}

#endif // end of _LOCATOR_SENSOR_MANAGER_H_
