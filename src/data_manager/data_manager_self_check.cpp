#include "data_manager.h"

namespace VIO {

// Self check.
bool DataManager::SelfCheckVisualLocalMap() {
    if (!visual_local_map_->SelfCheck()) {
        ReportError("[DataManager] Visual local map self check error.");
        return false;
    }

    // Iterate each frame to check all features.
    for (const auto &frame : visual_local_map_->frames()) {
        const auto frame_id = frame.id();
        for (const auto &pair : frame.features()) {
            const auto feature_id = pair.first;
            const auto feature_ptr = pair.second;
            if (feature_id != feature_ptr->id()) {
                ReportError("[DataManager] Visual local map self check frames, feature id error [" <<
                    feature_id << "] != [" << feature_ptr->id() << "].");
                return false;
            }

            if (feature_ptr->observe(frame_id).front().frame_time_stamp_s != frame.time_stamp_s()) {
                ReportError("[DataManager] Visual local map self check frames, feature observe timestamp error [" <<
                    feature_ptr->observe(frame_id).front().frame_time_stamp_s << "] != [" << frame.time_stamp_s() << "].");
                return false;
            }
        }
    }

    // Iterate each feature to check all observations.
    for (const auto &pair : visual_local_map_->features()) {
        const auto feature_id = pair.first;
        const auto &feature = pair.second;
        if (feature_id != feature.id()) {
            ReportError("[DataManager] Visual local map self check features, feature id error [" <<
                feature_id << "] != [" << feature.id() << "].");
            return false;
        }


        for (auto frame_id = feature.first_frame_id(); frame_id <= feature.final_frame_id(); ++frame_id) {
            const auto time_stamp_from_feature = feature.observe(frame_id).front().frame_time_stamp_s;
            const auto time_stamp_from_frame = visual_local_map_->frame(frame_id)->time_stamp_s();
            if (time_stamp_from_feature != time_stamp_from_frame) {
                ReportError("[DataManager] Visual local map self check features, feature observe timestamp error [" <<
                    time_stamp_from_feature << "] != [" << time_stamp_from_frame << "].");
                return false;
            }
        }
    }

    ReportInfo("[DataManager] Visual local map self check ok.");
    return true;
}

}
