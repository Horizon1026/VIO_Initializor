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
        for (const auto &pair : frame.features()) {
            const auto feature_id = pair.first;
            const auto feature_ptr = pair.second;
            if (feature_id != feature_ptr->id()) {
                ReportError("[DataManager] Visual local map self check frames, feature id error [" <<
                    feature_id << "] != [" << feature_ptr->id() << "].");
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
    }

    ReportInfo("[DataManager] Visual local map self check ok.");
    return true;
}

}
