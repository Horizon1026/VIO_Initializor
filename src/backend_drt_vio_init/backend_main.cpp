#include "backend.h"
#include "slam_log_reporter.h"
#include "tick_tock.h"

namespace vio {

bool Backend::RunOnce() {
    ReportInfo(MAGENTA "[Backend] Backend is triggerred to run once." RESET_COLOR);
    TickTock timer;

    if (!status_.is_initialized) {
        // Try to initialize vio if not initialized.
        timer.TockTickInMillisecond();
        const bool res = TryToInitialize();
        if (res) {
            status_.is_initialized = true;
            ReportColorInfo("[Backend] Backend succeed to initialize within " << timer.TockTickInMillisecond() << " ms.");
        } else {
            ResetToReintialize();
            ReportColorWarn("[Backend] Backend failed to initialize. All states will be reset for reinitialization.");
        }
    } else {
        // Quit when initialized.
        signals_.should_quit = true;
    }

    return true;
}

void Backend::Reset() {
    // Clear data manager.
    data_manager_->visual_local_map()->Clear();
    data_manager_->imu_based_frames().clear();

    // Clear states flag.
    status_.is_initialized = false;
}

void Backend::ResetToReintialize() {
    // Clear data manager.
    data_manager_->visual_local_map()->Clear();
    data_manager_->ControlSizeOfImuBasedFrames();

    // Clear states flag.
    status_.is_initialized = false;
}

}  // namespace vio
