#include "backend.h"
#include "log_report.h"
#include "tick_tock.h"

namespace VIO {

bool Backend::RunOnce() {
    ReportInfo(MAGENTA "[Backend] Backend is triggerred to run once." RESET_COLOR);

    // Record log.
    RecordBackendLogStates();
    return true;
}

void Backend::Reset() {

}

void Backend::ResetToReintialize() {

}

}
