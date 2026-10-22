#pragma once
#include "local_planner.h"

// Explicit dense acceptance API. Link planning::foot_acceptance (BUILD_TESTING).
// Normal planners and the executor do not link this library.
namespace planning {
void validateSpatialFootTrajectory(const FootLocalPlanRequest&, const FootLocalPlanResult&);
FootLocalPlanResult validateFootTrajectory(const FootLocalPlanRequest&, FootLocalPlanResult);
} // namespace planning
