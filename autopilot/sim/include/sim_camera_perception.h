#pragma once

#include "sim_vision_harness.h"

namespace sim {

// SimCameraPerception is an alias for SimVisionHarness for backwards compatibility.
// SimVisionHarness hooks directly into the actual vision-processing system (GateDetector / vision_node).
using SimCameraPerception = SimVisionHarness;

} // namespace sim
