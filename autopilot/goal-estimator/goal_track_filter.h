#pragma once
#include <map>
#include <cmath>
#include "tracked_goal.h"

class GoalTrackFilter {
public:
    explicit GoalTrackFilter(double alpha = 0.5);

    // Resets filter state (e.g., outlier glitch counters)
    void reset();
    void clearGoal(int key);

    // Updates an existing goal track with hypothesis data.
    // Handles spatial outlier rejection (>=3 consecutive frames to relocate),
    // confidence damping, exponential smoothing, and planar yaw symmetry projection.
    // Returns true if track was updated/accepted, false if rejected as single-frame glitch.
    bool updateGoalTrack(TrackedGoal& goal, const PoseHypothesis& hyp, double current_time, bool timed_out);

    // Initializes a new TrackedGoal from a hypothesis
    static TrackedGoal initializeTrackedGoal(int shape_type, const PoseHypothesis& hyp, double current_time);

    // Helper: extracts yaw, pitch, roll angles (radians) from quaternion
    static double extractYaw(const Quatf& q);
    static double extractPitch(const Quatf& q);
    static double extractRoll(const Quatf& q);

    double getAlpha() const { return alpha_; }
    void setAlpha(double alpha) { alpha_ = alpha; }

    int getOutlierJumpCount(int key) const;

private:
    double alpha_;
    std::map<int, int> outlier_jump_counts_;
};
