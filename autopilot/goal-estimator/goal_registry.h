#pragma once
#include <vector>
#include <mutex>
#include <map>
#include "tracked_goal.h"

class GoalRegistry {
public:
    explicit GoalRegistry(double timeout_sec = 2.0);

    void setTrackedGoal(int key, const TrackedGoal& goal);
    bool getTrackedGoal(int key, TrackedGoal& out_goal);
    bool getLastKnownGoal(int key, TrackedGoal& out_goal);
    
    std::vector<TrackedGoal> getActiveEstimates(double current_time);
    std::vector<TrackedGoal> getAllKnownGoals();
    
    void clearGoal(int key);
    void reset();

    double getTimeout() const { return timeout_sec_; }
    void setTimeout(double timeout_sec) { timeout_sec_ = timeout_sec; }

    std::map<int, TrackedGoal> getTrackedGoalsSnapshot();
    std::map<int, TrackedGoal> getLastKnownGoalsSnapshot();

    std::mutex& getMutex() { return mutex_; }
    std::map<int, TrackedGoal>& getTrackedGoalsRef() { return tracked_goals_; }
    std::map<int, TrackedGoal>& getLastKnownGoalsRef() { return last_known_goals_; }

private:
    std::map<int, TrackedGoal> tracked_goals_;
    std::map<int, TrackedGoal> last_known_goals_;
    std::mutex mutex_;
    double timeout_sec_;
};
