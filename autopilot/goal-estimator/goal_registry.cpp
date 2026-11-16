#include "goal_registry.h"

GoalRegistry::GoalRegistry(double timeout_sec)
    : timeout_sec_(timeout_sec) {}

void GoalRegistry::setTrackedGoal(int key, const TrackedGoal& goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    tracked_goals_[key] = goal;
    last_known_goals_[key] = goal;
}

bool GoalRegistry::getTrackedGoal(int key, TrackedGoal& out_goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = tracked_goals_.find(key);
    if (it != tracked_goals_.end()) {
        out_goal = it->second;
        return true;
    }
    return false;
}

bool GoalRegistry::getLastKnownGoal(int key, TrackedGoal& out_goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    auto it = last_known_goals_.find(key);
    if (it != last_known_goals_.end()) {
        out_goal = it->second;
        return true;
    }
    return false;
}

std::vector<TrackedGoal> GoalRegistry::getActiveEstimates(double current_time) {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TrackedGoal> result;
    auto it = tracked_goals_.begin();
    while (it != tracked_goals_.end()) {
        if (current_time - it->second.last_update_time > timeout_sec_) {
            it = tracked_goals_.erase(it);
        } else {
            result.push_back(it->second);
            ++it;
        }
    }
    return result;
}

std::vector<TrackedGoal> GoalRegistry::getAllKnownGoals() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<TrackedGoal> result;
    result.reserve(last_known_goals_.size());
    for (const auto& kv : last_known_goals_) {
        result.push_back(kv.second);
    }
    return result;
}

void GoalRegistry::clearGoal(int key) {
    std::lock_guard<std::mutex> lock(mutex_);
    tracked_goals_.erase(key);
    last_known_goals_.erase(key);
}

void GoalRegistry::reset() {
    std::lock_guard<std::mutex> lock(mutex_);
    tracked_goals_.clear();
    last_known_goals_.clear();
}

std::map<int, TrackedGoal> GoalRegistry::getTrackedGoalsSnapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return tracked_goals_;
}

std::map<int, TrackedGoal> GoalRegistry::getLastKnownGoalsSnapshot() {
    std::lock_guard<std::mutex> lock(mutex_);
    return last_known_goals_;
}
