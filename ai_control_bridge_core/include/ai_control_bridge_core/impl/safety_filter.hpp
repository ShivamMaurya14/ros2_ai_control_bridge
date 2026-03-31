#pragma once

#include "../safety_constraints.hpp"
#include <vector>
#include <cmath>
#include <algorithm>

namespace ai_control_bridge_core::impl {

/**
 * @brief Safety filter implementation - generalized for ANY number of joints.
 * 
 * Enforces constraints on:
 * - Position (joint limits)
 * - Velocity
 * - Acceleration
 * - Jerk
 * 
 * Implements the SafetyConstraints interface with generalized support
 * for robots with any number of DOF.
 */
class SafetyFilter : public SafetyConstraints {
public:
    struct Limits {
        std::vector<double> position_min;
        std::vector<double> position_max;
        std::vector<double> velocity_max;
        std::vector<double> acceleration_max;
        std::vector<double> jerk_max;
    };

    /// Constructor with number of joints (default 6)
    explicit SafetyFilter(size_t num_joints = 6) : num_joints_(num_joints) {
        // Initialize with default limits
        limits_.position_min.assign(num_joints, -M_PI);
        limits_.position_max.assign(num_joints, M_PI);
        limits_.velocity_max.assign(num_joints, 2.0);      // rad/s
        limits_.acceleration_max.assign(num_joints, 5.0);  // rad/s^2
        limits_.jerk_max.assign(num_joints, 50.0);         // rad/s^3
    }

    virtual ~SafetyFilter() = default;

    /// Set all constraints at once
    void set_limits(const Limits& lim) {
        limits_ = lim;
        num_joints_ = lim.position_min.size();
    }

    // SafetyConstraints interface implementation
    virtual bool check_position_limits(const std::vector<double>& position) const override {
        if (position.size() != num_joints_) return false;
        for (size_t i = 0; i < num_joints_; ++i) {
            if (position[i] < limits_.position_min[i] || 
                position[i] > limits_.position_max[i]) {
                return false;
            }
        }
        return true;
    }

    virtual bool check_velocity_limits(const std::vector<double>& velocity) const override {
        if (velocity.size() != num_joints_) return false;
        for (size_t i = 0; i < num_joints_; ++i) {
            if (std::abs(velocity[i]) > limits_.velocity_max[i]) {
                return false;
            }
        }
        return true;
    }

    virtual bool check_acceleration_limits(const std::vector<double>& acceleration) const override {
        if (acceleration.size() != num_joints_) return false;
        for (size_t i = 0; i < num_joints_; ++i) {
            if (std::abs(acceleration[i]) > limits_.acceleration_max[i]) {
                return false;
            }
        }
        return true;
    }

    virtual bool check_jerk_limits(const std::vector<double>& jerk) const override {
        if (jerk.size() != num_joints_) return false;
        for (size_t i = 0; i < num_joints_; ++i) {
            if (std::abs(jerk[i]) > limits_.jerk_max[i]) {
                return false;
            }
        }
        return true;
    }

    virtual void enforce_position_limits(std::vector<double>& position) const override {
        if (position.size() != num_joints_) return;
        for (size_t i = 0; i < num_joints_; ++i) {
            position[i] = std::clamp(position[i], 
                                    limits_.position_min[i], 
                                    limits_.position_max[i]);
        }
    }

    virtual void enforce_velocity_limits(std::vector<double>& velocity) const override {
        if (velocity.size() != num_joints_) return;
        
        // Find max violation ratio
        double max_scale = 1.0;
        for (size_t i = 0; i < num_joints_; ++i) {
            double abs_vel = std::abs(velocity[i]);
            if (abs_vel > limits_.velocity_max[i]) {
                max_scale = std::min(max_scale, limits_.velocity_max[i] / abs_vel);
            }
        }
        
        // Scale all velocities proportionally
        for (size_t i = 0; i < num_joints_; ++i) {
            velocity[i] *= max_scale;
        }
    }

    virtual void enforce_acceleration_limits(std::vector<double>& acceleration) const override {
        if (acceleration.size() != num_joints_) return;
        
        double max_scale = 1.0;
        for (size_t i = 0; i < num_joints_; ++i) {
            double abs_acc = std::abs(acceleration[i]);
            if (abs_acc > limits_.acceleration_max[i]) {
                max_scale = std::min(max_scale, limits_.acceleration_max[i] / abs_acc);
            }
        }
        
        for (size_t i = 0; i < num_joints_; ++i) {
            acceleration[i] *= max_scale;
        }
    }

    virtual size_t get_num_joints() const override { return num_joints_; }

    // Getters for limits
    const Limits& get_limits() const { return limits_; }

private:
    size_t num_joints_;
    Limits limits_;
};

}  // namespace ai_control_bridge_core::impl
