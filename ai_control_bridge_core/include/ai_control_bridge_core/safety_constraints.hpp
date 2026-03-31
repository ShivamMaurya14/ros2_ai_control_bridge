#pragma once

#include <vector>

namespace ai_control_bridge_core {

/**
 * @brief Abstract interface for safety constraint enforcement.
 * 
 * Enforces physical constraints on robot motion:
 * - Joint limits (position min/max)
 * - Velocity limits
 * - Acceleration limits
 * - Jerk limits
 * 
 * Implementations should use saturation or scaling to enforce limits
 * while maintaining trajectory smoothness.
 */
class SafetyConstraints {
public:
    virtual ~SafetyConstraints() = default;

    /**
     * @brief Check if position satisfies joint limits.
     * 
     * @param position Joint positions to check
     * @return true if all positions are within limits
     */
    virtual bool check_position_limits(const std::vector<double>& position) const = 0;

    /**
     * @brief Check if velocity satisfies velocity limits.
     * 
     * @param velocity Joint velocities to check
     * @return true if all velocities are within limits
     */
    virtual bool check_velocity_limits(const std::vector<double>& velocity) const = 0;

    /**
     * @brief Check if acceleration satisfies acceleration limits.
     * 
     * @param acceleration Joint accelerations to check
     * @return true if all accelerations are within limits
     */
    virtual bool check_acceleration_limits(const std::vector<double>& acceleration) const = 0;

    /**
     * @brief Check if jerk satisfies jerk rate limits.
     * 
     * @param jerk Joint jerk values to check
     * @return true if all jerks are within limits
     */
    virtual bool check_jerk_limits(const std::vector<double>& jerk) const = 0;

    /**
     * @brief Enforce position limits (clip to valid range).
     * 
     * @param position[in/out] Positions to constrain
     */
    virtual void enforce_position_limits(std::vector<double>& position) const = 0;

    /**
     * @brief Enforce velocity limits (scale if necessary).
     * 
     * Typically uses saturation: if any velocity exceeds limit,
     * scale all velocities proportionally to maintain direction.
     * 
     * @param velocity[in/out] Velocities to constrain
     */
    virtual void enforce_velocity_limits(std::vector<double>& velocity) const = 0;

    /**
     * @brief Enforce acceleration limits (scale if necessary).
     * 
     * @param acceleration[in/out] Accelerations to constrain
     */
    virtual void enforce_acceleration_limits(std::vector<double>& acceleration) const = 0;

    /**
     * @brief Get number of joints this safety filter manages.
     * @return Number of joints
     */
    virtual size_t get_num_joints() const = 0;
};

}  // namespace ai_control_bridge_core
