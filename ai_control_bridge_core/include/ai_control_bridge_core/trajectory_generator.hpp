#pragma once

#include <vector>

namespace ai_control_bridge_core {

/**
 * @brief Abstract interface for trajectory generation and evaluation.
 * 
 * Provides an extensible interface for different trajectory generation methods:
 * - Minimum Jerk Trajectory (smooth, C² continuous)
 * - Cubic splines
 * - Linear interpolation
 * - Custom implementations
 * 
 * Implementations must be designed for realtime evaluation (update() call).
 */
class TrajectoryGenerator {
public:
    virtual ~TrajectoryGenerator() = default;

    /**
     * @brief Initialize trajectory from start to end position.
     * 
     * @param start_pos Starting position(s)
     * @param end_pos Ending position(s)
     * @param duration Time to reach end position (seconds)
     * @return true if trajectory was successfully generated, false otherwise
     */
    virtual bool compute(
        const std::vector<double>& start_pos,
        const std::vector<double>& end_pos,
        double duration) = 0;

    /**
     * @brief Evaluate trajectory state at given time.
     * 
     * Called at high frequency in the realtime control loop.
     * Must be deterministic and lock-free.
     * 
     * @param t Current time along trajectory (seconds, from 0 to duration)
     * @param position Output: position at time t
     * @param velocity Output: velocity at time t (first derivative)
     * @param acceleration Output: acceleration at time t (second derivative)
     * @return true if evaluation successful, false otherwise
     */
    virtual bool evaluate(
        double t,
        std::vector<double>& position,
        std::vector<double>& velocity,
        std::vector<double>& acceleration) = 0;

    /**
     * @brief Get trajectory duration.
     * @return Total trajectory duration in seconds
     */
    virtual double get_duration() const = 0;

    /**
     * @brief Check if trajectory is currently valid.
     * @return true if trajectory is ready for evaluation
     */
    virtual bool is_valid() const = 0;
};

}  // namespace ai_control_bridge_core
