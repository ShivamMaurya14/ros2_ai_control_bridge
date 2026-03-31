/** @file trajectory.hpp
 *  @brief Minimum jerk trajectory generation for smooth robot motion.
 *  
 *  @section overview Overview
 *  
 *  This header defines MinimumJerkTrajectory, a mathematical framework for
 *  generating smooth, jerk-minimized trajectories for robot joints.
 *  
 *  **Applicable to:** Any robot with any DOF (Cartesian, joint space, etc.)
 *  
 *  @section why_minimum_jerk Why Minimize Jerk?
 *  
 *  Jerk is the rate of change of acceleration (d³x/dt³). Minimizing jerk
 *  produces smooth, comfortable motion:
 *  
 *  - ✓ Reduces vibration and ringing
 *  - ✓ Smoother motion for collaborative robots (human friendly)
 *  - ✓ Less stress on gearboxes and bearings
 *  - ✓ More efficient energy usage
 *  - ✓ Better trajectory tracking for sensitive tasks
 *  
 *  @section mathematics Mathematics
 *  
 *  A quintic (5th-order) polynomial satisfies all constraints:
 *  
 *  @f[
 *    x(t) = a_0 + a_1 t + a_2 t^2 + a_3 t^3 + a_4 t^4 + a_5 t^5
 *  @f]
 *  
 *  Subject to 6 constraints:
 *  - Position: $x(0) = x_0$, $x(T) = x_f$
 *  - Velocity: $\dot{x}(0) = 0$, $\dot{x}(T) = 0$
 *  - Acceleration: $\ddot{x}(0) = 0$, $\ddot{x}(T) = 0$
 *  
 *  Solving this system of equations yields:
 *  @f[
 *  \begin{align}
 *  a_0 &= x_0 \\
 *  a_1 &= 0 \\
 *  a_2 &= 0 \\
 *  a_3 &= \frac{10\Delta x}{T^3} \\
 *  a_4 &= -\frac{15\Delta x}{T^4} \\
 *  a_5 &= \frac{6\Delta x}{T^5}
 *  \end{align}
 *  @f]
 *  
 *  where $\Delta x = x_f - x_0$ is the distance to travel.
 *  
 *  @section derivatives Analytical Derivatives
 *  
 *  Velocity:
 *  @f[
 *    v(t) = a_1 + 2a_2 t + 3a_3 t^2 + 4a_4 t^3 + 5a_5 t^4
 *  @f]
 *  
 *  Acceleration:
 *  @f[
 *    a(t) = 2a_2 + 6a_3 t + 12a_4 t^2 + 20a_5 t^3
 *  @f]
 *  
 *  Jerk:
 *  @f[
 *    j(t) = 6a_3 + 24a_4 t + 60a_5 t^2
 *  @f]
 *  
 *  Note: All derivatives are computed **analytically** (not numerically),
 *  ensuring accuracy and numerical stability.
 *  
 *  @section properties Trajectory Properties
 *  
 *  | Property | Value | Notes |
 *  |----------|-------|-------|
 *  | Continuity | C³ | Position, velocity, acceleration all continuous |
 *  | Jerk | Bounded | Maximum at endpoints (not in middle) |
 *  | Computation | O(1) | Constant time regardless of DOF |
 *  | Memory | O(6) | Only 6 coefficients per trajectory |
 *  | Realtime Safe | ✓ | No allocation, no locks, deterministic |
 *  
 *  @section generality Generalized for Any Robot
 *  
 *  This implementation works for:
 *  - **Any number of DOF**: Trajectories are computed per-joint independently
 *  - **Any coordinate system**: Joint space, Cartesian, etc.
 *  - **Any robot type**: Serial arm, parallel manipulator, humanoid, etc.
 *  - **Any control frequency**: Evaluated at any timestep
 *  
 *  Example: 7-DOF robot
 *  @code
 *  // Create 7 separate trajectories (one per joint)
 *  std::vector<MinimumJerkTrajectory::Coefficients> trajectories(7);
 *  
 *  for (int i = 0; i < 7; ++i) {
 *      // Each joint has independent trajectory
 *      trajectories[i] = MinimumJerkTrajectory::compute(
 *          current_position[i],
 *          desired_position[i],
 *          duration_seconds);
 *  }
 *  
 *  // In control loop, evaluate all 7 trajectories
 *  for (int i = 0; i < 7; ++i) {
 *      double pos = MinimumJerkTrajectory::eval_position(
 *          trajectories[i], current_time);
 *      command[i] = pos;
 *  }
 *  @endcode
 *  
 *  @section usage Usage Example
 *  
 *  @code
 *  // Define start and end positions
 *  double start = 0.5;   // rad
 *  double goal = 2.0;    // rad
 *  double duration = 3.0;  // seconds
 *  
 *  // Compute trajectory coefficients once
 *  auto coeff = MinimumJerkTrajectory::compute(start, goal, duration);
 *  
 *  // Evaluate at different times during trajectory execution
 *  double time_elapsed = 0.1;  // seconds
 *  double position = MinimumJerkTrajectory::eval_position(coeff, time_elapsed);
 *  double velocity = MinimumJerkTrajectory::eval_velocity(coeff, time_elapsed);
 *  double acceleration = MinimumJerkTrajectory::eval_acceleration(
 *      coeff, time_elapsed);
 *  @endcode
 *  
 *  @section comparison Comparison to Other Trajectories
 *  
 *  | Method | Continuity | Jerk | Smoothness |
 *  |--------|-----------|------|-----------|
 *  | Linear | C⁰ | Infinite | Poor (jerky) |
 *  | Cubic Spline | C¹ | Discontinuous | Fair |
 *  | Quintic (MJT) | C³ | Bounded | Excellent |
 *  | Seventh-Order | C⁴ | Lower | Excellent (overkill) |
 *  
 *  @section references References
 *  
 *  - Hogan, N. (1984). "Impedance Control of Industrial Robots."
 *  - Flash, T., & Hogan, N. (1985). "The coordination of arm movements."
 *  - Siciliano, B., et al. "Robotics: Modelling, Planning and Control"
 *
 *  @author GSoC Contributors
 *  @date 2026
 */

#pragma once

#include <array>
#include <cmath>
#include <algorithm>

namespace ai_control_bridge_controller {

/**
 * @brief Minimum Jerk Trajectory (MJT) generator.
 * 
 * Generates C² continuous trajectories with zero jerk at endpoints.
 * Uses quintic polynomial (5th order) for smooth acceleration profiles.
 * 
 * Equation: x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
 * 
 * Constraints:
 * - x(0) = x_start, x(T) = x_end
 * - v(0) = 0, v(T) = 0
 * - a(0) = 0, a(T) = 0
 * - j(0) = 0, j(T) = 0
 */
class MinimumJerkTrajectory {
public:
    struct Coefficients {
        std::array<double, 6> a;
        double duration;
        double start_pos;
        double end_pos;
    };

    static Coefficients compute(double start_pos, double end_pos, double duration) {
        Coefficients coeff;
        coeff.start_pos = start_pos;
        coeff.end_pos = end_pos;
        coeff.duration = duration;
        
        if (duration <= 0.0) {
            coeff.a.fill(0.0);
            coeff.a[0] = start_pos;
            return coeff;
        }

        double T = duration;
        double T2 = T * T;
        double T3 = T2 * T;
        double T4 = T2 * T2;
        double T5 = T4 * T;

        coeff.a[0] = start_pos;
        coeff.a[1] = 0.0;
        coeff.a[2] = 0.0;
        coeff.a[3] = 10.0 * (end_pos - start_pos) / T3;
        coeff.a[4] = -15.0 * (end_pos - start_pos) / T4;
        coeff.a[5] = 6.0 * (end_pos - start_pos) / T5;

        return coeff;
    }

    static double eval_position(const Coefficients& coeff, double t) {
        if (t <= 0.0) return coeff.start_pos;
        if (t >= coeff.duration) return coeff.end_pos;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t2 * t2;
        double t5 = t4 * t;

        return coeff.a[0] + coeff.a[1] * t + coeff.a[2] * t2 + 
               coeff.a[3] * t3 + coeff.a[4] * t4 + coeff.a[5] * t5;
    }

    static double eval_velocity(const Coefficients& coeff, double t) {
        if (t <= 0.0 || t >= coeff.duration) return 0.0;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t2 * t2;

        return coeff.a[1] + 2.0 * coeff.a[2] * t + 3.0 * coeff.a[3] * t2 +
               4.0 * coeff.a[4] * t3 + 5.0 * coeff.a[5] * t4;
    }

    static double eval_acceleration(const Coefficients& coeff, double t) {
        if (t <= 0.0 || t >= coeff.duration) return 0.0;

        double t2 = t * t;
        double t3 = t2 * t;

        return 2.0 * coeff.a[2] + 6.0 * coeff.a[3] * t + 
               12.0 * coeff.a[4] * t2 + 20.0 * coeff.a[5] * t3;
    }

    static double eval_jerk(const Coefficients& coeff, double t) {
        if (t <= 0.0 || t >= coeff.duration) return 0.0;

        double t2 = t * t;

        return 6.0 * coeff.a[3] + 24.0 * coeff.a[4] * t + 60.0 * coeff.a[5] * t2;
    }

    static double get_peak_acceleration(const Coefficients& coeff) {
        double peak = 0.0;
        peak = std::max(peak, std::abs(eval_acceleration(coeff, 0.0)));
        peak = std::max(peak, std::abs(eval_acceleration(coeff, coeff.duration)));
        
        for (int i = 1; i < 10; ++i) {
            double t = coeff.duration * i / 10.0;
            peak = std::max(peak, std::abs(eval_acceleration(coeff, t)));
        }
        return peak;
    }

    static double get_peak_jerk(const Coefficients& coeff) {
        double peak = 0.0;
        for (int i = 0; i <= 10; ++i) {
            double t = coeff.duration * i / 10.0;
            peak = std::max(peak, std::abs(eval_jerk(coeff, t)));
        }
        return peak;
    }
};

}  // namespace ai_control_bridge_controller
