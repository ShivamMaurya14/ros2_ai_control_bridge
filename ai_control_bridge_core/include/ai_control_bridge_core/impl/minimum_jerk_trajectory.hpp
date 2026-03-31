#pragma once

#include "../trajectory_generator.hpp"
#include <array>
#include <cmath>
#include <algorithm>

namespace ai_control_bridge_core::impl {

/**
 * @brief Minimum Jerk Trajectory implementation.
 * 
 * Generates C² continuous trajectories using quintic polynomials.
 * Zero velocity, acceleration, and jerk at trajectory boundaries.
 * 
 * Per-joint implementation - one instance per joint.
 */
class MinimumJerkTrajectory {
public:
    struct Coefficients {
        std::array<double, 6> a;  // Quintic coefficients [a0, a1, ..., a5]
        double duration;
        double start_pos;
        double end_pos;
    };

    /**
     * @brief Compute trajectory coefficients from start to end position.
     * 
     * @param start_pos Initial position
     * @param end_pos Final position
     * @param duration Total trajectory duration (seconds)
     * @return Coefficients structure for evaluation
     */
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

        // Quintic spline coefficients for MJT with zero boundary conditions
        coeff.a[0] = start_pos;
        coeff.a[1] = 0.0;
        coeff.a[2] = 0.0;
        coeff.a[3] = 10.0 * (end_pos - start_pos) / T3;
        coeff.a[4] = -15.0 * (end_pos - start_pos) / T4;
        coeff.a[5] = 6.0 * (end_pos - start_pos) / T5;

        return coeff;
    }

    /// Evaluate position at time t
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

    /// Evaluate velocity (first derivative) at time t
    static double eval_velocity(const Coefficients& coeff, double t) {
        if (t <= 0.0 || t >= coeff.duration) return 0.0;

        double t2 = t * t;
        double t3 = t2 * t;
        double t4 = t2 * t2;

        return coeff.a[1] + 2.0 * coeff.a[2] * t + 3.0 * coeff.a[3] * t2 +
               4.0 * coeff.a[4] * t3 + 5.0 * coeff.a[5] * t4;
    }

    /// Evaluate acceleration (second derivative) at time t
    static double eval_acceleration(const Coefficients& coeff, double t) {
        if (t <= 0.0 || t >= coeff.duration) return 0.0;

        double t2 = t * t;
        double t3 = t2 * t;

        return 2.0 * coeff.a[2] + 6.0 * coeff.a[3] * t + 
               12.0 * coeff.a[4] * t2 + 20.0 * coeff.a[5] * t3;
    }

    /// Evaluate jerk (third derivative) at time t
    static double eval_jerk(const Coefficients& coeff, double t) {
        if (t <= 0.0 || t >= coeff.duration) return 0.0;

        double t2 = t * t;

        return 6.0 * coeff.a[3] + 24.0 * coeff.a[4] * t + 60.0 * coeff.a[5] * t2;
    }

    /// Get peak acceleration magnitude during trajectory
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

    /// Get peak jerk magnitude during trajectory
    static double get_peak_jerk(const Coefficients& coeff) {
        double peak = 0.0;
        for (int i = 0; i <= 10; ++i) {
            double t = coeff.duration * i / 10.0;
            peak = std::max(peak, std::abs(eval_jerk(coeff, t)));
        }
        return peak;
    }
};

}  // namespace ai_control_bridge_core::impl
