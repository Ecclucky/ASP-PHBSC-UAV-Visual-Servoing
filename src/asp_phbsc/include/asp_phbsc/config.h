#pragma once

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

namespace asp_phbsc {

// All control indices are zero based; intervals are [start_step, end_step).
struct AttackWindow { long long start_step; long long end_step; };

struct ControllerConfig {
    double control_period_s = 0.01;
    double measurement_period_s = 0.05;
    std::array<double, 2> feedback_gain{{4.0, 2.0}};
    // Documentary pair from the UAV section, not a complete augmented gain.
    std::array<double, 2> paper_observer_gain{{-0.6, -6.6}};
    // Explicit per-axis exosystem. No flight-model values are inferred.
    std::vector<std::vector<double> > disturbance_dynamics; // A_w, l_w x l_w
    std::vector<double> disturbance_output;                // C_w, 1 x l_w
    std::vector<double> augmented_observer_gain;            // L, (2+l_w) x 1
    double delta_0 = 0.02;
    double delta_1 = 0.02;
    int measurement_delay_steps = 10;
    int max_trigger_interval_steps = 20;
    int prediction_steps = 100; // N_p: a packet contains N_p + 1 inputs.
    double target_radius_m = 0.0;
    double target_period_s = 0.0;
    int max_consecutive_dos_steps = 0;
    bool wind_disturbance = false;
    bool dos_enabled = false;
    std::vector<AttackWindow> attack_windows_steps;
};

inline void requireConfig(bool condition, const std::string& message) {
    if (!condition) throw std::invalid_argument("ASP-PHBSC configuration: " + message);
}

inline void validateBaseConfig(const ControllerConfig& c) {
    requireConfig(std::isfinite(c.control_period_s) && c.control_period_s > 0.0,
                  "control_period_s must be finite and positive");
    requireConfig(std::isfinite(c.measurement_period_s) && c.measurement_period_s > 0.0,
                  "measurement_period_s must be finite and positive");
    const double ratio = c.measurement_period_s / c.control_period_s;
    requireConfig(std::isfinite(ratio) && ratio > 1.0 &&
                  ratio <= static_cast<double>(std::numeric_limits<int>::max()) &&
                  std::abs(ratio - std::round(ratio)) < 1e-9,
                  "measurement_period_s / control_period_s must be an integer greater than one");
    for (std::size_t i = 0; i < 2; ++i) {
        requireConfig(std::isfinite(c.feedback_gain[i]) && c.feedback_gain[i] > 0.0,
                      "feedback_gain must contain two finite positive values for the double integrator");
        requireConfig(std::isfinite(c.paper_observer_gain[i]),
                      "paper_observer_gain must contain two finite values");
    }
    requireConfig(std::isfinite(c.delta_0) && c.delta_0 >= 0.0 &&
                  std::isfinite(c.delta_1) && c.delta_1 >= 0.0,
                  "delta_0 and delta_1 must be finite and nonnegative");
    requireConfig(c.measurement_delay_steps > 0,
                  "measurement_delay_steps must be positive, as assumed in the manuscript");
    requireConfig(c.max_trigger_interval_steps > 0 && c.prediction_steps > 0 &&
                  c.prediction_steps < std::numeric_limits<int>::max() &&
                  c.max_consecutive_dos_steps > 0,
                  "prediction_steps, max_trigger_interval_steps, and max_consecutive_dos_steps must be positive");
    requireConfig(static_cast<long long>(c.prediction_steps) >=
                  static_cast<long long>(c.max_trigger_interval_steps) + c.max_consecutive_dos_steps,
                  "prediction_steps must be >= max_trigger_interval_steps + max_consecutive_dos_steps");
    requireConfig(std::isfinite(c.target_radius_m) && c.target_radius_m > 0.0 &&
                  std::isfinite(c.target_period_s) && c.target_period_s > 0.0,
                  "target_radius_m and target_period_s must be finite and positive case metadata");
    requireConfig(!c.dos_enabled || !c.attack_windows_steps.empty(),
                  "enabled DoS requires an explicit, nonempty attack_windows_steps list");
    long long previous_end = -1;
    long long consecutive_start = -1;
    for (const AttackWindow& window : c.attack_windows_steps) {
        requireConfig(window.start_step > 0 && window.end_step > window.start_step,
                      "attack windows must satisfy 0 < start < end; the initial packet at step 0 must succeed");
        requireConfig(previous_end < 0 || window.start_step >= previous_end,
                      "attack windows must be sorted and must not overlap");
        if (window.start_step != previous_end) consecutive_start = window.start_step;
        requireConfig(window.end_step - consecutive_start <= c.max_consecutive_dos_steps,
                      "a continuous attack run (including adjacent windows) exceeds max_consecutive_dos_steps");
        previous_end = window.end_step;
    }
}

inline void validateConfig(const ControllerConfig& c) {
    validateBaseConfig(c);
    const std::size_t nw = c.disturbance_dynamics.size();
    requireConfig(nw > 0,
        "missing UAV augmented disturbance model: supply disturbance_dynamics (A_w), "
        "disturbance_output (C_w), and the complete augmented_observer_gain (L). "
        "The manuscript's two-entry paper_observer_gain is insufficient for [position, velocity, w].");
    requireConfig(nw <= static_cast<std::size_t>(std::numeric_limits<int>::max() - 4),
                  "disturbance model dimension is too large");
    for (const auto& row : c.disturbance_dynamics) {
        requireConfig(row.size() == nw, "disturbance_dynamics must be a square matrix");
        for (double value : row) requireConfig(std::isfinite(value), "disturbance_dynamics must be finite");
    }
    requireConfig(c.disturbance_output.size() == nw,
                  "disturbance_output must have one entry per disturbance state");
    for (double value : c.disturbance_output)
        requireConfig(std::isfinite(value), "disturbance_output must be finite");
    requireConfig(c.augmented_observer_gain.size() == nw + 2,
                  "augmented_observer_gain must contain 2 + disturbance-dimension entries");
    for (double value : c.augmented_observer_gain)
        requireConfig(std::isfinite(value), "augmented_observer_gain must be finite");
}

inline int measurementSteps(const ControllerConfig& c) {
    return static_cast<int>(std::round(c.measurement_period_s / c.control_period_s));
}

inline bool attackActive(const ControllerConfig& c, long long zero_based_step) {
    if (!c.dos_enabled) return false;
    for (const AttackWindow& window : c.attack_windows_steps) {
        if (zero_based_step < window.start_step) return false;
        if (zero_based_step < window.end_step) return true;
    }
    return false;
}

} // namespace asp_phbsc
