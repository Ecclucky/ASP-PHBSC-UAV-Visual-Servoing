#pragma once

#include <asp_phbsc/config.h>
#include <ros/ros.h>
#include <XmlRpcValue.h>

namespace asp_phbsc {
namespace parameter_detail {

inline XmlRpc::XmlRpcValue required(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value;
    requireConfig(nh.getParam(key, value), "missing private parameter: " + key);
    return value;
}

inline double number(XmlRpc::XmlRpcValue& value, const std::string& key) {
    if (value.getType() == XmlRpc::XmlRpcValue::TypeDouble) return static_cast<double>(value);
    if (value.getType() == XmlRpc::XmlRpcValue::TypeInt) return static_cast<int>(value);
    throw std::invalid_argument("ASP-PHBSC configuration: " + key + " must be a number");
}

inline double requiredNumber(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value = required(nh, key);
    return number(value, key);
}

inline int integer(XmlRpc::XmlRpcValue& value, const std::string& key) {
    requireConfig(value.getType() == XmlRpc::XmlRpcValue::TypeInt, key + " must be an integer");
    return static_cast<int>(value);
}

inline int requiredInteger(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value = required(nh, key);
    return integer(value, key);
}

inline bool requiredBoolean(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value = required(nh, key);
    requireConfig(value.getType() == XmlRpc::XmlRpcValue::TypeBoolean, key + " must be a boolean");
    return static_cast<bool>(value);
}

inline std::array<double, 2> requiredPair(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value = required(nh, key);
    requireConfig(value.getType() == XmlRpc::XmlRpcValue::TypeArray && value.size() == 2,
                  key + " must contain exactly two numbers");
    return {{number(value[0], key), number(value[1], key)}};
}

inline std::vector<double> requiredVector(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value = required(nh, key);
    requireConfig(value.getType() == XmlRpc::XmlRpcValue::TypeArray,
                  key + " must be an array (leave [] only while its values are unresolved)");
    std::vector<double> result;
    for (int i = 0; i < value.size(); ++i) result.push_back(number(value[i], key));
    return result;
}

inline std::vector<std::vector<double> > requiredMatrix(ros::NodeHandle& nh, const std::string& key) {
    XmlRpc::XmlRpcValue value = required(nh, key);
    requireConfig(value.getType() == XmlRpc::XmlRpcValue::TypeArray,
                  key + " must be an array of rows (leave [] only while its values are unresolved)");
    std::vector<std::vector<double> > result;
    for (int r = 0; r < value.size(); ++r) {
        requireConfig(value[r].getType() == XmlRpc::XmlRpcValue::TypeArray,
                      key + " must contain numeric row arrays");
        std::vector<double> row;
        for (int s = 0; s < value[r].size(); ++s) row.push_back(number(value[r][s], key));
        result.push_back(row);
    }
    return result;
}

}  // namespace parameter_detail

inline ControllerConfig loadConfig(ros::NodeHandle& private_nh) {
    using namespace parameter_detail;
    ControllerConfig c;
    c.control_period_s = requiredNumber(private_nh, "control_period_s");
    c.measurement_period_s = requiredNumber(private_nh, "measurement_period_s");
    c.feedback_gain = requiredPair(private_nh, "feedback_gain");
    c.paper_observer_gain = requiredPair(private_nh, "paper_observer_gain");
    c.disturbance_dynamics = requiredMatrix(private_nh, "disturbance_dynamics");
    c.disturbance_output = requiredVector(private_nh, "disturbance_output");
    c.augmented_observer_gain = requiredVector(private_nh, "augmented_observer_gain");
    c.delta_0 = requiredNumber(private_nh, "delta_0");
    c.delta_1 = requiredNumber(private_nh, "delta_1");
    c.measurement_delay_steps = requiredInteger(private_nh, "measurement_delay_steps");
    c.max_trigger_interval_steps = requiredInteger(private_nh, "max_trigger_interval_steps");
    c.prediction_steps = requiredInteger(private_nh, "prediction_steps");
    c.target_radius_m = requiredNumber(private_nh, "target_radius_m");
    c.target_period_s = requiredNumber(private_nh, "target_period_s");
    c.max_consecutive_dos_steps = requiredInteger(private_nh, "max_consecutive_dos_steps");
    c.wind_disturbance = requiredBoolean(private_nh, "wind_disturbance");
    c.dos_enabled = requiredBoolean(private_nh, "dos/enabled");
    XmlRpc::XmlRpcValue windows = required(private_nh, "dos/attack_windows_steps");
    requireConfig(windows.getType() == XmlRpc::XmlRpcValue::TypeArray,
                  "dos/attack_windows_steps must be an array of [start, end) pairs");
    for (int i = 0; i < windows.size(); ++i) {
        XmlRpc::XmlRpcValue& window = windows[i];
        requireConfig(window.getType() == XmlRpc::XmlRpcValue::TypeArray && window.size() == 2,
                      "each dos/attack_windows_steps entry must contain [start, end)");
        c.attack_windows_steps.push_back({integer(window[0], "attack start step"),
                                          integer(window[1], "attack end step")});
    }
    validateConfig(c);
    return c;
}

}  // namespace asp_phbsc
