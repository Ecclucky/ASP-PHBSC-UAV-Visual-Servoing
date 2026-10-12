#include <ros/ros.h>
#include <asp_phbsc/controller.h>
#include <asp_phbsc/ros_config.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>
#include <stdexcept>
#include <vector>

// The core uses tracking-error coordinates: u = -a_q. This adapter alone
// converts the virtual input to the UAV's ENU acceleration command.
class AspPhbscRosAdapter {
public:
    explicit AspPhbscRosAdapter(const asp_phbsc::ControllerConfig& config)
        : nh_("~"), config_(config), controller_(config),
          desired_relative_(1.0, 0.0, 0.0), acceleration_limits_(0.5, 3.0, 3.0),
          previous_applied_virtual_(Eigen::Vector3d::Zero()),
          last_measurement_(Eigen::Vector3d::Zero()),
          uav_position_(Eigen::Vector3d::Zero()), uav_velocity_(Eigen::Vector3d::Zero()),
          target_position_(Eigen::Vector3d::Zero()), target_velocity_(Eigen::Vector3d::Zero()) {
        loadVector("desired_relative_position", desired_relative_, false);
        loadVector("acceleration_limits", acceleration_limits_, true);
        measurement_sub_ = nh_.subscribe("/point_with_fixed_delay", 32,
            &AspPhbscRosAdapter::measurementCallback, this, ros::TransportHints().tcpNoDelay());
        state_sub_ = nh_.subscribe("/px4_state_pub", 1,
            &AspPhbscRosAdapter::stateCallback, this, ros::TransportHints().tcpNoDelay());
        uav_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/pose", 10,
            &AspPhbscRosAdapter::uavPoseCallback, this);
        uav_velocity_sub_ = nh_.subscribe("/vrpn_client_node/YZU0/0/velocity", 10,
            &AspPhbscRosAdapter::uavVelocityCallback, this);
        target_pose_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/pose", 10,
            &AspPhbscRosAdapter::targetPoseCallback, this);
        target_velocity_sub_ = nh_.subscribe("/vrpn_client_node/YZU1/0/velocity", 10,
            &AspPhbscRosAdapter::targetVelocityCallback, this);
        command_pub_ = nh_.advertise<quadrotor_msgs::PositionCommand>("/acc_cmd", 1);
        log_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/hat_x_topic", 100);
        diagnostics_pub_ = nh_.advertise<std_msgs::Float64MultiArray>("/asp_phbsc/diagnostics", 100);
        timer_ = nh_.createTimer(ros::Duration(config_.control_period_s),
            &AspPhbscRosAdapter::controlTick, this);
        ROS_INFO("Measurement stamps must be acquisition times in the ROS clock domain. "
                 "Measurements are released at stamp + n_tau*T, never receipt + n_tau*T.");
        ROS_WARN("The C-A DoS channel is a software packet-reception emulator. "
                 "The actuator buffer continues to produce fast commands during packet loss.");
    }

    bool failed() const { return failed_; }

private:
    struct Measurement {
        ros::Time stamp;
        Eigen::Vector3d error;
    };

    void loadVector(const std::string& key, Eigen::Vector3d& output, bool positive) {
        std::vector<double> values;
        if (!nh_.hasParam(key)) return;
        if (!nh_.getParam(key, values) || values.size() != 3)
            throw std::invalid_argument(key + " must contain three numbers");
        for (int axis = 0; axis < 3; ++axis) {
            if (!std::isfinite(values[axis]) || (positive && values[axis] <= 0.0))
                throw std::invalid_argument(key + " contains an invalid value");
            output(axis) = values[axis];
        }
    }

    void resetRun() {
        controller_.reset();
        started_ = false;
        last_sample_step_ = -1;
        previous_applied_virtual_.setZero();
        last_measurement_.setZero();
    }

    void stateCallback(const std_msgs::Int32::ConstPtr& message) {
        if (message->data != px4_state_) {
            resetRun();
            measurements_.clear();
        }
        px4_state_ = message->data;
    }

    void measurementCallback(const std_msgs::Float64MultiArray::ConstPtr& message) {
        if (message->data.size() < 6 || message->data[4] != 0.0) return;
        for (int i = 0; i < 6; ++i) {
            if (!std::isfinite(message->data[i])) return;
        }
        const ros::Time now = ros::Time::now();
        const double stamp_s = message->data[3];
        const double tau = config_.measurement_delay_steps * config_.control_period_s;
        // Do not fabricate a timestamp, reuse a latched sample, or relabel a late
        // image as a current sample. Small delivery/fast-clock quantization is
        // bounded by one control period; this is not capture-clock calibration.
        if (stamp_s <= 0.0 || stamp_s <= last_received_stamp_s_ ||
            stamp_s > now.toSec() + 1e-6 ||
            now.toSec() - stamp_s > tau + config_.control_period_s + 1e-6) {
            ROS_WARN_THROTTLE(2.0, "Rejected duplicate, late, future, or invalid visual timestamp.");
            return;
        }
        Measurement measurement;
        measurement.stamp.fromSec(stamp_s);
        measurement.error = Eigen::Vector3d(message->data[0], message->data[1], message->data[2])
                            - desired_relative_;
        last_received_stamp_s_ = stamp_s;
        measurements_.push_back(measurement);
        if (measurements_.size() > 256) {
            measurements_.pop_front();
            ROS_WARN_THROTTLE(2.0, "Visual measurement queue overflow; oldest sample discarded.");
        }
    }

    bool takeDueMeasurement(const ros::Time& now, Eigen::Vector3d& error, double& age) {
        const double tau = config_.measurement_delay_steps * config_.control_period_s;
        bool found = false;
        while (!measurements_.empty()) {
            const double current_age = (now - measurements_.front().stamp).toSec();
            if (current_age + 1e-6 < tau) break;
            const Measurement measurement = measurements_.front();
            measurements_.pop_front();
            if (current_age > tau + config_.control_period_s + 1e-6) {
                ROS_WARN_THROTTLE(2.0, "Discarded a visual sample outside the fixed-delay release window.");
                continue;
            }
            // If multiple eligible samples arrive together, use the latest once.
            error = measurement.error;
            age = current_age;
            found = true;
        }
        return found;
    }

    void publishCommand(const ros::Time& stamp, const Eigen::Vector3d& acceleration) {
        quadrotor_msgs::PositionCommand command;
        command.header.stamp = stamp;
        command.header.frame_id = "world";
        command.acceleration.x = acceleration.x();
        command.acceleration.y = acceleration.y();
        command.acceleration.z = acceleration.z();
        // Preserve the recovered platform's fixed yaw reference and its PX4
        // attitude/thrust mapping. Gravity compensation is performed by px4ctrl.
        command.yaw = 0.0;
        command.yaw_dot = 0.0;
        command_pub_.publish(command);
    }

    void controlTick(const ros::TimerEvent&) {
        const auto start = std::chrono::steady_clock::now();
        const ros::Time now = ros::Time::now();
        if (started_ && !last_tick_.isZero()) {
            const double elapsed = (now - last_tick_).toSec();
            if (elapsed <= 0.0 || elapsed > 1.5 * config_.control_period_s) {
                // Resetting here would erase nonzero applied-input history and
                // replay the configured attack schedule at step zero.
                ROS_FATAL("Fast-clock discontinuity/overrun: ending this control run.");
                failed_ = true;
                publishCommand(now, Eigen::Vector3d::Zero());
                ros::shutdown();
                return;
            }
        }
        last_tick_ = now;
        Eigen::Vector3d measurement = Eigen::Vector3d::Zero();
        double sample_age = -1.0;
        const bool sample_available = takeDueMeasurement(now, measurement, sample_age);
        if (px4_state_ != 3 || (!started_ && !sample_available)) {
            publishCommand(now, Eigen::Vector3d::Zero());
            return;
        }
        started_ = true;
        if (sample_available) last_measurement_ = measurement;
        try {
            long long sample_gap_steps = -1;
            if (sample_available) {
                const long long current_step = controller_.nextStep();
                sample_gap_steps = last_sample_step_ < 0 ? 0 : current_step - last_sample_step_;
                // The first accepted measurement defines slow-sample phase zero.
                // Missing measurements may skip whole slow periods; a sample on
                // another phase cannot be relabelled or silently rescheduled.
                if (last_sample_step_ >= 0 &&
                    (sample_gap_steps <= 0 ||
                     sample_gap_steps % asp_phbsc::measurementSteps(config_) != 0))
                    throw std::runtime_error("visual measurement release is outside the configured slow-sampling grid; gap=" +
                                             std::to_string(sample_gap_steps) + " fast steps");
            }
            const asp_phbsc::StepResult result = controller_.step(
                sample_available, measurement, previous_applied_virtual_);
            if (sample_available) last_sample_step_ = result.step;
            Eigen::Vector3d acceleration = -result.buffered_virtual_input;
            bool saturated = false;
            for (int axis = 0; axis < 3; ++axis) {
                if (!std::isfinite(acceleration(axis)))
                    throw std::runtime_error("non-finite acceleration command");
                const double limited = std::max(-acceleration_limits_(axis),
                                               std::min(acceleration_limits_(axis), acceleration(axis)));
                saturated = saturated || limited != acceleration(axis);
                acceleration(axis) = limited;
            }
            // The observer must use the command actually sent after hardware
            // limits, not the unconstrained candidate or unreceived packet.
            previous_applied_virtual_ = -acceleration;
            publishCommand(now, acceleration);
            const double elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            publishLogs(now, result, acceleration, saturated, sample_age, sample_gap_steps, elapsed);
        } catch (const std::exception& error) {
            ROS_FATAL_STREAM("ASP-PHBSC control failed: " << error.what());
            failed_ = true;
            publishCommand(now, Eigen::Vector3d::Zero());
            ros::shutdown();
        }
    }

    void publishLogs(const ros::Time& stamp, const asp_phbsc::StepResult& result,
                     const Eigen::Vector3d& acceleration, bool saturated,
                     double sample_age, long long sample_gap_steps, double duration) {
        std_msgs::Float64MultiArray log;
        log.data.assign(38, 0.0);
        for (int axis = 0; axis < 3; ++axis) {
            const int offset = 12 * axis;
            log.data[offset] = result.current_x(0, axis);
            log.data[offset + 1] = result.current_x(1, axis);
            log.data[offset + 2] = acceleration(axis);
            log.data[offset + 3] = last_measurement_(axis);
            // Ground truth is logging-only; no core input uses these values.
            log.data[offset + 4] = uav_position_(axis);
            log.data[offset + 5] = uav_velocity_(axis);
            log.data[offset + 6] = target_position_(axis);
            log.data[offset + 7] = target_velocity_(axis);
            log.data[offset + 8] = result.buffered_x(0, axis);
            log.data[offset + 9] = result.buffered_x(1, axis);
        }
        log.data[10] = result.estimated_disturbance.x();
        log.data[11] = result.event_error;
        log.data[22] = result.dos_active ? 1.0 : 0.0;
        log.data[23] = result.estimated_disturbance.y();
        log.data[34] = result.estimated_disturbance.z();
        log.data[35] = result.transmission_accepted ? 1.0 : 0.0;
        log.data[36] = static_cast<double>(result.step);
        log.data[37] = duration;
        log_pub_.publish(log);

        std_msgs::Float64MultiArray diagnostic;
        diagnostic.data = {stamp.toSec(), static_cast<double>(result.step),
            result.sample_used ? 1.0 : 0.0, result.candidate_rebuilt ? 1.0 : 0.0,
            result.transmission_attempted ? 1.0 : 0.0,
            result.transmission_accepted ? 1.0 : 0.0, result.dos_active ? 1.0 : 0.0,
            saturated ? 1.0 : 0.0, sample_age, result.event_error,
            result.event_threshold, previous_applied_virtual_.x(),
            previous_applied_virtual_.y(), previous_applied_virtual_.z(),
            static_cast<double>(sample_gap_steps)};
        diagnostics_pub_.publish(diagnostic);
    }

    void uavPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& m) {
        uav_position_ = Eigen::Vector3d(m->pose.position.x, m->pose.position.y, m->pose.position.z);
    }
    void uavVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& m) {
        uav_velocity_ = Eigen::Vector3d(m->twist.linear.x, m->twist.linear.y, m->twist.linear.z);
    }
    void targetPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& m) {
        target_position_ = Eigen::Vector3d(m->pose.position.x, m->pose.position.y, m->pose.position.z);
    }
    void targetVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr& m) {
        target_velocity_ = Eigen::Vector3d(m->twist.linear.x, m->twist.linear.y, m->twist.linear.z);
    }

    ros::NodeHandle nh_;
    const asp_phbsc::ControllerConfig config_;
    AspPhbscController controller_;
    Eigen::Vector3d desired_relative_, acceleration_limits_, previous_applied_virtual_, last_measurement_;
    Eigen::Vector3d uav_position_, uav_velocity_, target_position_, target_velocity_;
    std::deque<Measurement> measurements_;
    int px4_state_ = 1;
    bool started_ = false;
    bool failed_ = false;
    long long last_sample_step_ = -1;
    double last_received_stamp_s_ = -1.0;
    ros::Time last_tick_;
    ros::Subscriber measurement_sub_, state_sub_, uav_pose_sub_, uav_velocity_sub_, target_pose_sub_, target_velocity_sub_;
    ros::Publisher command_pub_, log_pub_, diagnostics_pub_;
    ros::Timer timer_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "asp_phbsc_node");
    try {
        ros::NodeHandle private_nh("~");
        // Validate the full augmented model before creating any flight publisher.
        const asp_phbsc::ControllerConfig config = asp_phbsc::loadConfig(private_nh);
        AspPhbscRosAdapter adapter(config);
        ros::spin();
        return adapter.failed() ? 1 : 0;
    } catch (const std::exception& error) {
        ROS_FATAL_STREAM("ASP-PHBSC startup refused: " << error.what());
        return 1;
    }
    return 0;
}
