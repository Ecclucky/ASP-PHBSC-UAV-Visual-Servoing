#pragma once

#include <asp_phbsc/config.h>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <algorithm>
#include <deque>
#include <functional>
#include <stdexcept>
#include <utility>
#include <vector>

// Mathematical reconstruction from the manuscript, not a recovered flight
// executable. The caller supplies the missing UAV exosystem and full gain.
// This core has no ROS, camera, motor, or flight-controller side effects.
namespace asp_phbsc {

using PhysicalState = Eigen::Matrix<double, 2, 3>; // rows e_r,e_v; columns X,Y,Z
using ControlPacket = std::vector<Eigen::Vector3d>;
using StatePacket = std::vector<PhysicalState>;

// Independent controller/actuator buffer storage. Pi shifts once and appends
// zero, exactly as printed in the manuscript. No implicit terminal hold.
class PacketBuffer {
public:
    explicit PacketBuffer(std::size_t length) : values_(length, Eigen::Vector3d::Zero()),
        states_(length, PhysicalState::Zero()) {
        requireConfig(length > 0, "a packet must contain at least one input");
    }
    void reset() {
        std::fill(values_.begin(), values_.end(), Eigen::Vector3d::Zero());
        std::fill(states_.begin(), states_.end(), PhysicalState::Zero());
    }
    void shiftZero() {
        for (std::size_t j = 0; j + 1 < values_.size(); ++j) {
            values_[j] = values_[j + 1];
            states_[j] = states_[j + 1];
        }
        values_.back().setZero();
        states_.back().setZero();
    }
    void replace(const ControlPacket& values, const StatePacket& states) {
        if (values.size() != values_.size() || states.size() != states_.size())
            throw std::invalid_argument("ASP-PHBSC packet size mismatch");
        values_ = values;
        states_ = states;
    }
    const Eigen::Vector3d& first() const { return values_.front(); }
    const PhysicalState& firstState() const { return states_.front(); }
    const ControlPacket& values() const { return values_; }
    const StatePacket& states() const { return states_; }
private:
    ControlPacket values_;
    StatePacket states_;
};

struct StepResult {
    long long step = 0;
    PhysicalState current_x = PhysicalState::Zero();
    Eigen::MatrixXd current_w;
    Eigen::Vector3d estimated_disturbance = Eigen::Vector3d::Zero();
    Eigen::Vector3d candidate_virtual_input = Eigen::Vector3d::Zero();
    Eigen::Vector3d buffered_virtual_input = Eigen::Vector3d::Zero();
    PhysicalState buffered_x = PhysicalState::Zero();
    bool sample_used = false;
    bool candidate_rebuilt = false;
    bool transmission_attempted = false;
    bool transmission_accepted = false;
    bool dos_active = false;
    double event_error = 0.0;
    double event_threshold = 0.0;
};

// One event decision for the complete three-axis packet. Disturbance-state
// magnitude must not enter the threshold norm: the paper uses physical x.
inline bool eventTriggered(const Eigen::Vector3d& candidate_first,
                           const Eigen::Vector3d& actuator_first,
                           const PhysicalState& current_x, double delta_0,
                           double delta_1, long long step, long long last_event,
                           int max_interval) {
    return (candidate_first - actuator_first).norm() > delta_0 * current_x.norm() + delta_1 ||
           step - last_event >= max_interval;
}

} // namespace asp_phbsc

class AspPhbscController {
public:
    // Return true when this step's transmission would be received. This is an
    // injectable software gate, not a ROS/MAVLink link or a physical actuator.
    using ReceptionGate = std::function<bool(long long)>;

    explicit AspPhbscController(const asp_phbsc::ControllerConfig& config,
                               ReceptionGate gate = ReceptionGate())
        : config_(validated(config)), dimension_(2 + static_cast<int>(config.disturbance_dynamics.size())),
          candidate_(static_cast<std::size_t>(config.prediction_steps) + 1),
          actuator_(static_cast<std::size_t>(config.prediction_steps) + 1), gate_(std::move(gate)) {
        initializeModel();
        reset();
    }

    void reset() { reset(Eigen::MatrixXd::Zero(dimension_, 3)); }
    void reset(const Eigen::MatrixXd& delayed_augmented_state) {
        if (delayed_augmented_state.rows() != dimension_ || delayed_augmented_state.cols() != 3 ||
            !delayed_augmented_state.allFinite())
            throw std::invalid_argument("ASP-PHBSC initial delayed state must be finite and (2+l_w)-by-3");
        delayed_state_ = delayed_augmented_state;
        history_.clear();
        history_first_step_ = 0;
        next_step_ = 0;
        last_event_step_ = -1;
        last_success_step_ = -1;
        candidate_.reset();
        actuator_.reset();
    }

    // At call i, previous_applied_virtual_input is the actual u_{i-1}, including
    // any adapter saturation; it is ignored only on the initial call i=0.
    // delayed_tracking_error is y_i^tau (already offset into tracking error).
    // Returned virtual input u must be converted to UAV acceleration a_q=-u.
    asp_phbsc::StepResult step(bool sample_available,
                              const Eigen::Vector3d& delayed_tracking_error,
                              const Eigen::Vector3d& previous_applied_virtual_input) {
        if (!previous_applied_virtual_input.allFinite() ||
            (sample_available && !delayed_tracking_error.allFinite()))
            throw std::invalid_argument("ASP-PHBSC received a non-finite measurement or actual input");
        const long long i = next_step_;
        const bool reception_allowed = gate_ ? gate_(i) : !asp_phbsc::attackActive(config_, i);
        if (i == 0 && !reception_allowed)
            throw std::invalid_argument("ASP-PHBSC requires successful reception of the initial packet at step 0");
        if (i > 0) recordPreviousInput(i - 1, previous_applied_virtual_input);

        asp_phbsc::StepResult out;
        out.step = i;
        out.sample_used = sample_available;
        out.dos_active = !reception_allowed;
        const Eigen::MatrixXd current = predictCurrent(delayed_state_, i, i, asp_phbsc::ControlPacket());
        out.current_x = current.topRows<2>();
        out.current_w = current.bottomRows(dimension_ - 2);
        out.estimated_disturbance = (Cw_ * out.current_w).transpose();

        // Initial packet is generated from the explicitly initialized observer
        // even if a delayed sample has not arrived yet (initial-success premise).
        out.candidate_rebuilt = sample_available || i == 0;
        if (out.candidate_rebuilt) rebuildCandidate(i, sample_available, delayed_tracking_error);
        else candidate_.shiftZero();
        if (i > 0) actuator_.shiftZero();

        out.candidate_virtual_input = candidate_.first();
        out.event_error = (candidate_.first() - actuator_.first()).norm();
        out.event_threshold = config_.delta_0 * out.current_x.norm() + config_.delta_1;
        out.transmission_attempted = i == 0 || asp_phbsc::eventTriggered(
            candidate_.first(), actuator_.first(), out.current_x, config_.delta_0,
            config_.delta_1, i, last_event_step_, config_.max_trigger_interval_steps);
        // i_e in the manuscript is an event, not a successful reception.
        if (out.transmission_attempted) last_event_step_ = i;
        out.transmission_accepted = out.transmission_attempted && reception_allowed;
        if (out.transmission_accepted) {
            actuator_.replace(candidate_.values(), candidate_.states());
            last_success_step_ = i;
        }
        out.buffered_virtual_input = actuator_.first();
        out.buffered_x = actuator_.firstState();

        // Eq. (6a) advances z_i to z_{i+1}; do not label z_{i+1} as z_i in
        // the delay compensator or apply an undocumented same-step correction.
        const Eigen::Vector3d delayed_u = historicalInput(i - config_.measurement_delay_steps);
        delayed_state_ = advanceObserver(delayed_state_, delayed_u, sample_available, delayed_tracking_error);
        if (!delayed_state_.allFinite() || !current.allFinite() || !out.buffered_virtual_input.allFinite())
            throw std::runtime_error("ASP-PHBSC numerical state is not finite");
        ++next_step_;
        return out;
    }

    long long nextStep() const { return next_step_; }
    long long lastEventStep() const { return last_event_step_; }
    long long lastSuccessfulTransmissionStep() const { return last_success_step_; }
    std::size_t historySize() const { return history_.size(); }
    const asp_phbsc::ControlPacket& candidateControlSequence() const { return candidate_.values(); }
    const asp_phbsc::ControlPacket& appliedControlSequence() const { return actuator_.values(); }
    const Eigen::MatrixXd& delayedState() const { return delayed_state_; }
    const Eigen::MatrixXd& discreteA() const { return Ad_; }
    const Eigen::VectorXd& discreteB() const { return Bd_; }
    const Eigen::VectorXd& discreteL() const { return Ld_; }
    const asp_phbsc::ControllerConfig& config() const { return config_; }

private:
    static asp_phbsc::ControllerConfig validated(const asp_phbsc::ControllerConfig& c) {
        asp_phbsc::validateConfig(c);
        return c;
    }

    void initializeModel() {
        Eigen::MatrixXd A = Eigen::MatrixXd::Zero(dimension_, dimension_);
        Eigen::VectorXd B = Eigen::VectorXd::Zero(dimension_);
        Eigen::VectorXd L(dimension_);
        A(0, 1) = 1.0;
        B(1) = 1.0;
        const int nw = dimension_ - 2;
        Cw_.resize(nw);
        feedback_.resize(dimension_);
        feedback_(0) = config_.feedback_gain[0];
        feedback_(1) = config_.feedback_gain[1];
        for (int r = 0; r < nw; ++r) {
            Cw_(r) = config_.disturbance_output[static_cast<std::size_t>(r)];
            A(1, r + 2) = Cw_(r);
            feedback_(r + 2) = Cw_(r);
            for (int s = 0; s < nw; ++s)
                A(r + 2, s + 2) = config_.disturbance_dynamics[static_cast<std::size_t>(r)][static_cast<std::size_t>(s)];
        }
        for (int r = 0; r < dimension_; ++r)
            L(r) = config_.augmented_observer_gain[static_cast<std::size_t>(r)];

        Eigen::MatrixXd observer_error = A;
        observer_error.col(0) += L; // A + L C, C=[1,0,...,0]
        const Eigen::EigenSolver<Eigen::MatrixXd> spectrum(observer_error, false);
        asp_phbsc::requireConfig(spectrum.info() == Eigen::Success,
                                "failed to compute eigenvalues of A + L C");
        for (int r = 0; r < dimension_; ++r) {
            const auto eigenvalue = spectrum.eigenvalues()(r);
            asp_phbsc::requireConfig(std::isfinite(eigenvalue.real()) &&
                                    std::isfinite(eigenvalue.imag()) && eigenvalue.real() < 0.0,
                "A + L C must be Hurwitz; the supplied augmented observer gain is not admissible");
        }
        // This basic continuous observer check is not a verification of the
        // manuscript's complete sampled-data stability inequalities.

        // exp([[A,B,L],[0,0,0],[0,0,0]] T) yields all three exact ZOH blocks;
        // this works for singular A, including constant-disturbance modes.
        Eigen::MatrixXd block = Eigen::MatrixXd::Zero(dimension_ + 2, dimension_ + 2);
        block.topLeftCorner(dimension_, dimension_) = A;
        block.block(0, dimension_, dimension_, 1) = B;
        block.block(0, dimension_ + 1, dimension_, 1) = L;
        const Eigen::MatrixXd exponential = (block * config_.control_period_s).exp();
        Ad_ = exponential.topLeftCorner(dimension_, dimension_);
        Bd_ = exponential.block(0, dimension_, dimension_, 1);
        Ld_ = exponential.block(0, dimension_ + 1, dimension_, 1);
        if (!Ad_.allFinite() || !Bd_.allFinite() || !Ld_.allFinite())
            throw std::invalid_argument("ASP-PHBSC discretization is not finite for the supplied model");
    }

    void recordPreviousInput(long long index, const Eigen::Vector3d& actual) {
        if (history_.empty()) history_first_step_ = index;
        history_.push_back(actual);
        while (history_.size() > static_cast<std::size_t>(config_.measurement_delay_steps)) {
            history_.pop_front();
            ++history_first_step_;
        }
    }
    Eigen::Vector3d historicalInput(long long index) const {
        if (index < 0) return Eigen::Vector3d::Zero(); // explicit zero startup history
        if (index < history_first_step_ ||
            index >= history_first_step_ + static_cast<long long>(history_.size()))
            throw std::logic_error("ASP-PHBSC requested unavailable actual input history");
        return history_[static_cast<std::size_t>(index - history_first_step_)];
    }
    Eigen::Vector3d rolloutInput(long long index, long long origin,
                                const asp_phbsc::ControlPacket& future_inputs) const {
        if (index < origin) return historicalInput(index);
        const auto offset = static_cast<std::size_t>(index - origin);
        if (offset >= future_inputs.size())
            throw std::logic_error("ASP-PHBSC predictor requested an uncomputed future input");
        return future_inputs[offset];
    }
    Eigen::MatrixXd predictCurrent(Eigen::MatrixXd delayed, long long index, long long origin,
                                   const asp_phbsc::ControlPacket& future_inputs) const {
        // Chronological propagation equals A_d^n z + Gamma [u_{i-1};...;u_{i-n}].
        for (int r = 0; r < config_.measurement_delay_steps; ++r) {
            const auto u = rolloutInput(index - config_.measurement_delay_steps + r, origin, future_inputs);
            delayed = (Ad_ * delayed + Bd_ * u.transpose()).eval();
        }
        return delayed;
    }
    Eigen::MatrixXd advanceObserver(const Eigen::MatrixXd& delayed,
                                   const Eigen::Vector3d& delayed_input, bool has_measurement,
                                   const Eigen::Vector3d& measurement) const {
        const Eigen::MatrixXd open_loop = Ad_ * delayed + Bd_ * delayed_input.transpose();
        // Literal printed Eq. (6b): y_i^p = C A_d z_i + C B_d u_{i-n_tau}.
        // Its RHS is a one-step prediction despite the printed subscript i;
        // we preserve that formula explicitly rather than silently re-index it.
        const Eigen::RowVector3d mixed_output = has_measurement
            ? Eigen::RowVector3d(measurement.transpose()) : Eigen::RowVector3d(open_loop.row(0));
        return open_loop - Ld_ * (mixed_output - delayed.row(0));
    }
    void rebuildCandidate(long long index, bool sample_available, const Eigen::Vector3d& measurement) {
        asp_phbsc::ControlPacket values;
        asp_phbsc::StatePacket states;
        const std::size_t count = static_cast<std::size_t>(config_.prediction_steps) + 1;
        values.reserve(count);
        states.reserve(count);
        Eigen::MatrixXd predicted_delayed = delayed_state_;
        for (std::size_t n = 0; n < count; ++n) {
            const long long q = index + static_cast<long long>(n);
            const Eigen::MatrixXd current = predictCurrent(predicted_delayed, q, index, values);
            const Eigen::Vector3d u = -(feedback_ * current).transpose();
            if (!current.allFinite() || !u.allFinite())
                throw std::runtime_error("ASP-PHBSC candidate prediction is not finite");
            values.push_back(u);
            states.push_back(current.topRows<2>());
            if (n + 1 < count) {
                const auto delayed_u = rolloutInput(q - config_.measurement_delay_steps, index, values);
                // Future slow samples are unknown, including future multiples of M.
                predicted_delayed = advanceObserver(predicted_delayed, delayed_u,
                                                     n == 0 && sample_available, measurement);
            }
        }
        candidate_.replace(values, states);
    }

    const asp_phbsc::ControllerConfig config_;
    const int dimension_;
    Eigen::MatrixXd Ad_;
    Eigen::VectorXd Bd_, Ld_;
    Eigen::RowVectorXd Cw_, feedback_;
    Eigen::MatrixXd delayed_state_;
    std::deque<Eigen::Vector3d> history_;
    long long history_first_step_ = 0;
    long long next_step_ = 0;
    long long last_event_step_ = -1;
    long long last_success_step_ = -1;
    asp_phbsc::PacketBuffer candidate_;
    asp_phbsc::PacketBuffer actuator_;
    ReceptionGate gate_;
};
