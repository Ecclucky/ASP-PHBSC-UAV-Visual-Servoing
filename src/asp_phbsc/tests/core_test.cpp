// Executes the production mathematical core, without ROS or hardware.
// Every disturbance model below is a NEW TEST FIXTURE. None is asserted to be
// the unidentified model/gain used in the manuscript's UAV experiments.
#ifdef NDEBUG
#undef NDEBUG
#endif
#include <asp_phbsc/controller.h>
#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
using asp_phbsc::ControllerConfig;
using asp_phbsc::PhysicalState;
using Eigen::Vector3d;
const Vector3d zero = Vector3d::Zero();

template<class A, class B>
void expectNear(const Eigen::MatrixBase<A>& actual, const Eigen::MatrixBase<B>& expected,
                double tolerance = 1e-10) {
    assert(actual.rows() == expected.rows() && actual.cols() == expected.cols());
    assert((actual - expected).norm() <= tolerance * (1.0 + expected.norm()));
}
void expectNear(double actual, double expected, double tolerance = 1e-10) {
    assert(std::abs(actual - expected) <= tolerance * (1.0 + std::abs(expected)));
}
ControllerConfig testConfig(bool second_case = false) {
    ControllerConfig c;
    c.target_radius_m = 0.67;
    c.target_period_s = second_case ? 4.5 : 8.0;
    c.max_consecutive_dos_steps = second_case ? 40 : 60;
    c.wind_disturbance = second_case;
    // Newly selected constant-disturbance fixture; NOT a flight parameter claim.
    c.disturbance_dynamics = {{0.0}};
    c.disturbance_output = {1.0};
    c.augmented_observer_gain = {-0.6, -6.6, -1.0};
    return c;
}
void expectInvalid(const ControllerConfig& c) {
    bool rejected = false;
    try { AspPhbscController invalid(c); }
    catch (const std::invalid_argument&) { rejected = true; }
    assert(rejected);
}

void checkConfigurationAndInitialPremise() {
    auto c = testConfig();
    asp_phbsc::validateBaseConfig(c);
    assert(asp_phbsc::measurementSteps(c) == 5);
    c.disturbance_dynamics.clear(); c.disturbance_output.clear();
    c.augmented_observer_gain.clear();
    expectInvalid(c); // Production YAML intentionally has this unresolved state.
    c = testConfig(); c.augmented_observer_gain = {-0.6, -6.6}; expectInvalid(c);
    c = testConfig(); c.disturbance_dynamics = {{0.0, 1.0}}; expectInvalid(c);
    c = testConfig(); c.disturbance_output.clear(); expectInvalid(c);
    c = testConfig(); c.disturbance_output[0] = std::numeric_limits<double>::quiet_NaN(); expectInvalid(c);
    c = testConfig(); c.augmented_observer_gain = {0.0, 0.0, 0.0}; expectInvalid(c); // Not Hurwitz.
    c = testConfig(); c.prediction_steps = 79; expectInvalid(c);
    c = testConfig(); c.measurement_period_s = 0.055; expectInvalid(c);
    c = testConfig(); c.control_period_s = 0.0; expectInvalid(c);
    c = testConfig(); c.measurement_delay_steps = 0; expectInvalid(c);
    c = testConfig(); c.delta_1 = -0.01; expectInvalid(c);
    c = testConfig(); c.dos_enabled = true; expectInvalid(c);
    c = testConfig(); c.attack_windows_steps = {{0, 1}}; expectInvalid(c);
    c = testConfig(); c.attack_windows_steps = {{1, 61}, {60, 62}}; expectInvalid(c);
    c = testConfig(true); c.attack_windows_steps = {{1, 31}, {31, 51}}; expectInvalid(c);
    c = testConfig(); c.delta_0 = c.delta_1 = 0.0; AspPhbscController zero_thresholds(c);

    AspPhbscController refused(testConfig(), [](long long) { return false; });
    bool initial_rejected = false;
    try { refused.step(false, zero, zero); }
    catch (const std::invalid_argument&) { initial_rejected = true; }
    assert(initial_rejected && refused.nextStep() == 0);
}

void checkExactDiscretization() {
    const auto c = testConfig();
    AspPhbscController core(c);
    const double h = c.control_period_s;
    Eigen::Matrix3d expected_a;
    expected_a << 1.0, h, h*h/2.0, 0.0, 1.0, h, 0.0, 0.0, 1.0;
    Vector3d expected_b(h*h/2.0, h, 0.0);
    Vector3d expected_l(-0.6*h-6.6*h*h/2.0-h*h*h/6.0,
                        -6.6*h-h*h/2.0, -h);
    expectNear(core.discreteA(), expected_a);
    expectNear(core.discreteB(), expected_b);
    expectNear(core.discreteL(), expected_l);

    // A different, explicitly synthetic two-state harmonic exosystem checks
    // arbitrary augmentation dimension and a valid gain with a positive entry.
    auto harmonic = testConfig();
    harmonic.disturbance_dynamics = {{0.0, 1.0}, {-1.0, 0.0}};
    harmonic.disturbance_output = {1.0, 0.0};
    harmonic.augmented_observer_gain = {-4.0, -5.0, 0.0, 4.0}; // characteristic (s+1)^4
    AspPhbscController oscillatory(harmonic);
    const double cs = std::cos(h), sn = std::sin(h);
    Eigen::Matrix4d harmonic_a;
    harmonic_a << 1.0,h,1.0-cs,h-sn, 0.0,1.0,sn,1.0-cs,
                  0.0,0.0,cs,sn, 0.0,0.0,-sn,cs;
    Eigen::Vector4d harmonic_l(-4.0*h-h*h/2.0-4.0*(1.0-cs),
                               -h-4.0*sn, 4.0*(1.0-cs), 4.0*sn);
    expectNear(oscillatory.discreteA(), harmonic_a);
    expectNear(oscillatory.discreteL(), harmonic_l);
    expectNear(oscillatory.discreteB(), Eigen::Vector4d(h*h/2.0,h,0.0,0.0));
}

void checkObserverAndPredictorTiming() {
    const auto c = testConfig();
    AspPhbscController core(c);
    const Vector3d measured(1.0,-2.0,0.5);
    const auto first = core.step(true, measured, Vector3d::Constant(99.0)); // no u_-1 is consumed
    expectNear(first.current_x, PhysicalState::Zero());
    expectNear(first.buffered_virtual_input, zero);
    const Eigen::MatrixXd z1 = -core.discreteL() * measured.transpose();
    expectNear(core.delayedState(), z1); // Correct minus-L_d residual sign.
    const auto old_packet = core.candidateControlSequence();
    assert(old_packet.size() == 101);
    Eigen::MatrixXd predicted = z1;
    for (int j = 0; j < c.measurement_delay_steps; ++j)
        predicted = (core.discreteA() * predicted).eval();
    const auto second = core.step(false, zero, zero);
    expectNear(second.current_x, predicted.topRows<2>());
    expectNear(second.current_w, predicted.bottomRows(1));
    const Eigen::MatrixXd open_loop = core.discreteA() * z1;
    const Eigen::MatrixXd z2 = open_loop - core.discreteL() * (open_loop.row(0) - z1.row(0));
    expectNear(core.delayedState(), z2); // Literal printed intersample y_i^p.
    assert(!second.candidate_rebuilt);
    for (std::size_t j = 0; j + 1 < old_packet.size(); ++j)
        expectNear(core.candidateControlSequence()[j], old_packet[j+1]);
    expectNear(core.candidateControlSequence().back(), zero);
}

void checkActualInputHistoryAndGammaOrder() {
    auto c = testConfig();
    c.measurement_delay_steps = 2;
    AspPhbscController core(c);
    core.step(false, zero, zero);
    const Vector3d u0(2.0,-3.0,5.0), u1(7.0,11.0,-13.0), u2(-17.0,19.0,23.0);
    // Deliberately differ from returned commands. These are externally applied
    // virtual inputs, as after a hardware adapter's saturation.
    auto at1 = core.step(false, zero, u0);
    expectNear(at1.current_x.row(0), (0.5*c.control_period_s*c.control_period_s*u0).transpose());
    expectNear(at1.current_x.row(1), (c.control_period_s*u0).transpose());
    auto at2 = core.step(false, zero, u1);
    expectNear(at2.current_x.row(0),
               (c.control_period_s*c.control_period_s*(1.5*u0+0.5*u1)).transpose());
    expectNear(at2.current_x.row(1), (c.control_period_s*(u0+u1)).transpose());
    const Eigen::MatrixXd z3 = core.delayedState();
    const Eigen::MatrixXd expected = core.discreteA()*core.discreteA()*z3 +
        core.discreteA()*core.discreteB()*u1.transpose() + core.discreteB()*u2.transpose();
    auto at3 = core.step(false, zero, u2);
    expectNear(at3.current_x, expected.topRows<2>());
    expectNear(at3.current_w, expected.bottomRows(1));
    for (int j = 4; j < 205; ++j) {
        core.step(false, zero, zero);
        assert(core.historySize() == 2);
    }
}

void checkDisturbanceCompensationAndStateNorm() {
    auto c = testConfig();
    AspPhbscController core(c);
    Eigen::MatrixXd initial = Eigen::MatrixXd::Zero(3,3);
    initial.row(0) << 1.0,2.0,-3.0;
    initial.row(1) << 0.5,-0.2,0.3;
    initial.row(2) << 2.0,-4.0,6.0;
    core.reset(initial);
    const auto out = core.step(false, zero, zero);
    const double tau = c.measurement_delay_steps*c.control_period_s;
    PhysicalState expected;
    expected.row(0) = initial.row(0)+tau*initial.row(1)+0.5*tau*tau*initial.row(2);
    expected.row(1) = initial.row(1)+tau*initial.row(2);
    expectNear(out.current_x, expected);
    expectNear(out.estimated_disturbance, initial.row(2).transpose());
    const Vector3d expected_u = -(4.0*expected.row(0)+2.0*expected.row(1)+initial.row(2)).transpose();
    expectNear(out.candidate_virtual_input, expected_u);
    expectNear(out.buffered_virtual_input, expected_u);
    assert(out.transmission_accepted && out.step == 0);

    // Cancel the physical estimate at time 0 while keeping a large w estimate.
    // The ET threshold must remain delta_1, with no disturbance-state term.
    initial.setZero();
    initial.row(2).setConstant(100.0);
    initial.row(1).setConstant(-tau*100.0);
    initial.row(0).setConstant(0.5*tau*tau*100.0);
    core.reset(initial);
    const auto large_w = core.step(false, zero, zero);
    expectNear(large_w.current_x, PhysicalState::Zero());
    expectNear(large_w.event_threshold, c.delta_1);
    assert(large_w.current_w.norm() > 100.0);
}

void checkPacketsAndNoImaginaryMeasurements() {
    auto c = testConfig();
    AspPhbscController core(c);
    const auto first = core.step(true, Vector3d(1.0,-0.5,0.2), zero);
    assert(first.transmission_accepted && first.candidate_rebuilt);
    auto previous_actual = first.buffered_virtual_input;
    // Beyond several future slow-sensor instants, the originally predicted
    // packet must agree with live no-measurement propagation when it is applied.
    for (int i = 1; i <= 15; ++i) {
        const auto out = core.step(false, zero, previous_actual);
        const Vector3d feedback = -(4.0*out.current_x.row(0)+2.0*out.current_x.row(1)).transpose()
                                  - out.estimated_disturbance;
        expectNear(out.candidate_virtual_input, feedback);
        assert(!out.candidate_rebuilt);
        expectNear(out.buffered_virtual_input, out.candidate_virtual_input);
        previous_actual = out.buffered_virtual_input;
    }
    for (int i = 16; i <= 102; ++i) {
        const auto out = core.step(false, zero, previous_actual);
        previous_actual = out.buffered_virtual_input;
    }
    for (const auto& entry : core.candidateControlSequence()) expectNear(entry, zero);
    for (const auto& entry : core.appliedControlSequence()) expectNear(entry, zero);

    asp_phbsc::PacketBuffer buffer(3);
    asp_phbsc::ControlPacket entries{Vector3d::Constant(1),Vector3d::Constant(2),Vector3d::Constant(3)};
    asp_phbsc::StatePacket states(3, PhysicalState::Ones());
    buffer.replace(entries, states);
    buffer.shiftZero();
    expectNear(buffer.values()[0], entries[1]);
    expectNear(buffer.values()[1], entries[2]);
    expectNear(buffer.values()[2], zero);
    expectNear(buffer.states().back(), PhysicalState::Zero());
}

void checkFirstInputEventAndJointNorm() {
    const PhysicalState x = PhysicalState::Zero();
    assert(!asp_phbsc::eventTriggered(Vector3d(3,4,0),zero,x,0.0,5.0,1,0,20)); // strict >
    assert(asp_phbsc::eventTriggered(Vector3d(3,4,0),zero,x,0.0,4.999,1,0,20));
    PhysicalState physical;
    physical << 1,2,3,4,5,6;
    assert(!asp_phbsc::eventTriggered(Vector3d::Constant(0.1),zero,physical,0.02,0.02,1,0,20));

    AspPhbscController core(testConfig());
    auto first = core.step(true, Vector3d::Ones(), zero);
    const auto before = core.appliedControlSequence();
    // A radically changed fresh measurement changes FUTURE entries, but the
    // current estimate precedes z_i -> z_{i+1}; its first input is unchanged.
    auto second = core.step(true, Vector3d::Constant(100.0), first.buffered_virtual_input);
    expectNear(second.event_error, 0.0);
    assert(second.candidate_rebuilt && !second.transmission_attempted);
    double whole_packet_difference = 0.0;
    for (std::size_t j = 0; j + 1 < before.size(); ++j)
        whole_packet_difference += (core.candidateControlSequence()[j]-before[j+1]).squaredNorm();
    assert(whole_packet_difference > 1.0); // Would falsely trigger the old whole-packet norm.
    const auto third = core.step(false, zero, second.buffered_virtual_input);
    assert(third.transmission_attempted && third.transmission_accepted);
    assert(third.event_error > third.event_threshold);
}

void checkFailedEventClockAndGate() {
    auto c = testConfig();
    c.dos_enabled = true;
    c.attack_windows_steps = {{20,22}};
    AspPhbscController core(c);
    const auto first = core.step(false, zero, zero);
    assert(first.transmission_attempted && first.transmission_accepted);
    for (long long i = 1; i < 20; ++i) {
        const auto r = core.step(false,zero,zero);
        assert(r.step == i && !r.transmission_attempted);
    }
    const auto blocked = core.step(false,zero,zero);
    assert(blocked.transmission_attempted && !blocked.transmission_accepted && blocked.dos_active);
    assert(core.lastEventStep() == 20 && core.lastSuccessfulTransmissionStep() == 0);
    const auto following = core.step(false,zero,zero);
    assert(!following.transmission_attempted && following.dos_active);
    for (int i = 22; i < 40; ++i) {
        const auto r = core.step(false,zero,zero);
        assert(!r.dos_active && !r.transmission_attempted);
    }
    assert(core.step(false,zero,zero).transmission_accepted);
    assert(core.lastEventStep() == 40 && core.lastSuccessfulTransmissionStep() == 40);
    c.attack_windows_steps = {{200,240},{500,540}};
    asp_phbsc::validateConfig(c);
    assert(!asp_phbsc::attackActive(c,199));
    assert(asp_phbsc::attackActive(c,200));
    assert(asp_phbsc::attackActive(c,239));
    assert(!asp_phbsc::attackActive(c,240));
    core.reset();
    assert(core.nextStep() == 0 && core.historySize() == 0 && core.lastEventStep() == -1);
}

} // namespace

int main() {
    checkConfigurationAndInitialPremise();
    checkExactDiscretization();
    checkObserverAndPredictorTiming();
    checkActualInputHistoryAndGammaOrder();
    checkDisturbanceCompensationAndStateNorm();
    checkPacketsAndNoImaginaryMeasurements();
    checkFirstInputEventAndJointNorm();
    checkFailedEventClockAndGate();
    std::cout << "ASP-PHBSC mathematical core tests passed; models are synthetic fixtures, no ROS/hardware.\n";
}
