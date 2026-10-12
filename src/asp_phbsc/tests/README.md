# Offline core checks

`core_test.cpp` executes the same `AspPhbscController` header as the ROS node.
It requires only a C++11 compiler and Eigen3; it does not open camera, ROS,
network, motor, or flight-controller connections.

From the package directory on Linux with Eigen3 installed:

```sh
g++ -std=c++11 -Iinclude -I/usr/include/eigen3 tests/core_test.cpp -o /tmp/asp_phbsc_core_test
/tmp/asp_phbsc_core_test
```

The test is also registered with CTest when `CATKIN_ENABLE_TESTING` is enabled.
It covers missing-model and invalid-gain rejection, exact matrix-exponential
discretization against closed-form constant and harmonic models, the observer
residual sign, printed output-predictor timing, Gamma input-history ordering,
three-axis disturbance compensation, 101-entry packets, zero-fill shifting,
absence of fictitious future measurements, first-input event comparisons,
physical-state-only threshold norms, and event timing after failed reception.
Actual input history is checked with inputs different from the commands returned
by the core, as occurs when an adapter saturates a command.

The constant and harmonic exosystems and full gains in this test are newly
selected analytical fixtures. They are NOT claimed as the missing UAV
experimental parameters and are never loaded by the production ROS launch.
The production YAML deliberately leaves A_w, C_w, and the complete augmented
gain empty until their actual values are provided.

These checks exercise the same mathematical core as the ROS adapter. They do
not establish closed-loop flight performance, reproduce manuscript figures,
validate ROS timer scheduling or the full theoretical inequalities, or verify
the date/version of an original trial executable. The printed discrete output
predictor has an indexing ambiguity; the core follows its printed formula and
documents that convention rather than silently changing the manuscript.
