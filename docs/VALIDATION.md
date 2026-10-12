# Validation record

Preparation date: 2026-10-12. These checks concern the reconstructed source tree, not the historical experiment results.

## Completed locally

- Parsed the actual common, case, and demonstration YAML files and checked the Section VI parameter values, prediction-horizon bound, and consecutive attack-window bounds.
- Checked all eight package manifests, CMake target/source declarations, internal dependencies, launch file references, runtime dependencies, and shared visual measurement configuration.
- Checked generated message declarations and source header references; checked for stale package references and generated/local files in the release tree.
- Ran `bash -n` separately on both shell launch helpers using Git Bash. This parses scripts without executing them.
- Compared all 395 files in the original source directory against the pre-edit SHA-256 inventory: unchanged. The complete backup ZIP passed its integrity check. Archived files also matched the inventory.
- The earlier parameter-preparation stage preserved the PX4 control source. This equation-alignment stage intentionally changes thrust-attitude selection and adds an IMU validity gate, along with the documented launch remapping. Existing message definitions and license files remain unchanged.
- Confirmed that the strict configuration check rejects the intentionally missing experimental augmented model; no test fixture is silently loaded for runtime.

## C++ core check

The standalone test calls the actual controller header, without ROS or hardware. It covers missing/invalid parameters, matrix-exponential discretization against hand-derived constant/harmonic models, observer/predictor timing, the chronological actual-input history, disturbance compensation, 101-input zero-filled buffers, absence of imaginary future measurements, first-input joint event norms, and failed-attempt event-clock updates.

**The actual C++ test compiled and ran successfully**, with both exit codes equal to zero, using portable Zig 0.16.0 in C++11 mode and Eigen 3.4.1 on Windows. The compiler flag -mcpu=baseline avoids a host-specific Eigen AVX512 compilation issue. The compiler, libraries, executable and logs are outside the release tree. The initial incomplete compiler download from the earlier preparation stage was replaced with a complete checksum-verified package.

Test output:

~~~text
ASP-PHBSC mathematical core tests passed; models are synthetic fixtures, no ROS/hardware.
~~~

The constant and harmonic models in the tests were selected for verification, not recovered from the paper's flight experiments. Production model fields remain empty.

To run in a Linux environment with Eigen installed:

```bash
python3 scripts/validate_workspace.py
bash -n scripts/run_experiment.sh
bash -n scripts/request_mavlink_streams.sh
g++ -std=c++11 -O2 -Wall -Wextra -pedantic \
  -Isrc/asp_phbsc/include -I/usr/include/eigen3 \
  src/asp_phbsc/tests/core_test.cpp -o /tmp/asp_phbsc_core_test
/tmp/asp_phbsc_core_test
```

The Python checker requires PyYAML and verifies the shipped manuscript configuration; deliberately retuned experiments will need a corresponding update to its expected values. Its normal source-structure mode reports the pending model explicitly. Run with --require-model to additionally require all experimental model fields; this is expected to fail until they are supplied.

## Not verified

- No full catkin/ROS build was performed on the Windows preparation machine.
- No camera, motion-capture server, MAVROS connection, motor, fan, or aircraft was operated.
- No scheduling, acquisition-delay, real-network packet-loss, or closed-loop flight-performance test was performed.
- No reproduction of manuscript figures, RMSE results, or historical attack traces is claimed.
- The supplied GitHub Actions workflow has not been run on GitHub. It runs static checks and the standalone core test; it does not build the full ROS stack.

See [implementation status](IMPLEMENTATION_STATUS.md) for the retained mathematical and deployment differences. A passing software check does not resolve those differences.
