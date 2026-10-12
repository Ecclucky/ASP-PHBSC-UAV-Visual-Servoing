# ASP-PHBSC UAV Visual Servoing

[中文说明](README_zh.md)

UAV visual-servoing experiment code for the ASP-PHBSC algorithm, built as a ROS 1 / catkin workspace.

The workspace includes the controller, visual target tracking, PX4/MAVROS interfaces, motion-capture logging, and CSV recording.

**Configuration required:** `A_w`, `C_w`, and the full observer gain `L` are currently unset; the controller refuses to start until these parameters are configured.

## Environment

Ubuntu 20.04 with ROS Noetic. Dependencies include Eigen3, OpenCV, librealsense2, ViSP with AprilTag support, VRPN, MAVROS, and `cv_bridge`; see each package's `package.xml` for ROS dependencies.

## Build and run

From the workspace root:

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Configure the model and platform settings, then launch either experiment:

```bash
bash scripts/run_experiment.sh case1
bash scripts/run_experiment.sh case2
```

These commands start the controller and recorder. Hardware drivers are optional through `hardware:=true` and require the appropriate connection settings. Synthetic DoS injection is disabled by default and can be enabled with `synthetic_dos:=true`.

## Files

- Controller: [`src/asp_phbsc`](src/asp_phbsc)
- Shared parameters: [`common.yaml`](src/asp_phbsc/config/common.yaml)
- Experiment parameters: [`case1.yaml`](src/asp_phbsc/config/case1.yaml), [`case2.yaml`](src/asp_phbsc/config/case2.yaml)
- CSV fields: [data format](docs/DATA_FORMAT.md)

## Offline checks

The configuration checker requires Python 3 and PyYAML. The standalone C++ test requires Eigen3.

```bash
python3 scripts/validate_workspace.py
g++ -std=c++11 -O2 -Isrc/asp_phbsc/include -I/usr/include/eigen3 \
  src/asp_phbsc/tests/core_test.cpp -o /tmp/asp_phbsc_core_test
/tmp/asp_phbsc_core_test
```

Static checks and standalone core tests have passed locally; full ROS integration and flight operation have not been validated for this release.

Third-party license declarations are retained. See [third-party notices](THIRD_PARTY_NOTICES.md).
