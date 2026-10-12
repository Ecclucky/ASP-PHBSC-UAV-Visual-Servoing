# ASP-PHBSC UAV Visual Servoing

[English](README.md)

ASP-PHBSC 算法无人机视觉伺服实验代码，采用 ROS 1 / catkin 工程结构，包含控制器、视觉目标跟踪、PX4/MAVROS 接口、动捕记录和 CSV 数据记录。

**当前 `A_w`、`C_w` 和完整观测器增益 `L` 尚待配置，未配置时控制节点会拒绝启动。**

## 环境

Ubuntu 20.04、ROS Noetic，以及 Eigen3、OpenCV、librealsense2、支持 AprilTag 的 ViSP、VRPN、MAVROS 和 `cv_bridge`。其他 ROS 依赖见各功能包的 `package.xml`。

## 编译与运行

在工程根目录执行：

```bash
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

补齐模型参数并配置硬件接口后，选择实验场景：

```bash
bash scripts/run_experiment.sh case1
bash scripts/run_experiment.sh case2
```

默认启动控制和记录节点；启动硬件驱动需添加 `hardware:=true` 并提供对应连接设置。默认关闭 DoS 注入，可通过 `synthetic_dos:=true` 启用演示攻击序列。

## 主要文件

- 算法目录：[`src/asp_phbsc`](src/asp_phbsc)
- 公共参数：[`common.yaml`](src/asp_phbsc/config/common.yaml)
- 场景参数：[`case1.yaml`](src/asp_phbsc/config/case1.yaml)、[`case2.yaml`](src/asp_phbsc/config/case2.yaml)
- 数据说明：[CSV 字段](docs/DATA_FORMAT.md)

## 离线检查

配置检查需要 Python 3 和 PyYAML；独立 C++ 核心测试需要 Eigen3。

```bash
python3 scripts/validate_workspace.py
g++ -std=c++11 -O2 -Isrc/asp_phbsc/include -I/usr/include/eigen3 \
  src/asp_phbsc/tests/core_test.cpp -o /tmp/asp_phbsc_core_test
/tmp/asp_phbsc_core_test
```

已通过本地静态检查和独立核心测试；当前发布版本尚未完成 ROS 整体集成及实飞验证。

第三方许可声明予以保留，见 [第三方声明](THIRD_PARTY_NOTICES.md)。
