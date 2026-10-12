cmake_minimum_required(VERSION 3.10)
project(px4ctrl)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  sensor_msgs
  mavros_msgs
  quadrotor_msgs
  uav_utils
)
find_package(Eigen3 REQUIRED NO_MODULE)

catkin_package()

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)
target_compile_features(px4ctrl_node PRIVATE cxx_std_11)
target_compile_options(px4ctrl_node PRIVATE -Wall)
target_include_directories(px4ctrl_node PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(px4ctrl_node ${catkin_LIBRARIES} Eigen3::Eigen)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS px4ctrl_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
catkin_install_python(PROGRAMS thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
