cmake_minimum_required(VERSION 3.10)
project(visual_target)

find_package(catkin REQUIRED COMPONENTS roscpp sensor_msgs std_msgs cv_bridge)
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED CONFIG)
find_package(VISP REQUIRED)

catkin_package(CATKIN_DEPENDS roscpp sensor_msgs std_msgs cv_bridge)

add_executable(visual_target_node src/visual_target_node.cpp)
target_compile_features(visual_target_node PRIVATE cxx_std_11)
target_include_directories(visual_target_node PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${VISP_INCLUDE_DIRS}
)
target_link_libraries(visual_target_node
  ${catkin_LIBRARIES}
  Eigen3::Eigen
  ${OpenCV_LIBRARIES}
  ${VISP_LIBRARIES}
  realsense2::realsense2
)
if(catkin_EXPORTED_TARGETS)
  add_dependencies(visual_target_node ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS visual_target_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
