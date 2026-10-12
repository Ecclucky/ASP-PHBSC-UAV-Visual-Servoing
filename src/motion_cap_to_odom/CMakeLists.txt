cmake_minimum_required(VERSION 3.10)
project(motion_cap_to_odom)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs)
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs nav_msgs)

add_executable(motion_cap_to_odom src/motion_cap_to_odom.cpp)
target_compile_features(motion_cap_to_odom PRIVATE cxx_std_11)
target_include_directories(motion_cap_to_odom PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(motion_cap_to_odom ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(motion_cap_to_odom ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS motion_cap_to_odom
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
