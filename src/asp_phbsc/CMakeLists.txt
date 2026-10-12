cmake_minimum_required(VERSION 3.10)
project(asp_phbsc)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  quadrotor_msgs
  xmlrpcpp
)
find_package(Eigen3 REQUIRED)
# Export the include path required by the installed, ROS-independent core.
get_target_property(Eigen3_INCLUDE_DIRS Eigen3::Eigen INTERFACE_INCLUDE_DIRECTORIES)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs quadrotor_msgs xmlrpcpp
  DEPENDS Eigen3
)

add_executable(asp_phbsc_node src/asp_phbsc_node.cpp)
target_compile_features(asp_phbsc_node PRIVATE cxx_std_11)
target_include_directories(asp_phbsc_node PRIVATE include ${catkin_INCLUDE_DIRS})
target_link_libraries(asp_phbsc_node ${catkin_LIBRARIES} Eigen3::Eigen)
add_dependencies(asp_phbsc_node ${catkin_EXPORTED_TARGETS})

if(CATKIN_ENABLE_TESTING)
  add_executable(asp_phbsc_core_test tests/core_test.cpp)
  target_compile_features(asp_phbsc_core_test PRIVATE cxx_std_11)
  target_include_directories(asp_phbsc_core_test PRIVATE include)
  target_link_libraries(asp_phbsc_core_test Eigen3::Eigen)
  add_test(NAME asp_phbsc_core_test COMMAND $<TARGET_FILE:asp_phbsc_core_test>)
endif()

install(TARGETS asp_phbsc_node
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
