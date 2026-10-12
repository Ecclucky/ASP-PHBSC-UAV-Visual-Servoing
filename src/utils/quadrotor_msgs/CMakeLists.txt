cmake_minimum_required(VERSION 3.10)
project(quadrotor_msgs)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)
find_package(Eigen3 REQUIRED NO_MODULE)

add_message_files(FILES
  AuxCommand.msg
  Corrections.msg
  Gains.msg
  GoalSet.msg
  OutputData.msg
  PositionCommand.msg
  PPROutputData.msg
  Serial.msg
  SO3Command.msg
  StatusData.msg
  TRPYCommand.msg
  Odometry.msg
  PolynomialTrajectory.msg
  OptimalTimeAllocator.msg
  TrajectoryMatrix.msg
  ReplanCheck.msg
  SpatialTemporalTrajectory.msg
  Bspline.msg
  SwarmCommand.msg
  Replan.msg
  SwarmOdometry.msg
  SwarmInfo.msg
  Px4ctrlDebug.msg
  TakeoffLand.msg
)

generate_messages(DEPENDENCIES std_msgs geometry_msgs nav_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES encode_msgs decode_msgs
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs message_runtime
  DEPENDS EIGEN3
)

add_library(decode_msgs src/decode_msgs.cpp)
add_library(encode_msgs src/encode_msgs.cpp)
foreach(target decode_msgs encode_msgs)
  target_compile_features(${target} PRIVATE cxx_std_11)
  target_include_directories(${target} PUBLIC include ${catkin_INCLUDE_DIRS})
  target_link_libraries(${target} ${catkin_LIBRARIES} Eigen3::Eigen)
  add_dependencies(${target} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
endforeach()

install(TARGETS encode_msgs decode_msgs
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
