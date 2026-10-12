cmake_minimum_required(VERSION 3.10)
project(uav_utils)

find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs nav_msgs)
find_package(Eigen3 REQUIRED NO_MODULE)

# This package exports headers; it does not define a uav_utils library.
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs
  DEPENDS EIGEN3
)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS})

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(${PROJECT_NAME}-test src/${PROJECT_NAME}_test.cpp)
  if(TARGET ${PROJECT_NAME}-test)
    target_compile_features(${PROJECT_NAME}-test PRIVATE cxx_std_11)
    target_link_libraries(${PROJECT_NAME}-test ${catkin_LIBRARIES} Eigen3::Eigen)
    if(catkin_EXPORTED_TARGETS)
      add_dependencies(${PROJECT_NAME}-test ${catkin_EXPORTED_TARGETS})
    endif()
  endif()
endif()
