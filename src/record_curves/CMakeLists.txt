cmake_minimum_required(VERSION 3.10)
project(record_curves)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)
catkin_package(CATKIN_DEPENDS roscpp std_msgs)

add_executable(record_curves src/record_y_curves.cpp)
target_compile_features(record_curves PRIVATE cxx_std_11)
target_include_directories(record_curves PRIVATE ${catkin_INCLUDE_DIRS})
target_link_libraries(record_curves ${catkin_LIBRARIES})
if(catkin_EXPORTED_TARGETS)
  add_dependencies(record_curves ${catkin_EXPORTED_TARGETS})
endif()

install(TARGETS record_curves
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
