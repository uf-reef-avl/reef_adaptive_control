cmake_minimum_required(VERSION 2.8.3)
project(reef_adaptive_control)

set(CMAKE_CXX_FLAGS "-std=c++0x")

find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  dynamic_reconfigure
  geometry_msgs
  reef_msgs
  roscpp
  rosflight_msgs
  rospy
)

find_package(Eigen REQUIRED)
find_package(PkgConfig REQUIRED)

generate_dynamic_reconfigure_options(
        cfg/Gains.cfg
        cfg/Adaptive_Gains.cfg
)

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})


add_executable(pid_control_node      src/pid_node.cpp
        src/controller.cpp          include/controller.h
        src/PID.cpp                 include/PID.h
        src/simple_pid.cpp          include/simple_pid.h
        )

add_executable(adaptive_node        src/adaptive_node.cpp
        include/state_space.h       include/adaptive_control.h
        include/adaptive_io.h
        )

target_link_libraries(pid_control_node ${catkin_LIBRARIES})

target_link_libraries(adaptive_node ${catkin_LIBRARIES})
add_dependencies(adaptive_node ${PROJECT_NAME}_gencfg)

add_dependencies(pid_control_node ${PROJECT_NAME}_gencfg)
