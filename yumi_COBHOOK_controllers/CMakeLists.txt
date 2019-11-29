cmake_minimum_required(VERSION 2.8.3)
project(yumi_test_controllers)

find_package(Armadillo REQUIRED) # <<<
include_directories(${ARMADILLO_INCLUDE_DIRS}) # <<<

set(CMAKE_BUILD_TYPE RelWithDebInfo)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# copy the urdf file to be loaded by the KDL parser
file(COPY ${CMAKE_CURRENT_SOURCE_DIR}/yumi.urdf DESTINATION ${CMAKE_CURRENT_BINARY_DIR})

find_package(catkin REQUIRED COMPONENTS
  angles
  control_msgs
  control_toolbox
  controller_interface
  forward_command_controller
  realtime_tools
  sensor_msgs
#  geometry_msgs
  kdl_parser
)

catkin_package(
  CATKIN_DEPENDS
    angles
    control_msgs
    control_toolbox
    controller_interface
    forward_command_controller
    realtime_tools
    sensor_msgs
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
)

include_directories(include 
  ${Boost_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIRS} 
  ${catkin_INCLUDE_DIRS}
  ${ARMADILLO_INCLUDE_DIRS}
)


#add_executable(test_traj_vel_control src/test_traj_vel_control.cpp)
#target_link_libraries(test_traj_vel_control ${catkin_LIBRARIES})

#add_executable(test_vel_control src/test_vel_control.cpp)
#target_link_libraries(test_vel_control ${catkin_LIBRARIES})

#add_executable(test_IK_control src/test_IK_control.cpp)
#target_link_libraries(test_IK_control ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(sensor_calibration src/sensor_calibration.cpp)
target_link_libraries(sensor_calibration ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(test_MPC_pose src/test_IK_control_MPC_pose.cpp src/pose_MPC.cpp)
target_link_libraries(test_MPC_pose ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})

add_executable(position_controller src/test_pos_control.cpp)
target_link_libraries(position_controller ${catkin_LIBRARIES} ${ARMADILLO_LIBRARIES})
