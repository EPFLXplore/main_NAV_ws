cmake_minimum_required(VERSION 3.0.2)
project(controller_motors)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  message_filters
  message_generation
)


add_message_files(
  FILES
  WheelVelocity.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
)

 
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sync
  CATKIN_DEPENDS rospy roscpp message_runtime
#  DEPENDS system_lib
)


include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

message(STATUS "DIR : --------->  ${PROJECT_SOURCE_DIR}")


add_executable(controller_motors src/controller_motors_test.cpp)

target_link_libraries(controller_motors ${catkin_LIBRARIES} 
                      ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-core_ethercatcpp-core.so 
                      ${PROJECT_SOURCE_DIR}/lib/libethercatcpp-epos_ethercatcpp-epos.so 
                      ${PROJECT_SOURCE_DIR}/lib/libpid-os-utilities_pid-signal-manager.so 
                      ${PROJECT_SOURCE_DIR}/lib/libsoem.a )

add_dependencies(controller_motors controller_motors_generate_messages_cpp)

