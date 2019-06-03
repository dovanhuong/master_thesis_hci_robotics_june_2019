cmake_minimum_required(VERSION 2.8.3)
project(modman_comm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  modman_msgs
)

add_message_files(
  FILES
  modman_state.msg
)

generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
add_executable(server src/server.cpp)
add_executable(client src/client.cpp)
add_executable(modman_comm src/modman_comm.cpp)
add_executable(ur_comm src/ur_comm.cpp)
add_executable(ur_test src/ur_test.cpp)

target_link_libraries(server
   ${catkin_LIBRARIES}
)
target_link_libraries(client
   ${catkin_LIBRARIES}
)
target_link_libraries(modman_comm
   ${catkin_LIBRARIES}
)
target_link_libraries(ur_comm
   ${catkin_LIBRARIES}
)
target_link_libraries(ur_test
   ${catkin_LIBRARIES}
)

#common commands for building c++ executables and libraries
#rosbuild_add_library(${PROJECT_NAME} src/example.cpp)
#target_link_libraries(${PROJECT_NAME} another_library)
#rosbuild_add_boost_directories()
#rosbuild_link_boost(${PROJECT_NAME} thread)
#rosbuild_add_executable(example examples/example.cpp)
#target_link_libraries(example ${PROJECT_NAME})

