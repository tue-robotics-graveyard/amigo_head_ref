cmake_minimum_required(VERSION 2.8.3)
project(amigo_head_ref)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  roscpp
  tf
  tf_conversions
  urdf
  visualization_msgs
  message_generation
)

add_action_files(
  FILES
  HeadRef.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    visualization_msgs
)

catkin_package(
#  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib actionlib_msgs geometry_msgs roscpp tf tf_conversions urdf visualization_msgs message_runtime
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(amigo_head_ref src/head_ref.cpp)
add_executable(dummy_action_client src/dummy_action_client.cpp)
add_executable(head_ref_test_client src/test_client.cpp)

target_link_libraries(amigo_head_ref ${catkin_LIBRARIES})
target_link_libraries(dummy_action_client ${catkin_LIBRARIES})
target_link_libraries(head_ref_test_client ${catkin_LIBRARIES})

add_dependencies(amigo_head_ref ${catkin_EXPORTED_TARGETS})
add_dependencies(dummy_action_client ${catkin_EXPORTED_TARGETS})
add_dependencies(head_ref_test_client ${catkin_EXPORTED_TARGETS})
