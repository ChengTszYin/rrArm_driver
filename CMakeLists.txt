cmake_minimum_required(VERSION 3.1.3)
project(rArm)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  serial
  control_msgs
  trajectory_msgs
  actionlib 
  actionlib_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES rArm
 CATKIN_DEPENDS roscpp rospy sensor_msgs serial control_msgs trajectory_msgs actionlib actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME}
  src/rr_arm_ser.cpp
)

add_executable(rr_arm_node src/rr_arm_node.cpp)

target_link_libraries(rr_arm_node
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# #   FILES_MATCHING PATTERN "*.h"
# #   PATTERN ".svn" EXCLUDE
# )

install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
