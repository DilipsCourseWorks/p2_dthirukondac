cmake_minimum_required(VERSION 3.0.2)
project(p2_dthirukondac_rjain5)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  cv_bridge
  image_transport
  actionlib
  move_base_msgs
)

find_package(OpenCV REQUIRED)

catkin_package(
  CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    geometry_msgs
    sensor_msgs
    cv_bridge
    image_transport
    actionlib
    move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# ── Part 2: vision_follower C++ node ──────────────────────────────────────────
add_executable(vision_follower src/vision_follower.cpp)

target_link_libraries(vision_follower
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
)

add_dependencies(vision_follower
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)

# ── Part 1: navigation_node Python script ─────────────────────────────────────
catkin_install_python(PROGRAMS
  script/navigation_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS vision_follower
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
