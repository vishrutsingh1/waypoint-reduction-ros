cmake_minimum_required(VERSION 3.0.2)
project(waypoint_navigation)

add_compile_options(-std=c++14 -Wall -Wextra)

find_package(catkin REQUIRED COMPONENTS
  roscpp nav_msgs geometry_msgs visualization_msgs tf2 tf2_geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES waypoint_reducer
  CATKIN_DEPENDS roscpp nav_msgs geometry_msgs visualization_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

# ROS-free core library
add_library(waypoint_reducer src/waypoint_reducer.cpp)

# ROS service node
add_executable(waypoint_server src/waypoint_server.cpp)
target_link_libraries(waypoint_server waypoint_reducer ${catkin_LIBRARIES})

install(TARGETS waypoint_reducer waypoint_server
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/ DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(test_waypoint_reducer test/test_waypoint_reducer.cpp)
  target_link_libraries(test_waypoint_reducer waypoint_reducer)
endif()
