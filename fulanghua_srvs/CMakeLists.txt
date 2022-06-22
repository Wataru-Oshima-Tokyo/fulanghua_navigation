cmake_minimum_required(VERSION 2.8.3)
project(fulanghua_srvs)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  orne_waypoints_editor
)

## Generate services in the 'srv' folder
add_service_files(
    FILES
    Pose.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    orne_waypoints_editor
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

