message("${ColourBoldBlue}${package_name}(v. ${package_version})::${PROJECT_NAME}::ros ${ColourReset}")

add_definitions(-DIS_ROS)

find_package(catkin REQUIRED
  roscpp
)
  
catkin_package(
  INCLUDE_DIRS library/include
  LIBRARIES ${PROJECT_NAME}
)
