cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)
project(aduulm_logger)

find_package(catkin REQUIRED
  roscpp
)

# Get aduulm cmake macros for coloring, package_version extraction and find_aduulm_package()
find_package(aduulm_cmake_tools REQUIRED)
get_version_string_from_package_xml(.)
show_build_info("ros")

catkin_package(
  CFG_EXTRAS
    proj-extras.cmake
)

setup_git_hooks()
