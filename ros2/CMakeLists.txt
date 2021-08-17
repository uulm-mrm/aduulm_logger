cmake_minimum_required(VERSION 3.16)
project(aduulm_logger)

find_package(ament_cmake REQUIRED)

# Get aduulm cmake macros for coloring, package_version extraction and find_aduulm_package()
find_package(aduulm_cmake_tools REQUIRED)
get_version_string_from_package_xml(.)
show_build_info("ros")

find_package(aduulm_logger_lib REQUIRED)

add_library(aduulm_logger INTERFACE)
ament_target_dependencies(aduulm_logger INTERFACE aduulm_logger_lib)

ament_package()
setup_git_hooks()
