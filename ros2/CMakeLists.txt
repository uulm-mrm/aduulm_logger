cmake_minimum_required(VERSION 3.16)
project(aduulm_logger)

# Get aduulm cmake macros for coloring, package_version extraction and find_aduulm_package()
find_package(aduulm_cmake_tools REQUIRED)

aduulm_init_ros2_package()

aduulm_create_ros2_package()
setup_git_hooks()
