cmake_minimum_required(VERSION 3.3 FATAL_ERROR)
project(aduulm_logger_lib)

# Get aduulm cmake macros for coloring, package_version extraction and find_aduulm_package()
find_package(aduulm_cmake_tools REQUIRED)
get_version_string_from_package_xml(.)
show_build_info("library")
setup_git_hooks()

find_package(Boost REQUIRED)

# install python code
# PYTHON_INSTALL_DIR is defined by aduulm_cmake_tools
install(DIRECTORY ${PROJECT_SOURCE_DIR}/python/aduulm_logger_python/
        DESTINATION ${PYTHON_INSTALL_DIR}/aduulm_logger_python)

set(INCLUDE_INSTALL_DIR include/ CACHE STRING "Path to installed include directory")
set(LIB_INSTALL_DIR lib/ CACHE STRING "Path to installed lib directory")
set(BIN_INSTALL_DIR bin/ CACHE STRING "Path to installed bin directory")

# TARGETS

## Declare a C++ library
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

## Specify additional locations of header files
## Your package locations should be listed before other locations
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
)
target_include_directories(${PROJECT_NAME}
  SYSTEM INTERFACE
    ${Boost_INCLUDE_DIRS}
)
target_compile_features(${PROJECT_NAME} INTERFACE cxx_std_11)

target_link_libraries(${PROJECT_NAME} INTERFACE ${Boost_LIBRARIES})

set(EXTRA_PACKAGES_USED FALSE)
set(EXTRA_PACKAGES "")
if(IS_ROS)
  find_package(rclcpp QUIET)
  if(rclcpp_FOUND)
    # ROS2
    target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${rclcpp_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} INTERFACE ${rclcpp_LIBRARIES})
    target_compile_definitions(${PROJECT_NAME} INTERFACE -DIS_ROS2=1)
    set(EXTRA_INCLUDE_DIRS ${rclcpp_INCLUDE_DIRS})
    set(EXTRA_LIBRARIES ${rclcpp_LIBRARIES})
    set(EXTRA_PACKAGES_USED TRUE)
    set(EXTRA_PACKAGES rclcpp)
  else()
    # ROS1
    find_package(catkin REQUIRED COMPONENTS roscpp)
    target_include_directories(${PROJECT_NAME} SYSTEM INTERFACE ${catkin_INCLUDE_DIRS})
    target_link_libraries(${PROJECT_NAME} INTERFACE ${catkin_LIBRARIES})
    set(EXTRA_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
    set(EXTRA_LIBRARIES ${catkin_LIBRARIES})
  endif()
else()
  set(EXTRA_INCLUDE_DIRS "")
  set(EXTRA_LIBRARIES "")
endif()

# INSTALLATION

set(PACKAGE_LIBRARY_VERSION "${package_version}")

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
  VERSION "${PACKAGE_LIBRARY_VERSION}"
  COMPATIBILITY AnyNewerVersion
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  INCLUDES DESTINATION ${INCLUDE_INSTALL_DIR}
  LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT Runtime
  ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT Development
  RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT Runtime
  PUBLIC_HEADER DESTINATION ${INCLUDE_INSTALL_DIR} COMPONENT Development
  BUNDLE DESTINATION ${BIN_INSTALL_DIR} COMPONENT Runtime
)

include(CMakePackageConfigHelpers)
configure_package_config_file(
  "${PROJECT_SOURCE_DIR}/config.cmake.in"
  "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
  INSTALL_DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR BIN_INSTALL_DIR EXTRA_INCLUDE_DIRS EXTRA_LIBRARIES EXTRA_PACKAGES_USED EXTRA_PACKAGES
)

install(EXPORT ${PROJECT_NAME}Targets
  DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME}
  NAMESPACE ${PROJECT_NAME}::
)
# install CMake config and version file
install(FILES "${PROJECT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
              "${PROJECT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
        DESTINATION ${LIB_INSTALL_DIR}/cmake/${PROJECT_NAME})
# install header files
install(DIRECTORY ${PROJECT_SOURCE_DIR}/include/ DESTINATION ${INCLUDE_INSTALL_DIR})
