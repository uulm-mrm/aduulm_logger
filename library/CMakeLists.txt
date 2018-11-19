cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)
project(aduulm_logger_lib)

## status logging
message("${ColourBoldBlue}${package_name}(v. ${package_version})::${PROJECT_NAME}::library ${ColourReset}")

# we default to Release build type
if(NOT CMAKE_BUILD_TYPE)
	set(CMAKE_BUILD_TYPE "Release")
endif()

set(INCLUDE_INSTALL_DIR include/ CACHE STRING "Path to installed include directory")
set(LIB_INSTALL_DIR lib/ CACHE STRING "Path to installed lib directory")
set(BIN_INSTALL_DIR bin/ CACHE STRING "Path to installed bin directory")

# TARGETS
  
## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/aduulm_logger.cpp
)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

## Specify additional locations of header files
## Your package locations should be listed before other locations
target_include_directories(${PROJECT_NAME}
  PUBLIC 
    $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
)

if(IS_ROS)
  find_package(catkin REQUIRED COMPONENTS roscpp)
  target_include_directories(${PROJECT_NAME} PUBLIC ${catkin_INCLUDE_DIRS})
  target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})
  set(EXTRA_INCLUDE_DIRS ${catkin_INCLUDE_DIRS})
  set(EXTRA_LIBRARIES ${catkin_LIBRARIES})
else()
  set(EXTRA_INCLUDE_DIRS "")
  set(EXTRA_LIBRARIES "")
endif()

target_compile_options(${PROJECT_NAME} PRIVATE "-std=c++14")

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
  PATH_VARS INCLUDE_INSTALL_DIR LIB_INSTALL_DIR BIN_INSTALL_DIR EXTRA_INCLUDE_DIRS EXTRA_LIBRARIES
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
