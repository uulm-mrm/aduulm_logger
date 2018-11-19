message("${ColourBoldBlue}${package_name}(v. ${package_version})::${PROJECT_NAME}::ros ${ColourReset}")

find_package(catkin REQUIRED
  roscpp
)

catkin_package(
  INCLUDE_DIRS
    library/include
  LIBRARIES
    aduulm_logger_lib
  CFG_EXTRAS
    proj-extras.cmake
)

set(HOOKS_PATH "${CMAKE_BINARY_DIR}/../../root/git_hooks")
if(EXISTS "${HOOKS_PATH}")
  add_subdirectory("${HOOKS_PATH}" "${CMAKE_CURRENT_BINARY_DIR}/.hooks")
else()
  message(WARNING "Could not find git hooks. Git hooks are not active.")
endif()
