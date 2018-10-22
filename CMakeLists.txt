cmake_minimum_required(VERSION 2.8.3)
project(aduulm_logger)

# colour support
if (NOT WIN32)
  string(ASCII 27 Esc)
  set(ColourReset "${Esc}[0m")
  set(ColourBold  "${Esc}[1m")
  set(ColourRed         "${Esc}[31m")
  set(ColourGreen       "${Esc}[32m")
  set(ColourYellow      "${Esc}[33m")
  set(ColourBlue        "${Esc}[34m")
  set(ColourMagenta     "${Esc}[35m")
  set(ColourCyan        "${Esc}[36m")
  set(ColourWhite       "${Esc}[37m")
  set(ColourBoldRed     "${Esc}[1;31m")
  set(ColourBoldGreen   "${Esc}[1;32m")
  set(ColourBoldYellow  "${Esc}[1;33m")
  set(ColourBoldBlue    "${Esc}[1;34m")
  set(ColourBoldMagenta "${Esc}[1;35m")
  set(ColourBoldCyan    "${Esc}[1;36m")
  set(ColourBoldWhite   "${Esc}[1;37m")
endif()

# Get version string to c++
execute_process(COMMAND sh -c "grep \"<name>\" package.xml | sed -e 's+ *<name>\\(.*\\)</name>+\\1+'"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE package_name
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

execute_process(COMMAND sh -c "grep \"<version>\" package.xml | sed -e 's+ *<version>\\(.*\\)</version>+\\1+'"
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
  OUTPUT_VARIABLE package_version
  OUTPUT_STRIP_TRAILING_WHITESPACE
)

add_definitions("-D${package_name}_v=\"${package_version}\"")
message("${ColourBoldBlue}${package_name}(v. ${package_version})::${PROJECT_NAME} ${ColourReset}")

if (IS_ROS)
  # ROS node
  include(./ros/CMakeLists.txt)
  
  set(HOOKS_PATH "${CMAKE_BINARY_DIR}/../../root/git_hooks")
  if(EXISTS "${HOOKS_PATH}")
    add_subdirectory("${HOOKS_PATH}" "${CMAKE_CURRENT_BINARY_DIR}/.hooks")
  else()
    message(WARNING "Could not find git hooks. Git hooks are not active.")
  endif()

elseif(IS_ADTF)
  # ADTF Node

endif()

# Library (in this case needs to be built last!
include(./library/CMakeLists.txt)

#message("${catkin_INCLUDE_DIRS}")
#message("${catkin_LIBRARIES}")
