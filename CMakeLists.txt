cmake_minimum_required(VERSION 2.8.3)
project(aduulm_logger)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11 -fPIC)

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

execute_process(COMMAND sh -c "grep \"<version>\" package.xml | sed -e 's+ *<version>\\(.*\\)</version>+\\1+'"
  WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
  OUTPUT_VARIABLE ${PROJECT_NAME}_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)
  add_definitions("-D${PROJECT_NAME}_VERSION=\"${${PROJECT_NAME}_VERSION}\""
)
    #message("${${PROJECT_NAME}_VERSION}")


if (IS_ROS)
  add_definitions(-DIS_ROS)

  find_package(catkin REQUIRED
    roscpp
  )
  
  catkin_package(
    INCLUDE_DIRS library/include
    LIBRARIES ${PROJECT_NAME}
  )
  
  #include_directories(
  #  library/include
  #  ${catkin_INCLUDE_DIRS}
  #)
elseif(IS_ADTF)
  
endif()


include(./library/CMakeLists.txt)

if(IS_ROS)  
  add_subdirectory("${CMAKE_BINARY_DIR}/../../root/git_hooks" "${CMAKE_CURRENT_BINARY_DIR}/.hooks")
endif()


  message("${catkin_INCLUDE_DIRS}")
  message("${catkin_LIBRARIES}")
