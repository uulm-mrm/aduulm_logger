cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)
project(aduulm_logger_lib)

## status logging
message("${ColourBoldBlue}${PROJECT_NAME}::library${ColourReset}")

## options for compiler
#add_compile_options(-std=c++11 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter)

## Specify variables containing the src files of the different targets
set(sources
  src/aduulm_logger.cpp
)

set(headers
  include/aduulm_logger/aduulm_logger.hpp
)
  

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
#  library/include/${PROJECT_NAME}
  ${catkin_INCLUDE_DIRS}
#  ${Boost_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
  ${sources}
)

# Install things at the end
if(IS_ADTF)
  #add_dependencies(${PROJECT_NAME} ${TARGET_DEPENDENCIES} )
  #target_link_libraries(${PROJECT_NAME} PUBLIC ${TARGET_DEPENDENCIES})
  install(TARGETS ${PROJECT_NAME} DESTINATION lib)
  if (${INSTALL_HEADERS})
	install(DIRECTORY include/ DESTINATION include
	  FILES_MATCHING PATTERN "*.h")
  endif()
elseif(IS_ROS)

else()
  message("${ColourYellow}Library is not installed since no framework (ROS/ADTF) is given.${ColourReset}")
endif()

