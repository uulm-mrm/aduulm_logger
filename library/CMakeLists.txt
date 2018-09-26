cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)
project(aduulm_logger)

## status logging
message("${ColourBoldBlue}${PROJECT_NAME}::library${ColourReset}")

## options for compiler
#add_compile_options(-std=c++11 -Wall -Wextra -Wno-unused-variable -Wno-unused-parameter)

## Specify variables containing the src files of the different targets
set(sources
  library/src/aduulm_logger.cpp
)

set(headers
  library/include/aduulm_logger/aduulm_logger.hpp
)
  
## Declare a C++ library
add_library(${PROJECT_NAME} STATIC
  ${sources}
  ${headers}
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
target_include_directories(${PROJECT_NAME}
  PUBLIC library/include
  ${catkin_INCLUDE_DIRS}
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

