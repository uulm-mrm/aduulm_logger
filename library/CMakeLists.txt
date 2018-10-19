cmake_minimum_required(VERSION 2.8.9 FATAL_ERROR)

## status logging
message("${ColourBoldBlue}${package_name}(v. ${package_version})::${PROJECT_NAME}::library ${ColourReset}")

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
add_library(${PROJECT_NAME} SHARED
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
  install(TARGETS ${PROJECT_NAME}
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )

# Mark cpp header files for installation
  install(DIRECTORY library/include/${PROJECT_NAME}/
    DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  )
else()
  message("${ColourYellow}Library is not installed since no framework (ROS/ADTF) is given.${ColourReset}")
endif()
