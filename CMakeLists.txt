cmake_minimum_required(VERSION 3.0.2)
project(vdb_mapping)

## Compile as C++11, supported in ROS Kinetic and newer
# TODO possible alternatives are, checkout which suits the case best
# add_definitions
# set(CMAKE_CXX_FLAGS)
add_compile_options(-std=c++14 -Wall -pedantic)



##TODO should release by always enforced?
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")


## System dependencies are found with CMake's conventions
find_package(OpenVDB REQUIRED)
find_package(TBB REQUIRED)


###########
## Build ##
###########

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/VDBMapping.cpp
)
target_compile_options(${PROJECT_NAME} 
  PUBLIC 
    ${CXX14_FLAG}
    )
target_include_directories(${PROJECT_NAME}
  PRIVATE 
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${OpenVDB_INCLUDE_DIR}
    )

target_link_libraries(${PROJECT_NAME}
 ${OpenVDB_LIBRARIES}
 ${TBB_LIBRARIES}
 -lHalf
)

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME} EXPORT ${PROJECT_NAME}_targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  EXPORT ${PROJECT_NAME}_targets
  DESTINATION lib/cmake/${PROJECT_NAME}
  FILE ${PROJECT_NAME}Targets.cmake
  NAMESPACE ${PROJECT_NAME}::
  )

## Mark cpp header files for installation
 install(
   DIRECTORY include/${PROJECT_NAME}/
   DESTINATION include/${PROJECT_NAME}
   FILES_MATCHING 
   PATTERN "*.h"
   PATTERN "*.hpp"
 )

include(CMakePackageConfigHelpers)
write_basic_package_version_file(${PROJECT_NAME}ConfigVersion.cmake VERSION 1.0.0
  COMPATIBILITY SameMajorVersion)
install(
  FILES
    cmake/FindOpenVDB.cmake
    cmake/FindTBB.cmake
  DESTINATION
    lib/cmake/vdb_mapping
  )
install(
  FILES ${PROJECT_NAME}Config.cmake ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
  DESTINATION lib/cmake/${PROJECT_NAME}
  )


## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_vdb_mapping.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
