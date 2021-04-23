cmake_minimum_required(VERSION 3.0.2)
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules" ${CMAKE_MODULE_PATH})

project(vdb_mapping CXX)

if(NOT CMAKE_CONFIGURATION_TYPES AND NOT CMAKE_BUILD_TYPE)
  message("${PROJECT_NAME}: You did not request a specific build type: selecting 'Release'.")
  set(CMAKE_BUILD_TYPE Release)
endif()

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(OpenVDB REQUIRED)
find_package(PCL REQUIRED)
find_package(TBB REQUIRED)

###########
## Build ##
###########

##
## Check C++14 support/ enable global pedantic and Wall
##
include(DefineCXX14CompilerFlag)
DEFINE_CXX_14_COMPILER_FLAG(CXX14_FLAG)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -pedantic")

##
## Collect source format files
##
include(ClangformatMacros)
include(ClangtidyMacros)
add_format_source(${PROJECT_NAME} PATTERN "include/${PROJECT_NAME}/[a-zA-Z]*.h")
add_format_source(${PROJECT_NAME} PATTERN "src/[a-zA-Z]*.h")
add_format_source(${PROJECT_NAME} PATTERN "src/[a-zA-Z]*.cpp")
ADD_CLANGFORMAT_TARGET(${PROJECT_NAME})
ADD_CLANGTIDY_TARGET(${PROJECT_NAME})

## Declare a C++ library
add_library(${PROJECT_NAME} INTERFACE
  #src/VDBMapping.h 
  )

target_compile_options(${PROJECT_NAME} INTERFACE ${CXX14_FLAG})

#target_include_directories(${PROJECT_NAME}
  #PRIVATE
    #${CMAKE_CURRENT_SOURCE_DIR}/src
    #)
target_include_directories(${PROJECT_NAME}
  INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${EIGEN3_INCLUDE_DIR}
    ${OpenVDB_INCLUDE_DIRS}
    ${PCL_INCLUDE_DIRS}
    )

target_link_libraries(${PROJECT_NAME}
  INTERFACE
    ${OpenVDB_LIBRARIES}
    ${PCL_LIBRARIES}
    )

add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}_targets
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
    ${PROJECT_NAME}Config.cmake ${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake
    CMakeModules/FindOpenVDB.cmake
    CMakeModules/FindTBB.cmake
  DESTINATION
    lib/cmake/${PROJECT_NAME}
  )
