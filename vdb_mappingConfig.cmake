set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}" ${CMAKE_MODULE_PATH})

include(CMakeFindDependencyMacro)

find_dependency(OpenVDB)
find_dependency(TBB)

include("${CMAKE_CURRENT_LIST_DIR}/vdb_mappingTargets.cmake")
