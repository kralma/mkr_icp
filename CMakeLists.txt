cmake_minimum_required(VERSION 2.8)
     
PROJECT(IterativeClosestPoint)
 
find_package(VTK REQUIRED)
include(${VTK_USE_FILE})

add_subdirectory(dataLoader)
set(LIBS LaserDataLoader)

add_subdirectory(gui)
set(LIBS ${LIBS} Gui)

add_executable(IterativeClosestPoint icp_main.cpp)
 
target_link_libraries(IterativeClosestPoint ${LIBS} ${VTK_LIBRARIES})
