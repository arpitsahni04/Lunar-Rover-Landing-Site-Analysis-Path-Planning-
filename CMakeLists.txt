cmake_minimum_required(VERSION 3.13)

project(sitesearch)
set(CMAKE_CXX_STANDARD 20)
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "C:/Users/mukul/source/repos/FanFeast/Engineering_Computation/ext/Eigen3/cmake" )
set( CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "C:/Users/mukul/source/repos/FanFeast/Engineering_Computation/cmake" )
set( EMBREE_LOCATION "C:/Users/mukul/source/repos/FanFeast/Engineering_Computation/ext/embree 3.0")
set( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/Users/mukul/source/repos/FanFeast/Engineering_Computation/ext/Eigen3" )
set( CMAKE_PREFIX_PATH ${CMAKE_PREFIX_PATH} "C:/Users/mukul/source/repos/FanFeast/Engineering_Computation/ext/embree 3.0/lib/cmake/embree-3.13.5" )

find_package(Eigen3)

find_package(embree 3.0 REQUIRED)

find_package(OpenMP REQUIRED)

if(NOT TARGET OpenMP::OpenMP_CXX)
    find_package(Threads REQUIRED)
    add_library(OpenMP::OpenMP_CXX IMPORTED INTERFACE)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_COMPILE_OPTIONS ${OpenMP_CXX_FLAGS})
    # Only works if the same flag is passed to the linker; use CMake 3.9+ otherwise (Intel, AppleClang)
    set_property(TARGET OpenMP::OpenMP_CXX
                 PROPERTY INTERFACE_LINK_LIBRARIES ${OpenMP_CXX_FLAGS} Threads::Threads)

endif()

MESSAGE ("Source path: ${PROJECT_SOURCE_DIR}")

#add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp "ext/pixel2latlon.h")
add_executable(${PROJECT_NAME} ${PROJECT_SOURCE_DIR}/src/main.cpp ) #"ext/A_star.h"
target_include_directories(${PROJECT_NAME} PUBLIC src/ ext/ ${EIGEN3_INCLUDE_DIR} ${EMBREE_INCLUDE_DIR} )
target_link_libraries(${PROJECT_NAME} ${EMBREE_LIBRARY} OpenMP::OpenMP_CXX)