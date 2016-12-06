cmake_minimum_required(VERSION 2.8.8)

# Project name
project(SVO)

# Include our cmake files
SET(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmakeModules/)

# Include libraries
find_package(Eigen3 REQUIRED)
find_package(G2O REQUIRED)
find_package(SuiteSparse REQUIRED)
find_package(GTSAM REQUIRED)
find_package(PCL 1.8 REQUIRED)
find_package(Boost REQUIRED COMPONENTS system serialization system filesystem thread program_options date_time regex timer chrono)
find_package(OpenCV 3 REQUIRED core imgcodecs videoio ximgproc)
find_package(GeographicLib 1.34 REQUIRED)
find_package(Pangolin REQUIRED)
find_package(X11 REQUIRED)

# Try to compile with c++11
# http://stackoverflow.com/a/25836953
include(CheckCXXCompilerFlag)
CHECK_CXX_COMPILER_FLAG("-std=c++11" COMPILER_SUPPORTS_CXX11)
CHECK_CXX_COMPILER_FLAG("-std=c++0x" COMPILER_SUPPORTS_CXX0X)
if(COMPILER_SUPPORTS_CXX11)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
elseif(COMPILER_SUPPORTS_CXX0X)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++0x")
else()
    message(STATUS "The compiler ${CMAKE_CXX_COMPILER} has no C++11 support. Please use a different C++ compiler.")
endif()

# Enable compile optimizations
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

# Enable debug flags
#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g3  -Wall")


# SSE3 is needed by libviso2 (its a cpu arch attribute)
# If we are on ARM we need to find an alternative
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -msse -msse2 -msse3")


# Include our header files
include_directories(
        ${Pangolin_INCLUDE_DIR}
        ${X11_INCLUDE_DIRS}
        src
        thirdParty
        ${EIGEN3_INCLUDE_DIR}
        ${GTSAM_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        ${Boost_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
        ${GeographicLib_INCLUDE_DIRS}
        ${G2O_INCLUDE_DIRS}
        ${CSPARSE_INCLUDE_DIR} #Has been set by SuiteParse
        ${CHOLMOD_INCLUDE_DIR} #Has been set by SuiteParse
)

# Include all thirdparty source files in build
file(GLOB_RECURSE sources_thirdParty thirdParty/*.cpp)

#####################################
# Main class files
#####################################
set(sources_SVO
    src/SVO/stereo.cpp
    src/SVO/drawer.cpp
)


# Create the main executable, and link libraries
add_executable(main_SVO_gt
        src/main_SVO_gt.cpp
        ${sources_SVO}
        ${sources_thirdParty}
)

target_link_libraries(main_SVO_gt
        X11
        ${Pangolin_LIBRARY}
        gtsam
        tbb
        tbbmalloc
        png
        GL
        ${GTSAM_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GeographicLib_LIBRARIES}
)


# Test main for running GTSAM code
add_executable(main_gtsam 
	           src/main_gtsam.cpp)
target_link_libraries(main_gtsam
        gtsam
        tbb
        tbbmalloc
        ${GTSAM_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GeographicLib_LIBRARIES}
        )

# Test main for running libviso2 code
add_executable(main_libviso2
               src/main_libviso2.cpp
               ${sources_SVO}
               ${sources_thirdParty} )

target_link_libraries(main_libviso2
        X11
        ${Pangolin_LIBRARIES}
        gtsam
        tbb
        tbbmalloc
        png
        GL
        ${GTSAM_LIBS}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GeographicLib_LIBRARIES}
        )

# Test main for running g2o code
add_executable(main_g2o
        src/main_g2o.cpp
)

target_link_libraries(main_g2o
        csparse
        cxsparse
        ${G2O_LIBRARIES}
        ${PCL_LIBRARIES}
        ${Boost_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${GeographicLib_LIBRARIES}
        )
