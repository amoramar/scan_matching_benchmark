cmake_minimum_required(VERSION 3.5)
project(scan_matching_benchmark)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")

## CMake module to locate external packages via pkg-config. We use this where the external
## package does not provide cmake support.
find_package(PkgConfig REQUIRED)

# Add Ceres package.
find_package(Ceres REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

## Find PCL, Eigen and Lapack packages
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
add_definitions(-DPCL_NO_PRECOMPILE)

FIND_PACKAGE( LAPACK )
if(NOT LAPACK_FOUND)
	unset(LAPACK_LIBRARIES CACHE)
	ADD_DEFINITIONS(-DNO_LAPACK_FLAG)
endif()


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if your package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need

## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
 INCLUDE_DIRS include
 LIBRARIES scan_matching_benchmark
 CATKIN_DEPENDS roscpp
 DEPENDS EIGEN3
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(${PROJECT_SOURCE_DIR}/include
	${PCL_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
	${CERES_INCLUDE_DIRS}
	${catkin_INCLUDE_DIRS}
)


# imu_filter_node --- Implementation of a median filter of adjustable window
# 										size as a ROS node.
add_executable(compile_test
  src/compile_test.cpp
)

target_link_libraries(compile_test
	${catkin_LIBRARIES}
	${Boost_LIBRARIES}
)

## Since all code in the project is defined in header files (.h, .hpp),
## NO library is generated (.o). However, since ROS requires both includes
## and a library to link, generate a "dummy" library with a "dummy" .cpp
## ('main' with no functionality).
add_library(${PROJECT_NAME}
	src/compile_test.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
