cmake_minimum_required(VERSION 3.25)
project(TrajectoryAlign)

set(CMAKE_CXX_STANDARD 17)

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(Gflags REQUIRED)
include_directories(${GFLAGS_INCLUDE_DIRS})

find_package(Glog REQUIRED)
include_directories(${GLOG_INCLUDE_DIRS})

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})

find_package(Ceres REQUIRED)
include_directories(${Ceres_INCLUDE_DIRS})

FIND_PACKAGE( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

set(THIRTY_LIBS
        ${GFLAGS_LIBRARIES}
        -lglog
        ${spdlog_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${Sophus_LIBRARIES}
        ${CERES_LIBRARIES}
        )

include_directories(src)


add_executable(trajectory_align src/main.cpp src/utils.cpp  src/factor/pose_factor.cpp
        src/factor/local_parameterization.h
        src/factor/local_parameterization.cpp
        src/factor/math_util.hpp)
target_link_libraries(trajectory_align ${THIRTY_LIBS} )

