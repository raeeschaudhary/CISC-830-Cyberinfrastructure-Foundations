cmake_minimum_required(VERSION 3.5 FATAL_ERROR)

project(error_calculation)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")

find_package(PCL 1.2 REQUIRED)
find_package(OpenMP REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable (CD_kd_omp CD_kd_omp.cpp)
target_link_libraries (CD_kd_omp PUBLIC OpenMP::OpenMP_CXX ${PCL_LIBRARIES} )

