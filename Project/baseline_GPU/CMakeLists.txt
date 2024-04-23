cmake_minimum_required(VERSION 3.20 FATAL_ERROR)
project(rmse_cuda)


# set(CMAKE_CXX_STANDARD 17)
# set(CMAKE_CUDA_STANDARD 17)
# set(CMAKE_CUDA_STANDARD_REQUIRED TRUE)
# set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

# Set compiler-specific flags
# if (CMAKE_CXX_COMPILER_ID STREQUAL "Intel")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -diag-suppress 2809-D")
# elseif (CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#     set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-2809-D")
# endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -w")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS};-std=c++17)



# Find CUDA
find_package(CUDA REQUIRED)
include_directories(${CUDA_INCLUDE_DIRS})

# Find PCL
find_package(PCL 1.11 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# link_libraries(stdc++fs)


#COMPONENTS common io
# Add executable
cuda_add_executable(CD_baseline_GPU CD_baseline_GPU.cu)

# Link against CUDA and PCL
target_link_libraries(CD_baseline_GPU ${CUDA_LIBRARIES} ${PCL_LIBRARIES})
