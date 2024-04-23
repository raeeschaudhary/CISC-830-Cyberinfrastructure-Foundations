# Introduction

This repository provieds a codebase to compute Chamfer Distance (CD) between two point clouds using different methods. The CD metric provides a quantitative measure of the discrepancy or similarity between two shapes represented by point clouds. The mathematical expression to compute CD is as follow: <br>
$$CD (P_1, P_2) = \frac{1}{2} \left(\frac{1}{N} \sum_{x \in P_1} \min_{y \in P_2} \lVert x-y \rVert_2  + \frac{1}{N} \sum_{y \in P_2} \min_{x \in P_1} \lVert x-y \rVert_2 \right)$$

where 
- P<sub>1</sub> -> point cloud 1 
- P<sub>2</sub> -> point cloud 2
- x -> 3D points in P<sub>1</sub>
- y-> 3D points in P<sub>2</sub>


By calculating the CD, users can assess the degree of resemblance or difference between objects or scenes captured as point clouds. The smaller the value of CD, the simililar the point clouds and vice versa. Users can incorporate this project into their workflows by providing two sets of point clouds as input. 


## Prerequisite

The project uses cmake to compile programs. 
```
# install cmake 
sudo apt-get update
sudo apt-get install cmake
```

```
# install pcl library
sudo apt-get update
sudo apt-get install libpcl-dev
```

## Build and Run
Run the script `run.sh` in the root folder to generate all the results. The script will generate data and run all the [cases](#implementation-cases) one by one. The GPU case will only run on the machine with the GPU.

```
# Set the execute permission 
chmod +x run.sh
# Run the script
./run.sh 
```

All the executables run on the same data. the output of the baseline will be used to test the output of other executables. In case, to run an individual case use the following structure. 

```
# Move to data_generator, build and run.
cd data_generator
cmake . 
make  
chmod +x run.sh
./run.sh
cd ..
# Move to individual case, build and run. (e.g., cd {baseline|baseline_omp|baselineGPU|kd|kd_omp})
cd kd_omp
cmake .
make 
chmod +x run.sh
./run.sh
cd ..
```

## Example

The `run.sh` file takes long time to run becasue of huge test cses. To quickly test the output of different implementation cases, we provide `example.sh` that run all the cases for smaller test cases. Compilation may take time.

```
# Set the execute permission 
chmod +x example.sh
# Run the script
./example.sh 
```

## Implementation Cases
    ├── baseline
    │   ├── The baseline implementation uses the brute-force method.
    ├── baseline_omp
    │   ├── Similar to baseline implementation but with OpenMP.
    ├── baseline_GPU
    │   ├── Similar to baseline implementation but with cuda (GPU).
    ├── kd
    │   ├── Implementation using the KD-True to improve the baseline implementations.
    ├── kd_omp
    │   ├── Implementation using the KD-True with OpenMP to improve single thread KD-tree implementation.
    ├── kd_cuda
    │   ├── Implementation using the KD-True with GPU (yet to implement).

## Directory structures
    ├── baseline
    │   ├── CMakeLists.txt
    │   ├── CD_baseline.cpp
	│   └── run.sh
    ├── baseline_omp
    │   ├── CMakeLists.txt
    │   ├── CD_baseline_omp.cpp
	│   └── run.sh
    ├── baseline_GPU
    │   ├── CMakeLists.txt
    │   ├── CD_baseline_GPU.cu
	│   └── run.sh
    ├── kd
    │   ├── CMakeLists.txt
    │   ├── CD_kd.cpp
	│   └── run.sh
    ├── kd_omp
    │   ├── CMakeLists.txt
    │   ├── CD_kd_omp.cpp
	│   └── run.sh
