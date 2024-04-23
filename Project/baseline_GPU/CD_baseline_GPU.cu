/**
 * @file chamfer_distance_gpu.cu
 * @brief Calculate the Chamfer Distance between two point clouds using GPU acceleration.
 */

#include <iostream>
#include <vector>
#include <cmath>
#include <string>
#include <limits>
#include <filesystem>
#include <algorithm>
#include <cuda_runtime.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <cfloat> // Include this header for FLT_MAX

// CUDA kernel to compute distances between points in two point clouds
__global__ void computeDistances(const float* cloud1_data, int cloud1_size,
                                 const float* cloud2_data, int cloud2_size,
                                 float* distances) {
    int idx = threadIdx.x + blockIdx.x * blockDim.x;
    if (idx < cloud1_size) {
        float x1 = cloud1_data[3 * idx];
        float y1 = cloud1_data[3 * idx + 1];
        float z1 = cloud1_data[3 * idx + 2];

        float min_dist = FLT_MAX; // Use FLT_MAX here
        for (int i = 0; i < cloud2_size; ++i) {
            float x2 = cloud2_data[3 * i];
            float y2 = cloud2_data[3 * i + 1];
            float z2 = cloud2_data[3 * i + 2];

            float dx = x2 - x1;
            float dy = y2 - y1;
            float dz = z2 - z1;

            float dist = sqrt(dx * dx + dy * dy + dz * dz);
            min_dist = fminf(min_dist, dist);
        }
        distances[idx] = min_dist;
    }
}

// Function to compute Chamfer distance between two point clouds using GPU
void computeChamferDistanceGPU(const std::vector<pcl::PointXYZ>& cloud1,
                                const std::vector<pcl::PointXYZ>& cloud2,
                                float& chamfer_distance1, float& chamfer_distance2) {
    // Convert point clouds to arrays
    std::vector<float> cloud1_data, cloud2_data;
    for (const auto& point : cloud1) {
        cloud1_data.push_back(point.x);
        cloud1_data.push_back(point.y);
        cloud1_data.push_back(point.z);
    }
    for (const auto& point : cloud2) {
        cloud2_data.push_back(point.x);
        cloud2_data.push_back(point.y);
        cloud2_data.push_back(point.z);
    }

    // Allocate GPU memory
    float *d_cloud1, *d_cloud2, *d_distances1, *d_distances2;
    cudaMalloc((void **)&d_cloud1, sizeof(float) * cloud1_data.size());
    cudaMalloc((void **)&d_cloud2, sizeof(float) * cloud2_data.size());
    cudaMalloc((void **)&d_distances1, sizeof(float) * cloud1.size());
    cudaMalloc((void **)&d_distances2, sizeof(float) * cloud2.size());

    // Copy data from host to device
    cudaMemcpy(d_cloud1, cloud1_data.data(), sizeof(float) * cloud1_data.size(), cudaMemcpyHostToDevice);
    cudaMemcpy(d_cloud2, cloud2_data.data(), sizeof(float) * cloud2_data.size(), cudaMemcpyHostToDevice);

    // Launch CUDA kernel
    int block_size = 256;
    int num_blocks_cloud1 = (cloud1.size() + block_size - 1) / block_size;
    int num_blocks_cloud2 = (cloud2.size() + block_size - 1) / block_size;
    computeDistances<<<num_blocks_cloud1, block_size>>>(d_cloud1, cloud1.size(), d_cloud2, cloud2.size(), d_distances1);
    computeDistances<<<num_blocks_cloud2, block_size>>>(d_cloud2, cloud2.size(), d_cloud1, cloud1.size(), d_distances2);
    cudaDeviceSynchronize();

    // Copy data from device to host
    std::vector<float> distances1(cloud1.size()), distances2(cloud2.size());
    cudaMemcpy(distances1.data(), d_distances1, sizeof(float) * cloud1.size(), cudaMemcpyDeviceToHost);
    cudaMemcpy(distances2.data(), d_distances2, sizeof(float) * cloud2.size(), cudaMemcpyDeviceToHost);

    // Compute Chamfer distance
    chamfer_distance1 = std::accumulate(distances1.begin(), distances1.end(), 0.0f) / (float)cloud1.size();
    chamfer_distance2 = std::accumulate(distances2.begin(), distances2.end(), 0.0f) / (float)cloud2.size();

    // Free GPU memory
    cudaFree(d_cloud1);
    cudaFree(d_cloud2);
    cudaFree(d_distances1);
    cudaFree(d_distances2);
}

int main(int argc, char** argv) {
    // Parse the number of points from command-line argument
    const int num_points = std::stoi(argv[1]);
    // Filenames for input point clouds
    std::string filename1 = "../data_generator/points1_" + std::to_string(num_points) + ".txt";
    std::string filename2 = "../data_generator/points2_" + std::to_string(num_points) + ".txt";
    // Load point cloud 1
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud1(new pcl::PointCloud<pcl::PointXYZ>);
    cloud1->width = num_points;
    cloud1->height = 1;
    cloud1->points.resize(num_points);

    std::ifstream inFile1(filename1);

    if (!inFile1) {
        std::cerr << "Error: Could not open the file1." << std::endl;
        return 1;
    }

    float x, y, z;
    int count = 0 ;

    std::string line;
    while (inFile1 >> x >> y >> z) {
        cloud1->points[count].x = x;
        cloud1->points[count].y = y;
        cloud1->points[count].z = z;
        count++;
    }
    inFile1.close();

    // Load point cloud 2
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud2(new pcl::PointCloud<pcl::PointXYZ>);
    cloud2->width = num_points;
    cloud2->height = 1;
    cloud2->points.resize(num_points);

    std::ifstream inFile2(filename2);
    if (!inFile2) {
        std::cerr << "Error: Could not open the file2." << std::endl;
        return 1;
    }

    count = 0 ;
    while (inFile2 >> x >> y >> z) {
        cloud2->points[count].x = x;
        cloud2->points[count].y = y;
        cloud2->points[count].z = z;
        count++;
    }
    inFile2.close();


    float CD = 0.0f;

    // Convert point clouds to vectors
    std::vector<pcl::PointXYZ> points1(cloud1->begin(), cloud1->end());
    std::vector<pcl::PointXYZ> points2(cloud2->begin(), cloud2->end());

    float chamfer_distance1, chamfer_distance2;
    computeChamferDistanceGPU(points1, points2, chamfer_distance1, chamfer_distance2);
    CD = (chamfer_distance1 + chamfer_distance2) /2.0;

    // Output the computed Chamfer Distance
    std::cout << CD << std::endl;

    return 0;
}
