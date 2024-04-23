/**
 * @file chamfer_distance.cpp
 * @brief Calculate the Chamfer Distance between two point clouds using the standard Euclidean distance method.
 */

#include <iostream>
#include <vector> 
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <ctime>
#include <cfloat> // Include this header for FLT_MAX

/**
 * @brief Compute the Chamfer Distance between two point clouds.
 * @param cloud_source Pointer to the source point cloud.
 * @param cloud_target Pointer to the target point cloud.
 * @return The computed Chamfer Distance as a float value.
 */

float compute (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target)
{
  float rmse = 0.0f;
  // Vector to store distances from source to target points
  std::vector<float> distances(cloud_source->size());
  // Iterate over source points
  for (std::size_t point_i = 0; point_i < cloud_source->size(); ++ point_i)
  {
    float min_d = FLT_MAX;
    // Iterate over target points
    for (std::size_t point_j = 0; point_j < cloud_target->size(); ++ point_j)
    {
      // Calculate Euclidean distance between source and target points
      float dx = (*cloud_target)[point_j].x - (*cloud_source)[point_i].x;
      float dy = (*cloud_target)[point_j].y - (*cloud_source)[point_i].y;
      float dz = (*cloud_target)[point_j].z - (*cloud_source)[point_i].z;
      float dist = sqrt(dx * dx + dy * dy + dz * dz);
      // Update minimum distance
      min_d = fminf(min_d, dist); 
    }
    // Store minimum distance for each source point
    distances[point_i] = min_d;
  }
  // Calculate root mean square error (RMSE)
  rmse = std::accumulate(distances.begin(), distances.end(), 0.0);
  rmse = rmse / (float)cloud_source->size();
  return rmse;
}


int main(int argc, char** argv)
{
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
  // Compute Chamfer Distance between the two point clouds
  float CD1, CD2, CD;
  CD1 = compute (cloud1, cloud2);
  CD2 = compute (cloud1, cloud2);
  CD = (CD1 + CD2) /2;
  // Output the computed Chamfer Distance
  std::cout<< CD << std::endl;

  return 0;
}