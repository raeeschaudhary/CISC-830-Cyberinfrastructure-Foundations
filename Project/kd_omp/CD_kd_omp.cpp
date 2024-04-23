/**
 * @file chamfer_distance_openmp.cpp
 * @brief Calculate the Chamfer Distance between two point clouds using the nearest neighbor search method with OpenMP.
 */

#include <iostream>
#include <vector> 
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <filesystem>
#include <omp.h>


/**
 * @brief Compute the Chamfer Distance between two point clouds using OpenMP for parallelization.
 * @param cloud_source Pointer to the source point cloud.
 * @param cloud_target Pointer to the target point cloud.
 * @param K The number of nearest neighbors to search for.
 * @return The computed Chamfer Distance as a float value.
 */
float compute (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_source, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_target, int K)
{
  float rmse = 0.0f;
  pcl::KdTreeFLANN<pcl::PointXYZ> tree;
  tree.setInputCloud (cloud_target);
  // Parallelize the loop using OpenMP
  #pragma omp parallel for reduction(+:rmse)
  for (std::size_t point_i = 0; point_i < cloud_source->size(); ++ point_i)
  {
    std::vector<int> pointIdxNKNSearch(K);
    std::vector<float> pointNKNSquaredDistance(K);
    // Perform nearest neighbor search for the current point
    if ( tree.nearestKSearch ((*cloud_source)[point_i], K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0 )
    {
      // Accumulate squared distances to the nearest neighbors
      for (std::size_t i = 0; i < pointIdxNKNSearch.size (); ++i)
      {
        rmse += std::sqrt(pointNKNSquaredDistance[i]);
      }
      
    }
  }
  // Compute the root-mean-square error
  rmse = rmse / (float)cloud_source->size();
  return rmse;
}


int main(int argc, char** argv)
{
  // Parse the number of points from the command-line argument
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

  // Compute Chamfer Distance between the two point clouds  with K=1
  float CD1, CD2, CD;
  CD1 = compute (cloud1, cloud2, 1);
  CD2 = compute (cloud1, cloud2, 1);
  CD = (CD1 + CD2) /2;
  // Output the computed Chamfer Distance
  std::cout << CD << std::endl;
   
  return 0;
}
