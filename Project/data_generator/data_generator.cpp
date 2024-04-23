/**
 * @file data_generator.cpp
 * @brief Generate random data points and save them to text files.
 */

#include <iostream>
#include <fstream>

int main(int argc, char** argv)
{
  // Parse the number of points from the command-line argument
  const int num_points = std::stoi(argv[1]);
  // Filenames for output text files
  std::string filename1 = "points1_" + std::to_string(num_points) + ".txt";
  std::string filename2 = "points2_" + std::to_string(num_points) + ".txt";

  // Open output file streams for writing
  std::ofstream outFile1(filename1);
  // Generate random data points and write them to the first output file
  for (int i = 0; i < 3*num_points; ++i)
  {
    float x = 1024 * rand () / (RAND_MAX + 1.0f);
    outFile1 << x << std::endl;
  }
  outFile1.close();
  // Open output file streams for writing
  std::ofstream outFile2(filename2);
  // Generate random data points and write them to the first output file
  for (int i = 0; i < 3*num_points; ++i)
  {
    float x = 1024 * rand () / (RAND_MAX + 1.0f);
    outFile2 << x << std::endl;
  }
  outFile2.close();

  return 0;
}