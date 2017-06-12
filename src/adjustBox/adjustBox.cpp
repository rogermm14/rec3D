/*
 * 
 * Project: adjustBox 
 * 
 * This program is used to adjust the bbox resulting from a SfM run
 * The initial structure point cloud is filtered to remove possible outliers
 * The StatisticalOutlierRemoval filter from PCL library is used to do it
 * The new bbox maximum and minimum coordinates in each dimension X,Y,Z are computed
 * from the filtered structure point cloud
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <getopt.h>

#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/ply_io.h>

using namespace cv;
using namespace std;

int main (int argc, char** argv){
  
  // Initialize parameters
  float neighborsPercent = 0.3;
  double stdDevThr = 1.0; 
  string inputStructure, outputFilteredStructure, bboxFile;
  
  const char* const short_opts = "i:o:b:k:t:h";
  
  static struct option long_opts[] =
  {
    {"i",     required_argument, 0, 'i'},
    {"o",     required_argument, 0, 'o'},
    {"bbox",  required_argument, 0, 'b'},
    {"knn",   required_argument, 0, 'k'},
    {"t",     required_argument, 0, 't'},
    {"help",  no_argument,       0, 'h'},
    {0, 0, 0, 0}
  };
 
  int opt;
  int option_index = 0;
  
  while ((opt = getopt_long(argc, argv, short_opts,long_opts,&option_index)) != -1){
    switch (opt){
      case 'i':
	inputStructure = optarg;
	break;
      case 'o':
	outputFilteredStructure = optarg;
        break;
      case 'b':
	bboxFile = optarg; 
	break;
      case 'k':
	neighborsPercent = atof(optarg);
	break;
      case 't':
	stdDevThr = atof(optarg);
	break;
      case 'h':
	std::cout << "\nList of available options:\n\n"
	     << "	--i     <input .ply file with raw structure>\n"
	     << "	--o     <output .ply file with filtered structure>\n"
	     << "	--bbox  <output txt file with bbox coordinates>\n"
	     << "	--knn   <amount of neighbor points (less than 1)>  [OPTIONAL] Default: 0.3\n"
	     << "	--t     <deviation threshold>                      [OPTIONAL] Default: 0.5\n"
	     << std::endl;
	return  EXIT_SUCCESS;
	break;
      }
  }
  
  std::cout << "\nYou called:\n\n"
      << "	--i     <input .ply file with raw structure>        " + inputStructure  + "\n"
      << "	--o     <output .ply file with filtered structure>  " + outputFilteredStructure + "\n"
      << "	--bbox  <output txt file with bbox coordinates>     " + bboxFile + "\n"
      << "	--knn   <amount of neighbor points (less than 1)>   " << neighborsPercent << "\n"
      << "	--t     <deviation threshold>                       " << stdDevThr << "\n"
      << std::endl;
  
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PLYReader reader;
  pcl::PLYWriter writer;

  // Read input structure point cloud
  reader.read(inputStructure.c_str(), *cloud);
  
  // Filter using the StatisticalOutlierRemoval filter
  pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
  sor.setInputCloud(cloud);
  float K = (float)(cloud->points.size()) * neighborsPercent;
  sor.setMeanK( (int)K );
  sor.setStddevMulThresh( stdDevThr );
  sor.filter(*cloud_filtered);
 
  float minX = 1e10;
  float maxX = -1e10; 
  float minY = 1e10;
  float maxY = -1e10; 
  float minZ = 1e10;
  float maxZ = -1e10; 
  
  // Compute the new bbox from the filtered structure
  for (size_t i = 0; i < cloud_filtered->points.size(); ++i){
   
    float xcoord = cloud_filtered->points[i].x;
    float ycoord = cloud_filtered->points[i].y;
    float zcoord = cloud_filtered->points[i].z;
    
    if (minX > xcoord) minX = xcoord;
    if (maxX < xcoord) maxX = xcoord;
    if (minY > ycoord) minY = ycoord;
    if (maxY < ycoord) maxY = ycoord;
    if (minZ > zcoord) minZ = zcoord;
    if (maxZ < zcoord) maxZ = zcoord;
  }    

  float xmin = minX;
  float ymin = minY;
  float zmin = minZ;
  float xmax = maxX;
  float ymax = maxY;
  float zmax = maxZ;
  
  // Increase the new bbox by a certain margin in each direction
  // This is done to avoid losing info in case the filtering was too aggressive
  float margin = 0.1;
  xmin = xmin - (xmax - xmin) * margin;
  xmax = xmax + (xmax - xmin) * margin;
  ymin = ymin - (ymax - ymin) * margin;
  ymax = ymax + (ymax - ymin) * margin;
  zmin = zmin - (zmax - zmin) * margin;
  zmax = zmax + (zmax - zmin) * margin;
  
  // Write filtered structure (useful to debug) 
  writer.write( outputFilteredStructure.c_str(), *cloud_filtered );

  // Write output bbox
  std::ofstream myfile;
  myfile.open(bboxFile.c_str());
  myfile << xmin << " " << ymin << " " << zmin << "\n";
  myfile << xmax << " " << ymax << " " << zmax << "\n";
  myfile.close();

  return(0);
}
