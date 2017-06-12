/*
 * 
 * Project: rec3D 
 * 
 * BBOX CLASS
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#ifndef __BBOX_INCLUDED__  
#define __BBOX_INCLUDED__ 

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#ifndef __CAMERA_INCLUDED__ 
#include "camera.cpp"
#endif

using namespace cv;
using namespace std;

class BoundingBox {
  
  private:

    void Update();                             // Function to update the coordinates, nx, ny, nz and nvoxels attributes
    
  public:
  
    String filename;                           // Name of the txt file containing the bbox information
    Mat coordinates;                           // 3x8 matrix containing the coordinates of the 8 vertices delimiting the bbox
    float xmin, xmax, ymin, ymax, zmin, zmax;  // Maximum and minimum coordinates of the bbox in each dimension (X,Y,Z)
    float voxelSize;                           // Length of the voxel edges
    int nx, ny, nz;                            // Number of voxels in each direction (X,Y,Z)
    int nvoxels;                               // Total number of voxels

    BoundingBox(String bboxFile, float resolution);  // Constructor -> Build bbox from txt file
    BoundingBox();                                   // Function to initialize a bbox 
    void Save(String fileToSave);                    // Function to save the bbox in a txt file
    void SavePLY(String fileToSave);                 // Function to save the bbox in a ply file
    void AdjustFromDepth(vector<Camera> P, vector< Mat_<float> > depthmap, int minN);  // Adjust bbox using depth reliability
};


void BoundingBox::Update(){

  // Build the 3x8 matrix containing the coordinates of the 8 vertices delimiting the bbox
  coordinates =(Mat_<float>(3,8) << xmin, xmin, xmin, xmin, xmax, xmax, xmax, xmax,
		                    ymin, ymax, ymin, ymax, ymin, ymax, ymin, ymax,
			            zmin, zmin, zmax, zmax, zmin, zmin, zmax, zmax);
  
  // Get the number of voxels in each dimension
  nx = ceil((xmax-xmin)/voxelSize);
  ny = ceil((ymax-ymin)/voxelSize);
  nz = ceil((zmax-zmin)/voxelSize);
  
  // Get the total number of voxels inside the bbox
  nvoxels = nx*ny*nz;
  
}


BoundingBox::BoundingBox(){
  // This is just to initialize the bbox
}


BoundingBox::BoundingBox(String bboxFile, float voxels){
  
  filename = bboxFile;    // Get camera filename
  nvoxels = voxels;       // Get voxel size
  
  // Read bboxFile (e.g. "bbox.txt") and push all its information into a 'data' vector
  ifstream file(bboxFile.c_str());
  vector<float> data;
  string line;
  while (getline(file, line)) {  
    istringstream stream(line);
    float x;
    while (stream >> x) {
      data.push_back(x);
    }
  }
  
  // Get the maximum and minimum coordinates of the bbox in each dimension (X,Y,Z) from 'data'
  xmin = data[0];
  ymin = data[1];
  zmin = data[2];
  xmax = data[3];
  ymax = data[4];
  zmax = data[5];

  double bboxVolume = (xmax-xmin)*(ymax-ymin)*(zmax-zmin);
  double voxelVolume = bboxVolume/nvoxels;
  voxelSize = cbrt(voxelVolume);
  
  if (voxelSize < 0.005) voxelSize = 0.005;
  
  // Update the rest of the bbox attributes using the update function
  Update();
  
}


void BoundingBox::AdjustFromDepth( vector<Camera> P, vector< Mat_<float> > depthmap, int minN){
  
  // 'P' is the vector containing the cameras used to create the depthmaps
  // 'depthmap' is the vector containing the depthmaps created with the plane sweep algorithm
  // 'minN' is the minimum number of depthmaps where the projection of a voxel must have reliabile 
  //        depth in order to keep that voxel within the bbox
  
  float minX = +1e10;
  float maxX = -1e10; 
  float minY = +1e10;
  float maxY = -1e10; 
  float minZ = +1e10;
  float maxZ = -1e10; 
  
  bool minXchanged = false;
  bool maxXchanged = false;
  bool minYchanged = false;
  bool maxYchanged = false;
  bool minZchanged = false;
  bool maxZchanged = false;
  
  // Iterate for all the voxels of the bbox
  for (int z=0; z < nz; z++){
    for (int y=0; y < ny; y++){
      for (int x=0; x < nx; x++){	
	
	// Current voxel index (from 3D array to 1D array)
	float xcoord = x*voxelSize + xmin;
	float ycoord = y*voxelSize + ymin;
	float zcoord = z*voxelSize + zmin;
	
	int count = 0; // This will count the number of depthmaps where the voxel projection is assigned a reliable depth
	  
	for (int v=0; v < depthmap.size(); v++){
	    
	  // Project the voxel from the 3D space to the depthmap
	  Mat projection = P[v].P*(Mat_<float>(4,1) << xcoord,ycoord,zcoord,1.0);
	  float xp = projection.at<float>(0,0);
	  float yp = projection.at<float>(1,0);
	  float zp = projection.at<float>(2,0);
	  xp = cvRound(xp/zp);
	  yp = cvRound(yp/zp);
	  
	  // If the voxel falls within the depthmap and projects into a pixel with reliable depth, increase the counter
	  if (xp >= 0 && xp < P[v].w && yp >= 0 && yp < P[v].h && depthmap[v].at<float>(yp,xp) != 255.0) count++;
	  
	}
	
	// Update bbox dimensions according taking into account only those voxels with counter >= minN
	if (count >= minN){
	  
	  if (minX > xcoord){
	    minX = xcoord;
	    minXchanged = true;
	  }
	  if (maxX < xcoord){
	    maxX = xcoord;
	    maxXchanged = true;
	  }
	  if (minY > ycoord){
	    minY = ycoord;
	    minYchanged = true;
	  }
	  if (maxY < ycoord){
	    maxY = ycoord;
	    maxYchanged = true;
	  }
	  if (minZ > zcoord){
	    minZ = zcoord;
	    minZchanged = true;
	  }
	  if (maxZ < zcoord){
	    maxZ = zcoord;
	    maxZchanged = true;
	  }
	}
	  
      }
    }
  }
  
  // Update the maximum and minimum coordinates of the bbox in each dimension (X,Y,Z)
  if(minXchanged) xmin = minX;
  if(minYchanged) ymin = minY;
  if(minZchanged) zmin = minZ;
  if(maxXchanged) xmax = maxX;
  if(maxYchanged) ymax = maxY;
  if(maxZchanged) zmax = maxZ;
  
  // Update the rest of the bbox attributes using the update function
  Update();
  
}


void BoundingBox::Save( String fileToSave ){
  
  // Save txt file with the maxmimum and minimum bbox coordinates in each direction
  
  system(("if [ -f "+fileToSave+" ]; then rm "+fileToSave+"; fi").c_str()); // If file already exists, then remove it
  // Write the file
  ofstream myfile;
  myfile.open (fileToSave.c_str());
  myfile << xmin << " " << ymin << " " << zmin << "\n";
  myfile << xmax << " " << ymax << " " << zmax << "\n";
  myfile.close();
}


void BoundingBox::SavePLY( String fileToSave ){
  
  // Save ply file with the edges of the bbox (this is useful to check visually if the bbox is nicely adjusted to the 3D object)
  
  // Init the point cloud that will containg the points from the edges of the bbox 
  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  
  // Get the points from the edges of the bbox and push them to the point cloud
  
  // Get the 4 edges where x and z are fixed
  for (int y=0; y < ny; y++){
    
    float ycoord = y*voxelSize + ymin;
    
    // Edge 1 -> x=0 && z=0
    pcl::PointXYZ point1;
    point1.x = xmin;
    point1.y = ycoord;
    point1.z = zmin;
    
    // Edge 2 -> x=0 && z=nz-1
    pcl::PointXYZ point2;
    point2.x = xmin;
    point2.y = ycoord;
    point2.z = (nz-1)*voxelSize + zmin;
    
    // Edge 3 -> x=nx-1 && z=0
    pcl::PointXYZ point3;
    point3.x = (nx-1)*voxelSize + xmin;
    point3.y = ycoord;
    point3.z = zmin;
    
    // Edge 4 -> x=nx-1 && z=nz-1
    pcl::PointXYZ point4;
    point4.x = (nx-1)*voxelSize + xmin;
    point4.y = ycoord;
    point4.z = (nz-1)*voxelSize + zmin;
    
    basic_cloud_ptr->points.push_back(point1);
    basic_cloud_ptr->points.push_back(point2);
    basic_cloud_ptr->points.push_back(point3);
    basic_cloud_ptr->points.push_back(point4);
  }
  
  // Get the 4 edges where x and y are fixed
  for (int z=0; z < nz; z++){
    
    float zcoord = z*voxelSize + zmin;
    
    // Edge 1 -> x=0 && z=0
    pcl::PointXYZ point1;
    point1.x = xmin;
    point1.y = ymin;
    point1.z = zcoord;
    
    // Edge 2 -> x=0 && y=ny-1
    pcl::PointXYZ point2;
    point2.x = xmin;
    point2.y = (ny-1)*voxelSize + ymin;
    point2.z = zcoord;
    
    // Edge 3 -> x=nx-1 && y=0
    pcl::PointXYZ point3;
    point3.x = (nx-1)*voxelSize + xmin;
    point3.y = ymin;
    point3.z = zcoord;
    
    // Edge 4 -> x=nx-1 && y=ny-1
    pcl::PointXYZ point4;
    point4.x = (nx-1)*voxelSize + xmin;
    point4.y = (ny-1)*voxelSize + ymin;
    point4.z = zcoord;
    
    basic_cloud_ptr->points.push_back(point1);
    basic_cloud_ptr->points.push_back(point2);
    basic_cloud_ptr->points.push_back(point3);
    basic_cloud_ptr->points.push_back(point4);
  }
  
  // Get the 4 edges where y and z are fixed
  for (int x=0; x < nx; x++){
    
    float xcoord = x*voxelSize + xmin;
    
    // Edge 1 -> y=0 && z=0
    pcl::PointXYZ point1;
    point1.x = xcoord;
    point1.y = ymin;
    point1.z = zmin;
    
    // Edge 2 -> y=ny-1 && z=0
    pcl::PointXYZ point2;
    point2.x = xcoord;
    point2.y = (ny-1)*voxelSize + ymin;
    point2.z = zmin;
    
    // Edge 3 -> y=0 && z=nz-1
    pcl::PointXYZ point3;
    point3.x = xcoord;
    point3.y = ymin;
    point3.z = (nz-1)*voxelSize + zmin;
    
    // Edge 4 -> y=ny-1 && z=nz-1
    pcl::PointXYZ point4;
    point4.x = xcoord;
    point4.y = (ny-1)*voxelSize + ymin;
    point4.z = (nz-1)*voxelSize + zmin;
    
    basic_cloud_ptr->points.push_back(point1);
    basic_cloud_ptr->points.push_back(point2);
    basic_cloud_ptr->points.push_back(point3);
    basic_cloud_ptr->points.push_back(point4);
  }
  
  // Set point cloud size
  basic_cloud_ptr->width = (int)basic_cloud_ptr->points.size();
  basic_cloud_ptr->height = 1;
  
  // Write point cloud in ply file
  if (basic_cloud_ptr->width > 0){
    pcl::PLYWriter writer; 	
    writer.write(fileToSave.c_str(), *basic_cloud_ptr);
  }
  else{
    cout << "WARNING: Point cloud has no points." << endl;
  }
  
}

#endif