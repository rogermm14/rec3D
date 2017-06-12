/*
 * 
 * Project: rec3D 
 * 
 * WRITE POINTCLOUDS AND RAW FILES
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#ifndef __POINTCLOUD_INCLUDED__  
#define __POINTCLOUD_INCLUDED__ 

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

#ifndef __CAMERA_INCLUDED__ 
#include "camera.cpp"
#endif

#ifndef __BBOX_INCLUDED__ 
#include "bbox.cpp"
#endif

using namespace cv;
using namespace std;


float getMedian(vector<float> f){
  
  int length = f.size();
 
  sort(f.begin(), f.end());
  float median;
  if (f.size() % 2 == 0){ 
    median = (f[length/2 - 1] + f[length/2]) / 2.0;
  }
  else{
    median = f[floor(length/2)];
  }
  
  return median;
}


float getMean(vector<float> f){
  
  int length = f.size();
  float sum;
  for (int i = 0; i < length; i++){
   sum += f[i];  
  }
  
  return (float)sum/length;
}


void computePointCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, Mat_<float> f, float margin, BoundingBox bbox){
  
  for (int z=0; z < bbox.nz; z++){
      for (int y=0; y < bbox.ny; y++){
	for (int x=0; x < bbox.nx; x++){
	
	int n = x + bbox.nx * (y + bbox.ny * z);

	if (f.at<float>(n) < margin && f.at<float>(n) > -margin){
	  
	  pcl::PointXYZ point;
	  
	  float xcoord = x*bbox.voxelSize + bbox.xmin;
	  float ycoord = y*bbox.voxelSize + bbox.ymin;
	  float zcoord = z*bbox.voxelSize + bbox.zmin;
	  
	  point.x = xcoord;
	  point.y = ycoord;
	  point.z = zcoord;
	  
	  cloud->points.push_back(point);
	}
	
      }
    }
  }
  
  cloud->width = (int) cloud->points.size ();
  cloud->height = 1;
}

  
void paintPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGBcloud, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, 
		     vector<int> views_ind, vector<Mat> oResizedImages, vector<Camera> P, string paintingMethod)
{
  
  int N = P.size();
  
  for (size_t i = 0; i < cloud->points.size(); ++i)
  {
    //init RGB values
    int R = 0;
    int G = 0;
    int B = 0;
    vector<float> Rvec,Bvec,Gvec;
    
    //get current 3D point coordinates
    float xcoord = cloud->points[i].x;
    float ycoord = cloud->points[i].y;
    float zcoord = cloud->points[i].z;

    for(std::vector<int>::iterator it = views_ind.begin(); it != views_ind.end(); ++it)
    {
      //project voxel onto the image v
      Mat projection = P[*it].P*(Mat_<float>(4,1) << xcoord,ycoord,zcoord,1.0);
      float xp = projection.at<float>(0,0);
      float yp = projection.at<float>(1,0);
      float zp = projection.at<float>(2,0);
      xp = cvRound(xp/zp);
      yp = cvRound(yp/zp);
      
      //check if the projection falls inside the image limits
      if (xp >= 0 && xp < P[*it].w && yp >= 0 && yp < P[*it].h)
      {
	// Get color	
	Bvec.push_back(oResizedImages[*it].at<Vec3b>(yp,xp)[0]);
	Gvec.push_back(oResizedImages[*it].at<Vec3b>(yp,xp)[1]);
	Rvec.push_back(oResizedImages[*it].at<Vec3b>(yp,xp)[2]);
      }
    }
    
    //save the point 3D coordinates
    pcl::PointXYZRGB point;
    point.x = xcoord;
    point.y = ycoord;
    point.z = zcoord;
    
    if (Rvec.size() > 0){  
      // If the current vertex is not occluded in some view(s), 
      // then paint it according to the selected painting method
      if(paintingMethod == "mean"){
	R = (int)getMean(Rvec);
	G = (int)getMean(Gvec);
	B = (int)getMean(Bvec);
      }
      else{
	R = (int)getMedian(Rvec);
	G = (int)getMedian(Gvec);
	B = (int)getMedian(Bvec);
      }
    }
    
    //the final colour of the point will be the mean value in each of the RGB channels
    point.r = R;
    point.g = G;
    point.b = B;
      
    //add the point to the RGB point cloud
    RGBcloud->points.push_back(point);
  }
 
  RGBcloud->width = (int) RGBcloud->points.size ();
  RGBcloud->height = 1;
}


void writeBasicPointCloudPLY( std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
  
  if (cloud->width > 0){
    pcl::PLYWriter writer; 	
    writer.write(filename, *cloud);
  }
  else{
    cout << "WARNING: Point cloud has no points." << endl;
  }
  
}


void writeRGBPointCloudPLY( std::string filename, pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){
  
  if (cloud->width > 0){
    pcl::PLYWriter writer; 	
    writer.write(filename, *cloud);
  }
  else{
    cout << "WARNING: Point cloud has no points." << endl;
  }
  
}


void saveDistancefieldPLY(Mat_<float> distancefield, float margin, std::string outputPLY, BoundingBox bbox,
			  bool COMPUTE_COLOR, vector<int> views_ind, vector<Mat> oResizedImages, vector<Camera> P){

  pcl::PointCloud<pcl::PointXYZ>::Ptr basic_cloud_ptr (new pcl::PointCloud<pcl::PointXYZ>);
  computePointCloud(basic_cloud_ptr, distancefield, margin, bbox);
  
  if (!COMPUTE_COLOR){
    writeBasicPointCloudPLY(outputPLY, basic_cloud_ptr);
  }
  else{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr RGB_cloud_ptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    paintPointCloud(RGB_cloud_ptr, basic_cloud_ptr, views_ind, oResizedImages, P, "median");
    writeRGBPointCloudPLY(outputPLY, RGB_cloud_ptr);
  }
}


void writeIsoSurfaceRAW( String filename, Mat f ){
  
  FILE *fw = NULL;
  float val;
  fw=fopen(filename.c_str(),"w+");

  for (int n=0; n < f.cols; n++){
    val = f.at<float>(n);
    fwrite(&val,sizeof(float),1,fw);
  }
  fclose(fw);
}

void writeIsoSurfaceVTK( String filename, Mat f, int nx, int ny, int nz ){
  
  float minvalue = 0;
  float maxvalue = 1;
  
  ofstream fout(filename.c_str());
  
  fout << "# vtk DataFile Version 2.0\n";
  fout << "Probability Volume\n";
  fout << "ASCII\n";
  fout << "\n";
  fout << "DATASET STRUCTURED_POINTS\n";
  fout << "DIMENSIONS " << nx << " "
                        << ny << " "
                        << nz << "\n";
  fout << "ORIGIN 0 0 0\n";
  fout << "SPACING 1 1 1\n";

  fout << "POINT_DATA " << nx * ny * nz << "\n";
  fout << "SCALARS scalars unsigned_char 1\n";
  fout << "LOOKUP_TABLE default\n";
  
  for (int n=0; n < f.cols; n++){
    int v = int(255 * (f.at<float>(n) - minvalue) / (maxvalue - minvalue));
    fout << std::max(0, std::min(v, 255)) << "\n";
  }
}

#endif