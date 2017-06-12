/*
 * 
 * Project: rec3D 
 * 
 * CAMERA CLASS
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#ifndef __CAMERA_INCLUDED__  
#define __CAMERA_INCLUDED__ 

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>

using namespace cv;
using namespace std;

class Camera {
  
  private:
    
    void Update();              // Function used to update the camera center and orientation
    
  public:
  
    String filename;            // Name of the txt file containing the camera information
    Mat P;                      // Projection matrix
    Mat K;                      // Intrinsics matrix                      
    Mat R, t;                   // Rotation matrix (R) and translation vector (t)
    Mat center, orientation;    // Optical center and orientation vector
    double mindepth, maxdepth;  // Minimum depth and maximum depth of the bbox in the camera reference system
    int h, w;                   // Height and width of the camera
    int index;                  // Index of the current camera in the set of N views
    int format;                 // Format of the input camera file (1 or 2)
 
    Camera(String cameraFile, int viewInd);              // Constructor -> Build camera from txt file
    void Resize(float scale);                            // Function to resize camera
    void GetMinMaxDepth(Mat bboxWorldCoord);             // Function to get mindepth and maxdepth attributes
    void DrawCamera(std::string outputPLY, float scale); // Function to draw a camera in a ply file
    void GetClosestCamera(vector<Camera> P, Mat_<int> depthmapList, int* closestCameraInd); //Function to find the closest camera
    
  
};


void Camera::Update(){
  
  // Compute camera center
  Mat S, U, VT;
  SVD::compute(P, S, U, VT, SVD::FULL_UV);
  Mat V = VT.t();
  center = V(Range(0,3), Range(3,4)) / V.at<float>(3,3);
  
  // Compute camera orientation
  Mat KR = K*R;
  orientation = KR.inv() * (Mat_<float>(3,1) << w/2,h/2,1);
}


Camera::Camera(String cameraFile, int viewInd){
  
  filename = cameraFile;  // Get camera filename
  index = viewInd;        // Get camera index
  
  // Initialize P,K,R,t
  P = Mat::zeros(3,4, CV_32FC1);
  K = Mat::zeros(3,3, CV_32FC1);
  R = Mat::zeros(3,3, CV_32FC1);
  t = Mat::zeros(3,1, CV_32FC1);
  center = Mat::zeros(3,1, CV_32FC1);
  orientation = Mat::zeros(3,1, CV_32FC1);
  mindepth = +1e10;
  maxdepth = -1e10;
 
  // Read input cameraFile (e.g. "XXXXX_cam.txt") and push all its information into a 'data' vector
  ifstream file(filename.c_str());
  vector<float> data;
  string line;
  while (getline(file, line)) {  
    istringstream stream(line);
    float x;
    while (stream >> x) {
      data.push_back(x);
    }
  }
  
  // Fill the rest of the camera attributes using the information in 'data'
  
  format = data[0]; // Get camera file format
  w = data[1];      // Get the camera width
  h = data[2];      // Get the camera height
  
  int a = 3;
  switch (format){
    case 1:
    {
      // If camera file format is 1, read K, R and t from 'data' and then compute P
      for (int i = 0; i< K.rows; ++i){
	for (int j = 0; j< K.cols; ++j){
	  K.at<float>(i,j) = data[a];
	  a++;
	}
      }
      for (int i = 0; i< R.rows; ++i){
	for (int j = 0; j< R.cols; ++j){
	  R.at<float>(i,j) = data[a];
	  a++;
	}
      }
      for (int i = 0; i< t.rows; ++i){
	t.at<float>(i,0) = data[a];
	a++;
      }
      Mat extrinsics;
      hconcat(R, t, extrinsics);
      P = K * extrinsics;   
      break;
    }  
    case 2:
    {
      // If camera file format is 2, read P from 'data' and then compute K, R and t
      for (int i = 0; i< P.rows; ++i){
	for (int j = 0; j< P.cols; ++j){
	  P.at<float>(i,j) = data[a];
	  a++;
	}
      }
      Mat tempT;
      decomposeProjectionMatrix(P, K, R, tempT);
      t = tempT(Range(0,3),Range::all());
      break;
    }
  }
  
  Update(); // Update the camera center and orientation
}
  
  
void Camera::GetMinMaxDepth(Mat bboxWorldCoord){
  
  // Project the bbox, with its coordinates expressed in the world reference system into the camera reference system
  Mat temp = bboxWorldCoord.clone();
  temp.push_back( Mat::ones(1,bboxWorldCoord.cols,CV_32FC1));
  Mat bboxCamCoord = P*temp;
  
  // Get the minimum and maximum depth from the thrid row of the projected vertices
  minMaxLoc(bboxCamCoord(Range(2,3),Range::all()), &mindepth, &maxdepth);
}


void Camera::Resize(float scale){
  
  // Use the following matrix H to resize the camera matrices P and K according to the desired scale factor
  Mat_<float> H = (Mat_<float>(3,3) << scale,0.0,0.0,0.0,scale,0.0,0.0,0.0,1.0);
  P = H*P;
  K = H*K;
  
  // Also scale the camera size
  h = scale * h;
  w = scale * w;
  
  Update(); // Update the camera center and orientation
}


void Camera::GetClosestCamera(vector<Camera> P, Mat_< int > depthmapList, int* closestCameraInd){
  
  // The closest camera is the one whose orientation vector has the minimum angle with respect to this camera's orientation vector
  
  float minangle = +1e10; // Initialize the minimum angle
  int chosenCamera;       // Index of the chosen camera
  
  for (int i = 0; i < P.size(); i++){ 
    
    // Reference camera i cannot take camera i as a second view 
    if (i != index){ 
      
      // Check that the pair (index,i) has not been already used to compute the depthmap (i,index)
      // We can check this using the depthmapList
      // Annotation: (index,i) means "index" is the index of the reference camera and "i" is the index of the second camera
      bool repeated = false;
      if( depthmapList.at<int>(i+1,0)==i && depthmapList.at<int>(i+1,1)==index) repeated = true;
      
      // If the pair was not used, then camera i is a possible candidate to be the closest camera to this camera
      // Compute the angle between the orientation vectors of the two cameras and update minangle and chosenCamera
      // The dot product between the two orientation vectors is used below to compute the angle
      if (!repeated){
	//Mat dotproduct = orientation.t() * P[i].orientation;
	//float cosine = dotproduct.at<float>(0) / ( norm(orientation,cv::NORM_L2) * norm(P[i].orientation,cv::NORM_L2) ) ;
	//float angle = acos (cosine) * 180.0 / 3.14159265;
	float angle = norm(center-P[i].center,cv::NORM_L2);
	
	//cout << "     -> Angle with respect camera " << i << " : " << angle << " degrees." << endl;
	if (angle < minangle){
	  chosenCamera = i;
	  minangle = angle;
	}
      }   
    }
  }
  
  *closestCameraInd = chosenCamera; // Return the index of the closest camera
}


void Camera::DrawCamera( std::string outputPLY, float scale){
  
  // Compute the 4 points determining the boundaries of the image associated to the camera
  Mat KR = K*R;
  Mat p4 = center + KR.inv() * (Mat_<float>(3,1) << 0,0,1) * scale;
  Mat p3 = center + KR.inv() * (Mat_<float>(3,1) << w,0,1) * scale;
  Mat p2 = center + KR.inv() * (Mat_<float>(3,1) << w,h,1) * scale;
  Mat p1 = center + KR.inv() * (Mat_<float>(3,1) << 0,h,1) * scale;
  
  // Normalize the coordinates of the previous points (necessary for texture mapping)
  Mat np1 = p1 / norm(p1,cv::NORM_L2);
  Mat np2 = p2 / norm(p2,cv::NORM_L2);
  Mat np3 = p3 / norm(p3,cv::NORM_L2);
  Mat np4 = p4 / norm(p4,cv::NORM_L2);
  
  // Write ply file
  ofstream outFile( outputPLY.c_str() );
  outFile << "ply" << endl;
  outFile << "format ascii 1.0" << endl;
  outFile << "comment TextureFile cam_" << index << ".jpg" << endl;
  outFile << "element vertex 4" << endl;
  outFile << "property float x" << endl;
  outFile << "property float y" << endl;
  outFile << "property float z" << endl;
  outFile << "property float nx" << endl;
  outFile << "property float ny" << endl;
  outFile << "property float nz" << endl;
  outFile << "property uchar red" << endl;
  outFile << "property uchar green" << endl;
  outFile << "property uchar blue" << endl;
  outFile << "property float texture_u" << endl;
  outFile << "property float texture_v" << endl;
  outFile << "element face 1" << endl;
  outFile << "property list uchar int vertex_indices" << endl;
  outFile << "end_header" << endl;
  outFile << p1.at<float>(0) << " " << p1.at<float>(1) << " " << p1.at<float>(2) << " "
          << np1.at<float>(0) << " " << np1.at<float>(1) << " " << np1.at<float>(2) << " "
	  << "255 255 255 0 0" << endl;
  outFile << p2.at<float>(0) << " " << p2.at<float>(1) << " " << p2.at<float>(2) << " "
          << np2.at<float>(0) << " " << np2.at<float>(1) << " " << np2.at<float>(2) << " "
	  << "255 255 255 1 0" << endl;	  
  outFile << p3.at<float>(0) << " " << p3.at<float>(1) << " " << p3.at<float>(2) << " "
	  << np3.at<float>(0) << " " << np3.at<float>(1) << " " << np3.at<float>(2) << " "
	  << "255 255 255 1 1" << endl;  
  outFile << p4.at<float>(0) << " " << p4.at<float>(1) << " " << p4.at<float>(2) << " "
          << np4.at<float>(0) << " " << np4.at<float>(1) << " " << np4.at<float>(2) << " "
          << "255 255 255 0 1" << endl; 
  outFile << "4 0 1 2 3" << endl;
}

#endif  
