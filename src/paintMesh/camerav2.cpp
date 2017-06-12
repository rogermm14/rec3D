/*
 * 
 * Project: paintMesh
 * 
 * This is a simple version of the camera class from rec3D project
 * It is simply used to load the camera projection matrices 
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#include <opencv2/opencv.hpp>
#include <unistd.h>

#ifndef __CAMERA_V2_INCLUDED__  
#define __CAMERA_V2_INCLUDED__ 

using namespace cv;
using namespace std;

class Camera {
 
  public:
  
    //attributes
    Mat P, K, R, t;
    int h, w, index, format;
 
    //methods
    Camera(String camerafile, int viewInd);
  
};

Camera::Camera(String camerafile, int viewInd){
  
  index = viewInd;
  P = Mat::zeros(3,4, CV_32FC1);
  K = Mat::zeros(3,3, CV_32FC1);
  R = Mat::zeros(3,3, CV_32FC1);
  t = Mat::zeros(3,1, CV_32FC1);
  
  ifstream file(camerafile.c_str());
  vector<float> data;
  string line;
  while (getline(file, line)) {  
    istringstream stream(line);
    float x;
    while (stream >> x) {
      data.push_back(x);
    }
  }
  format = data[0];
  w = data[1];
  h = data[2];
  int a = 3;
  
  switch (format){
    case 1:
    {
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
}

#endif  
