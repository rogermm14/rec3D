/*
 * 
 * Project: rec3D 
 * 
 * PLANE SWEEP ALGORITHM
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#ifndef __PLANESWEEP_INCLUDED__  
#define __PLANESWEEP_INCLUDED__ 

#include <opencv2/opencv.hpp>
#include <unistd.h>

using namespace cv;
using namespace std;

/*
 * Function name: 'linspace'
 * Description: C++ of the Matlab function 'linspace'. Returns a vector of N uniformly distributed values between a and b. 
 *
 */
vector<float> linspace(float a, float b, int N) {
    vector<float> array;
    float step = (float)((b-a)/(N-1));
    while(a <= b) {
        array.push_back(a);
        a += step;
    }
    return array;
}


/*
 * Function name: 'computeNCC'
 * Description: This function computes the Normalized Cross Correlation (NCC) similarity cost between two images
 *
 */
Mat_<float> computeNCC(Mat image1, Mat image2, int windowSize) {
  
  int sigma = windowSize/2; // Set standard deviation for the Gaussian filter
  
  // The formulation of the Normalized Cross Correlation being applied is the following one:
  // Gaussian filter(sigma,windowSize) * ( (I1 - muI1)/sigmaI1 * (I2 - muI2)/sigmaI2 )
  
  // Obtain I1, muI1 and sigmaI1
  Mat_<float> I1 = image1.clone();
  Mat_<float> muI1, I1s, muI1s, muI1I1, sigmaI1, tempI1;
  multiply(I1, I1, I1s);
  GaussianBlur(I1, muI1, Size(windowSize, windowSize), sigma, sigma);    
  GaussianBlur(I1s, muI1I1, Size(windowSize, windowSize), sigma, sigma);
  multiply(muI1, muI1, muI1s);
  subtract(muI1I1, muI1s,tempI1);
  tempI1 = tempI1 + 0.000001;
  sqrt(tempI1, sigmaI1);

  // Obtain I2, muI2 and sigmaI2
  Mat_<float> I2 = image2.clone();	
  Mat_<float> muI2, I2s, muI2s, muI2I2, sigmaI2, tempI2;
  multiply(I2, I2, I2s);
  GaussianBlur(I2, muI2, Size(windowSize, windowSize), sigma, sigma);     
  GaussianBlur(I2s, muI2I2, Size(windowSize, windowSize), sigma, sigma);
  multiply(muI2, muI2, muI2s);
  subtract(muI2I2, muI2s,tempI2);
  tempI2 = tempI2 + 0.000001;
  sqrt(tempI2, sigmaI2);
  
  // Compute NCC using the formula noted above
  Mat_<float> I1c, I2c, A, B, C, cost;
  subtract(I1,muI1,I1c);
  subtract(I2,muI2,I2c);
  divide(I1c,sigmaI1,A);
  divide(I2c,sigmaI2,B);
  multiply(A,B,C);
  GaussianBlur(C, cost, Size(windowSize, windowSize), sigma, sigma);
  
  // Delete margins
  Mat final_cost = (-1.0)*Mat::ones(cost.rows, cost.cols, CV_32FC1);
  Mat temp = cost(Rect(windowSize/2+1, windowSize/2+1, cost.cols-windowSize-1, cost.rows-windowSize-1));
  temp.copyTo(final_cost(Rect(windowSize/2+1, windowSize/2+1, cost.cols-windowSize-1, cost.rows-windowSize-1)));
  
  return final_cost;
}

/*
 * Function name: 'computeSSD'
 * Description: This function computes the normalized Sum of Squared Differences (SSD) similarity cost between two images
 *
 */
Mat_<float> computeSSD(Mat im1, Mat im2, int windowSize) {

  Mat difference, SSD, cost;
  subtract(im1,im2,difference);
  multiply(difference, difference, SSD);
  SSD = SSD/(255.0*255.0); // normalize
  
  /*
  double sigmaColor = (double)windowSize/2; // Set standard deviation for the Bilateral filter
  double sigmaSpace = (double)windowSize/2; // Set standard deviation for the Bilateral filter
  bilateralFilter(SSD,cost, windowSize, sigmaColor, sigmaSpace );
  */
  
  GaussianBlur(SSD, cost, Size(windowSize, windowSize), windowSize/2, windowSize/2);
  
  // Delete margins
  Mat final_cost = Mat::ones(cost.rows, cost.cols, CV_32FC1);
  Mat temp = cost(Rect(windowSize/2+1, windowSize/2+1, cost.cols-windowSize-1, cost.rows-windowSize-1));
  temp.copyTo(final_cost(Rect(windowSize/2+1, windowSize/2+1, cost.cols-windowSize-1, cost.rows-windowSize-1)));
  
  return final_cost;
}

/*
 * Function name: 'computeSAD'
 * Description: This function computes the normalized Sum of Absolute Differences (SAD) similarity cost between two images
 *
 */
Mat_<float> computeSAD(Mat im1, Mat im2, int windowSize) {

  Mat SAD, cost;
  absdiff(im1, im2, SAD);
  SAD = SAD/255.0; // normalize
  
  /*
  double sigmaColor = (double)windowSize/2; // Set standard deviation for the Bilateral filter
  double sigmaSpace = (double)windowSize/2; // Set standard deviation for the Bilateral filter
  bilateralFilter(SAD,cost, windowSize, sigmaColor, sigmaSpace );
  */
  
  GaussianBlur(SAD, cost, Size(windowSize, windowSize), windowSize/2, windowSize/2);
  
  // Delete margins
  Mat final_cost = Mat::ones(cost.rows, cost.cols, CV_32FC1);
  Mat temp = cost(Rect(windowSize/2+1, windowSize/2+1, cost.cols-windowSize-1, cost.rows-windowSize-1));
  temp.copyTo(final_cost(Rect(windowSize/2+1, windowSize/2+1, cost.cols-windowSize-1, cost.rows-windowSize-1)));
  
  return final_cost;
}


/*
 * Function name: 'planeSweep'
 * Description: This function implements the Plane Sweep Algorithm for depthmap computation
 *
 */
void planeSweep(Mat I1, Mat I2, Mat P1, Mat P2, Mat M, Mat* outputDepthmap, String similarityMeasure,
		float conf, float mindepth, float maxdepth, int nplanes, int windowSize){
  
  // About the input parameters:
  //    I1 -> Matrix with the reference view
  //    I2 -> Matrix with the second view
  //    P1 -> Projection matrix of the camera used to take the reference view
  //    P2 -> Projection matrix of the camera used to take the second view
  //    M  -> Mask associated to the reference view
  //    outputDepthmap -> Pointer to the matrix where the output depthmap will be stored
  //    conf -> Minimum similarity value necessary to consider a depth estimation reliable
  //    mindepth -> Depth of the first fronto-parallel plane 
  //    maxdepth -> Depth of the last fronto-parallel plane
  //    nplanes -> Number of fronto-parallel planes to be used by the plane sweep algorithm
  //    windowSize -> Sides length of the neighbourhood window used for the similarity measure

  // Get input images size
  int h = I1.rows;
  int w = I2.cols;
  
  // Convert type of input images to signed float32 
  I1.convertTo(I1, CV_32FC1);
  I2.convertTo(I2, CV_32FC1);
  
  // Create 1xN vector with the depths of the N fronto-prallel planes
  // The N fronto-parallel planes are distributed between mindepth and maxdepth uniformly using the 'linspace' function 
  vector<float> depths = linspace(mindepth,maxdepth,nplanes);
  
  // Create hxw matrix to store the result of the similarity measure obtained at a certain depth per pixel
  Mat score = Mat::zeros(h, w, CV_32FC1);
  
  // Create hxw matrix to store the best depth (that is the one that produces the highest similarity measure) per pixel
  Mat bestDepth = Mat::zeros(h, w, CV_32FC1);
  
  // Create hxw matrix to store the best score obtained along all the iterations of the algorithm 
  Mat bestScore;
  if (similarityMeasure == "NCC") bestScore = (-1.0)*Mat::ones(h, w, CV_32FC1);
  else bestScore = Mat::ones(h, w, CV_32FC1);
  
  // Support matrix used to compare 'score' and 'bestScore'
  Mat newBestScore = Mat::zeros(h, w, CV_32FC1);
  
  for (int i = 0; i < depths.size(); i++){
    
    // Compute the fronto-parallel plane at the current depth
    Mat plane = P1.row(2) - (Mat_<float>(1,4) << 0,0,0,depths[i]);

    // Compute the homography from reference view to second view via the current fronto-parallel plane
    Mat_<float> A = P1.clone();
    A.push_back(plane);
    Mat invA = A.inv();
    Mat Hplane = P2*invA(Range::all(),Range(0,3));

    // Warp the second view to the reference view using the previous homography
    Mat warped;
    warpPerspective(I2, warped, Hplane.inv(), warped.size());
    
    // Check warped images (useful to debug)
    //namedWindow( "Display Image", CV_WINDOW_AUTOSIZE );
    //imshow( "Display Image", warped/255 );
    //waitKey(0);
    //destroyWindow("Display Image");
    
    // Compute similarity between the warpped image and the reference view
    if (similarityMeasure == "SAD"){
      score = computeSAD(I1, warped, windowSize);
      min(score,bestScore,newBestScore);
    }
    else if (similarityMeasure == "SSD"){
      score = computeSSD(I1, warped, windowSize);
      min(score,bestScore,newBestScore);
    }
    else{
      score = computeNCC(I1, warped, windowSize);
      max(score,bestScore,newBestScore);
    }
    
    // Update the best similarity score
    bestScore = newBestScore;
    
    // Set best depth according to best similarity score
    bestDepth.setTo(depths[i],score==bestScore);
  }
 
  // Create hxw matrix that will retain only the interesting depths
  Mat_<float> clippedDepth = Mat::zeros(h, w, CV_32FC1);
  
  // Build first mask (the one given by the user as input)
  Mat mask1 = Mat::zeros(h, w, CV_32FC1);
  compare(M,0.9,mask1,cv::CMP_GE);
  mask1 = mask1/255.0;
  
  // Build second mask (this one considers only those depths with similarity score > conf)  
  Mat mask2 = Mat::zeros(h, w, CV_32FC1);
  if (similarityMeasure == "NCC") compare(bestScore,conf,mask2,cv::CMP_GT);
  else compare(bestScore,conf,mask2,cv::CMP_LT);
  mask2 = mask2/255.0; 
  
  // Build final mask out of mask1 and mask2
  Mat_<float> maskFinal = Mat::zeros(h, w, CV_32FC1);
  multiply(mask1,mask2,maskFinal);

  // Clip depths outside the final mask and assign them a dedicated value: 255.0
  multiply(bestDepth,maskFinal,clippedDepth);
  clippedDepth.setTo(255.0,maskFinal==0);
  
  // Return output depthmap and associated depth confidences
  *outputDepthmap = clippedDepth;
}


/*
 * Function name: 'saveDepthmapPNG'
 * Description: This function writes depthmaps as png images
 *
 */
void saveDephtmapPNG(Mat_<float> depthmap, std::string outputPNGfilename ){
  
  Mat depthmapPNG = depthmap.clone();
  
  // In this function we want to maximize the range of display for the png depthmap 
  // This way we will be able to appreciate a wider variety of greyscale values representing different depths
  // In other words, we are not interested in representing 200 different depths in the range (e.g.) (1.0, 1.7)
  // It is nicer if we represent 200 different depths in the range (0,255)
  
  depthmapPNG.setTo(NAN,depthmap==255.0); // Set pixels outside the mask to NAN so that they do not affect the minMaxLoc function
  double min, max;                        // Annotation: min and max must be double to get minMaxLoc to work properly
  cv::minMaxLoc(depthmapPNG, &min, &max);
  //cout << min << endl;
  //cout << max << endl;
  
  // Up until now, depths had values in the range (min, max)
  // As said before, we want to set these values in the range (0, 255) to produce a nicer png image
  // The lines below use linear interpolation to do that
  float newmin = 0, newmax = 255;
  float factor = (newmax-newmin)/(max-min);
  depthmapPNG = (depthmapPNG - min) * factor + newmin;
  
  depthmapPNG.setTo(255.0,depthmap==255.0); // Set the pixels outside the mask to white by assigning them value 255.0
  
  depthmapPNG.convertTo(depthmapPNG, CV_8UC1);       // Convert matrix from signed float32 to unsigned int8 (with 1 channel)
  imwrite( outputPNGfilename.c_str(), depthmapPNG ); // Save the depthmap as a png image
 
}

void addSaltPepperNoiseDephtmap(Mat inputCleanDepthmap, Mat* outputNoisyDepthmap){
  
  Mat depthmap = inputCleanDepthmap.clone();
  
  depthmap.setTo(NAN,inputCleanDepthmap==255.0);
  double min, max;                        
  cv::minMaxLoc(depthmap, &min, &max);
  depthmap.setTo(255.0,inputCleanDepthmap==255.0); 
  
  Mat saltpepper_noise = Mat::zeros(depthmap.rows, depthmap.cols,CV_32FC1);
  randu(saltpepper_noise,min,max);

  Mat black = saltpepper_noise < min + (max-min)*0.1;
  Mat white = saltpepper_noise > max - (max-min)*0.1;

  Mat saltpepper_depthmap = depthmap.clone();
  saltpepper_depthmap.setTo(max,white);
  saltpepper_depthmap.setTo(min,black);
  
  *outputNoisyDepthmap = saltpepper_depthmap;
  
}

#endif  
