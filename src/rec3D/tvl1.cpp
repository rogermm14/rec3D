/*
 * 
 * Project: rec3D 
 * 
 * TVL1 AND TVL2 DEPTH FUSION
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#ifndef __TVL1_INCLUDED__  
#define __TVL1_INCLUDED__ 

#include <opencv2/opencv.hpp>
#include <unistd.h>
#include <omp.h>

#ifndef __CAMERA_INCLUDED__ 
#include "camera.cpp"
#endif

#ifndef __BBOX_INCLUDED__ 
#include "bbox.cpp"
#endif

#ifndef __POINTCLOUD_INCLUDED__ 
#include "pointcloud.cpp"
#endif

using namespace cv;
using namespace std;

/*
 * Function name: 'divergence'
 * Description: This function computes the divergence at each voxel inside a 3D bbox
 *
 */
void divergence(Mat_<float> divp, Mat_<float> p1, Mat_<float> p2, Mat_<float> p3, int nx, int ny, int nz){
  
  divp = Mat::zeros(1, nx*ny*nz, CV_32FC1); // Initialize divergence at all voxels with 0 value
  
  omp_set_num_threads(omp_get_max_threads());
  #pragma omp parallel for
  for(int z = 0; z < nz; z++){
    for (int x = 0; x < nx; x++){
      for(int y = 0; y < ny; y++){
	  
	int ind = x + nx * (y + ny * z); // Index of the current voxel, which is the one at (x,y,z)
	int ind_xm1 = ind - 1;           // Index of the voxel at (x-1,y,z)
	int ind_ym1 = ind - nx;          // Index of the voxel at (x,y-1,z)
	int ind_zm1 = ind - nx*ny;       // Index of the voxel at (x,y,z-1)
	
	// Divergence formula: divp = dp/dx + dp/dy + dp/dz
	
	// Compute dp/dx at the current voxel and add it to its divergence
	if (x==0) divp.at<float>(ind) += p1.at<float>(ind);
	else if (x < nx-1) divp.at<float>(ind) += ( p1.at<float>(ind) - p1.at<float>(ind_xm1) );
	else divp.at<float>(ind) -= p1.at<float>(ind_xm1);

	// Compute dp/dy at the current voxel and add it to its divergence
	if (y==0) divp.at<float>(ind) += p2.at<float>(ind);
	else if (y < ny-1) divp.at<float>(ind) += ( p2.at<float>(ind) - p2.at<float>(ind_ym1) );
	else divp.at<float>(ind) -= p2.at<float>(ind_ym1);
	
	// Compute dp/dz at the current voxel and add it to its divergence
	if (z==0) divp.at<float>(ind) += p3.at<float>(ind);
	else if (z < nz-1) divp.at<float>(ind) += ( p3.at<float>(ind) - p3.at<float>(ind_zm1) );
	else divp.at<float>(ind) -= p3.at<float>(ind_zm1);
	
      }
    }
  }

}


/*
 * Function name: 'forwardGradient'
 * Description: This function computes the forward gradient at each voxel inside a 3D bbox
 *
 */
void forwardGradient(Mat f, Mat fx, Mat fy, Mat fz, int nx, int ny, int nz){
  
  omp_set_num_threads(omp_get_max_threads());
  #pragma omp parallel for
  for(int z = 0; z < nz; z++){
    for (int x = 0; x < nx; x++){
      for(int y = 0; y < ny; y++){
	
	int ind = x + nx * (y + ny * z); // Index of the current voxel, which is the one at (x,y,z)
	int ind_xp1 = ind + 1;           // Index of the voxel at (x+1,y,z)
	int ind_yp1 = ind + nx;          // Index of the voxel at (x,y+1,z)
	int ind_zp1 = ind + nx*ny;       // Index of the voxel at (x,y,z+1)
	
	// Compute df/dx
	if(x < nx - 1) fx.at<float>(ind) = f.at<float>(ind_xp1) - f.at<float>(ind);
	else fx.at<float>(ind) = 0.0;
	
	// Compute df/dy  
	if(y < ny - 1) fy.at<float>(ind) = f.at<float>(ind_yp1) - f.at<float>(ind);
	else fy.at<float>(ind) = 0.0;
	
	// Compute df/dz
	if(z < nz - 1) fz.at<float>(ind) = f.at<float>(ind_zp1) - f.at<float>(ind);
	else fz.at<float>(ind) = 0.0;
	
      }
    }
  }

}


/*
 * Function name: 'solveTVL2'
 * Description: This function optimizes the TVL2 functional for depthmap fusion
 *
 */
void solveTVL2(Mat_<float> u, Mat* tvl2, BoundingBox bbox, vector< Mat_<float> > distancefield,
	       float lambda, float theta, int maxIterations, float tau, float tolerance,
	       String tvl2Dir, vector<Mat> Icolor, vector<Camera> P, bool SAVE_CLOUD){

  // Initialize the auxiliary variable v:
  Mat_<float> v = u.clone();
 
  // Initialize divergence and p vector
  Mat_<float> divp = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> p1 = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> p2 = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> p3 = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  
  // Initialize forward gradient
  Mat_<float> ux = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> uy = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> uz = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  
  vector<float> energy_per_it;   // This vector will store the energy of the functional evaluated at each iteration 
  vector<float> error_per_it;    // This vector will store the difference of u between successive iterations 
  error_per_it.push_back(+1e10); // Init 'error_per_it' with infinite error. Just for first iteration, this will be removed later
  bool converged = false;        // 1st convergence criteria: difference with respect previous iteration is smaller than tolerance
  bool small_variation = false;  // 2nd convergence criteria: energy decreased less than 1% in 5 consecutive iterations
  
  int n = 0; // Iterations counter
  
  while (n < maxIterations && converged == false && small_variation == false){
    n++;
      
    // DUAL PROBLEM STEP 1: For u fixed update v
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){	
      float sumW = 0.0;
      float sumTSDF = 0.0;
      for (int k = 0; k < distancefield.size(); k++){
	if (distancefield[k].at<float>(i)!=-5.0){
	  sumTSDF += distancefield[k].at<float>(i);
	  sumW += 1.0;
	}
      }
      float temp = 2.0*lambda*sumTSDF + u.at<float>(i)/theta;
      v.at<float>(i) = temp / ((1.0/theta)+2.0*lambda*sumW);
    }
    
    // DUAL PROBLEM STEP 2: For v fixed update u
    divergence(divp,p1,p2,p3,bbox.nx,bbox.ny,bbox.nz);  // Compute divergence of p
    float est_error = 0.0;                              // Init current iteration error
    
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      float prev_u = u.at<float>(i); 
      u.at<float>(i) = v.at<float>(i) + theta * divp.at<float>(i);
      #pragma omp critical
      est_error += (u.at<float>(i) - prev_u) * (u.at<float>(i) - prev_u);
    }
    
    // Compute error (difference with respect to the previous iteration)
    est_error = est_error/(float)(bbox.nvoxels);
    error_per_it.push_back(est_error);
    if ( est_error < abs(tolerance) ){
      converged = true;
      cout << "\n  Algorithm converged in " << error_per_it.size()-1 << " iterations" << endl;
      cout << "  The difference between 2 consecutive iterations was smaller than the tolerance threshold" << endl;
      break;
    }
    
    if(SAVE_CLOUD){
      // Write intermediate surface in a ply file
      ostringstream outputTVL2PLY;
      outputTVL2PLY << tvl2Dir << "/tvl2_" << n << ".ply";
      vector<int> views_ind;
      for (int v1 = 0; v1 < Icolor.size(); v1++){
        views_ind.push_back(v1);
      }
      saveDistancefieldPLY(u, 0.3, outputTVL2PLY.str(), bbox, true, views_ind, Icolor, P);
    }
    
    // Compute forward gradient
    forwardGradient(u,ux,uy,uz,bbox.nx,bbox.ny,bbox.nz);
    
    // Evaluate energy
    float energy = 0.0;
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      Mat_<float> g = (Mat_<float>(1,3) << ux.at<float>(i), uy.at<float>(i), uz.at<float>(i));
      float A = norm(g,cv::NORM_L2);
      float B = ( ( u.at<float>(i) - v.at<float>(i) ) * ( u.at<float>(i) - v.at<float>(i) ) ) / (2.0 * theta);
      float C = 0.0;
      for (int k = 0; k < distancefield.size(); k++){
	if (distancefield[k].at<float>(i)!=-5.0){
	  C += ( ( v.at<float>(i) - distancefield[k].at<float>(i) ) * ( v.at<float>(i) - distancefield[k].at<float>(i) ) );
	}  
      }
      #pragma omp critical
      energy += (A + B + C);
    }
    energy_per_it.push_back(energy);
    if (energy_per_it.size() > 5 && abs(energy_per_it[n-5] - energy_per_it[n-1]) < energy_per_it[n-5] * 0.01 ){
      small_variation = true;
      cout << "\n  Algorithm converged in " << error_per_it.size()-1 << " iterations" << endl;
      cout << "  Energy decreased less than 1% in 5 consecutive iterations" << endl;  
      break;
    }
    
    // Update vector p
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      float taut = tau / theta;
      Mat_<float> g = (Mat_<float>(1,3) << ux.at<float>(i), uy.at<float>(i), uz.at<float>(i));
      float ng  = 1.0 + taut * norm(g,cv::NORM_L2);
      p1.at<float>(i) = (p1.at<float>(i) + taut * ux.at<float>(i)) / ng;
      p2.at<float>(i) = (p2.at<float>(i) + taut * uy.at<float>(i)) / ng;
      p3.at<float>(i) = (p3.at<float>(i) + taut * uz.at<float>(i)) / ng;
    }
  }
  
  if (converged == false && small_variation == false){
    cout << "\n  Algorithm reached the maximum number of iterations" << endl;
  }
  
  // Write txt file with error per iteration (to debug)
  String errorTXT = tvl2Dir + "/tvl2_error.txt";
  system(("if [ -f "+errorTXT+" ]; then rm "+errorTXT+"; fi").c_str());
  ofstream errorfile;
  errorfile.open (errorTXT.c_str());
  for(int n = 1; n < error_per_it.size() ; n++){
      errorfile << std::fixed << std::setprecision(10) << error_per_it[n] << "\n";
  }
  errorfile.close();
  
  // Write txt file with energy per iteration (to debug)
  String energyTXT = tvl2Dir + "/tvl2_energy.txt";
  system(("if [ -f "+energyTXT+" ]; then rm "+energyTXT+"; fi").c_str());
  ofstream energyfile;
  energyfile.open (energyTXT.c_str());
  for(int n = 0; n < energy_per_it.size() ; n++){
      energyfile << std::fixed << std::setprecision(10) << energy_per_it[n] << "\n";
  }
  energyfile.close();
  
  // Write the output surface in a xml file
  ostringstream outputTVL2XML;
  outputTVL2XML << tvl2Dir << "/tvl2.xml";
  FileStorage file(outputTVL2XML.str().c_str(), FileStorage::WRITE);
  file << "Data" << u;
  file.release();
  
  // Write the output surface in a raw file
  //writeIsoSurfaceRAW(tvl2Dir + "/tvl2.raw", u);
  
  // Write the output surface in a vtk file
  writeIsoSurfaceVTK(tvl2Dir + "/tvl2.vtk", u, bbox.nx, bbox.ny, bbox.nz);
  
  *tvl2 = u; // Return output surface
  
}
  
/*
 * Function name: 'solveTVL1'
 * Description: This function optimizes the TVL1 functional for depthmap fusion
 *
 */
void solveTVL1(Mat_<float> u, Mat* tvl1, BoundingBox bbox, vector< Mat_<float> > distancefield, 
	       float lambda, float theta, int maxIterations, float tau, float tolerance,
	       String tvl1Dir, vector<Mat> Icolor, vector<Camera> P, bool SAVE_CLOUD){
  
  // Initialize the auxiliary variable v:
  Mat_<float> v = u.clone();
 
  // Initialize divergence and p vector
  Mat_<float> divp = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> p1 = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> p2 = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> p3 = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  
  // Initialize forward gradient
  Mat_<float> ux = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> uy = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  Mat_<float> uz = Mat::zeros(1, bbox.nvoxels, CV_32FC1);
  
  vector<float> energy_per_it;   // This vector will store the energy of the functional evaluated at each iteration 
  vector<float> error_per_it;    // This vector will store the difference of u between successive iterations 
  error_per_it.push_back(+1e10); // Init 'error_per_it' with infinite error. Just for first iteration, this will be removed later
  bool converged = false;        // 1st convergence criteria: difference with respect previous iteration is smaller than tolerance
  bool small_variation = false;  // 2nd convergence criteria: energy decreased less than 1% in 5 consecutive iterations
  
  int n = 0; // Iterations counter
  
  while (n < maxIterations && converged == false && small_variation == false){
    n++;
      
    // DUAL PROBLEM STEP 1: For u fixed update v
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      
      // Build the sorted sequence f and extract the median
      vector<float> f;
      f.push_back(-1e10);
      for (int k = 0; k < distancefield.size(); k++){
	if (distancefield[k].at<float>(i)!=-5.0f) f.push_back(distancefield[k].at<float>(i));
      }
      f.push_back(+1e10);
      int length = f.size();
      
      if ( length > 2 ){
	// If the current voxel has weight different from 0 in some distancefields 
	sort(f.begin(), f.end());
	float median;
	if (f.size() % 2 == 0){ 
	  median = (f[length/2 - 1] + f[length/2]) / 2.0;
	}
	else{
	  median = f[floor(length/2)];
	}
	
	// CASE 1: The stationary point lays inside an interval (f[k],f[k+1])
	bool entered_case1 = false;
	//solve
	for (int k = 0; k < length-1; k++) {
	  float temp = u.at<float>(i) - lambda*theta*(2.0*k-(length-2)); // u - lambda*theta*(2k-|I|)
	  
	  if (temp > f[k] && temp < f[k+1] ){
	    v.at<float>(i) = temp;
	    entered_case1 = true;
	  }
	}
	
	// CASE 2: The stationary point can be found among the different f values
	if(!entered_case1){
	  
	  /*
	  // WITHOUT USING SUBCASES
	  float mincost = 0.0;
	  for (int j=1; j < length-1 ; j++){
	    float cost = 0.0;
	    for (int i2 = 1; i2 < length-1; i2++){
	      cost += abs(f[j]-f[i2]);
	    }
	    if (cost < mincost){
	      mincost = cost;
	      v.at<float>(i) = f[j];
	    }
	  }
	  */
	 
	  // SUBCASE 2.1: u <= median
	  if (u.at<float>(i) <= median){
	    float mincost = 0.0;
	    for (int j=1; j <= length/2; j++){
	      float cost = 0.0;
	      for (int i2 = 1; i2 < length-1; i2++){
		cost += abs(f[j]-f[i2]);
	      }
	      if (cost < mincost){
		mincost = cost;
		v.at<float>(i) = f[j];
	      }
	    }
	  }
	  // SUBCASE 2.2: // u > median
	  else{
	    float mincost = 0.0;
	    for (int j = ceil(length/2); j < length-1; j++){
	      float cost = 0.0;
	      for (int i2 = 1; i2 < length-1; i2++){
		cost += abs(f[j]-f[i2]);
	      }
	      if (cost < mincost){
		mincost = cost;
		v.at<float>(i) = f[j];
	      }
	    }
	  }
	  
	  
	}
      }
      else{
	// If the current voxel has weight equal to 0 in all distancefields 
	v.at<float>(i) = u.at<float>(i);
      }
    }
    
    // DUAL PROBLEM STEP 2: For v fixed update u
    divergence(divp,p1,p2,p3,bbox.nx,bbox.ny,bbox.nz);  // Compute divergence of p
    float est_error = 0.0;                              // Init current iteration error
    
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      float prev_u = u.at<float>(i); 
      u.at<float>(i) = v.at<float>(i) + theta * divp.at<float>(i);
      #pragma omp critical
      est_error += (u.at<float>(i) - prev_u) * (u.at<float>(i) - prev_u);
    }
    
    // Compute error (difference with respect to the previous iteration)
    est_error = est_error/(float)(bbox.nvoxels);
    error_per_it.push_back(est_error);
    if ( est_error <=  abs(tolerance) ){
      converged = true;
      cout << "\n  Algorithm converged in " << error_per_it.size()-1 << " iterations" << endl;
      cout << "  The difference between 2 consecutive iterations was smaller than the tolerance threshold" << endl;
      break;
    }
    
    if(SAVE_CLOUD){
      // Write intermediate surface in a ply file
      ostringstream outputTVL1PLY;
      outputTVL1PLY << tvl1Dir << "/tvl1_" << n << ".ply";
      vector<int> views_ind;
      for (int v1 = 0; v1 < Icolor.size(); v1++){
        views_ind.push_back(v1);
      }
      saveDistancefieldPLY(u, 0.3, outputTVL1PLY.str(), bbox, true, views_ind, Icolor, P);
    }
    
    // Compute forward gradient
    forwardGradient(u,ux,uy,uz,bbox.nx,bbox.ny,bbox.nz);
    
    // Evaluate energy
    float energy = 0.0;
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      Mat_<float> g = (Mat_<float>(1,3) << ux.at<float>(i), uy.at<float>(i), uz.at<float>(i));
      float A = norm(g,cv::NORM_L2);
      float B = ( ( u.at<float>(i) - v.at<float>(i) ) * ( u.at<float>(i) - v.at<float>(i) ) ) / (2.0 * theta);
      float C = 0.0;
      for (int k = 0; k < distancefield.size(); k++){
	if (distancefield[k].at<float>(i)!=-5.0){
	  C += abs( v.at<float>(i) - distancefield[k].at<float>(i) );
	}
      }
      #pragma omp critical
      energy += (A + B + C);
    }
    energy_per_it.push_back(energy);
    if (energy_per_it.size() > 5 && abs(energy_per_it[n-5] - energy_per_it[n-1]) < energy_per_it[n-5] * 0.01 ){
      small_variation = true;
      cout << "\n  Algorithm converged in " << error_per_it.size()-1 << " iterations" << endl;
      cout << "  Energy decreased less than 1% in 5 consecutive iterations" << endl;  
      break;
    }
    
    // Update vector p
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for (int i = 0; i < bbox.nvoxels; i++){
      float taut = tau / theta;
      Mat_<float> g = (Mat_<float>(1,3) << ux.at<float>(i), uy.at<float>(i), uz.at<float>(i));
      float ng  = 1.0 + taut * norm(g,cv::NORM_L2);
      p1.at<float>(i) = (p1.at<float>(i) + taut * ux.at<float>(i)) / ng;
      p2.at<float>(i) = (p2.at<float>(i) + taut * uy.at<float>(i)) / ng;
      p3.at<float>(i) = (p3.at<float>(i) + taut * uz.at<float>(i)) / ng;
    }
  }

  if (converged == false && small_variation == false){
    cout << "\n  Algorithm reached the maximum number of iterations." << endl;
  }
  
  // Write txt file with error per iteration (to debug)
  String errorTXT = tvl1Dir + "/tvl1_error.txt";
  system(("if [ -f "+errorTXT+" ]; then rm "+errorTXT+"; fi").c_str());
  ofstream errorfile;
  errorfile.open (errorTXT.c_str());
  for(int n = 1; n < error_per_it.size() ; n++){
      errorfile << std::fixed << std::setprecision(10) << error_per_it[n] << "\n";
  }
  errorfile.close();
  
  // Write txt file with energy per iteration (to debug)
  String energyTXT = tvl1Dir + "/tvl1_energy.txt";
  system(("if [ -f "+energyTXT+" ]; then rm "+energyTXT+"; fi").c_str());
  ofstream energyfile;
  energyfile.open (energyTXT.c_str());
  for(int n = 0; n < energy_per_it.size() ; n++){
      energyfile << std::fixed << std::setprecision(10) << energy_per_it[n] << "\n";
  }
  energyfile.close();
  
  // Write the output surface in a xml file
  ostringstream outputTVL1XML;
  outputTVL1XML << tvl1Dir << "/tvl1.xml";
  FileStorage file(outputTVL1XML.str().c_str(), FileStorage::WRITE);
  file << "Data" << u;
  file.release();
  
  // Write the output surface in a raw file
  //writeIsoSurfaceRAW(tvl1Dir + "/tvl1.raw", u);
  
  // Write the output surface in a vtk file
  writeIsoSurfaceVTK(tvl1Dir + "/tvl1.vtk", u, bbox.nx, bbox.ny, bbox.nz);
  
  *tvl1 = u; // Return output surface
}
	  
#endif  