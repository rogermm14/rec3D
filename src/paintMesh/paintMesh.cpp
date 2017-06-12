/*
 * 
 * Project: paintMesh
 * 
 * This program is used to paint the mesh resulting from a multi-view 3D reconstruction
 * The vertices of the mesh are projected to the images of the multi-view dataset to decide 
 * which is the most suitable color for each vertex
 * The painted mesh is written in an output ply file
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <getopt.h>
#include <omp.h>

#include "vtkSmartPointer.h"
#include <vtkSTLReader.h>
#include <vtkPLYReader.h>
#include <vtkPLYWriter.h>
#include <vtkPolyData.h>
#include <vtkTriangle.h>
#include <vtkTriangleFilter.h>

#ifndef __CAMERA_V2_INCLUDED__ 
#include "camerav2.cpp"
#endif


using namespace cv;
using namespace std;

// Define 'Vertex' and 'Face' structures
// They will be used later to write the painted mesh in the output ply file
struct Vertex {
    float x,y,z;
    int r,g,b;
  };
struct Face {
    int v0_ind, v1_ind, v2_ind;
  };
  
/*
 * Function name: 'saveDepthProjectionsPNG'
 * Description: This function writes depthmaps as png images
 *
 */
void saveDepthimagePNG(Mat_<float> depthmap, std::string outputPNGfilename ){
  
  Mat depthmapPNG = depthmap.clone();
  
  // In this function we want to maximize the range of display for the png image containing the depth projections 
  // This way we will be able to appreciate a wider variety of greyscale values representing different depths
  // In other words, we are not interested in representing 200 different depths in the range (e.g.) (1.0, 1.7)
  // It is nicer if we represent 200 different depths in the range (0,255)
  
  depthmapPNG.setTo(NAN,depthmap>200.0); // Set pixels with no depth associated to NAN so that they do not affect the minMaxLoc function
  double min, max;                       // Annotation: min and max must be double to get minMaxLoc to work properly
  cv::minMaxLoc(depthmapPNG, &min, &max);
  //cout << min << endl;
  //cout << max << endl;
  
  // Up until now, depths had values in the range (min, max)
  // As said before, we want to set these values in the range (0, 255) to produce a nicer png image
  // The lines below use linear interpolation to do that
  float newmin = 0, newmax = 255;
  float factor = (newmax-newmin)/(max-min);
  depthmapPNG = (depthmapPNG - min) * factor + newmin;
  
  depthmapPNG.setTo(255.0,depthmap==255.0); // Assign the value 255.0 to the pixels with no depth associated
  
  depthmapPNG.convertTo(depthmapPNG, CV_8UC1);       // Convert matrix from signed float32 to unsigned int8 (with 1 channel)
  imwrite( outputPNGfilename.c_str(), depthmapPNG ); // Save the depthmap as a png image
 
}

/*
 * Function name: 'getMedian'
 * Description: This function returns the median value of an input vector f
 *
 */
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

/*
 * Function name: 'getMean'
 * Description: This function returns the mean value of an input vector f
 *
 */
float getMean(vector<float> f){
  
  int length = f.size();
  float sum;
  for (int i = 0; i < length; i++){
   sum += f[i];  
  }
  
  return (float)sum/length;
}


// *****************************************************
// ***************** MAIN PROGRAM START ****************
// *****************************************************

int main( int argc, char** argv ){

  int64 e1 = getTickCount();
  
  // Initialize parameters
  string inputSimpleMesh, outputDir, camerasFile, gridFile, imagesFile;
  string paintingMethod = "median";
  bool DEBUG = false;
  
  const char* const short_opts = "i:o:v:l:m:d:h";
  
  static struct option long_opts[] =
  {
    {"i",     required_argument, 0, 'i'},
    {"o",     required_argument, 0, 'o'},
    {"img",   required_argument, 0, 'a'},
    {"bbox",  required_argument, 0, 'b'},
    {"cam",   required_argument, 0, 'c'},
    {"method",required_argument, 0, 'm'},
    {"debug", no_argument,       0, 'd'},
    {"help",  no_argument,       0, 'h'},
    {0, 0, 0, 0}
  };
 
  int opt;
  int option_index = 0;
  
  while ((opt = getopt_long(argc, argv, short_opts,long_opts,&option_index)) != -1){
    switch (opt){
      case 'i':
	inputSimpleMesh = optarg;
	break;
      case 'o':
	outputDir = optarg;
        break;
      case 'a':
	imagesFile = optarg; 
	break;
      case 'b':
	gridFile = optarg;
	break;
      case 'c':
	camerasFile = optarg;
	break;
      case 'm':
	paintingMethod = optarg;
	break;
      case 'd':
	DEBUG = true;
	break;
      case 'h':
	cout << "\nList of available options:\n\n"
	     << "	--i       <input .ply file containing input mesh>\n"
	     << "	--o       <output directory>\n"
	     << "	--img     <txt file with images paths>\n"
	     << "	--bbox    <txt file with grid details>\n"
	     << "	--cam     <txt file with camera paths>\n"
	     << "	--method  <painting method: 'mean' or 'median'>  [OPTIONAL] Default: median\n"
	     << "	--debug   <debug mode (saves synthetic depths)>  [OPTIONAL] Default: OFF \n"
	     << endl;
	return  EXIT_SUCCESS;
	break;
      }
  }
  
  // Check if input command is valid according to the number of aguments
  if (argc < 11){
    cerr << "\nERROR: Invalid command.\n"
	 << "\nValid command example: $ ./paintMesh --i no_color_mesh.ply --o color_mesh.ply --img images.txt --bbox grid.txt --cam cameras.txt\n"
	 << "\nCheck list of available options: $ ./paintMesh --help\n" << endl;
    return EXIT_FAILURE;
  }
  
  cout << "\nYou called:\n\n"
       << "	--i       <input .ply file containing input mesh>  " + inputSimpleMesh  + "\n"
       << "	--o       <output directory>                       " + outputDir + "\n"
       << "	--img     <txt file with images paths>             " + imagesFile + "\n"
       << "	--bbox    <txt file with grid details>             " + gridFile + "\n"
       << "	--cam     <txt file with camera paths>             " + camerasFile + "\n"
       << "	--method  <painting method: 'mean' or 'median'>    " + paintingMethod + "\n"
       << "	--debug   <debug mode (saves synthetic depths)>    "; (!DEBUG)         ? cout << "OFF\n" : cout << "ON\n";
  
  // Check if input .ply file exists
  if (system(("test -f "+inputSimpleMesh).c_str())!=0){
    cerr << "\nERROR: Input mesh does not exist." << endl;
    return EXIT_FAILURE;
  }
  
  // Output coloured mesh will have the same name as the input one followed by '-painted'
  string inputFilenameWithExtension = inputSimpleMesh.substr(inputSimpleMesh.find_last_of("/") + 1);
  string inputFilename = inputFilenameWithExtension.substr(0, inputFilenameWithExtension.find_last_of(".") );
  String outputRGBMesh = outputDir + "/" + inputFilename + "-painted.ply";
  
  
  // *****************************************************
  // ***************** LOAD INPUT DATA *******************
  // *****************************************************
  
  // Read non-colored mesh from input .ply file
  vtkSmartPointer<vtkPLYReader> readerPLY = vtkSmartPointer<vtkPLYReader>::New();
  vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New();
  readerPLY->SetFileName(inputSimpleMesh.c_str());
  readerPLY->Update();
  mesh = readerPLY->GetOutput();
  triangleFilter->SetInputConnection(readerPLY->GetOutputPort());
  triangleFilter->Update();
 
  // Get number of vertices and faces
  int numberOfVertices = mesh->GetNumberOfPoints();
  int numberOfFaces = mesh->GetNumberOfCells();
  
  // Read input bbox 
  ifstream bboxfile(gridFile.c_str());
  vector<float> bboxdata;
  string bboxline;
  while (getline(bboxfile, bboxline)) {  
    istringstream stream(bboxline);
    float x;
    while (stream >> x) {
      bboxdata.push_back(x);
    }
  }
  int nx = bboxdata[0];
  int ny = bboxdata[1];
  int nz = bboxdata[2];
  float xmin = bboxdata[3];
  float ymin = bboxdata[4];
  float zmin = bboxdata[5];
  float voxelSize = bboxdata[6];
  
  // Read input images
  ifstream imagesfile(imagesFile.c_str());
  vector<std::string> imagesdata;
  string imagesline;
  while (std::getline(imagesfile, imagesline)) {  
    istringstream stream(imagesline);
    string x;
    while (stream >> x) {
      imagesdata.push_back(x);
    }
  }
  std::vector<Mat> images;
  for (int i = 0; i < imagesdata.size(); i++){
    images.push_back(imread(imagesdata[i].c_str(),CV_LOAD_IMAGE_COLOR));
  }

  // Read input cameras
  ifstream camerasfile(camerasFile.c_str());
  vector<std::string> camerasdata;
  string camerasline;
  while (std::getline(camerasfile, camerasline)) {  
    istringstream stream(camerasline);
    string x;
    while (stream >> x) {
      camerasdata.push_back(x);
    }
  }
  vector<Camera> P;
  for(size_t i = 0; i < images.size(); i++){  
   P.push_back(Camera(camerasdata[i],i));
  }
  
  // Check that the input number of images and cameras is the same
  if( camerasdata.size()!= imagesdata.size()){
    cerr << "\nERROR: The number of camera files and image files is not the same." << endl;
    return EXIT_FAILURE;
  }

  
  // *****************************************************
  // ********* CREATE SYNTHETIC DEPTH IMAGES *************
  // *****************************************************

  // Here we build one synthetic depth image per camera by projecting 
  // the vertices of the mesh using the associated camera matrix
  // We keep only the closest vertices to each camera
  // This will eventually be used to figure out which vertices are occluded 
  // in each view and use only the color of the non-occluded ones to paint the mesh
  
  vector< Mat_<float> > depthimages;
  vector<Vertex> vertices(numberOfVertices);
  
  for (int i=0; i < camerasdata.size(); i++){
      depthimages.push_back( (255.0)*Mat::ones(P[0].h, P[0].w, CV_32FC1) );
  }
  
  omp_set_num_threads(omp_get_max_threads());
  #pragma omp parallel for
  for(int i = 0; i < numberOfVertices; i++){
    
    // Get current vertex coordinates in the 3D space
    double p[3];
    mesh->GetPoint(i,p);
    float xcoord = p[0]*voxelSize + xmin;
    float ycoord = p[1]*voxelSize + ymin;
    float zcoord = p[2]*voxelSize + zmin;
 
    // Add the current vertex to the vertices vector
    Vertex currentVertex;
    currentVertex.x = p[0];
    currentVertex.y = p[1];
    currentVertex.z = p[2];
    currentVertex.r = -1;
    currentVertex.g = -1;
    currentVertex.b = -1;
    vertices[i] = currentVertex;
    
    
    for(int v = 0; v < images.size(); ++v){
      
      // Project vertex onto the image v
      Mat projection = P[v].P*(Mat_<float>(4,1) << xcoord,ycoord,zcoord,1.0);
      float xp = projection.at<float>(0,0);
      float yp = projection.at<float>(1,0);
      float zp = projection.at<float>(2,0);
      xp = cvRound(xp/zp);
      yp = cvRound(yp/zp);
      
      // Check if the projection falls inside the image limits and if this voxel is the nearest to the camera plane
      if (xp >= 0 && xp < P[v].w && yp >= 0 && yp < P[v].h && zp < depthimages[v].at<float>(yp,xp)){
	depthimages[v].at<float>(yp,xp) = zp;
      }   
    }
  }
  
  
  // *****************************************************
  // ******* INTERPOLATE SYNTHETIC DEPTH IMAGES **********
  // *****************************************************
  
  vector< Mat_<float> > interpol_depth;
  for (int i=0; i < camerasdata.size(); i++){
      interpol_depth.push_back( (255.0)*Mat::ones(P[0].h, P[0].w, CV_32FC1) );
  }
  
  // Interpolate
  int w = 7; // Window size
  
  omp_set_num_threads(omp_get_max_threads());
  #pragma omp parallel for
  for(int v = 0; v < images.size(); ++v){
    
    for (int i = w/2; i < P[v].w-w/2; i++){
      for (int j = w/2; j < P[v].h-w/2; j++){

	  Mat patch = depthimages[v](Range(j-w/2,j+w/2+1),Range(i-w/2,i+w/2+1));
	  double min, max;                       
	  cv::minMaxLoc(patch, &min, &max);
	  if (min != 255.0){
	    interpol_depth[v](Range(j-w/2,j+w/2+1),Range(i-w/2,i+w/2+1)) = min*Mat::ones(w,w,CV_32FC1);
	  }
      }
    }
    
    // Apply bilateral filter to smooth the result while preserving edges
    Mat temp;
    bilateralFilter(interpol_depth[v],temp, (double)w, (double)w/6.0, (double)w/6.0 );
    interpol_depth[v] = temp;
  }
  
  if(DEBUG){
  
    // Save the resulting images (useful to debug)
    system(("mkdir -p "+outputDir+"/depth-painting").c_str());
    for (int i=0; i < images.size(); i++){
      ostringstream outputDepthimagePNG;
      outputDepthimagePNG << outputDir << "/synthetic-depths-interpolated/depthimage_" << i << ".png";
      saveDepthimagePNG(interpol_depth[i], outputDepthimagePNG.str());
    }
  
    system(("mkdir -p "+outputDir+"/depth-painting2").c_str());
    for (int i=0; i < images.size(); i++){
      ostringstream outputDepthimagePNG;
      outputDepthimagePNG << outputDir << "/synthetic-depth/depthimage_" << i << ".png";
      saveDepthimagePNG(depthimages[i], outputDepthimagePNG.str());
    }
  }
  
  // *****************************************************
  // ********* GET VERTICES AND COLOR THEM ***************
  // *****************************************************
  
  int nonPaintedVertices = numberOfVertices;
  float diffMargin = 0.005;
  int maxIter = 30;
  int it = 0;
  
  while (nonPaintedVertices > 0 && it < maxIter){
  
    diffMargin = diffMargin*2;
    it++;
    
    cout << "Difference margin: " << diffMargin << endl;
    cout << "Iteration: " << it << endl;
    
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for(int i = 0; i < numberOfVertices; i++){
      
      if(vertices[i].r == -1 && vertices[i].g == -1 && vertices[i].b == -1){
      
	// Get current vertex coordinates in the 3D space
	double p[3];
	mesh->GetPoint(i,p);
	float xcoord = p[0]*voxelSize + xmin;
	float ycoord = p[1]*voxelSize + ymin;
	float zcoord = p[2]*voxelSize + zmin;
	
	// Initialize RGB values
	int R,G,B;
	vector<float> Rvec,Bvec,Gvec;

	for(int v = 0; v < images.size(); ++v){
	  
	  // Project vertex onto the image v
	  Mat projection = P[v].P*(Mat_<float>(4,1) << xcoord,ycoord,zcoord,1.0);
	  float xp = projection.at<float>(0,0);
	  float yp = projection.at<float>(1,0);
	  float zp = projection.at<float>(2,0);
	  xp = cvRound(xp/zp);
	  yp = cvRound(yp/zp);
	  
	  // Check if the projection falls inside the image limits and it is not occluded
	  if (xp>=0 && xp<P[v].w && yp>=0 && yp<P[v].h && abs(zp-interpol_depth[v].at<float>(yp,xp))<=diffMargin){
	    if (images[v].at<Vec3b>(yp,xp)[0] != 0 && images[v].at<Vec3b>(yp,xp)[1] != 0 && images[v].at<Vec3b>(yp,xp)[2] != 0){
	      // Get color	
	      Bvec.push_back(images[v].at<Vec3b>(yp,xp)[0]);
	      Gvec.push_back(images[v].at<Vec3b>(yp,xp)[1]);
	      Rvec.push_back(images[v].at<Vec3b>(yp,xp)[2]);
	    }
	  }
	  
	}
	
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
	  vertices[i].r = R;
	  vertices[i].g = G;
	  vertices[i].b = B;

	  if(R == 0 && G == 0 && B == 0){
	    for (int i = 0; i < Rvec.size(); i++) cout << Rvec[i] << endl;
	  }
	  
          #pragma omp critical
	  nonPaintedVertices--;
	}
	else{
	  // The current vertex remains not painted and the algorithm will try to paint it
	  // in the following iteration by enlarging the depth difference margin ('diffMargin')
	} 
      }
      
    }
    
    cout << "Non painted vertices: " << nonPaintedVertices << endl;
  
  }
  
  // Just in case there are vertices not painted, then set their color to black
  if (nonPaintedVertices > 0){
    cout << "Some vertices were not painted." << endl;
    omp_set_num_threads(omp_get_max_threads());
    #pragma omp parallel for
    for(int i = 0; i < numberOfVertices; i++){  
      if(vertices[i].r == -1 && vertices[i].g == -1 && vertices[i].b == -1){ 
        vertices[i].r = 0;
	vertices[i].g = 0;
	vertices[i].b = 0;
      }
    }
  }


  // *****************************************************
  // ***************** GET FACES *************************
  // *****************************************************
  
  vector<Face> faces(numberOfFaces);
  
  for (vtkIdType i = 0; i < numberOfFaces; i++){
   
    // Get ids of the face vertices
    vtkSmartPointer<vtkIdList> cellPointIds = vtkSmartPointer<vtkIdList>::New();
    triangleFilter->GetOutput()->GetCellPoints(i, cellPointIds);
    int v0_ind = cellPointIds->GetId(0);
    int v1_ind = cellPointIds->GetId(1);
    int v2_ind = cellPointIds->GetId(2);
    
    // Add the current face to the faces vector
    Face currentFace;
    currentFace.v0_ind = v0_ind;
    currentFace.v1_ind = v1_ind;
    currentFace.v2_ind = v2_ind;
    
    faces[i] = currentFace;
  }

  
  // *****************************************************
  // **************** WRITE OUTPUT PLY *******************
  // *****************************************************
  
  ofstream outFile( outputRGBMesh.c_str());
  // Write file header
  outFile << "ply" << endl;
  outFile << "format ascii 1.0" << endl;
  outFile << "element vertex " << numberOfVertices << endl;
  outFile << "property float x" << endl;
  outFile << "property float y" << endl;
  outFile << "property float z" << endl;
  outFile << "property uchar red" << endl;
  outFile << "property uchar green" << endl;
  outFile << "property uchar blue" << endl;
  outFile << "element face " << numberOfFaces << endl;
  outFile << "property list uchar int vertex_indices" << endl;
  outFile << "end_header" << endl;
  // Write vertices
  for(int i = 0; i < numberOfVertices; i++){
    outFile << vertices[i].x << " " << vertices[i].y << " " << vertices[i].z << " " <<
               vertices[i].r << " " << vertices[i].g << " " << vertices[i].b << " " << endl;
  }
  // Write faces
  for(int i = 0; i < numberOfFaces; i++){
    outFile << "3 " << faces[i].v0_ind << " " << faces[i].v1_ind << " " << faces[i].v2_ind << endl;
  }
  outFile.close();
  
  int64 e2 = getTickCount();
  double time = (e2 - e1)/getTickFrequency();
  cout << "\nTotal execution time took: " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;
 
  return EXIT_SUCCESS;
  
}
