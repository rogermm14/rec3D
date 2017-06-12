/*
 * 
 * Project: marchingCubes 
 * 
 * This program is used to extract a mesh from a vtk file containing an iso-surface
 * The algorithm Marching Cubes from the VTK library is used to extract the mesh
 * The resulting model is written in a .ply file
 * 
 * Author: Roger Marí
 * Universitat Pompeu Fabra, Barcelona
 * 2017
 * 
 */

#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <getopt.h>

#include "vtkSmartPointer.h"
#include "vtkStructuredPointsReader.h"
#include "vtkMarchingCubes.h"
#include "vtkPolyDataConnectivityFilter.h"
#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkTriangle.h>

using namespace std;
using namespace cv;

int main(int argc, char* argv[])
{
  
  // Initialize parameters
  string inputIsoSurface, outputMesh;
  float isoValue = 50;
  bool LARGEST_REGION = false;
 
  const char* const short_opts = "i:o:v:l:h";
  
  static struct option long_opts[] =
  {
    {"i",      	required_argument, 0, 'i'},
    {"o",      	required_argument, 0, 'o'},
    {"iso",	required_argument, 0, 'v'},
    {"largest",	no_argument,       0, 'l'},
    {"help",    no_argument,       0, 'h'},
    {0, 0, 0, 0}
  };
 
  int opt;
  int option_index = 0;
  
  while ((opt = getopt_long(argc, argv, short_opts,long_opts,&option_index)) != -1){
    switch (opt){
      case 'i':
	inputIsoSurface = optarg;
	break;
      case 'o':
	outputMesh = optarg;
        break;
      case 'v':
	isoValue = atof(optarg); 
	break;
      case 'l':
	LARGEST_REGION = true;
	break;
      case 'h':
	cout << "\nList of available options:\n\n"
	     << "	--i        <input .vtk file containing iso-surface>\n"
	     << "	--o        <output .ply file>\n"
	     << "	--iso      <marching cubes threshold between 0 and 255>  [OPTIONAL] Default: 50\n"
	     << "	--largest  <keep only largest continuous region>          [OPTIONAL] Default: OFF\n" << endl;
	return  EXIT_SUCCESS;
	break;
      }
  }
  
  // Check if input command is valid according to the number of aguments
  // At least the input and output files must be specified
  if (argc < 5){
    cerr << "\nERROR: Invalid command.\n"
	 << "\nValid command example: $ ./marchingCubes --i iso_surface.vtk --o output_mesh.ply --iso 50 --largest\n"
	 << "\nCheck list of available options: $ ./marchingCubes --help\n" << endl;
    return EXIT_FAILURE;
  }
  
  // Print information about the parameters selected for the current execution
  cout << "\nYou called:\n\n"
       << "	--i        <input .vtk file containing iso-surface>      " + inputIsoSurface  + "\n"
       << "	--o        <output .ply file>                            " + outputMesh + "\n"
       << "	--iso      <marching cubes threshold between 0 and 255>  " << isoValue << "\n";
  cout << "	--largest  <keep only largest continuous region>         "; (!LARGEST_REGION) ? cout << "OFF\n" : cout << "ON\n";

  // Check input vtk file exists
  if (system(("test -f "+inputIsoSurface).c_str())!=0){
    cerr << "\nERROR: Input file does not exist." << endl;
    return EXIT_FAILURE;
  }
  
  int64 e1 = getTickCount();
  cout << "\nRunning marching cubes...\n" << endl; 
  
  // Load data
  vtkSmartPointer<vtkStructuredPointsReader> reader = vtkSmartPointer<vtkStructuredPointsReader>::New();
  reader->SetFileName(inputIsoSurface.c_str());
 
  // Run marching cubes
  vtkSmartPointer<vtkMarchingCubes> mc = vtkSmartPointer<vtkMarchingCubes>::New();
  mc->SetInputConnection(reader->GetOutputPort());
  mc->ComputeNormalsOn();
  mc->ComputeGradientsOn();
  mc->SetValue(0, isoValue);  // isoValue acts as threshold
  mc->Update();
  
  // Extract the mesh from the marching cubes output
  // If the user has enabled the 'LARGEST_REGION' option, then only the largest continuous part
  // of the 3D model will be extracted with vtkPolyDataConnectivityFilter, while the rest will be discarded
  vtkSmartPointer<vtkPolyData> mesh = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkPolyDataConnectivityFilter> confilter = vtkSmartPointer<vtkPolyDataConnectivityFilter>::New();
  // The triangleFilter below will be used later to get the indeces of the 3 vertices forming each face
  vtkSmartPointer<vtkTriangleFilter> triangleFilter = vtkSmartPointer<vtkTriangleFilter>::New(); 
  
  if (LARGEST_REGION){
    // To pick only largest continuous region
    confilter->SetInputConnection(mc->GetOutputPort());
    confilter->SetExtractionModeToLargestRegion();
    confilter->Update();
    mesh = confilter->GetOutput();
    triangleFilter->SetInputConnection(confilter->GetOutputPort());
    
  }
  else{
    mesh = mc->GetOutput();
    triangleFilter->SetInputConnection(mc->GetOutputPort());
  }
  triangleFilter->Update();
  
  // Get number of vertices and faces 
  int numberOfVertices = mesh->GetNumberOfPoints();
  int numberOfFaces = mesh->GetNumberOfCells();
  cout << "  Output mesh contains " << numberOfVertices << " vertices and " << numberOfFaces << " triangular faces" << endl;
  
  // Write output ply file with the extracted mesh
  ofstream outFile( outputMesh.c_str());
  // Write file header
  outFile << "ply" << endl;
  outFile << "format ascii 1.0" << endl;
  outFile << "element vertex " << numberOfVertices << endl;
  outFile << "property float x" << endl;
  outFile << "property float y" << endl;
  outFile << "property float z" << endl;
  outFile << "element face " << numberOfFaces << endl;
  outFile << "property list uchar int vertex_indices" << endl;
  outFile << "end_header" << endl;
  // Write vertices
  for(int i = 0; i < numberOfVertices; i++){
    double p[3];
    mesh->GetPoint(i,p);
    outFile << p[0] << " " << p[1] << " " << p[2] << " " << endl;
  }
  // Write faces
  for (vtkIdType i = 0; i < numberOfFaces; i++){
    // Get ids of the face vertices
    vtkSmartPointer<vtkIdList> cellPointIds = vtkSmartPointer<vtkIdList>::New();
    triangleFilter->GetOutput()->GetCellPoints(i, cellPointIds);
    int v0_ind = cellPointIds->GetId(0);
    int v1_ind = cellPointIds->GetId(1);
    int v2_ind = cellPointIds->GetId(2);
    outFile << "3 " << v0_ind << " " << v2_ind << " " << v1_ind << endl;
  }
  outFile.close();
  
  int64 e2 = getTickCount();
  double time = (e2 - e1)/getTickFrequency();
  cout << "\nMarching cubes successfully completed in " << floor(time/60) << " min " << fmod(time,60) << " s\n" << endl;
  
  return EXIT_SUCCESS;
}
