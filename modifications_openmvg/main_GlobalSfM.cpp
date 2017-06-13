
// Copyright (c) 2012, 2013, 2014 Pierre MOULON.

// This Source Code Form is subject to the terms of the Mozilla Public
// License, v. 2.0. If a copy of the MPL was not distributed with this
// file, You can obtain one at http://mozilla.org/MPL/2.0/.

#include <cstdlib>

#include "openMVG/sfm/pipelines/global/sfm_global_engine_relative_motions.hpp"
#include "openMVG/system/timer.hpp"
#include "openMVG/cameras/Cameras_Common_command_line_helper.hpp"
#include "openMVG/sfm/sfm.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;

#include "third_party/cmdLine/cmdLine.h"
#include "third_party/stlplus3/filesystemSimplified/file_system.hpp"

bool saveIntrinsics(const SfM_Data & data, const std::string & path){

  // Check and create path if necessary
  if (!stlplus::folder_exists(path) && !stlplus::folder_create(path)) {
    std::cerr << "\nCannot create output directory to write 'intrinsics_OpenMVG.txt'" << std::endl;
    return false;
  }

  bool checkIntrinsics = false;

  unsigned i = 0;
  while (i < data.views.size() && checkIntrinsics == false){
    const View & view = *(data.views.at(i));
    if(data.IsPoseAndIntrinsicDefined(&view)){

      // Get intrinsics from SfM data
      const IntrinsicBase * baseintrinsics = data.intrinsics.at(view.id_intrinsic).get();
      const Pinhole_Intrinsic_Radial_K3 & intrinsics = *(dynamic_cast<const Pinhole_Intrinsic_Radial_K3*>(baseintrinsics));
      const std::vector<double> params = intrinsics.getParams();
      
      if ( !checkIntrinsics ){

        // Create the stream and check it is ok
        std::ofstream stream(stlplus::create_filespec(path, "intrinsics_OpenMVG.txt").c_str(), std::ios::binary | std::ios::out);
        if (!stream.is_open()) return false;

        // Write intrinsics_OpenMVG.txt
        stream << intrinsics.K() << std::endl;
        stream << params[params.size()-3] << " " << params[params.size()-2] << " " << params[params.size()-1] << " " << std::endl;	    
        stream.close();

        checkIntrinsics = true;
      }
    }

  i++;
  }

  return true;
}


bool saveSFM(const SfM_Data & data, const std::string & path){

  // Check and create path if necessary
  if (!stlplus::folder_exists(path) && !stlplus::folder_create(path)) {
    std::cerr << "\nCannot create output sfm directory" << std::endl;
    return false;
  }
  
  ///// Camera files

  for (unsigned i = 0; i < data.views.size(); ++i){

    const View & view = *(data.views.at(i));
    if(data.IsPoseAndIntrinsicDefined(&view)){

      // Get current camera intrinsics and extrinsics from SfM data
      const IntrinsicBase * baseintrinsics = data.intrinsics.at(view.id_intrinsic).get();
      const Pinhole_Intrinsic_Radial_K3 & intrinsics = *(dynamic_cast<const Pinhole_Intrinsic_Radial_K3*>(baseintrinsics));
      const geometry::Pose3 & extrinsics = data.poses.at(view.id_pose);
      const std::vector<double> params = intrinsics.getParams();
      
      // Pinhole_Intrinsic_Radial_K3 model format
      // params[0] = focal
      // params[1] = ppx
      // params[2] = ppy
      // params[3] = K1
      // params[4] = K2
      // params[5] = K3

      // Set filename and path
      std::ostringstream filename;
      std::string fullname = data.views.at(i)->s_Img_path;
      size_t lastindex = fullname.find_last_of("."); 
      std::string rawname = fullname.substr(0, lastindex); 
      filename << rawname + "_cam.txt";

      // Create the stream and check it is ok
      std::ofstream stream(stlplus::create_filespec(path, filename.str()), std::ios::binary | std::ios::out);
      if (!stream.is_open()) return false;

      // Write current camera file
      stream << "1" << std::endl;
      stream << intrinsics.w_ << " " << intrinsics.h_ << std::endl;
      stream << intrinsics.K() << std::endl;
      stream << extrinsics.rotation() << std::endl;
      stream << extrinsics.translation() << std::endl;
      stream << params[params.size()-3] << " " << params[params.size()-2] << " " << params[params.size()-1] << " " << std::endl;	    
      stream.close();
    }
  }

  ///// Structure
  {
    // Create the stream and check it is ok
    std::ofstream stream(stlplus::create_filespec(path, "structure_raw.ply").c_str(), std::ios::binary | std::ios::out);
    if (!stream.is_open()) return false;

    // Write structure in a ply file
    stream << "ply" << std::endl;
    stream << "format ascii 1.0" << std::endl;
    stream << "element vertex " << data.structure.size() << std::endl;
    stream << "property float x" << std::endl;
    stream << "property float y" << std::endl;
    stream << "property float z" << std::endl;
    stream << "end_header" << std::endl;
    Landmarks::const_iterator it;
    for (it = data.structure.begin(); it != data.structure.end(); ++it) {
      const Vec3 & P = it->second.X;
      stream << P[0] << " " << P[1] << " " << P[2] << std::endl;
    }
    stream.close();
  }

  ///// Bounding box
  {
    // Compute bounding box
    Vec3 BBmin(1e10,1e10,1e10), BBmax(-1e10,-1e10,-1e10);
    Landmarks::const_iterator it;
    for (it = data.structure.begin(); it != data.structure.end(); ++it) {
      const Vec3 & P = it->second.X;
      for (unsigned i = 0; i < 3; ++i) {
        if (P[i] < BBmin[i]) BBmin[i] = P[i];
        else if (P[i] > BBmax[i]) BBmax[i] = P[i];
      }
    }
    
    // Create the stream and check it is ok
    std::ofstream stream(stlplus::create_filespec(path, "bbox_raw.txt").c_str(), std::ios::binary | std::ios::out);
    if (!stream.is_open()) return false;

    // Write bbox in a txt file
    stream << BBmin[0] << " " << BBmin[1] << " " << BBmin[2] << "\n" 
           << BBmax[0] << " " << BBmax[1] << " " << BBmax[2] << std::endl;
    stream.close();
  }

  return true;
}


int main(int argc, char **argv)
{
  using namespace std;
  std::cout << std::endl
    << "-----------------------------------------------------------\n"
    << "Global Structure from Motion:\n"
    << "-----------------------------------------------------------\n"
    << "Open Source implementation of the paper:\n"
    << "\"Global Fusion of Relative Motions for "
    << "Robust, Accurate and Scalable Structure from Motion.\"\n"
    << "Pierre Moulon, Pascal Monasse and Renaud Marlet. "
    << " ICCV 2013." << std::endl
    << "------------------------------------------------------------"
    << std::endl;


  CmdLine cmd;

  std::string sSfM_Data_Filename;
  std::string sMatchesDir;
  std::string sOutDir = "";
  int iRotationAveragingMethod = int (ROTATION_AVERAGING_L2);
  int iTranslationAveragingMethod = int (TRANSLATION_AVERAGING_SOFTL1);
  std::string sIntrinsic_refinement_options = "ADJUST_ALL";
  bool b_use_motion_priors = false;

  cmd.add( make_option('i', sSfM_Data_Filename, "input_file") );
  cmd.add( make_option('m', sMatchesDir, "matchdir") );
  cmd.add( make_option('o', sOutDir, "outdir") );
  cmd.add( make_option('r', iRotationAveragingMethod, "rotationAveraging") );
  cmd.add( make_option('t', iTranslationAveragingMethod, "translationAveraging") );
  cmd.add( make_option('f', sIntrinsic_refinement_options, "refineIntrinsics") );
  cmd.add( make_switch('P', "prior_usage") );

  try {
    if (argc == 1) throw std::string("Invalid parameter.");
    cmd.process(argc, argv);
  } catch(const std::string& s) {
    std::cerr << "Usage: " << argv[0] << '\n'
    << "[-i|--input_file] path to a SfM_Data scene\n"
    << "[-m|--matchdir] path to the matches that corresponds to the provided SfM_Data scene\n"
    << "[-o|--outdir] path where the output data will be stored\n"
    << "\n[Optional]\n"
    << "[-r|--rotationAveraging]\n"
      << "\t 1 -> L1 minimization\n"
      << "\t 2 -> L2 minimization (default)\n"
    << "[-t|--translationAveraging]:\n"
      << "\t 1 -> L1 minimization\n"
      << "\t 2 -> L2 minimization of sum of squared Chordal distances\n"
      << "\t 3 -> SoftL1 minimization (default)\n"
    << "[-f|--refineIntrinsics] Intrinsic parameters refinement option\n"
      << "\t ADJUST_ALL -> refine all existing parameters (default) \n"
      << "\t NONE -> intrinsic parameters are held as constant\n"
      << "\t ADJUST_FOCAL_LENGTH -> refine only the focal length\n"
      << "\t ADJUST_PRINCIPAL_POINT -> refine only the principal point position\n"
      << "\t ADJUST_DISTORTION -> refine only the distortion coefficient(s) (if any)\n"
      << "\t -> NOTE: options can be combined thanks to '|'\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_PRINCIPAL_POINT\n"
      <<      "\t\t-> refine the focal length & the principal point position\n"
      << "\t ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION\n"
      <<      "\t\t-> refine the focal length & the distortion coefficient(s) (if any)\n"
      << "\t ADJUST_PRINCIPAL_POINT|ADJUST_DISTORTION\n"
      <<      "\t\t-> refine the principal point position & the distortion coefficient(s) (if any)\n"
    << "[-P|--prior_usage] Enable usage of motion priors (i.e GPS positions)\n"
    << std::endl;

    std::cerr << s << std::endl;
    return EXIT_FAILURE;
  }

  if (iRotationAveragingMethod < ROTATION_AVERAGING_L1 ||
      iRotationAveragingMethod > ROTATION_AVERAGING_L2 )  {
    std::cerr << "\n Rotation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  const cameras::Intrinsic_Parameter_Type intrinsic_refinement_options =
    cameras::StringTo_Intrinsic_Parameter_Type(sIntrinsic_refinement_options);

  if (iTranslationAveragingMethod < TRANSLATION_AVERAGING_L1 ||
      iTranslationAveragingMethod > TRANSLATION_AVERAGING_SOFTL1 )  {
    std::cerr << "\n Translation averaging method is invalid" << std::endl;
    return EXIT_FAILURE;
  }

  // Load input SfM_Data scene
  SfM_Data sfm_data;
  if (!Load(sfm_data, sSfM_Data_Filename, ESfM_Data(VIEWS|INTRINSICS))) {
    std::cerr << std::endl
      << "The input SfM_Data file \""<< sSfM_Data_Filename << "\" cannot be read." << std::endl;
    return EXIT_FAILURE;
  }

  // Init the regions_type from the image describer file (used for image regions extraction)
  using namespace openMVG::features;
  const std::string sImage_describer = stlplus::create_filespec(sMatchesDir, "image_describer", "json");
  std::unique_ptr<Regions> regions_type = Init_region_type_from_file(sImage_describer);
  if (!regions_type)
  {
    std::cerr << "Invalid: "
      << sImage_describer << " regions type file." << std::endl;
    return EXIT_FAILURE;
  }

  // Features reading
  std::shared_ptr<Features_Provider> feats_provider = std::make_shared<Features_Provider>();
  if (!feats_provider->load(sfm_data, sMatchesDir, regions_type)) {
    std::cerr << std::endl
      << "Invalid features." << std::endl;
    return EXIT_FAILURE;
  }
  // Matches reading
  std::shared_ptr<Matches_Provider> matches_provider = std::make_shared<Matches_Provider>();
  if // Try to read the two matches file formats
  (
    !(matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.txt")) ||
      matches_provider->load(sfm_data, stlplus::create_filespec(sMatchesDir, "matches.e.bin")))
  )
  {
    std::cerr << std::endl
      << "Invalid matches file." << std::endl;
    return EXIT_FAILURE;
  }

  if (sOutDir.empty())  {
    std::cerr << "\nIt is an invalid output directory" << std::endl;
    return EXIT_FAILURE;
  }

  if (!stlplus::folder_exists(sOutDir))
  {
    if (!stlplus::folder_create(sOutDir))
    {
      std::cerr << "\nCannot create the output directory" << std::endl;
    }
  }

  //---------------------------------------
  // Global SfM reconstruction process
  //---------------------------------------

  openMVG::system::Timer timer;
  GlobalSfMReconstructionEngine_RelativeMotions sfmEngine(
    sfm_data,
    sOutDir,
    stlplus::create_filespec(sOutDir, "Reconstruction_Report.html"));

  // Configure the features_provider & the matches_provider
  sfmEngine.SetFeaturesProvider(feats_provider.get());
  sfmEngine.SetMatchesProvider(matches_provider.get());

  // Configure reconstruction parameters
  sfmEngine.Set_Intrinsics_Refinement_Type(intrinsic_refinement_options);
  b_use_motion_priors = cmd.used('P');
  sfmEngine.Set_Use_Motion_Prior(b_use_motion_priors);

  // Configure motion averaging method
  sfmEngine.SetRotationAveragingMethod(
    ERotationAveragingMethod(iRotationAveragingMethod));
  sfmEngine.SetTranslationAveragingMethod(
    ETranslationAveragingMethod(iTranslationAveragingMethod));

  if (sfmEngine.Process())
  {
    std::cout << std::endl << " Total Ac-Global-Sfm took (s): " << timer.elapsed() << std::endl;

    std::cout << "...Generating SfM_Report.html" << std::endl;
    Generate_SfM_Report(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "SfMReconstruction_Report.html"));

    //-- Export to disk computed scene (data & visualizable results)
    std::cout << "...Export SfM_Data to disk." << std::endl;
    Save(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "sfm_data", ".bin"),
      ESfM_Data(ALL));

    Save(sfmEngine.Get_SfM_Data(),
      stlplus::create_filespec(sOutDir, "cloud_and_poses", ".ply"),
      ESfM_Data(ALL));

    //-- Save sfm files for 3D reconstruction
    saveIntrinsics(sfmEngine.Get_SfM_Data(), sOutDir);
    saveSFM(sfmEngine.Get_SfM_Data(), stlplus::create_filespec(sOutDir, "sfm"));

    return EXIT_SUCCESS;
  }
  return EXIT_FAILURE;
}
