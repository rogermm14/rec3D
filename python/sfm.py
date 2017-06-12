# This Python file uses the following encoding: utf-8
#
# Name: sfm.py 
# 
# This script is used to run the OpenMVG SfM tools on a multi-view image dataset
#
# usage: python sfm.py 'SFM_METHOD' 'DESCRIBER_METHOD' 'DESCRIBER_SEARCH' 'MATCHING_METHOD' 'GEOMETRIC_MODEL'
# example: python sfm.py incremental SIFT HIGH ANNL2 FUNDAMENTAL 1
# 
# SFM_METHOD options       : incremental | global
# DESCRIBER_METHOD options : SIFT | AKAZE_FLOAT
# DESCRIBER_SEARCH options : NORMAL | HIGH | ULTRA
# MATCHING_METHOD options  : BRUTEFORCEL2 | ANNL2 | FASTCASCADEHASHINGL2
# GEOMETRIC_MODEL options  : FUNDAMENTAL | ESSENTIAL
#
# Author: Roger Marí
# Universitat Pompeu Fabra, Barcelona
# 2017
# 
              
# IMPORTANT! Indicate here the path to OpenMVG binary directory
OPENMVG_SFM_BIN = "/home/usuario/openMVG_Build/Linux-x86_64-RELEASE"

# Indicate here the openMVG camera sensor width directory
CAMERA_SENSOR_WIDTH_DIRECTORY = "/home/usuario/openMVG/src/openMVG/exif/sensor_width_database"

import commands
import os
import subprocess
import sys
import shutil

def get_parent_dir(directory):
    import os
    return os.path.dirname(directory)

session_output_dir = os.getcwd()
input_dir = os.path.join(session_output_dir, "resize")
output_dir = os.path.join(session_output_dir, "openmvg")

print 'Input dir  :', input_dir
print 'Output_dir :', output_dir

matches_dir = os.path.join(output_dir, "matches")
camera_file_params = os.path.join(CAMERA_SENSOR_WIDTH_DIRECTORY, "sensor_width_camera_database.txt")

# Create ouput folder if not present
if not os.path.exists(output_dir):
  os.mkdir(output_dir)

# Create matches folder if not present
if not os.path.exists(matches_dir):
  os.mkdir(matches_dir)

sfmMethod=sys.argv[1]
describerMethod=sys.argv[2]
describerSearch=sys.argv[3]
matchingMethod=sys.argv[4]
geometricModel=sys.argv[5]

print '\nInput parameters:'
print '	SfM method              :', sfmMethod
print '	Describer method        :', describerMethod
print '	Describer search        :', describerSearch
print '	Matching method         :', matchingMethod
print '	Geometric model         :', geometricModel

if geometricModel == "FUNDAMENTAL" :
  geometricModel = "f"
else :
  geometricModel = "e"

# List input images and set initial intrinsics
print '\n1. Intrinsics analysis'
pIntrisics = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_SfMInit_ImageListing"), "-i", input_dir, "-o", matches_dir, "-d", camera_file_params, "-c", "3"] )
pIntrisics.wait()

# Compute features
print '\n2. Compute features'
pFeatures = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeFeatures"), "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-f" , "0", "-m", describerMethod, "-p", describerSearch, "-n", "8"] )
pFeatures.wait()

mean_features_file = open(matches_dir+"/mean_features.txt")
mean_features = mean_features_file.readline()

# Compute matches
print '\n3. Compute matches'
pMatches = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ComputeMatches"), "-i", matches_dir+"/sfm_data.json", "-o", matches_dir, "-f", "0", "-g", geometricModel, "-r", "0.8", "-n", matchingMethod, "-v", "-1"] )
pMatches.wait()

# Compute SfM
if sfmMethod == "incremental" :
  reconstruction_dir = os.path.join(output_dir, "incremental")
  if geometricModel == "e" :
    os.rename(matches_dir+"/matches.e.bin", matches_dir+"/matches.f.bin");
  print '\n4. Incremental SfM'
  pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_IncrementalSfM"), "-i", matches_dir+"/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir, "-c", "3", "-f", "ADJUST_FOCAL_LENGTH|ADJUST_DISTORTION"] )
  pRecons.wait()
else :
  reconstruction_dir = os.path.join(output_dir, "global")
  if geometricModel == "f" :
    os.rename(matches_dir+"/matches.f.bin", matches_dir+"/matches.e.bin");
  print '\n4. Global SfM'
  pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_GlobalSfM"), "-i", matches_dir+"/sfm_data.json", "-m", matches_dir, "-o", reconstruction_dir ])
  pRecons.wait()

# Undistort images
pRecons = subprocess.Popen( [os.path.join(OPENMVG_SFM_BIN, "openMVG_main_ExportUndistortedImages"),  "-i", reconstruction_dir+"/sfm_data.bin", "-o", session_output_dir+"/undistort", "-r", "1", "-n", "8"] )
pRecons.wait()
