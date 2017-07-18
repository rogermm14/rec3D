#########################################################################
#
# Name: reconstruct_pipeline.sh 
# 
# This script is used to execute the complete 3D reconstruction pipeline
# 
# Author: Roger Marí
# Universitat Pompeu Fabra, Barcelona
# 2017
# 
#########################################################################

# IMPORTANT! Indicate here the path to the rec3D folder in your computer
REC3D_PATH=/home/usuario/rec3D

# Exit whenever an error is found
set -e 

START=$(date +%s)
DATASET=$1
BIN_PATH=$REC3D_PATH/binaries

TVL1=true
TVL2=false

cd
cd $REC3D_PATH/data/$DATASET
mkdir -p output

echo "Starting 3D reconstruction ..."; echo;

# Resize images
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; 
      echo "%%%%%% Resizing input images %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo; 
mkdir -p output/resize
rsync -av --exclude='output' --exclude='data.txt' . output/resize > /dev/null
cd output
python -u $REC3D_PATH/python/resize.py 700

# SfM
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; 
      echo "%%%%%% SfM %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo; 
SFM_METHOD=incremental
python -u $REC3D_PATH/python/sfm.py $SFM_METHOD AKAZE_FLOAT ULTRA FASTCASCADEHASHINGL2 ESSENTIAL
mkdir -p undistort
mkdir -p sfm
cp -r openmvg/$SFM_METHOD/sfm/* sfm
echo;

# Adjust bbox
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; 
      echo "%%%%%% Adjusting bbox %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo; 
if [ "$SFM_METHOD" == "incremental" ] ; then 
  $BIN_PATH/adjustBox --i sfm/structure_raw.ply --o sfm/structure_filt.ply --bbox sfm/bbox_filt.txt --knn 0.3 --t 0.5
else
  $BIN_PATH/adjustBox --i sfm/structure_raw.ply --o sfm/structure_filt.ply --bbox sfm/bbox_filt.txt --knn 0.2 --t 1.0
fi

# Dense reconstruction with plane sweep and depthmap fusion
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; 
      echo "%%%%%% 3D reconstruction via depthmap fusion %%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo; 
mkdir -p rec3D
V=1e7
S=1.0
$BIN_PATH/rec3D --img undistort --sfm sfm --out rec3D --s $S --voxels $V --debug --depth --cost SAD --planes 150 --wsize 13 --conf 0.3

$BIN_PATH/rec3D --img undistort --sfm sfm --out rec3D --s $S --voxels $V --tsdf --delta 0.01 --eta 0.2

if $TVL1 ; then 
  $BIN_PATH/rec3D --img undistort --sfm sfm --out rec3D --s $S --voxels $V --tvl1 --lambda 0.1 --theta 0.25 --tau 0.15 --maxit 50
fi
if $TVL2 ; then
  $BIN_PATH/rec3D --img undistort --sfm sfm --out rec3D --s $S --voxels $V --tvl2 --lambda 0.1 --theta 0.25 --tau 0.15 --maxit 50
fi

cd rec3D

# Plot energy and error
if $TVL1 ; then 
  python -u $REC3D_PATH/python/plotvector.py tvl1/tvl1_energy.txt tvl1/tvl1_energy.png Energy Iterations TVL1 > /dev/null
  python -u $REC3D_PATH/python/plotvector.py tvl1/tvl1_error.txt tvl1/tvl1_error.png Difference Iterations TVL1 > /dev/null
fi
if $TVL2 ; then 
  python -u $REC3D_PATH/python/plotvector.py tvl2/tvl2_energy.txt tvl2/tvl2_energy.png Energy Iterations TVL2> /dev/null
  python -u $REC3D_PATH/python/plotvector.py tvl2/tvl2_error.txt tvl2/tvl2_error.png Difference Iterations TVL2 > /dev/null
fi

# Mesh extraction with marching cubes
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; 
      echo "%%%%%% Mesh extraction with marching cubes %%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo;
if $TVL1 ; then 
  echo; echo "Extracting TVL1 mesh...";
  $BIN_PATH/marchingCubes --i tvl1/tvl1.vtk --o tvl1/tvl1_mesh.ply
fi
if $TVL2 ; then
  echo; echo "Extracting TVL2 mesh...";
  $BIN_PATH/marchingCubes --i tvl2/tvl2.vtk --o tvl2/tvl2_mesh.ply
fi

# Paint mesh
echo; echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; 
      echo "%%%%%% Painting mesh %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%";
      echo "%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%"; echo;
if $TVL1 ; then 
  echo; echo "Painting TVL1 mesh..."; echo;
  $BIN_PATH/paintMesh --i tvl1/tvl1_mesh.ply --o tvl1 --img images.txt --bbox grid.txt --cam cameras.txt
fi
if $TVL2 ; then
  echo; echo "Painting TVL2 mesh..."; echo;
  $BIN_PATH/paintMesh --i tvl2/tvl2_mesh.ply --o tvl2 --img images.txt --bbox grid.txt --cam cameras.txt
fi

# How long did it take to reconstruct the dataset?
END=$(date +%s)
ELAPSED="$(( (($END-$START)/60)%60 )) min $(( ($END-$START)%60 ))sec"
echo; echo "************ Total time: " $ELAPSED "************"; echo;
