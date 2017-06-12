#########################################################################
# 
# Name: reconstruct_DATASET.py
# 
# This script is used to reconstruct a DATASET
# 
# Author: Roger Marí
# Universitat Pompeu Fabra, Barcelona
# 2017
# 
#########################################################################

# IMPORTANT! Indicate here the path to the rec3D folder in your computer
REC3D_PATH=/home/usuario/rec3D

# Get dataset name
DATASET=$1

# All information about the 3D reconstruction will be saved in the data.txt file 
DATA=$REC3D_PATH/data/$DATASET/data.txt

# Check for existing dataset
if [ ! -d "$REC3D_PATH/data/$DATASET" ]; then 
	echo "ERROR: Dataset directory was not found."
	exit 1
fi

# Check number of images >= 2
cd $REC3D_PATH/data/$DATASET
IMAGES=`ls | wc -l`
if [ $IMAGES -lt 3 ] ; then 
	echo "ERROR: Number of views is less than 2."
	exit 1
fi

# Dataset exists and number of images is >=2, reconstruction pipeline can start
. $REC3D_PATH/reconstruct_pipeline.sh $DATASET >> $DATA 2>> $DATA

# Notify if some error occurred. Otherwise, reconstruction ended syccessfully
if [ $? -gt 0 ] ; then
	echo >> $DATA; echo "ERROR: Some unexpected error occurred. 3D reconstruction aborted." >> $DATA; echo >> $DATA; 
	exit 1
fi
echo "... 3D reconstruction ended successfully!" >> $DATA
