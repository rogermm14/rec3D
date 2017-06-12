# This Python file uses the following encoding: utf-8
#
# Name: resize.py 
# 
# This script is used to resize an image dataset given a maximum size in pixels
#
# usage: python resize.py 'maxSize'
# example: python resize.py 2000
# 
# Author: Roger Marí
# Universitat Pompeu Fabra, Barcelona
# 2017
# 

from PIL import Image
import os
import time
import math
import sys
import pyexiv2

print 'Resizing input images...\n '
start_time = time.time()
 
# Get session path
session_dir = os.getcwd()

# Get images path
images_dir = os.path.join(session_dir, "resize")

# Set maximum dimension
maxdim = float(sys.argv[1])

# Iterate through every image in the images directory in alphabetic order
images_list = sorted(os.listdir(images_dir))

for image in images_list :    

    # Open image and read size
    current_image = os.path.join(images_dir, image)
    img = Image.open(current_image)
    width, height = img.size

    # Check if input image has exif data
    metadata = pyexiv2.ImageMetadata(current_image)
    metadata.read()

    hasexif = True
    if not metadata.exif_keys :
      hasexif = False
      print 'Resizing image ', image, ' (exif data not found)'   
    else : 
      print 'Resizing image ', image, ' (exif data found)'  

    # Resize image if size exceeds maximum dimension
    if (max(width, height) > maxdim) :
      resize_factor = float( maxdim ) / max(width, height)
      newwidth = math.floor(width * resize_factor)
      newheight = math.floor(height * resize_factor) 
      img = img.resize((int(newwidth), int(newheight)), Image.BILINEAR)

    # Save resized image in the output directory
    output_name = os.path.splitext(image)[0]+".jpg" 
    if hasexif :
      exif_data = img.info['exif']
      os.remove(current_image)
      img.save(os.path.join(images_dir,output_name), exif=exif_data)
    else :
      os.remove(current_image)
      img.save(os.path.join(images_dir,output_name))

print '\nResizing images completed in ', time.time() - start_time, ' (s)\n'

