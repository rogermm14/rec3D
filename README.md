=====================================
rec3D
=====================================


---------------------------------------------
Linux compilation
---------------------------------------------

This software was developed using Ubuntu 16.04

Required tools:
- Cmake
- C/C++ compiler

Required external libraries:
- OpenMVG (https://github.com/openMVG/openMVG) [1]
- OpenCV (https://github.com/opencv/opencv)
- PCL (https://github.com/PointCloudLibrary/pcl)
- VTK (https://github.com/Kitware/VTK)

Python:
- python-numpy
- python-matplotlib
- python-pyexiv2

You will need to apply some changes to the OpenMVG library to run the software successfully.
All the necessary modifications and files are listed in modifications_openmvg.

You can use the INSTALL.sh script to compile the software after all the previous tools listed
above are correctly setup. Alternatively, you can use the already compiled binaries instead.

Do not forget to change the necessary paths at the beginning of the following scripts:
- reconstruct_dataset.sh
- reconstruct_pipeline.sh
- sfm.py 


---------------------------------------------
How to use the software
---------------------------------------------

To reconstruct a dataset: 

1. Copy ONLY the collection of input images (.jpg and .png formats are accepted) in a folder 
   located at the rec3D/data directory.

2. Run the script reconstruct_dataset.sh passing as argument the name of the dataset folder.
   The default parameters will be used for the 3D reconstruction.
   - Example call: $ bash reconstruct_dataset.sh castle
  
3. Feel free to modify the 3D reconstruction paramters in the reconstruct_pipeline.sh script.
   Read [2] for further information about the algorithms implemented in the software.
   - To see the list of available paramters for depth map generation and fusion: $ ./rec3D --help
   - To see the list of available paramters for mesh extraction: $ ./marchingCubes --help
   - To see the list of available paramters for mesh painting: $ ./paintMesh --help
   - To see the list of available paramters for bbox adjustment: $ ./adjustBox --help
   


---------------------------------------------
Citations
---------------------------------------------

[1] Moulon Pierre, Monasse Pascal, Marlet Renaud, et al. (2013). OpenMVG: an Open Multiple View Geometry library.

[2] Marí Roger (2017). Multi-view 3D Reconstruction via Depth Map Fusion for a Smartphone Application.

If you find this software or some part of it useful, please cite it as:

```
  @misc{mari2017depthfusion,
    author = "Roger Mar{\'i}",
    title = "Multi-view 3D Reconstruction via Depth Map Fusion for a Smartphone Application",
    year = 2017,
    howpublished = "\url{https://github.com/rogermm14/rec3D}" 
  }
```
