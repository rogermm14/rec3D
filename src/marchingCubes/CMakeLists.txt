cmake_minimum_required(VERSION 2.8)
project(marchingCubes)

find_package(VTK REQUIRED)
if(VTK_FOUND)
  message("Found VTK. Version:" ${VTK_VERSION})
endif(VTK_FOUND)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
   message("Found OpenCV.")
endif(OpenCV_FOUND)

include(${VTK_USE_FILE})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(marchingCubes marchingCubes.cpp)
target_link_libraries(marchingCubes ${VTK_LIBRARIES} ${OpenCV_LIBS})
