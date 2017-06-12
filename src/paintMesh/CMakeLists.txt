cmake_minimum_required(VERSION 2.8)
project(paintMesh)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

find_package(VTK REQUIRED)
if(VTK_FOUND)
  message("found VTK. Version:" ${VTK_VERSION})
endif(VTK_FOUND)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
   message("Found OpenCV.")
endif(OpenCV_FOUND)

find_package(OpenMP REQUIRED)
if (OPENMP_FOUND)
    message("Found OpenMP.")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif(OPENMP_FOUND)

include(${VTK_USE_FILE})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(paintMesh paintMesh.cpp)
target_link_libraries(paintMesh ${VTK_LIBRARIES} ${OpenCV_LIBS})
