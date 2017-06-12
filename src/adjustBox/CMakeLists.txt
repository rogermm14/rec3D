cmake_minimum_required(VERSION 2.6)
project(adjustBox)

find_package(PCL REQUIRED)
if(PCL_FOUND)
   message("Found PCL")
endif(PCL_FOUND)

find_package(OpenCV REQUIRED)
if(OpenCV_FOUND)
   message("Found OpenCV.")
endif(OpenCV_FOUND)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(adjustBox adjustBox.cpp)
target_link_libraries(adjustBox ${PCL_COMMON_LIBRARIES} ${PCL_IO_LIBRARIES} ${PCL_FILTERS_LIBRARIES} ${OpenCV_LIBS})
