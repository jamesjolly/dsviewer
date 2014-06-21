
cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
project(ds_pcl_viewer)
find_package(PCL 1.2 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS} 
"/opt/softkinetic/DepthSenseSDK/include"
)
link_directories(${PCL_LIBRARY_DIRS}
"/opt/softkinetic/DepthSenseSDK/lib"
)
add_definitions(${PCL_DEFINITIONS})
add_executable (ds_pcl_viewer main.cxx)
target_link_libraries (ds_pcl_viewer ${PCL_LIBRARIES} usb-1.0 turbojpeg DepthSense DepthSensePlugins)

