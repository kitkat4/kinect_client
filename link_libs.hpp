#pragma once


// link OpenCV2 libraries
#define MY_OPENCV2_VERSION "2412"
#ifdef _DEBUG
#define MY_OPENCV2_LIB_SUFFIX "" MY_OPENCV2_VERSION "d"
#else
#define MY_OPENCV2_LIB_SUFFIX "" MY_OPENCV2_VERSION ""
#endif
#pragma comment(lib,"opencv_calib3d" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_contrib" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_flann" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_ml" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_ocl" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_photo" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_stitching" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_superres" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_ts" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_videostab" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_core" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_imgproc" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_highgui" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_objdetect" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_contrib" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_features2d" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_gpu" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_legacy" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_video" MY_OPENCV2_LIB_SUFFIX ".lib")
#pragma comment(lib,"opencv_nonfree" MY_OPENCV2_LIB_SUFFIX ".lib")
#undef MY_OPENCV2_LIB_SUFFIX
#undef MY_OPENCV2_VERSION

// link PCL libraries
#ifdef _DEBUG
#define MY_PCL_LIB_SUFFIX "debug"
#else
#define MY_PCL_LIB_SUFFIX "release"
#endif
#pragma comment(lib,"pcl_common_" MY_PCL_LIB_SUFFIX ".lib")
#pragma comment(lib,"pcl_features_" MY_PCL_LIB_SUFFIX ".lib")
#pragma comment(lib,"pcl_filters_" MY_PCL_LIB_SUFFIX ".lib")
#pragma comment(lib,"pcl_io_" MY_PCL_LIB_SUFFIX ".lib")
#pragma comment(lib,"pcl_search_" MY_PCL_LIB_SUFFIX ".lib")
#pragma comment(lib,"pcl_segmentation_" MY_PCL_LIB_SUFFIX ".lib")
#pragma comment(lib,"pcl_visualization_" MY_PCL_LIB_SUFFIX ".lib")
#undef MY_PCL_LIB_SUFFIX

// link VTK libraries
#define MY_VTK_VERSION "6.2"
#ifdef _DEBUG
#define MY_VTK_LIB_SUFFIX "" MY_VTK_VERSION "-gd"
#else
#define MY_VTK_LIB_SUFFIX "" MY_VTK_VERSION ""
#endif
#pragma comment(lib,"vtkRenderingCore-" MY_VTK_LIB_SUFFIX ".lib")
#pragma comment(lib,"vtkCommonCore-" MY_VTK_LIB_SUFFIX ".lib")
#pragma comment(lib,"vtkCommonMath-" MY_VTK_LIB_SUFFIX ".lib")
#pragma comment(lib,"vtkCommonDataModel-" MY_VTK_LIB_SUFFIX ".lib")
#pragma comment(lib,"vtkCommonExecutionModel-" MY_VTK_LIB_SUFFIX ".lib")
#undef MY_VTK_LIB_SUFFIX
#undef MY_VTK_VERSION


// link Kinect SDK v2.0 libraries
// #pragma comment(lib, "Kinect20.lib")

// link libfreenect2 libraries
#pragma comment(lib, "freenect2.lib")

//#pragma comment( lib, "winmm.lib" )

