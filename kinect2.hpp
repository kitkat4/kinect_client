/*
  Katsumasa Kitajima
  kitajima@ynl.t.u-tokyo.ac.jp
 */

#pragma once

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include "kinect_logger.hpp"

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>

#include <iostream>
#include <fstream>
#include <algorithm>
#include <vector>
#include <memory>
#include <string>
#include <stdexcept>
#include <thread>
#include <chrono>



class Kinect2{

    typedef libfreenect2::Frame frame_t;
    typedef libfreenect2::Freenect2Device device_t;
    typedef device_t::IrCameraParams ir_param_t;
    typedef device_t::ColorCameraParams color_param_t;

    enum{
        Good = 0,
        UnspecifiedError = 1 << 0,
        NoKinectFound = 1 << 1,
        FailedToOpenKinect = 1 << 2,
        FailedToStartKinect = 1 << 3,
    };
    
public:

    typedef uint8_t color_ch_t ;
    typedef float depth_ch_t;

    Kinect2( const std::string& name, const char* const serial_num = nullptr );
    Kinect2();
    ~Kinect2();
    uint32_t open( const std::string& name, const char* const serial_num = nullptr );
    void close();
    
    // Saves ir/color camera parameters
    int saveKinectParams()const{ return saveKinectParams( std::string("./") + name_ + ".dat" ); }
    int saveKinectParams( const std::string& file_path )const;
    // Loads ir/color camera parameters and reset registration object.
    int loadKinectParams(){ return loadKinectParams( std::string("./") + name_ + ".dat" ); }
    int loadKinectParams( const std::string& file_path );
    
    // Waits for the next frame and store data to the specified pointers.
    // Specify nullptr as those you don't need.
    // The unit of timestamp is 0.1[ms].
    void waitForNewFrame( color_ch_t* const color,
                          depth_ch_t* const depth,
                          color_ch_t* const registered,
                          uint32_t* const timestamp = nullptr );
    // Just waits for the next frame and does nothing.
    // The unit of timestamp is 0.1[ms].
    void waitForNewFrame( uint32_t* const timestamp = nullptr ){
        waitForNewFrame( nullptr, nullptr, nullptr, timestamp );
    }
    // Converts depth pixel location and depth value to 3D point.
    void mapDepthPointToCameraSpace( const int row,
                                     const int col,
                                     const float depth,
                                     float& x,
                                     float& y,
                                     float& z )const;
    void getPointCloudRGB( depth_ch_t const * const depth,
                           color_ch_t const * const registered,
                           pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                           const double* const homogenous_tansformation_mat = nullptr )const;

    std::string getName()const{ return name_; }
    operator bool()const{ return state_ == Good; }

    static float cvtUcharBgrToFloat( const uint8_t b, const uint8_t g, const uint8_t r );
    static void cvtFloatBgrToUchar( const float bgr, uint8_t& b, uint8_t& g, uint8_t& r );

    // color frame constants
    static const int kCWidth  = 1920;
    static const int kCHeight = 1080;
    static const int kCNumOfPixels = kCWidth * kCHeight;
    static const int kCNumOfChannelsPerPixel = 4; // BGRX
    static const int kCNumOfChannels = kCNumOfPixels * kCNumOfChannelsPerPixel;
    static const int kCNumOfBytesPerPixel = sizeof(color_ch_t) * kCNumOfChannelsPerPixel;
    static const int kCNumOfBytes = kCNumOfBytesPerPixel * kCNumOfPixels;

    // depth frame constants
    static const int kDWidth  = 512;
    static const int kDHeight = 424;
    static const int kDNumOfPixels = kDWidth * kDHeight;
    static const int kDNumOfChannelsPerPixel = 1;
    static const int kDNumOfChannels = kDNumOfPixels * kDNumOfChannelsPerPixel;
    static const int kDNumOfBytesPerPixel = sizeof(depth_ch_t) * kDNumOfChannelsPerPixel;
    static const int kDNumOfBytes = kDNumOfBytesPerPixel * kDNumOfPixels;

    
private:


    void set( uint32_t new_state ){ state_ |= new_state; }
    bool succeeded()const{ return state_ == Good; }
    bool failed()const{ return ! succeeded(); }

    uint32_t state_;
    
    std::string log_file_path_;
    std::unique_ptr<KinectLogger> logger_;


    libfreenect2::Freenect2 freenect2_;    
    device_t* device_;
    std::unique_ptr<libfreenect2::PacketPipeline> pipeline_;
    std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener_;
    ir_param_t ir_param_;
    color_param_t color_param_;
    std::unique_ptr<libfreenect2::Registration> registration_;
    libfreenect2::FrameMap frames_;
    
    std::string name_;


};


