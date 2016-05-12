/*
  Kinect v2 recorder.
  Katsumasa Kitajima
  kitajima@ynl.t.u-tokyo.ac.jp
 */


#pragma once


#include "kinect2.hpp"
#include "link_libs.hpp"


#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <boost/asio.hpp>

#include <thread>
#include <atomic>
#include <chrono>
#include <memory>
#include <string>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>
#include <vector>
#include <queue>
#include <stdexcept>
#include <ctime>
#include <algorithm>
#include <cmath>

#include <windows.h>


#if defined(_MSC_VER) && _MSC_VER < 1800
#ifndef __func__
#define __func__ __FUNCTION__
#endif
#define ISNAN(f) _isnan(f)
#else
#define ISNAN(f) std::isnan(f)
#endif


class KinectRecorder{

    typedef Kinect2::color_ch_t color_ch_t; // channel type of color frame
    typedef Kinect2::depth_ch_t depth_ch_t;   // channel type of depth frame
    typedef libfreenect2::Frame frame_t;
    typedef libfreenect2::Freenect2Device device_t;
    typedef boost::asio::ip::udp udp_t;
    typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
    typedef pcl::PointCloud<pcl::PointXYZRGB> cloudRGB_t;
    
public:
    
    typedef enum{
        Error,
        InitialState,
        WaitingForFpsStabilized,
        ReadyToRecord,
        WritingData,
        Recording,
        ReadyToCalibrate,
        Calibrating,
        Exiting,
    }RecorderState;

    KinectRecorder( const std::string& config_file_path );
    ~KinectRecorder();

    // Specify device_index explicitly when you use multiple kinect sensors at the same time as follows:
    // 0 for the first device, 1 for the second device, etc.
    void init();
    
    void startKinectAndCreateWindow();
    void calibrate();
    void stopKinectAndDestroyWindow(){
        cv::destroyAllWindows();
        set( Exiting );
    }
    

    void saveCurrentColorFrame(){
        if( push_color_queue_.load() )
            std::cerr << "warning: a color frame to be saved has been lost ("
                      << __func__ << ")." << std::endl;
        else
            push_color_queue_ = current_frame_color_;
    }
    void saveCurrentDepthFrame(){
        if( push_depth_queue_.load() )
            std::cerr << "warning: a depth frame to be saved has been lost ("
                      << __func__ << ")." << std::endl;
        else
            push_depth_queue_ = current_frame_depth_;
    }
    void saveCurrentRegFrame(){
        if( push_reg_queue_.load() )
            std::cerr << "warning: a 'registered' frame to be saved has been lost ("
                      << __func__ << ")." << std::endl;
        else
            push_reg_queue_ = current_frame_reg_;
    }
    void saveCurrentFrame(){
        saveCurrentColorFrame();
        saveCurrentDepthFrame();
        saveCurrentRegFrame();
    }

    void enterMainLoop();
    bool configure( const std::string& config_file_path );
    operator bool(){ return recorder_state_ != Error; }
    
private:
    void update();
    void updateQueue();

    void save();

    bool saveColor( cv::VideoWriter& video_writer ); // when save color frames as a video file
    bool saveColor( const std::string& file_path ); // when save color frames as pictures
    bool saveDepth( const std::string& file_path );
    int saveKinectParams( const std::string& file_path )const{
        kinect_->saveKinectParams( file_path );
    }

    int loadKinectParams( const std::string& file_path ){
        kinect_->loadKinectParams( file_path );
    }
    void createSceneDir();
    void waitVideoWriterToBeOpened( cv::VideoWriter& video_writer );
    void sync();
    void processRecvBuf( const std::vector<char>* buf, // somehow buf as reference doesn't work
                         const boost::system::error_code& error,
                         const std::size_t size );
    bool fpsKeepsHigh( const double fps )const;
    void showImgAndInfo();
    void putText( cv::Mat& img, const std::string& text, const bool newline = true,
                  const cv::Scalar& color = kSkyBlue,
                  const cv::Point& org = cv::Point(-1,-1) )const;
    void putText( cv::Mat& img, const std::string& text, const bool newline,
                  const cv::Point& org )const{
        putText( img ,text, newline, kSkyBlue, org );
    };
    
    std::string toBinaryString( const float value )const;
    float toFloat( const std::string& str )const;
    std::string toString( const double value )const;
    std::string toString( const int value )const;


    bool is( const uint32_t state )const{ return recorder_state_ == state; }
    void set( const uint32_t state ){ recorder_state_ = state; }
    bool isStandalone()const{ return (recorder_mode_ & 1) == 0; }
    bool specifyEachFrame()const{ return (recorder_mode_ & 2) == 2; }
    bool saveAsVideo()const{ return (recorder_mode_ & 4) == 4; }
    bool queuesAreEmpty()const{
        return color_queue_.empty() && depth_queue_.empty() && reg_queue_.empty();
    }

    // project src onto plane ax+by+cz+d=0
    pcl::PointXYZ project( const float a, const float b, const float c, const float d,
                           const pcl::PointXYZ& src )const{
        return pcl::PointXYZ( - src.x * d / ( a*src.x + b*src.y + c*src.z ),
                              - src.y * d / ( a*src.x + b*src.y + c*src.z ),
                              - src.z * d / ( a*src.x + b*src.y + c*src.z ) );
    }


    class FpsCalculator{
    public:
        FpsCalculator( const int update_cycle )
            : loop_count_( 0 ),
              update_cycle_( update_cycle ),
              fps_( 0.0 ){

            QueryPerformanceCounter( &timestamp_ );
            QueryPerformanceFrequency( &freq_ );
        }
        ~FpsCalculator(){}
        bool fps( double& out_fps ){
            bool ret = false;
            if( ++loop_count_ % update_cycle_ == 0 ){
                LARGE_INTEGER now;
                QueryPerformanceCounter( &now );
                fps_ = (double)(update_cycle_*freq_.QuadPart)/(now.QuadPart - timestamp_.QuadPart);
                timestamp_ = now;
                ret = true;
                loop_count_ = 0;
            }
            out_fps = fps_;
            return ret;
        }
        double fps(){
            double ret = 0;
            fps( ret );
            return ret;
        }

    private:
        int loop_count_;
        LARGE_INTEGER timestamp_, freq_;
        int update_cycle_;
        double fps_;
    };


    std::unique_ptr<Kinect2> kinect_;
    std::string kinect_name_;
    std::string serial_num_;

    
    std::string out_dir_;
    std::string scene_dir_;
    std::string motion_name_;

    int freq_;
    
    static const int kBufSize = 600;

    std::vector<color_ch_t> color_buf_idle_;
    std::vector<color_ch_t> color_buf_[kBufSize];
    std::vector<color_ch_t> * current_frame_color_;
    std::queue<std::vector<color_ch_t> * > color_queue_;
    std::atomic< std::vector<color_ch_t>* > push_color_queue_;
    std::atomic_bool pop_color_queue_;
    int fourcc_color_;
    double fps_color_video_;

    std::vector<depth_ch_t> depth_buf_idle_;
    std::vector<depth_ch_t> depth_buf_[kBufSize];
    std::vector<depth_ch_t>* current_frame_depth_;
    std::queue<std::vector<depth_ch_t> * > depth_queue_;
    std::atomic< std::vector<depth_ch_t>* > push_depth_queue_;
    std::atomic_bool pop_depth_queue_;

    std::vector<color_ch_t> reg_buf_idle_;
    std::vector<color_ch_t> reg_buf_[kBufSize];
    std::vector<color_ch_t> * current_frame_reg_;
    std::queue<std::vector<color_ch_t> * > reg_queue_;
    std::atomic< std::vector<color_ch_t>* > push_reg_queue_;
    std::atomic_bool pop_reg_queue_;
    
    cv::Mat img_to_show_;
    cv::Mat1d H_; // homogenous transformation matrix

    // open video file from main thread via this pointers: opening one from child threads may fail.
    std::atomic< cv::VideoWriter* > video_writer_for_main_thread_;

    std::string color_img_extension_;

    std::string server_ip_, server_port_;
    
    std::atomic_uint32_t recorder_state_;
    std::atomic_uint32_t recorder_mode_;

    // color frame constants
    static const int kCWidth  = Kinect2::kCWidth;
    static const int kCHeight = Kinect2::kCHeight;
    static const int kCNumOfPixels = kCWidth * kCHeight;
    static const int kCNumOfChannelsPerPixel = 4; // BGRX
    static const int kCNumOfChannels = kCNumOfPixels * kCNumOfChannelsPerPixel;
    static const int kCNumOfBytesPerPixel = sizeof(color_ch_t) * kCNumOfChannelsPerPixel;
    static const int kCNumOfBytes = kCNumOfBytesPerPixel * kCNumOfPixels;

    // depth frame constants
    static const int kDWidth  = Kinect2::kDWidth;
    static const int kDHeight = Kinect2::kDHeight;
    static const int kDNumOfPixels = kDWidth * kDHeight;
    static const int kDNumOfChannelsPerPixel = 1;
    static const int kDNumOfChannels = kDNumOfPixels * kDNumOfChannelsPerPixel;
    static const int kDNumOfBytesPerPixel = sizeof(depth_ch_t) * kDNumOfChannelsPerPixel;
    static const int kDNumOfBytes = kDNumOfBytesPerPixel * kDNumOfPixels;

    // 'registered' frame constants
    static const int kRWidth  = kDWidth;
    static const int kRHeight = kDHeight;
    static const int kRNumOfPixels = kRWidth * kRHeight;
    static const int kRNumOfChannelsPerPixel = 4;
    static const int kRNumOfChannels = kRNumOfPixels * kRNumOfChannelsPerPixel;
    static const int kRNumOfBytesPerPixel = sizeof(color_ch_t) * kRNumOfChannelsPerPixel;
    static const int kRNumOfBytes = kRNumOfBytesPerPixel * kRNumOfPixels;
    
    // chessboard constants
    static const int kCornersWidth = 10; 
    static const int kCornersHeight = 7; 

    // constants for puttext
    static const int kTextThickness = 2;
    static const double kFontScale;
    static const cv::Scalar kRed, kGreen, kBlue, kOrange, kYellow, kSkyBlue;
    
    static const double kResizeScale;

    static const int kNumSetw = 8;

    static const int kKinectIdealFps = 30;

    std::atomic<double> fps_update_loop_;
    std::atomic<double> fps_push_;
    std::atomic<double> fps_pop_;
    std::atomic<double> fps_main_loop_;
    std::atomic<double> fps_sync_loop_;

    std::thread save_thread_;
    std::thread update_thread_;
    std::thread update_queue_thread_;
    
    // used when synchronizing with server is needed 
    std::thread sync_thread_;
    udp_t::endpoint remote_endpoint_;
    udp_t::endpoint local_endpoint_sync_;
    udp_t::endpoint local_endpoint_calib_;
    uint16_t local_endpoint_port_sync_ ;
    uint16_t local_endpoint_port_calib_;

    // Declaration of io_service_ should be prior to that of sockets.
    // This order of declaration affects destruction order and prevents segmentation fault.
    boost::asio::io_service io_service_; 
    std::unique_ptr<udp_t::socket> socket_calib_;
    std::unique_ptr<udp_t::socket> socket_sync_;

    std::atomic_int key_;

};



