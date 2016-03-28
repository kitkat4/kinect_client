/*
  Kinect v2 recorder.

  created by Katsumasa Kitajima
 */


#pragma once

#include "link_libs.hpp"

#include <opencv2/opencv.hpp>

#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/logger.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>

#include <boost/asio.hpp>

#include <thread>
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

//#include <mmsystem.h> // timeGetTime()
#include <windows.h>


#if defined(_MSC_VER) && _MSC_VER < 1800
#ifndef __func__
#define __func__ __FUNCTION__
#endif
#define ISNAN(f) _isnan(f)
#else
#define ISNAN(f) std::isnan(f)
#endif



class MyFileLogger: public libfreenect2::Logger{
    
private:
    std::ofstream logfile_;
public:
    MyFileLogger(const char *filename){
        open( filename );
        level_ = Debug;

    }
    ~MyFileLogger(){
        logfile_.close();
    }
    void open( const char* filename ){
        if( filename )
            logfile_.open(filename, std::ofstream::app );
        if( good() ){
            setlocale( LC_ALL, "JPN" );
            time_t tmp_time = time( nullptr );
            logfile_ << std::endl
                     << asctime( localtime( &tmp_time ) ); 
        }
    }

    bool good(){
        return logfile_.is_open() && logfile_.good();
    }
    virtual void log(Level level, const std::string &message);
};



class KinectManager{

    typedef uint8_t color_ch_t; // channel type of color frame
    typedef float depth_ch_t;   // channel type of depth frame
    typedef libfreenect2::Frame frame_t;
    typedef boost::asio::ip::udp udp_t;
    typedef pcl::PointCloud<pcl::PointXYZ> cloud_t;
    typedef pcl::PointCloud<pcl::PointXYZRGB> cloudRGB_t;
    
public:
    
    typedef enum{
        InitialState,
        WaitingForKinectToBeStarted,
        WaitingForFpsStabilized,
        ReadyToRecord,
        WritingData,
        Recording,
        ReadyToCalibrate,
        Calibrating,
        Exiting,
    }RecorderState;

    KinectManager( const std::string& out_dir,
                   const std::string& server_ip = "",
                   const std::string& server_port = "",
                   const bool specify_each_frame = false,
                   const int save_fps = kKinectIdealFps,
                   const double fps_color_video = 29.97,
                   const std::string& log_file_name = "kinect.log",
                   const int fourcc_color = // CV_FOURCC( 'M', 'J', 'P', 'G' )
                   CV_FOURCC('X','V','I','D')
                   );
    ~KinectManager();

    void init();
    void startKinectAndCreateWindow();
    void calibrate();
    void stopKinectAndDestroyWindow(){
        cv::destroyAllWindows();
        device_->stop();
        set( Exiting );
    }
    

    void saveCurrentColorFrame(){
        if( push_color_queue_ )
            std::cerr << "warning: a color frame to be saved has been lost ("
                      << __func__ << ")." << std::endl;
        else
            push_color_queue_ = current_frame_color_;
    }
    void saveCurrentDepthFrame(){
        if( push_depth_queue_ )
            std::cerr << "warning: a depth frame to be saved has been lost ("
                      << __func__ << ")." << std::endl;
        else
            push_depth_queue_ = current_frame_depth_;
    }
    void saveCurrentFrame(){
        saveCurrentColorFrame();
        saveCurrentDepthFrame();
    }

    void enterMainLoop();
    
private:
    void update();
    void updateQueue();
    void save();

    bool saveColor( cv::VideoWriter& video_writer ); // when save color frames as a video file
    bool saveColor( const std::string& file_path ); // when save color frames as pictures
    bool saveDepth( const std::string& file_path );
    void sync();
    void processRecvBuf( const std::vector<char>* buf, // somehow buf as reference doesn't work
                         const boost::system::error_code& error,
                         const std::size_t size );
    void showImgAndInfo();
    void putText( cv::Mat& img, const std::string& text, const bool newline = true,
                  const cv::Scalar& color = kSkyBlue,
                  const cv::Point& org = cv::Point(-1,-1) )const;
    void putText( cv::Mat& img, const std::string& text, const bool newline,
                  const cv::Point& org )const{
        putText( img ,text, newline, kSkyBlue, org );
    };

    std::string toString( const volatile double value ){
        static std::stringstream sstream;
        sstream.str("");
        sstream << std::fixed << std::setprecision(3) << value;
        return sstream.str();
    }
    std::string toString( const volatile int value ){
        static std::stringstream sstream;
        sstream.str("");
        sstream << value;
        return sstream.str();
    }

    bool is( const uint32_t state )const{ return recorder_state_ == state; }
    void set( const uint32_t state ){ recorder_state_ = state; }
    bool isStandalone()const{ return (recorder_mode_ & 1) == 0; }
    bool specifyEachFrame()const{ return (recorder_mode_ & 2) == 2; }
    bool saveAsVideo()const{ return (recorder_mode_ & 4) == 4; }
    bool queuesAreEmpty()const{ return color_queue_.empty() && depth_queue_.empty(); }

    // project src onto plane ax+by+cz+d=0
    pcl::PointXYZ project( const float a, const float b, const float c, const float d,
                           const pcl::PointXYZ& src )const{
        return pcl::PointXYZ( - src.x * d / ( a*src.x + b*src.y + c*src.z ),
                              - src.y * d / ( a*src.x + b*src.y + c*src.z ),
                              - src.z * d / ( a*src.x + b*src.y + c*src.z ) );
    }

    // to thin out frames according to fps
    bool notToBeThinnedOut( const uint64_t frame_count, const int fps, int loop_fps )const{
        return loop_fps <= 0 ? true : ( frame_count * fps ) % loop_fps < fps;
    }

    // class FpsCalculator{
    // public:
    //     FpsCalculator( const int update_cycle )
    //         : loop_count_( 0 ),
    //           timestamp_( clock() ),
    //           update_cycle_( update_cycle ),
    //           fps_( 0.0 ){}
    //     ~FpsCalculator(){}
    //     bool fps( volatile double& out_fps ){
    //         bool ret = false;
    //         if( ++loop_count_ % update_cycle_ == 0 ){
    //             clock_t now = clock();
    //             fps_ = (double)(update_cycle_*CLOCKS_PER_SEC)/( now - timestamp_ );
    //             timestamp_ = now;
    //             ret = true;
    //             loop_count_ = 0;
    //         }
    //         out_fps = fps_;
    //         return ret;
    //     }
    //     double fps(){
    //         double ret = 0;
    //         fps( ret );
    //         return ret;
    //     }

    // private:
    //     int loop_count_;
    //     clock_t timestamp_;
    //     int update_cycle_;
    //     double fps_;
    // };

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
        bool fps( volatile double& out_fps ){
            bool ret = false;
            if( ++loop_count_ % update_cycle_ == 0 ){
                LARGE_INTEGER now;
                QueryPerformanceCounter( &now );
                fps_ = (double)(update_cycle_*freq_.QuadPart)/( now.QuadPart - timestamp_.QuadPart );
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


    MyFileLogger logger_;
    
    libfreenect2::Freenect2 freenect2_;
    libfreenect2::Freenect2Device* device_;
    std::unique_ptr<libfreenect2::SyncMultiFrameListener> listener_;
    libfreenect2::FrameMap frames_;
    std::unique_ptr<libfreenect2::Registration> registration_;

    std::string out_dir_;
    std::string scene_dir_;
    std::string motion_name_;
    
    static const int kBufSize = 600;

    std::vector<color_ch_t> color_buf_idle_;
    std::vector<color_ch_t> color_buf_[kBufSize];
    std::vector<color_ch_t> * current_frame_color_;
    std::queue<std::vector<color_ch_t> * > color_queue_;
    std::vector<color_ch_t>* volatile push_color_queue_;
    volatile bool pop_color_queue_;
    int fourcc_color_;
    double fps_color_video_;

    std::vector<depth_ch_t> depth_buf_idle_;
    std::vector<depth_ch_t> depth_buf_[kBufSize];
    std::vector<depth_ch_t>* current_frame_depth_;
    std::queue<std::vector<depth_ch_t> * > depth_queue_;
    std::vector<depth_ch_t>* volatile push_depth_queue_;
    volatile bool pop_depth_queue_;
    
    cv::Mat img_to_show_;
    cv::Mat1f H_; // homogenous transformation matrix

    // open video file from main thread via this pointers: opening one from child threads may fail.
    cv::VideoWriter* volatile video_writer_for_main_thread_;

    std::string server_ip_, server_port_;
    
    volatile uint32_t recorder_state_;
    volatile uint32_t recorder_mode_;

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



    const int fps_save_;
    
    volatile double fps_update_loop_;
    volatile double fps_push_;
    volatile double fps_pop_;
    volatile double fps_main_loop_;
    volatile double fps_sync_loop_;

    std::thread save_thread_;
    std::thread update_thread_;
    
    // used when synchronizing with server is needed 
    std::thread sync_thread_;
    udp_t::endpoint remote_endpoint_;
    udp_t::endpoint local_endpoint_sync_;
    udp_t::endpoint local_endpoint_calib_;
    // Declaration of io_service_ should be prior to that of sockets.
    // This order of declaration affects destruction order and prevents segmentation fault.
    boost::asio::io_service io_service_; 
    std::unique_ptr<udp_t::socket> socket_calib_;
    std::unique_ptr<udp_t::socket> socket_sync_;
    static const uint16_t kLocalEndpointPortSync  = 49998;
    static const uint16_t kLocalEndpointPortCalib = 49999;

    volatile int key_;

};



