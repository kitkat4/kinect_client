/*
  Kinect v2 recorder.

  created by Katsumasa Kitajima
 */


#include "kinect_manager.hpp"


KinectManager::KinectManager( const std::string& out_dir,
                              const std::string& server_ip,
                              const std::string& server_port,
                              const bool specify_each_frame,
                              const std::string& log_file_name,
                              const double fps_color,
                              const int fourcc_color )
    : device_( nullptr ),
      motion_name_( "scene" ),
      recorder_state_( InitialState ),
      current_frame_color_( nullptr ),
      current_frame_depth_( nullptr ),
      push_color_queue_( nullptr ),
      push_depth_queue_( nullptr ),
      pop_color_queue_( false ),
      pop_depth_queue_( false ),
      fps_update_( 0.0 ),
      fps_push_( 0.0 ),
      fps_pop_( 0.0 ),
      fps_main_( 0.0 ),
      fps_sync_( 0.0 ),
      H_( cv::Mat::eye( 4, 4, CV_32F ) ),
      video_writer_for_main_thread_( nullptr ),
      dir_path_for_main_thread_( nullptr ),
      server_ip_( server_ip ),
      server_port_( server_port ),
      local_endpoint_calib_( boost::asio::ip::udp::endpoint( boost::asio::ip::udp::v4(), kLocalEndpointPortCalib ) ),
      local_endpoint_sync_( boost::asio::ip::udp::endpoint( boost::asio::ip::udp::v4(), kLocalEndpointPortSync ) ),
      key_(0),
      recorder_mode_(0),
      out_dir_( out_dir ),
      fps_color_( fps_color ),
      fourcc_color_( fourcc_color ),
      logger_( (out_dir + "/" + log_file_name).c_str() ){
    

    if( server_ip_ == "" || server_port_ == "" )
        recorder_mode_ &= ~1; // manual
    else
        recorder_mode_ |= 1; // from server
    
    if( specify_each_frame )
        recorder_mode_ |= 2;
    else
        recorder_mode_ &= ~2;
}

KinectManager::~KinectManager(){

    device_->close();
    if( socket_calib_ ){
        if( socket_calib_->is_open() )
            socket_calib_->close();
        
        // Just in case. Not needed when some specific fields socket_calib_ depends on (io_service_)
        // are declared before the declaration of socket_calib_ in the class header file.
        socket_calib_.reset(); 
    }
    if( socket_sync_  ){
        if( socket_sync_->is_open() )
            socket_sync_->close();
        socket_sync_.reset(); // see the comment above
    }
}

void KinectManager::init(){

    try{

        libfreenect2::setGlobalLogger( libfreenect2::createConsoleLogger( libfreenect2::Logger::Warning ) );
        if( logger_.good() )
            libfreenect2::setGlobalLogger( &logger_ );
        
        if( freenect2_.enumerateDevices() == 0 ){
            std::cerr << "error: no Kinect v2 sensor connected.(" << __func__ << ")" << std::endl;
            return;
        }
        device_ = freenect2_.openDevice( freenect2_.getDefaultDeviceSerialNumber() ); // open kinect
        if( device_ == 0 ){
            std::cerr << "error: failed to open Kinect v2 device.(" << __func__ << ")" << std::endl;
            return;
        }

        listener_.reset( new libfreenect2::SyncMultiFrameListener( libfreenect2::Frame::Color |
                                                                   libfreenect2::Frame::Depth |
                                                                   libfreenect2::Frame::Ir ) );

        device_->setColorFrameListener( listener_.get() );
        device_->setIrAndDepthFrameListener( listener_.get() );

        if( ! isManual() ){
            if( server_ip_ == "" || server_ip_ == "" )
                throw std::runtime_error("no valid ip/port");

            boost::asio::ip::udp::resolver resolver(io_service_);
            boost::asio::ip::udp::resolver::query query( boost::asio::ip::udp::v4(),
                                                         server_ip_, server_port_ );
            remote_endpoint_ = *resolver.resolve(query);
            socket_calib_.reset( new boost::asio::ip::udp::socket( io_service_, local_endpoint_calib_ ) );
            socket_sync_.reset(  new boost::asio::ip::udp::socket( io_service_, local_endpoint_sync_ ) );

            socket_calib_->send_to( boost::asio::buffer( std::string("[Kinect] I am calibrator") ),
                                    remote_endpoint_ );
        }

        save_thread_ = std::thread( &KinectManager::save, this );
        update_thread_ = std::thread( &KinectManager::update, this );

        if( ! isManual() )
            sync_thread_ = std::thread( &KinectManager::sync, this );
        
        // allocate memory in advance to avoid overhead
        for( int i = 0; i < kBufSize; i++ ){
            color_buf_[i].resize( kColorVecSize );
            depth_buf_[i].resize( kDepthVecSize );
        }
        color_buf_idle_.resize( kColorVecSize );
        depth_buf_idle_.resize( kDepthVecSize );        
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }

    set( WaitingForKinectToBeStarted ); 

    return;
}

void KinectManager::startKinectAndCreateWindow(){

    try{
        if( ! is( WaitingForKinectToBeStarted ) || ! device_->start() ){
            std::cerr << "failed to start kinect sensor." << std::endl;
            return;
        }
        cv::namedWindow( "color", CV_WINDOW_AUTOSIZE );


        registration_.reset( new libfreenect2::Registration( device_->getIrCameraParams(),
                                               device_->getColorCameraParams() ) );
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
    set( WaitingForFpsStabilized );
}

void KinectManager::calibrate(){

    static libfreenect2::Frame undistorted( kDepthFrameWidth, kDepthFrameHeight, kDepthBytesPerPixel );
    static libfreenect2::Frame registered( kDepthFrameWidth, kDepthFrameHeight, kDepthBytesPerPixel );
    static cv::Mat registered_to_show;

    try{
    
        std::vector<cv::Point2f> corners;
        while( true ){

        
            libfreenect2::Frame color_frame( kColorFrameWidth, kColorFrameHeight, kColorBytesPerPixel,
                                             reinterpret_cast<uint8_t*>(current_frame_color_->data()) );
            libfreenect2::Frame depth_frame( kDepthFrameWidth, kDepthFrameHeight, kDepthBytesPerPixel,
                                             reinterpret_cast<uint8_t*>(current_frame_depth_->data()) );

            registration_->apply( &color_frame, &depth_frame, &undistorted, &registered );

            cv::Mat color_8UC3;
            cv::cvtColor( cv::Mat( kDepthFrameHeight, kDepthFrameWidth, CV_8UC4, registered.data ),
                          color_8UC3, CV_BGRA2BGR );
        
            registered_to_show = color_8UC3.clone();
        
            if( cv::findChessboardCorners( color_8UC3, cv::Size( kCornersWidth, kCornersHeight ), corners,
                                           CV_CALIB_CB_ADAPTIVE_THRESH + CV_CALIB_CB_NORMALIZE_IMAGE ) ){
                drawChessboardCorners( registered_to_show, cv::Size( kCornersWidth, kCornersHeight ),
                                       cv::Mat(corners), true );
                                   
                if( is( Calibrating ) )
                    break;
            }

            if( ! is( ReadyToCalibrate ) && ! is( Calibrating ) )
                return;

            cv::imshow( "registered", registered_to_show );
        }

        cv::Mat1f H_board_to_target(4, 4);
        H_board_to_target = cv::Mat1f::eye(4,4);

        if( server_ip_ != "" && server_port_ != "" && socket_calib_ ){ // fill in H_board_to_target

            socket_calib_->send_to( boost::asio::buffer( std::string("[Kinect] QUERY: H_mocap_to_chessboard") ), remote_endpoint_ );

            std::vector<double> recv_buf(12);
            socket_calib_->receive( boost::asio::buffer(recv_buf) );

            std::cerr << "received: " << std::endl;
            for( int i_row = 0; i_row < 3; i_row++ ){
                for( int i_col = 0; i_col < 4; i_col++ )
                    std::cerr << recv_buf[ i_row * 4 + i_col ] << " ";
                std::cerr << std::endl;
            }
            
            for( int i_row = 0; i_row < 3; i_row++ )
                for( int i_col = 0; i_col < 4; i_col++ )
                    H_board_to_target( i_row, i_col ) = recv_buf[ i_row * 4 + i_col ];
            
            
            H_board_to_target( 3, 0 ) = H_board_to_target( 3, 1 ) = H_board_to_target( 3, 2 ) = 0;
            H_board_to_target( 3, 3 ) = 1;
        }

        int min_x=10000, min_y=10000, max_x=0, max_y=0;
        for( int i = 0; i < corners.size(); i++ ){
            min_x = min_x <= corners[i].x ? min_x : corners[i].x;
            min_y = min_y <= corners[i].y ? min_y : corners[i].y;
            max_x = max_x >= corners[i].x ? max_x : corners[i].x;
            max_y = max_y >= corners[i].y ? max_y : corners[i].y;
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZ>( max_x - min_x + 1,
                                                                                       max_y - min_y + 1 ) );
        for( int i_row = min_y; i_row <= max_y; i_row++ ){ // populate the cloud
            for( int i_col = min_x; i_col <= max_x; i_col++ ){
                pcl::PointXYZ point;
                float rgb_buf;
                registration_->getPointXYZRGB( &undistorted, &registered, i_row, i_col,
                                               point.x, point.y, point.z, rgb_buf );
                point.y *= -1;
                (*cloud)( i_col - min_x, i_row - min_y ) =  point;
            }
        }
    
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZ> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setDistanceThreshold (0.01);
        seg.setInputCloud (cloud);
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
        seg.segment (*inliers, *coefficients);

        if (inliers->indices.size () == 0) {
            PCL_ERROR ("Could not estimate a planar model for the given dataset.");
            return;
        }

        float a = coefficients->values[0];
        float b = coefficients->values[1];
        float c = coefficients->values[2];
        float d = coefficients->values[3];
    
        cv::Mat1f z_axis(3,1); // normalized z-axis of board coords (oriented toward kinect sensor)
        z_axis(0) = a;
        z_axis(1) = b;
        z_axis(2) = c;
        cv::normalize( z_axis, z_axis );
        if( z_axis(2) > 0 )
            z_axis *= -1;

        cv::Mat1f x_axis(3,1);    // normalized x-axis of board coords (from left to right)
        {
            pcl::PointXYZ total_x(0,0,0);
            for( int i_row = 0; i_row < kCornersHeight; i_row++ ){
                pcl::PointXYZ p1, p2;
                float rgb_buf;
                registration_->getPointXYZRGB( &undistorted, &registered,
                                               corners[ i_row * kCornersWidth ].y,
                                               corners[ i_row * kCornersWidth ].x,
                                               p1.x, p1.y, p1.z, rgb_buf );
                p1.y *= -1;
                registration_->getPointXYZRGB( &undistorted, &registered,
                                               corners[ i_row * kCornersWidth + kCornersWidth - 1 ].y,
                                               corners[ i_row * kCornersWidth + kCornersWidth - 1 ].x,
                                               p2.x, p2.y, p2.z, rgb_buf );
                p2.y *= -1;
                pcl::PointXYZ projected1 = project( a, b, c, d, p1 ), projected2 = project( a, b, c, d, p2 );
                total_x.x += projected1.x - projected2.x; // images taken from kinect are mirrored
                total_x.y += projected1.y - projected2.y;
                total_x.z += projected1.z - projected2.z;
            }
            x_axis(0) = total_x.x/kCornersHeight;
            x_axis(1) = total_x.y/kCornersHeight;
            x_axis(2) = total_x.z/kCornersHeight;
            cv::normalize( x_axis, x_axis );
        }

        cv::Mat1f y_axis(3,1);    // normalized y-axis of board coords 
        cv::normalize( z_axis.cross( x_axis ), y_axis );

        cv::Mat1f center(3,1);       // origin of board coords (center of chessboard)
        {
            pcl::PointXYZ total(0,0,0);
            for( int i = 0; i < kCornersWidth * kCornersHeight; i++ ){
                pcl::PointXYZ p;
                float rgb_buf;
                registration_->getPointXYZRGB( &undistorted, &registered,
                                               corners[i].y, corners[i].x,
                                               p.x, p.y, p.z, rgb_buf );
                p.y *= -1;
                total.x += p.x;
                total.y += p.y;
                total.z += p.z;
            }
            center(0) = total.x / (kCornersWidth * kCornersHeight);
            center(1) = total.y / (kCornersWidth * kCornersHeight);
            center(2) = total.z / (kCornersWidth * kCornersHeight);
        }


        cv::Mat1f H_board_to_kinect(4, 4);
        for( int i=0; i < 3; i++ ){
            H_board_to_kinect(i,0) = x_axis(i);
            H_board_to_kinect(i,1) = y_axis(i);
            H_board_to_kinect(i,2) = z_axis(i);
            H_board_to_kinect(i,3) = center(i);
        }
        H_board_to_kinect(3,0) = H_board_to_kinect(3,1) = H_board_to_kinect(3,2) = 0;
        H_board_to_kinect(3,3) = 1.0;

        H_ = H_board_to_target * H_board_to_kinect.inv();
        
        std::cerr  << "H_board_to_kinect.inv(): " << std::endl
                   << H_board_to_kinect.inv() << std::endl
                   << "H_board_to_target: " << std::endl
                   << H_board_to_target << std::endl
                   << "H_: " << std::endl
                   << H_ << std::endl;

        bool h_is_ok = true;
        for( int i = 0; i < 4; i++ )
            for( int j = 0; j < 4; j++ )
                if( ISNAN( H_(i,j) ) )
                    h_is_ok = false;

        if( ! h_is_ok )
            std::cerr << "error: invalid homogenous transformation matrix H_." << std::endl;
        

        set( ReadyToCalibrate );
        

    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
}


    
void KinectManager::saveCurrentFrame(){
    
    if( push_color_queue_ )
        std::cerr << "warning: a color frame to be saved has been lost ("
                  << __func__ << ")." << std::endl;
    else
        push_color_queue_ = current_frame_color_;
    
    if( push_depth_queue_ )
        std::cerr << "warning: a depth frame to be saved has been lost ("
                  << __func__ << ")." << std::endl;
    else
        push_depth_queue_ = current_frame_depth_;
}

void KinectManager::enterMainLoop(){

    while( ! is( Exiting ) ){

        static FpsCalculator fps_calc( 30 );
        fps_main_ = fps_calc.fps();
        
        showImgAndInfo();
        
        updateQueue();
        
        if( is( Exiting ) )
            continue;

        if( video_writer_for_main_thread_ && dir_path_for_main_thread_ ){
            video_writer_for_main_thread_->open( *dir_path_for_main_thread_ + "/color.avi",
                                                 fourcc_color_, fps_color_,
                                                 cv::Size( kColorFrameWidth, kColorFrameHeight ));
            video_writer_for_main_thread_ = nullptr;
            dir_path_for_main_thread_ = nullptr;
        }

        if( is( WritingData ) && queuesAreEmpty() ){
            set( ReadyToRecord );
            if( ! isManual() )
                socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] ready")), remote_endpoint_ );
        }

        
        int key = cv::waitKey(1);
        if( isManual() )
            key_ = key;
        if( key_ == 'q' ){
            set( Exiting );
            std::cerr << "quit" << std::endl;
        }else if( key_ == 'c' ){
            if( is( ReadyToCalibrate ) || is( Calibrating ) ){
                set( ReadyToRecord );
                cv::destroyWindow( "registered" );
            }else if( is( ReadyToRecord ) ){
                cv::namedWindow( "registered", CV_WINDOW_AUTOSIZE );
                set( ReadyToCalibrate );
            }
        }else if( key_ == 's' ){
            if( is( ReadyToCalibrate ) ){
                std::cerr << "start calibrating" << std::endl;
                set( Calibrating );
            }else if( is( Recording ) ){
                std::cerr << "stop recording" << std::endl;
                set( WritingData );
                fps_push_ = 0.0;
            }else if( is( ReadyToRecord) && ! specifyEachFrame() ){
                std::cerr << "start recording" << std::endl;
                set( Recording );
            }
        }else if( key == 'r' ){ // this may not be used, but is implemented for debugging.
            if( isManual() && specifyEachFrame() ){
                if( is( ReadyToRecord ) )
                    set( Recording );
                saveCurrentFrame(); 
            }
        }

        key_ = 0;
    }

    if( ! isManual() ){
        io_service_.stop();
        sync_thread_.join();
    }
    save_thread_.join();
    update_thread_.join();
    
    return;
}

void KinectManager::update(){

    static int buf_index = 0;
    static uint8_t* data_dst_color = nullptr;
    static float* data_dst_depth = nullptr;

    try{

        while( ! is( WaitingForFpsStabilized ) && ! is( ReadyToRecord ) )
            std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
        
        while( ! is( Exiting ) ){

            listener_->waitForNewFrame( frames_ );

            if( is( Recording ) ){
                data_dst_color = &color_buf_[buf_index][0];
                data_dst_depth = &depth_buf_[buf_index][0];
            }else{
                data_dst_color = &color_buf_idle_[0];
                data_dst_depth = &depth_buf_idle_[0];
            }

            std::copy( frames_[libfreenect2::Frame::Color]->data,
                       frames_[libfreenect2::Frame::Color]->data + kColorVecSize,
                       data_dst_color );
            std::copy( (float*)frames_[libfreenect2::Frame::Depth]->data,
                       (float*)frames_[libfreenect2::Frame::Depth]->data + kDepthVecSize,
                       data_dst_depth );

            cv::resize( cv::Mat( kColorFrameHeight, kColorFrameWidth,CV_8UC4,
                                 data_dst_color ),
                        img_to_show_, cv::Size(), kResizeScale, kResizeScale );

            if( is( Recording ) ){
                current_frame_color_ = color_buf_ + buf_index;
                current_frame_depth_ = depth_buf_ + buf_index;
                if( ++buf_index == kBufSize )
                    buf_index = 0;
            }else{
                current_frame_color_ = &color_buf_idle_;
                current_frame_depth_ = &depth_buf_idle_;
            }

            
            listener_->release( frames_ );

            static FpsCalculator fps_calc( 30 );
            
            if( fps_calc.fps( fps_update_ ) ){
                if( is( WaitingForFpsStabilized ) ){
                    
                    static int high_fps_count = 0;
                    
                    if( fps_update_ >= 29.0 )
                        high_fps_count++;
                    else
                        high_fps_count = 0;
                    
                    if( high_fps_count == 3 ){
                        set( ReadyToRecord );
                        if( ! isManual() )
                            socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] ready")), remote_endpoint_ );
                    }
                }
            }
            
            if( ! specifyEachFrame() && is( Recording ) ){
                saveCurrentFrame();
                fps_push_ = fps_update_;
            }

        }
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
}

void KinectManager::updateQueue(){
    
    if( push_color_queue_ ){
        color_queue_.push( current_frame_color_ );
        push_color_queue_ = nullptr;
    }
    if( push_depth_queue_ ){
        depth_queue_.push( current_frame_depth_ );
        push_depth_queue_ = nullptr;
    }
    if( pop_color_queue_ && pop_depth_queue_ && ! color_queue_.empty() && ! depth_queue_.empty() ){
        
        color_queue_.pop();
        depth_queue_.pop();
        pop_color_queue_ = false;
        pop_depth_queue_ = false;

        static FpsCalculator fps_calc( 3 );
        fps_pop_ = fps_calc.fps();
    }

    return;
}

void KinectManager::save(){

    try{
    
        while( ! is( Exiting ) ){ // scene loop

            while( is( ReadyToCalibrate ) )
                calibrate();

            if( ! is( Recording ) ){
                std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
                continue;
            }
            
            std::string scene_dir;
            { // create scene directory
                for( int i = 1; ; i++ ){ // begin from 1 for the compatibility with Cortex.
                    std::stringstream sstream;
                    sstream << out_dir_ + "/" + motion_name_ << std::setw( kNumSetw ) << std::setfill('0') << i;
                    if( ! boost::filesystem::exists( boost::filesystem::path( sstream.str() ) ) ){
                        scene_dir = sstream.str();
                        break;
                    }
                }
                boost::filesystem::create_directory( boost::filesystem::path( scene_dir ) );
            }
            dir_path_for_main_thread_ = &scene_dir;

            // open video file
            cv::VideoWriter video_writer_color;
            video_writer_for_main_thread_ = &video_writer_color;
            { // wait for video_writer_color to be opened
                int count = 0;
                while( video_writer_for_main_thread_ ){ 
                    std::this_thread::sleep_for( std::chrono::milliseconds( 1 ));
                    if( count++ > 5000 ){
                        std::cerr << "error: failed to open " << scene_dir + "/color.avi" << std::endl;
                        return;
                    }
                }
            }

            int frame_count = 0;
            while( is( Recording ) || ! queuesAreEmpty() ){ // frame loop
                if( is( Exiting ) )
                    break;
                std::stringstream sstream;
                sstream << scene_dir + "/"
                        << std::setw( kNumSetw ) << std::setfill('0') << frame_count << ".pcd";
                if( saveDepth( sstream.str() ) )
                    frame_count++;
                saveColor( video_writer_color );
            } // end of frame loop
            fps_pop_ = 0.0;
        } // end of scene loop
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": "  << ex.what() << std::endl;
        throw;
    }
}

void KinectManager::saveColor( cv::VideoWriter& video_writer ){

    if( pop_color_queue_ )
        return;

    if( color_queue_.empty() )
        return;

    if( color_queue_.front() == nullptr ){
        pop_color_queue_ = true;
        return;
    }

    static cv::Mat tmp_color_img;
    cv::cvtColor( cv::Mat( kColorFrameHeight, kColorFrameWidth, CV_8UC4, &color_queue_.front()->front() ),
                  tmp_color_img, CV_BGRA2BGR );
    cv::flip( tmp_color_img, tmp_color_img, 1 ); // color images taken from kinect are mirrored
    video_writer << tmp_color_img;
    
    pop_color_queue_ = true;
    return;
}

bool KinectManager::saveDepth( const std::string& file_path ){

    static libfreenect2::Frame undistorted( kDepthFrameWidth, kDepthFrameHeight, kDepthBytesPerPixel );
    static libfreenect2::Frame registered( kDepthFrameWidth, kDepthFrameHeight, kDepthBytesPerPixel );
    
    if( pop_depth_queue_ )
        return false;
    if( depth_queue_.empty() )
        return false;
    if( depth_queue_.front() == nullptr ){
        pop_depth_queue_ = true;
        return false;
    }

    libfreenect2::Frame color_frame( kColorFrameWidth, kColorFrameHeight, kColorBytesPerPixel,
                                     &color_queue_.front()->at(0) );
    libfreenect2::Frame depth_frame( kDepthFrameWidth, kDepthFrameHeight, kDepthBytesPerPixel,
                                     reinterpret_cast<uint8_t*>(&depth_queue_.front()->at(0)) );

    registration_->apply( &color_frame, &depth_frame, &undistorted, &registered );

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud( new pcl::PointCloud<pcl::PointXYZRGB> );

    float rgb_buf;
    for( int i_row = 0; i_row < kDepthFrameHeight; i_row++ ){
        for( int i_column = 0; i_column < kDepthFrameWidth; i_column++ ){
            pcl::PointXYZRGB point;
            float x,y,z;
            registration_->getPointXYZRGB( &undistorted, &registered, i_row, i_column,
                                           x, y, z, rgb_buf );
            y *= -1;
            
            cv::Mat1f vec_kinect_coords( 4, 1 ), vec_output_coords( 4, 1 );

            vec_kinect_coords( 0 ) = x;
            vec_kinect_coords( 1 ) = y;
            vec_kinect_coords( 2 ) = z;
            vec_kinect_coords( 3 ) = 1;
            
            vec_output_coords = H_ * vec_kinect_coords ;
            
            point.x = vec_output_coords( 0 );
            point.y = vec_output_coords( 1 );
            point.z = vec_output_coords( 2 );

            point.rgb = rgb_buf;
            
            cloud->push_back( point );
        }
    }
    pcl::io::savePCDFileBinary<pcl::PointXYZRGB>( file_path, *cloud );
    pop_depth_queue_ = true;
    return true;
}

void KinectManager::sync(){

    try{
        socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] I am synchronizer") ),
                               remote_endpoint_ );

        static std::vector<char> buf( 256, '\0' );
        
        while( ! is( Exiting ) ){

            static FpsCalculator fps_calc( 30 );
            fps_sync_ = fps_calc.fps();
            
            io_service_.reset();
            
            memset( &buf[0], '\0', 256 );
            
            socket_sync_->async_receive( boost::asio::buffer( buf ),
                                         std::bind(&KinectManager::processRecvBuf,
                                                   this, &buf,
                                                   std::placeholders::_1,
                                                   std::placeholders::_2));

            io_service_.run();
        } // end of while( ! is( Exiting ) )
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
    
    return;
}

void KinectManager::processRecvBuf( const std::vector<char>* buf,
                                    const boost::system::error_code& error,
                                    const std::size_t size ){

    static int32_t frame_id = 0, frame_id_prev = 0;
    // static clock_t timestamp = clock();
        
    if( (*buf)[0] == 'c' ){
        if( 0 == strcmp( &(*buf)[0], "cestimate delay" ) ){
            socket_sync_->send_to( boost::asio::buffer( *buf ), remote_endpoint_ );
            std::cerr << "estimate delay: echoed back" << std::endl;
        }else if( 0 == strncmp( &(*buf)[0], "cset motion name ", strlen("cset motion name ") ) ){
            motion_name_ = &(*buf)[0] + strlen("cset motion name ");
            std::cerr << "set motion name as " + motion_name_ << std::endl;
        }
        else{
            key_ = (*buf)[1];
            std::cerr << "key_: " << (char)key_ << std::endl;
        }
    }
    else if( (*buf)[0] == 'i' ){

        frame_id = *(int32_t*)( &(*buf)[1] );
        std::cerr << "\rreceived frame_id: " << std::setw(5) << frame_id << "  "
                  << std::flush;
        if( frame_id == 0 && is( ReadyToRecord ) ){ // start recording 
            if( ! queuesAreEmpty() )
                throw std::runtime_error( "not ready to start recording." );
            set( Recording );
            saveCurrentFrame();
            frame_id_prev = 0;
        }else if( is( Recording ) ){

            static FpsCalculator fps_calc( 15 );
            fps_push_ = fps_calc.fps();
            // std::cout << fps_push_ << std::endl;

            if( frame_id == frame_id_prev + 1 )
                saveCurrentFrame();
            else
                throw std::runtime_error( "non-sequential frame id arrived." );
            
            frame_id_prev = frame_id;
        }else
            throw std::runtime_error( "unexpected error." );                    
    }else
        throw std::runtime_error( "invalid data arrived." );

}

void KinectManager::showImgAndInfo(){

    if( ! current_frame_color_ )
        return;

    std::stringstream sstream;

    if( is( Recording ) )
        if( isManual() && ! specifyEachFrame() )
            cv::putText( img_to_show_, "Recording ( s: stop recording, q: abort )" , cv::Point(10,40),
                         cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,0,255), kTextThickness );
        else
            cv::putText( img_to_show_, "Recording" , cv::Point(10,40),
                         cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,0,255), kTextThickness );
    else if( is( ReadyToCalibrate ) )
        if( isManual() )
            cv::putText( img_to_show_, "Ready to start calibration ( s: start calibration, c: recorder mode, q: abort )",
                         cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255),
                         kTextThickness );
        else
            cv::putText( img_to_show_, "Ready to start calibration",
                         cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, 0.7, cv::Scalar(0,255,255),
                         kTextThickness );
    else if( is( WaitingForFpsStabilized ) )
        cv::putText( img_to_show_, "Not Ready", cv::Point(10,40),
                     cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,100,255), kTextThickness );
    else if( is( Calibrating ) )
        if( isManual() )
            cv::putText( img_to_show_, "Calibrating ( q: abort )", cv::Point(10,40),
                         cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,100,255), kTextThickness );
        else
            cv::putText( img_to_show_, "Calibrating", cv::Point(10,40),
                         cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,100,255), kTextThickness );
    else if( is( WritingData ) )
        if( isManual() && ! specifyEachFrame() )
            cv::putText( img_to_show_, "Writing files... ( q: abort )" , cv::Point(10,40),
                         cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,100,255), kTextThickness );
        else
            cv::putText( img_to_show_, "Writing files..." , cv::Point(10,40),
                         cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,100,255), kTextThickness );
    else if( is( ReadyToRecord ) )
        if( isManual() && ! specifyEachFrame() )
            cv::putText( img_to_show_, "Ready to start recording ( s: start recording, c: calib mode, q: quit )",
                         cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(0,255,0),
                         kTextThickness );
        else
            cv::putText( img_to_show_, "Ready to start recording",
                         cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, kFontScale,cv::Scalar(0,255,0),
                         kTextThickness );
    else
        cv::putText( img_to_show_, "Error: unknown state!",
                     cv::Point(10,40), cv::FONT_HERSHEY_SIMPLEX, kFontScale,cv::Scalar(0,0,255),
                     kTextThickness );
        

    sstream << std::fixed << std::setprecision(3) << fps_update_ ;
    cv::putText( img_to_show_, "update: " + sstream.str() + " fps", cv::Point(10,100),
                 cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0), kTextThickness );
    sstream.str("");
    sstream << std::fixed << std::setprecision(3) << fps_push_;
    cv::putText( img_to_show_, "push  : " + sstream.str() + " fps", cv::Point(10,130),
                 cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0), kTextThickness );
    sstream.str("");
    sstream << std::fixed << std::setprecision(3) << fps_pop_;
    cv::putText( img_to_show_, "pop   : " + sstream.str() + " fps", cv::Point(10,160),
                 cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0), kTextThickness );
    sstream.str("");
    sstream << std::fixed << std::setprecision(3) << fps_main_;
    cv::putText( img_to_show_, "main  : " + sstream.str() + " fps", cv::Point(10,190),
                 cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0), kTextThickness );
    sstream.str("");
    sstream << std::fixed << std::setprecision(3) << fps_sync_;
    cv::putText( img_to_show_, "sync  : " + sstream.str() + " fps", cv::Point(10,220),
                 cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0), kTextThickness );

    
    int left_color_queue_size = kBufSize - color_queue_.size();
    int left_depth_queue_size = kBufSize - depth_queue_.size();
        
    if( left_color_queue_size == 0 || left_depth_queue_size == 0 )
        std::cerr << "error: run out of queues" << std::endl;
    if( left_color_queue_size != left_depth_queue_size )
        std::cerr << "error: invalid queue size" << std::endl;
        
    sstream.str("");
    sstream  << left_color_queue_size << std::flush;
    cv::putText( img_to_show_, "left color queue size: " + sstream.str(),
                 cv::Point(10,250), cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0),
                 kTextThickness );
    sstream.str("");
    sstream  << left_depth_queue_size << std::flush;
    cv::putText( img_to_show_, "left depth queue size: " + sstream.str(),
                 cv::Point(10,280), cv::FONT_HERSHEY_SIMPLEX, kFontScale, cv::Scalar(255,255,0),
                 kTextThickness );

    cv::imshow( "color", img_to_show_ );
}

const double KinectManager::kResizeScale = 0.5;
const double KinectManager::kFontScale = 0.6;


void MyFileLogger::log(Level level, const std::string &message){

    if( level >= Info ){
        // this message is originally "Info", but treated as "Warning".
        if( message.find( "packets were lost" ) != std::string::npos ){ 
            std::cerr << "[Warning] " << message << std::endl;
            logfile_ << "[Warning] " << message << std::endl;
        }else
            logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message
                     << std::endl;
    }else{
        std::cerr << "[" << libfreenect2::Logger::level2str(level) << "] " << message
                  << std::endl;
        logfile_ << "[" << libfreenect2::Logger::level2str(level) << "] " << message
                 << std::endl;
    }
}
