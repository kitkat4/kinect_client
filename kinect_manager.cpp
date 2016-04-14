/*
  Kinect v2 recorder.

  created by Katsumasa Kitajima
 */


#include "kinect_manager.hpp"


KinectManager::KinectManager( const std::string& out_dir,
                              const std::string& server_ip,
                              const std::string& server_port,
                              const bool specify_each_frame,
                              const int fps_save,
                              const double fps_color_video,
                              const std::string& log_file_name,
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
      fps_update_loop_( 0.0 ),
      fps_push_( 0.0 ),
      fps_pop_( 0.0 ),
      fps_main_loop_( 0.0 ),
      fps_sync_loop_( 0.0 ),
      H_( cv::Mat::eye( 4, 4, CV_32F ) ),
      video_writer_for_main_thread_( nullptr ),
      server_ip_( server_ip ),
      server_port_( server_port ),
      local_endpoint_calib_( udp_t::endpoint( udp_t::v4(), kLocalEndpointPortCalib ) ),
      local_endpoint_sync_( udp_t::endpoint( udp_t::v4(), kLocalEndpointPortSync ) ),
      key_(0),
      recorder_mode_(0),
      out_dir_( out_dir ),
      fps_save_( fps_save <= kKinectIdealFps ? fps_save : kKinectIdealFps ),
      fps_color_video_( fps_color_video ),
      fourcc_color_( fourcc_color ),
      logger_( (out_dir + "/" + log_file_name).c_str() ){
    

    // whether this works as a client or a standalone program
    if( server_ip_ == "" || server_port_ == "" )
        recorder_mode_ &= ~1; // standalone
    else
        recorder_mode_ |= 1;  // client

    // whether you specify each frame or start/stop
    if( specify_each_frame )
        recorder_mode_ |= 2;  // specify each frame
    else
        recorder_mode_ &= ~2; // specify start/stop

    // whether you save color frames as a video or a set of pictures
    if( fps_color_video > 0.0 )
        recorder_mode_ |= 4;  // video
    else
        recorder_mode_ &= ~4; // pictures

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

        listener_.reset( new libfreenect2::SyncMultiFrameListener( frame_t::Color |
                                                                   frame_t::Depth |
                                                                   frame_t::Ir ) );

        device_->setColorFrameListener( listener_.get() );
        device_->setIrAndDepthFrameListener( listener_.get() );

        if( ! isStandalone() ){
            if( server_ip_ == "" || server_ip_ == "" )
                throw std::runtime_error("no valid ip/port");

            udp_t::resolver resolver(io_service_);
            udp_t::resolver::query query( udp_t::v4(),
                                                         server_ip_, server_port_ );
            remote_endpoint_ = *resolver.resolve(query);
            socket_calib_.reset( new udp_t::socket( io_service_, local_endpoint_calib_ ) );
            socket_sync_.reset(  new udp_t::socket( io_service_, local_endpoint_sync_ ) );

            socket_calib_->send_to( boost::asio::buffer( std::string("[Kinect] I am calibrator") ),
                                    remote_endpoint_ );
        }

        save_thread_ = std::thread( &KinectManager::save, this );
        update_thread_ = std::thread( &KinectManager::update, this );

        if( ! isStandalone() )
            sync_thread_ = std::thread( &KinectManager::sync, this );
        
        // allocate memory in advance to avoid overhead
        for( int i = 0; i < kBufSize; i++ ){
            color_buf_[i].resize( kCNumOfChannels );
            depth_buf_[i].resize( kDNumOfChannels );
        }
        color_buf_idle_.resize( kCNumOfChannels );
        depth_buf_idle_.resize( kDNumOfChannels );        
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

        if( saveKinectParams( out_dir_ + "/kinect" + device_->getSerialNumber() + "_params.txt" ) )
            throw std::runtime_error("failed to write kinect parameters.");
        
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
    
    set( WaitingForFpsStabilized );
}

void KinectManager::calibrate(){

    static frame_t undistorted( kDWidth, kDHeight, kDNumOfBytesPerPixel );
    static frame_t registered( kDWidth, kDHeight, kDNumOfBytesPerPixel );
    static cv::Mat registered_to_show;

    try{
    
        std::vector<cv::Point2f> corners;
        while( true ){

        
            frame_t color_frame( kCWidth, kCHeight, kCNumOfBytesPerPixel,
                                 reinterpret_cast<uint8_t*>(current_frame_color_->data()) );
            frame_t depth_frame( kDWidth, kDHeight, kDNumOfBytesPerPixel,
                                 reinterpret_cast<uint8_t*>(current_frame_depth_->data()) );

            registration_->apply( &color_frame, &depth_frame, &undistorted, &registered );

            cv::Mat color_8UC3;
            cv::cvtColor( cv::Mat( kDHeight, kDWidth, CV_8UC4, registered.data ),
                          color_8UC3, CV_BGRA2BGR );
        
            registered_to_show = color_8UC3.clone();
        
            if( cv::findChessboardCorners( color_8UC3, cv::Size( kCornersWidth, kCornersHeight ),
                                           corners,
                                           CV_CALIB_CB_ADAPTIVE_THRESH +
                                           CV_CALIB_CB_NORMALIZE_IMAGE ) ){
                drawChessboardCorners( registered_to_show,
                                       cv::Size( kCornersWidth, kCornersHeight ),
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

        cloud_t::Ptr cloud( new cloud_t( max_x - min_x + 1, max_y - min_y + 1 ) );
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
                                               corners[i_row * kCornersWidth + kCornersWidth - 1].y,
                                               corners[i_row * kCornersWidth + kCornersWidth - 1].x,
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


void KinectManager::enterMainLoop(){

    while( ! is( Exiting ) ){

        static FpsCalculator fps_calc( 30 );
        fps_main_loop_ = fps_calc.fps();
        
        showImgAndInfo();
        
        updateQueue();
        
        if( is( Exiting ) )
            continue;

        if( video_writer_for_main_thread_ ){
            video_writer_for_main_thread_->open( scene_dir_ + "/color.avi",
                                                 fourcc_color_, fps_color_video_,
                                                 cv::Size( kCWidth, kCHeight ));
            video_writer_for_main_thread_ = nullptr;
        }

        if( is( WritingData ) && queuesAreEmpty() ){
            set( ReadyToRecord );
            if( ! isStandalone() )
                socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] ready")), remote_endpoint_ );
        }

        
        int key = cv::waitKey(1);
        if( isStandalone() )
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
            if( isStandalone() && specifyEachFrame() ){
                if( is( ReadyToRecord ) )
                    set( Recording );
                saveCurrentFrame(); 
            }
        }

        key_ = 0;
    }

    if( ! isStandalone() ){
        io_service_.stop();
        sync_thread_.join();
    }
    save_thread_.join();
    update_thread_.join();


    return;
}

void KinectManager::update(){

    static int buf_index = 0;
    static color_ch_t* data_dst_color = nullptr;
    static depth_ch_t* data_dst_depth = nullptr;

    try{

        while( ! is( WaitingForFpsStabilized ) && ! is( ReadyToRecord ) )
            std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );

        uint64_t loop_count = 0;
        LARGE_INTEGER timestamp, freq;
        QueryPerformanceCounter( &timestamp );
        QueryPerformanceFrequency( &freq );
        while( ! is( Exiting ) ){

            static double tmp_fps = 0.0;

            listener_->waitForNewFrame( frames_ );

            LARGE_INTEGER now;
            QueryPerformanceCounter( &now );            
            if( (double)(now.QuadPart- timestamp.QuadPart)/freq.QuadPart <= 0.05 ){
                listener_->release( frames_ );
                continue;
            }
            timestamp = now;

            static FpsCalculator fps_calc( 15 );
            fps_update_loop_ = fps_calc.fps();
                
            if( is( Recording ) ){
                data_dst_color = &color_buf_[ buf_index ][0];
                data_dst_depth = &depth_buf_[ buf_index ][0];
            }else{
                data_dst_color = &color_buf_idle_[0];
                data_dst_depth = &depth_buf_idle_[0];
            }

            std::copy( frames_[frame_t::Color]->data,
                       frames_[frame_t::Color]->data + kCNumOfChannels,
                       data_dst_color );
            std::copy( (depth_ch_t*)frames_[frame_t::Depth]->data,
                       (depth_ch_t*)frames_[frame_t::Depth]->data + kDNumOfChannels,
                       data_dst_depth );

            cv::resize( cv::Mat( kCHeight, kCWidth,CV_8UC4,
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

            static FpsCalculator tmp_fps_calc( 15 );
            if( tmp_fps_calc.fps( tmp_fps ) ){
                if( is( WaitingForFpsStabilized ) ){
                    
                    static int high_fps_count = 0;
                    
                    if( tmp_fps >= 14.7 )
                        high_fps_count++;
                    else
                        high_fps_count = 0;
                    
                    if( high_fps_count == 3 ){
                        set( ReadyToRecord );
                        if( ! isStandalone() )
                            socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] ready")), remote_endpoint_ );
                    }
                }
            }
            
            if( ! specifyEachFrame() && is( Recording ) && notToBeThinnedOut( loop_count, fps_save_, 15 ) ){
                saveCurrentFrame();
                static FpsCalculator fps_calc_push( fps_save_ );
                fps_push_ = fps_calc_push.fps();
            }

            loop_count++;
        }
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
}

void KinectManager::updateQueue(){
    
    if( push_color_queue_ ){
        color_queue_.push( const_cast<std::vector<color_ch_t>*>( push_color_queue_ ) );
        push_color_queue_ = nullptr;
    }
    if( push_depth_queue_ ){
        depth_queue_.push( const_cast<std::vector<depth_ch_t>*>( push_depth_queue_ ) );
        push_depth_queue_ = nullptr;
    }
    if( pop_color_queue_ && pop_depth_queue_ && ! color_queue_.empty() && ! depth_queue_.empty() ){
        
        color_queue_.pop();
        depth_queue_.pop();
        pop_color_queue_ = false;
        pop_depth_queue_ = false;

        static FpsCalculator fps_calc( fps_save_ );
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

            createSceneDir();

            // open video file
            cv::VideoWriter video_writer_color;
            if( saveAsVideo() ){
                waitVideoWriterToBeOpened( video_writer_color );

            }

            int frame_count = 0;

            while( is( Recording ) || ! queuesAreEmpty() ){ // frame loop
                if( is( Exiting ) )
                    break;

                static std::stringstream sstream;
                sstream.str("");
                sstream << scene_dir_ + "/"
                        << std::setw( kNumSetw ) << std::setfill('0') << frame_count << ".pcd";

                if( saveDepth( sstream.str() ) ){

                    if( saveAsVideo() ){
                        if( saveColor( video_writer_color ) ){
                            frame_count++;
                        }else throw std::runtime_error("saveDepth returned true, but saveColor returned false.");
                    }else{
                        static std::stringstream sstream;
                        sstream.str("");
                        sstream << scene_dir_ << "/color"
                                << std::setw( kNumSetw ) << std::setfill('0') << frame_count
                                << ".bmp";

                        if( saveColor( sstream.str() ) )
                            frame_count++;
                        else throw std::runtime_error("saveDepth returned true, but saveColor returned false.");
                    }
                }
            } // end of frame loop

            fps_pop_ = 0.0;
        } // end of scene loop
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": "  << ex.what() << std::endl;
        throw;
    }
}


bool KinectManager::saveColor( cv::VideoWriter& video_writer ){

    if( pop_color_queue_ || color_queue_.empty() ){
        std::this_thread::sleep_for( std::chrono::milliseconds( 30 ));
        return false;
    }

    static cv::Mat tmp_color_img;
    cv::cvtColor( cv::Mat( kCHeight, kCWidth, CV_8UC4, &color_queue_.front()->front() ),
                  tmp_color_img, CV_BGRA2BGR );
    cv::flip( tmp_color_img, tmp_color_img, 1 ); // color images taken from kinect are mirrored
    video_writer << tmp_color_img;

    
    pop_color_queue_ = true;
    return true;
}

bool KinectManager::saveColor( const std::string& file_path ){

    if( pop_color_queue_ || color_queue_.empty() ){
        std::this_thread::sleep_for( std::chrono::milliseconds( 30 ));
        return false;
    }

    static cv::Mat tmp_color_img;
    cv::flip( cv::Mat( kCHeight, kCWidth, CV_8UC4, &color_queue_.front()->front() ),
                       tmp_color_img, 1 ); // color images taken from kinect are mirrored


    cv::imwrite( file_path, tmp_color_img );
     
    pop_color_queue_ = true;
    return true;
}

bool KinectManager::saveDepth( const std::string& file_path ){

    static frame_t undistorted( kDWidth, kDHeight, kDNumOfBytesPerPixel );
    static frame_t registered( kDWidth, kDHeight, kDNumOfBytesPerPixel );

    if( pop_depth_queue_ || depth_queue_.empty() ){
        std::this_thread::sleep_for( std::chrono::milliseconds( 30 ));
        return false;
    }

    frame_t color_frame( kCWidth, kCHeight, kCNumOfBytesPerPixel,
                         reinterpret_cast<uint8_t*>(&color_queue_.front()->at(0)) );
    frame_t depth_frame( kDWidth, kDHeight, kDNumOfBytesPerPixel,
                         reinterpret_cast<uint8_t*>(&depth_queue_.front()->at(0)) );

    registration_->apply( &color_frame, &depth_frame, &undistorted, &registered );

    static cloudRGB_t::Ptr cloud( new cloudRGB_t(kDWidth, kDHeight) );

    float rgb_buf;
    for( int i_row = 0; i_row < kDHeight; i_row++ ){
        for( int i_column = 0; i_column < kDWidth; i_column++ ){
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

            (*cloud)( i_column, i_row ) = point;
        }
    }
    pcl::io::savePCDFileBinary<pcl::PointXYZRGB>( file_path, *cloud );
    pop_depth_queue_ = true;

    
    return true;
}

int KinectManager::saveKinectParams( const std::string& file_path,
                                     const device_t::IrCameraParams& ip,
                                     const device_t::ColorCameraParams& cp )const{

    cv::FileStorage fs( file_path, cv::FileStorage::WRITE );
    if( ! fs.isOpened() )
        return 1;

    setlocale( LC_ALL, "JPN" );
    time_t tmp_time = time( nullptr );
    fs << "date" << asctime( localtime( &tmp_time ) );

    fs << "IrCameraParams" << "{";
    {
        fs << "fx" << "[" << ip.fx << toBinaryString( ip.fx ) << "]";
        fs << "fy" << "[" << ip.fy << toBinaryString( ip.fy ) << "]";
        fs << "cx" << "[" << ip.cx << toBinaryString( ip.cx ) << "]";
        fs << "cy" << "[" << ip.cy << toBinaryString( ip.cy ) << "]";
        fs << "k1" << "[" << ip.k1 << toBinaryString( ip.k1 ) << "]";
        fs << "k2" << "[" << ip.k2 << toBinaryString( ip.k2 ) << "]";
        fs << "k3" << "[" << ip.k3 << toBinaryString( ip.k3 ) << "]";
        fs << "p1" << "[" << ip.p1 << toBinaryString( ip.p1 ) << "]";    
        fs << "p2" << "[" << ip.p2 << toBinaryString( ip.p2 ) << "]";
    }
    fs << "}";

    fs << "ColorCameraParams" << "{";
    {
        fs << "fx" << "[" << cp.fx << toBinaryString( cp.fx )<< "]";
        fs << "fy" << "[" << cp.fy << toBinaryString( cp.fy )<< "]";
        fs << "cx" << "[" << cp.cx << toBinaryString( cp.cx )<< "]";
        fs << "cy" << "[" << cp.cy << toBinaryString( cp.cy )<< "]";
        fs << "shift_d" << "[" << cp.shift_d << toBinaryString( cp.shift_d )<< "]";
        fs << "shift_m" << "[" << cp.shift_m << toBinaryString( cp.shift_m )<< "]";
        fs << "mx_x3y0" << "[" << cp.mx_x3y0 << toBinaryString( cp.mx_x3y0 )<< "]";
        fs << "mx_x0y3" << "[" << cp.mx_x0y3 << toBinaryString( cp.mx_x0y3 )<< "]";
        fs << "mx_x2y1" << "[" << cp.mx_x2y1 << toBinaryString( cp.mx_x2y1 )<< "]";
        fs << "mx_x1y2" << "[" << cp.mx_x1y2 << toBinaryString( cp.mx_x1y2 )<< "]";
        fs << "mx_x2y0" << "[" << cp.mx_x2y0 << toBinaryString( cp.mx_x2y0 )<< "]";
        fs << "mx_x0y2" << "[" << cp.mx_x0y2 << toBinaryString( cp.mx_x0y2 )<< "]";
        fs << "mx_x1y1" << "[" << cp.mx_x1y1 << toBinaryString( cp.mx_x1y1 )<< "]";
        fs << "mx_x1y0" << "[" << cp.mx_x1y0 << toBinaryString( cp.mx_x1y0 )<< "]";
        fs << "mx_x0y1" << "[" << cp.mx_x0y1 << toBinaryString( cp.mx_x0y1 )<< "]";
        fs << "mx_x0y0" << "[" << cp.mx_x0y0 << toBinaryString( cp.mx_x0y0 )<< "]";
        fs << "my_x3y0" << "[" << cp.my_x3y0 << toBinaryString( cp.my_x3y0 )<< "]";
        fs << "my_x0y3" << "[" << cp.my_x0y3 << toBinaryString( cp.my_x0y3 )<< "]";
        fs << "my_x2y1" << "[" << cp.my_x2y1 << toBinaryString( cp.my_x2y1 )<< "]";
        fs << "my_x1y2" << "[" << cp.my_x1y2 << toBinaryString( cp.my_x1y2 )<< "]";
        fs << "my_x2y0" << "[" << cp.my_x2y0 << toBinaryString( cp.my_x2y0 )<< "]";
        fs << "my_x0y2" << "[" << cp.my_x0y2 << toBinaryString( cp.my_x0y2 )<< "]";
        fs << "my_x1y1" << "[" << cp.my_x1y1 << toBinaryString( cp.my_x1y1 )<< "]";
        fs << "my_x1y0" << "[" << cp.my_x1y0 << toBinaryString( cp.my_x1y0 )<< "]";
        fs << "my_x0y1" << "[" << cp.my_x0y1 << toBinaryString( cp.my_x0y1 )<< "]";
        fs << "my_x0y0" << "[" << cp.my_x0y0 << toBinaryString( cp.my_x0y0 )<< "]";
    }
    fs << "}";

    fs.release();
    return 0;
}

int KinectManager::loadKinectParams( const std::string& file_path,
                                     device_t::IrCameraParams& ip,
                                     device_t::ColorCameraParams& cp )const{

    cv::FileStorage fs( file_path, cv::FileStorage::READ );
    if( ! fs.isOpened() )
        return 1;
    
    ip.fx = toFloat( (std::string)fs["IrCameraParams"]["fx"][1] );
    ip.fy = toFloat( (std::string)fs["IrCameraParams"]["fy"][1] );
    ip.cx = toFloat( (std::string)fs["IrCameraParams"]["cx"][1] );
    ip.cy = toFloat( (std::string)fs["IrCameraParams"]["cy"][1] );
    ip.k1 = toFloat( (std::string)fs["IrCameraParams"]["k1"][1] );
    ip.k2 = toFloat( (std::string)fs["IrCameraParams"]["k2"][1] );
    ip.k3 = toFloat( (std::string)fs["IrCameraParams"]["k3"][1] );
    ip.p1 = toFloat( (std::string)fs["IrCameraParams"]["p1"][1] );
    ip.p2 = toFloat( (std::string)fs["IrCameraParams"]["p2"][1] );

    cp.fx = toFloat((std::string)fs["ColorCameraParams"]["fx"][1] );
    cp.fy = toFloat((std::string)fs["ColorCameraParams"]["fy"][1] );
    cp.cx = toFloat((std::string)fs["ColorCameraParams"]["cx"][1] );
    cp.cy = toFloat((std::string)fs["ColorCameraParams"]["cy"][1] );
    cp.shift_d = toFloat((std::string)fs["ColorCameraParams"]["shift_d"][1] );
    cp.shift_m = toFloat((std::string)fs["ColorCameraParams"]["shift_m"][1] );
    cp.mx_x3y0 = toFloat((std::string)fs["ColorCameraParams"]["mx_x3y0"][1] );
    cp.mx_x0y3 = toFloat((std::string)fs["ColorCameraParams"]["mx_x0y3"][1] );
    cp.mx_x2y1 = toFloat((std::string)fs["ColorCameraParams"]["mx_x2y1"][1] );
    cp.mx_x1y2 = toFloat((std::string)fs["ColorCameraParams"]["mx_x1y2"][1] );
    cp.mx_x2y0 = toFloat((std::string)fs["ColorCameraParams"]["mx_x2y0"][1] );
    cp.mx_x0y2 = toFloat((std::string)fs["ColorCameraParams"]["mx_x0y2"][1] );
    cp.mx_x1y1 = toFloat((std::string)fs["ColorCameraParams"]["mx_x1y1"][1] );
    cp.mx_x1y0 = toFloat((std::string)fs["ColorCameraParams"]["mx_x1y0"][1] );
    cp.mx_x0y1 = toFloat((std::string)fs["ColorCameraParams"]["mx_x0y1"][1] );
    cp.mx_x0y0 = toFloat((std::string)fs["ColorCameraParams"]["mx_x0y0"][1] );
    cp.my_x3y0 = toFloat((std::string)fs["ColorCameraParams"]["my_x3y0"][1] );
    cp.my_x0y3 = toFloat((std::string)fs["ColorCameraParams"]["my_x0y3"][1] );
    cp.my_x2y1 = toFloat((std::string)fs["ColorCameraParams"]["my_x2y1"][1] );
    cp.my_x1y2 = toFloat((std::string)fs["ColorCameraParams"]["my_x1y2"][1] );
    cp.my_x2y0 = toFloat((std::string)fs["ColorCameraParams"]["my_x2y0"][1] );
    cp.my_x0y2 = toFloat((std::string)fs["ColorCameraParams"]["my_x0y2"][1] );
    cp.my_x1y1 = toFloat((std::string)fs["ColorCameraParams"]["my_x1y1"][1] );
    cp.my_x1y0 = toFloat((std::string)fs["ColorCameraParams"]["my_x1y0"][1] );
    cp.my_x0y1 = toFloat((std::string)fs["ColorCameraParams"]["my_x0y1"][1] );
    cp.my_x0y0 = toFloat((std::string)fs["ColorCameraParams"]["my_x0y0"][1] );
    
    return 0;
}

void KinectManager::createSceneDir(){
    
    for( int i = 1; ; i++ ){ // begin from 1 for the compatibility with Cortex.
        std::stringstream sstream;
        sstream << out_dir_ + "/" + motion_name_ << std::setw( kNumSetw ) << std::setfill('0') << i;
        if( ! boost::filesystem::exists( boost::filesystem::path( sstream.str() ) ) ){
            scene_dir_ = sstream.str();
            break;
        }
    }
    boost::filesystem::create_directory( boost::filesystem::path( scene_dir_ ) );
    return;
}

void KinectManager::waitVideoWriterToBeOpened( cv::VideoWriter& video_writer ){
    video_writer_for_main_thread_ = &video_writer;
    // wait for video_writer_color to be opened
    int count = 0;
    while( video_writer_for_main_thread_ ){ 
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ));
        if( count++ > 5000 ){
            std::cerr << "error: failed to open " << scene_dir_ + "/color.avi"
                      << std::endl;
            return;
        }
    }
}

void KinectManager::sync(){

    try{
        socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] I am synchronizer") ),
                               remote_endpoint_ );

        static std::vector<char> buf( 256, '\0' );
        
        while( ! is( Exiting ) ){

            static FpsCalculator fps_calc( 30 );
            fps_sync_loop_ = fps_calc.fps();
            
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
        if( isStandalone() && ! specifyEachFrame() )
            putText( img_to_show_, "Recording ( s: stop recording, q: abort )", false, kRed, cv::Point(10,40) );
        else
            putText( img_to_show_, "Recording", false, kRed, cv::Point(10,40) );
    else if( is( ReadyToCalibrate ) )
        if( isStandalone() )
            putText( img_to_show_,
                     "Ready to start calibration (s: start calibration, c: recorder mode, q: abort)", false, kYellow, cv::Point(10,40) );
        else
            putText( img_to_show_, "Ready to start calibration", false, kYellow, cv::Point(10,40));
    else if( is( WaitingForFpsStabilized ) )
        putText( img_to_show_, "Not Ready", false, kOrange, cv::Point(10,40) );
    else if( is( Calibrating ) )
        if( isStandalone() )
            putText( img_to_show_, "Calibrating ( q: abort )", false, kOrange, cv::Point(10,40) );
        else
            putText( img_to_show_, "Calibrating", false, kOrange, cv::Point(10,40) );
    else if( is( WritingData ) )
        if( isStandalone() && ! specifyEachFrame() )
            putText( img_to_show_, "Writing files... ( q: abort )", false, kOrange, cv::Point(10,40) );
        else
            putText( img_to_show_, "Writing files..." , false, kOrange, cv::Point(10,40) );
    else if( is( ReadyToRecord ) )
        if( isStandalone() && ! specifyEachFrame() )
            putText( img_to_show_,
                     "Ready to start recording ( s: start recording, c: calib mode, q: quit )",
                     false, kGreen, cv::Point(10,40) );
        else
            putText( img_to_show_, "Ready to start recording", false, kGreen, cv::Point(10,40) );
    else
        putText( img_to_show_, "Error: unknown state!", false, kRed, cv::Point(10,40) );
        
    putText( img_to_show_, "update fps:" );
    putText( img_to_show_, toString( fps_update_loop_ ), false, cv::Point(220, -1) );
    putText( img_to_show_, "push fps:" );
    putText( img_to_show_, toString( fps_push_ ), false, cv::Point(220, -1) );
    putText( img_to_show_, "pop fps:" );
    putText( img_to_show_, toString( fps_pop_ ), false, cv::Point(220, -1) );
    putText( img_to_show_, "main loop fps:" );
    putText( img_to_show_, toString( fps_main_loop_ ), false, cv::Point(220, -1) );
    putText( img_to_show_, "sync fps:" );
    putText( img_to_show_, toString( fps_sync_loop_ ), false, cv::Point(220, -1) );
    
    int left_color_queue_size = kBufSize - color_queue_.size();
    int left_depth_queue_size = kBufSize - depth_queue_.size();
        
    if( left_color_queue_size == 0 || left_depth_queue_size == 0 )
        std::cerr << "error: run out of queues" << std::endl;
    if( left_color_queue_size != left_depth_queue_size )
        std::cerr << "error: invalid queue size" << std::endl;

    putText( img_to_show_, "left color queue size:" );
    putText( img_to_show_, toString( left_color_queue_size ), false, cv::Point(220, -1) );
    putText( img_to_show_, "left depth queue size:" );
    putText( img_to_show_, toString( left_depth_queue_size ), false, cv::Point(220, -1) );

    cv::imshow( "color", img_to_show_ );
}

void KinectManager::putText( cv::Mat& img, const std::string& text, const bool newline,
                             const cv::Scalar& color, const cv::Point& org )const{
    static int x = 10;
    static int y = 40;
    x = org.x >= 0 ? org.x : 10;
    y = org.y >= 0 ? org.y : ( newline ? y + 30 : y ) ; 
    cv::putText( img, text, cv::Point( x, y ), cv::FONT_HERSHEY_SIMPLEX,
                 kFontScale, color, kTextThickness );
}

const double KinectManager::kResizeScale = 0.5;
const double KinectManager::kFontScale = 0.6;
const cv::Scalar KinectManager::kRed = cv::Scalar(0,0,255);
const cv::Scalar KinectManager::kGreen = cv::Scalar(0,255,0);
const cv::Scalar KinectManager::kBlue = cv::Scalar(255,0,0);
const cv::Scalar KinectManager::kOrange = cv::Scalar(0,100,255);
const cv::Scalar KinectManager::kYellow = cv::Scalar(0,255,255);
const cv::Scalar KinectManager::kSkyBlue = cv::Scalar(255,255,0);

void MyFileLogger::log(Level level, const std::string &message){

    if( level == Debug ){}
    else if( level == Info ){
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
