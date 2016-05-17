/*
  Kinect v2 recorder.
  Katsumasa Kitajima
  kitajima@ynl.t.u-tokyo.ac.jp
 */


#include "kinect_recorder.hpp"


KinectRecorder::KinectRecorder( const std::string& config_file_path )
    : H_( cv::Mat::eye( 4, 4, CV_32F ) ),
      current_frame_color_( nullptr ),
      current_frame_depth_( nullptr ),
      current_frame_reg_( nullptr ),
      serial_num_( "" )
{

    push_color_queue_.store( nullptr );
    push_depth_queue_.store( nullptr );
    push_reg_queue_.store( nullptr );
    video_writer_for_main_thread_.store( nullptr );
    
    pop_color_queue_.store( false );
    pop_depth_queue_.store( false );
    pop_reg_queue_.store( false );
        
    fps_update_loop_.store( 0.0 );
    fps_push_.store( 0.0 );
    fps_pop_.store( 0.0 );
    fps_main_loop_.store( 0.0 );
    fps_sync_loop_.store( 0.0 );
    
    key_.store( 0 );
    recorder_state_.store( Error );
    recorder_mode_.store( 0 );
    
    if( configure( config_file_path ) ) // if succeeded
        recorder_state_ = InitialState;

    if( ! isStandalone() ){
        local_endpoint_calib_ = udp_t::endpoint( udp_t::v4(), local_endpoint_port_calib_ );
        local_endpoint_sync_ = udp_t::endpoint( udp_t::v4(), local_endpoint_port_sync_ );
    }
}

bool KinectRecorder::configure( const std::string& config_file_path ){

    cv::FileStorage fs( config_file_path, cv::FileStorage::READ );
    if( ! fs.isOpened() )
        return false;
    
    motion_name_ = fs["motion name"].empty() ? "scene" : (std::string)fs["motion name"];
    out_dir_ = fs["output directory"].empty() ? "." : (std::string)fs["output directory"];
    kinect_name_ = fs["kinect name"].empty() ? "kinect" : (std::string)fs["kinect name"];
    serial_num_ = fs["serial number"].empty() ? "" : (std::string)fs["serial number"];
    
    if( (std::string)fs["use as a client?"] == "true" ){
        recorder_mode_ |= 1;  // client        
        local_endpoint_port_sync_ = fs["local port1"].empty() ? 49998 : (int)fs["local port1"];
        local_endpoint_port_calib_ = fs["local port2"].empty() ? 49999 : (int)fs["local port2"];
        cv::FileNode fn = fs["server config"];
        server_ip_ = (std::string)fn["server ip"];
        server_port_ = (std::string)fn["server port"];
    }else
        recorder_mode_ &= ~1; // standalone

    if( isStandalone() ){
        freq_ = fs["frequency to get data"].empty() ? 30 : (int)fs["frequency to get data"];
        if( freq_ > 30 || freq_ <= 0 )
            freq_ = 30;
    }else freq_ = 30;
    
    if( (std::string)fs["specify each frame to save"] == "true" )
        recorder_mode_ |= 2;  // specify each frame        
    else
        recorder_mode_ &= ~2; // specify start/stop        
    
    if( (std::string)fs["save color images as a video?"] == "true" ){
        recorder_mode_ |= 4;  // video
        cv::FileNode fn = fs["color video config"];
        fps_color_video_ = (double)fn["color video fps"];
        std::string fourcc = (std::string)fn["color video fourcc"];
        if( fourcc.size() == 4 )
            fourcc_color_ = CV_FOURCC( fourcc[0], fourcc[1], fourcc[2], fourcc[3] );
        else{
            std::cerr << "warning: Invalid fourcc specified. Default value will be set." << std::endl;
            fourcc_color_ = CV_FOURCC( 'X', 'V', 'I', 'D' );
        }
    }else{
        cv::FileNode fn = fs["color image config"];
        color_img_extension_ = fn["extension"].empty() ? ".jpg" : (std::string)fn["extension"];
        if( color_img_extension_[0] != '.' )
            color_img_extension_.insert( 0, 1, '.' );
        recorder_mode_ &= ~4; // set of pictures
    }

    return true;
}


KinectRecorder::~KinectRecorder(){

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

void KinectRecorder::init(){

    try{

        if( serial_num_ != "" )
            kinect_.reset( new Kinect2( kinect_name_, serial_num_.c_str() ) );
        else 
            kinect_.reset( new Kinect2( kinect_name_ ) );
        if( ! kinect_ )
            throw std::runtime_error( std::string("failed to open") + kinect_name_ );
        
        if( ! isStandalone() ){
            if( server_ip_ == "" || server_ip_ == "" )
                throw std::runtime_error("no valid ip/port");

            udp_t::resolver resolver(io_service_);
            udp_t::resolver::query query( udp_t::v4(), server_ip_, server_port_ );
            remote_endpoint_ = *resolver.resolve(query);
            socket_calib_.reset( new udp_t::socket( io_service_, local_endpoint_calib_ ) );
            socket_sync_.reset(  new udp_t::socket( io_service_, local_endpoint_sync_ ) );

            socket_calib_->send_to( boost::asio::buffer( std::string("[Kinect] I am calibrator") ),
                                    remote_endpoint_ );
        }

        cv::namedWindow( "color", CV_WINDOW_AUTOSIZE );
        // cv::namedWindow( "color", CV_WINDOW_OPENGL );
        
        save_thread_ = std::thread( &KinectRecorder::save, this );
        update_thread_ = std::thread( &KinectRecorder::update, this );
        update_queue_thread_ = std::thread( &KinectRecorder::updateQueue, this );

        if( ! isStandalone() )
            sync_thread_ = std::thread( &KinectRecorder::sync, this );
        
        // allocate memory in advance to avoid overhead
        for( int i = 0; i < kBufSize; i++ ){
            color_buf_[i].resize( kCNumOfChannels );
            depth_buf_[i].resize( kDNumOfChannels );
            reg_buf_[i].resize( kRNumOfChannels );
        }


        if( kinect_->saveKinectParams( out_dir_ + "/" + kinect_name_ + ".dat" ) )
            throw std::runtime_error("failed to write kinect parameters.");
        
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }

    set( WaitingForFpsStabilized );

    return;
}

void KinectRecorder::calibrate(){

    static std::vector<depth_ch_t> depth_frame( kDNumOfChannels );
    static std::vector<color_ch_t> reg_frame( kCNumOfChannels );
    static cv::Mat registered_to_show;

    try{
        std::vector<cv::Point2f> corners;
        while( true ){

            std::copy( current_frame_depth_->begin(),
                       current_frame_depth_->end(),
                       depth_frame.begin() );
            std::copy( current_frame_reg_->begin(),
                       current_frame_reg_->end(),
                       reg_frame.begin() );
            
            cv::Mat color_8UC3;
            cv::cvtColor( cv::Mat( kDHeight, kDWidth, CV_8UC4,
                                   reinterpret_cast<uint8_t*>( &reg_frame[0] ) ),
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

        if( ! isStandalone() && server_ip_ != "" && server_port_ != "" && socket_calib_ ){ // fill in H_board_to_target

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
                depth_ch_t depth = depth_frame[ i_row * kDWidth + i_col ];
                kinect_->mapDepthPointToCameraSpace( i_row,
                                                     i_col,
                                                     depth,
                                                     point.x,
                                                     point.y,
                                                     point.z );
                if( ISNAN( point.x ) ) std::cerr << "nan in " << __LINE__ << std::endl; 
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

        if( inliers->indices.size() == 0 ) {
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
                pcl::PointXYZ p1;
                {// populate p1
                    int x1 = corners[ i_row * kCornersWidth ].x,
                        y1 = corners[ i_row * kCornersWidth ].y;
                    depth_ch_t d1 =
                        depth_frame[ y1 * kDWidth + x1 ];
                    kinect_->mapDepthPointToCameraSpace( y1, x1, d1, p1.x, p1.y, p1.z );
                    if( ISNAN( p1.x ) ) std::cerr << "nan in " << __LINE__ << std::endl;

                }
                pcl::PointXYZ p2;
                {// populate p2
                    int x2 = corners[ i_row * kCornersWidth + kCornersWidth - 1 ].x,
                        y2 = corners[ i_row * kCornersWidth + kCornersWidth - 1 ].y;
                    depth_ch_t d2 =
                        depth_frame[ y2 * kDWidth + x2 ];
                    kinect_->mapDepthPointToCameraSpace( y2, x2, d2, p2.x, p2.y, p2.z );
                    if( ISNAN( p2.x ) ) std::cerr << "nan in " << __LINE__ << std::endl;
                }
                pcl::PointXYZ projected1 = project( a, b, c, d, p1 ),
                    projected2 = project( a, b, c, d, p2 );
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
                int x = corners[i].x;
                int y = corners[i].y;
                depth_ch_t depth
                    = depth_frame[ y * kDWidth + x ];
                std::cerr << x << " "<< y <<" " <<depth <<std::endl;
                kinect_->mapDepthPointToCameraSpace( y, x, depth,
                                                     p.x, p.y, p.z );
                if( ISNAN( p.x ) ) std::cerr << "nan in " << __LINE__ << std::endl;                
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


void KinectRecorder::enterMainLoop(){

    while( ! is( Exiting ) ){

        static FpsCalculator fps_calc( 30 );
        fps_main_loop_ = fps_calc.fps();
        
        // updateQueue();

        
        showImgAndInfo();
        
        if( is( Exiting ) )
            continue;

        if( video_writer_for_main_thread_.load() ){
            video_writer_for_main_thread_.load()->open( scene_dir_ + "/color.avi",
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
    update_queue_thread_.join();

    return;
}

void KinectRecorder::update(){

    static int buf_index = 0;
    static color_ch_t* data_dst_color = nullptr;
    static depth_ch_t* data_dst_depth = nullptr;
    static color_ch_t* data_dst_reg = nullptr;

    try{

        while( ! is( WaitingForFpsStabilized ) && ! is( ReadyToRecord ) )
            std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );

        {
            LARGE_INTEGER timestamp, freq;
            QueryPerformanceCounter( &timestamp );
            QueryPerformanceFrequency( &freq );
        }
        for( int i_frame = 0; ! is( Exiting ); ){


            data_dst_color = &color_buf_[ buf_index ][0];
            data_dst_depth = &depth_buf_[ buf_index ][0];
            data_dst_reg = &reg_buf_[ buf_index][0];

            double offset;
            {
                LARGE_INTEGER timestamp;
                if( freq_ < 30 )
                    QueryPerformanceCounter( &timestamp );

                kinect_->waitForNewFrame( data_dst_color, data_dst_depth, data_dst_reg );

                static LARGE_INTEGER initial_timestamp = timestamp, freq;
                if( freq_ < 30 )
                    QueryPerformanceFrequency( &freq );

                if( freq_ < 30 )
                    offset = (double)1000*(timestamp.QuadPart - initial_timestamp.QuadPart)/freq.QuadPart - (double)i_frame * 1000 / freq_ ;
            }
            
            if( freq_ < 30 && offset < -16.66 )
                continue;
            
            static FpsCalculator fps_calc( 30 );
            fps_update_loop_ = fps_calc.fps();
                
            cv::resize( cv::Mat( kCHeight, kCWidth,CV_8UC4,
                                 data_dst_color ),
                        img_to_show_, cv::Size(), kResizeScale, kResizeScale );

            current_frame_color_ = color_buf_ + buf_index;
            current_frame_depth_ = depth_buf_ + buf_index;
            current_frame_reg_ = reg_buf_ + buf_index;
            if( ++buf_index == kBufSize )
                buf_index = 0;
            
            if( is( WaitingForFpsStabilized ) ){
                double tmp_fps = 0.0;
                static FpsCalculator tmp_fps_calc( 30 );
                if( tmp_fps_calc.fps( tmp_fps ) && fpsKeepsHigh( tmp_fps ) ){
                    set( ReadyToRecord );
                    if( ! isStandalone() )
                        socket_sync_->send_to( boost::asio::buffer( std::string("[Kinect] ready")),
                                               remote_endpoint_ );
                }
            }
            
            if( ! specifyEachFrame() && is( Recording )){
                saveCurrentFrame();
                static FpsCalculator fps_calc_push( 30 );
                fps_push_ = fps_calc_push.fps();
            }

            i_frame++;

        }
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": " << ex.what() << std::endl;
        throw;
    }
}


void KinectRecorder::updateQueue(){

    while( ! is( Exiting ) ){
        
    if( push_color_queue_.load() ){
        color_queue_.push( push_color_queue_ );
        push_color_queue_ = nullptr;
    }
    if( push_depth_queue_.load() ){
        depth_queue_.push( push_depth_queue_ );
        push_depth_queue_ = nullptr;
    }
    if( push_reg_queue_.load() ){
        reg_queue_.push( push_reg_queue_ );
        push_reg_queue_ = nullptr;
    }
    if( pop_color_queue_ && pop_depth_queue_ && pop_reg_queue_ &&
        ! color_queue_.empty() && ! depth_queue_.empty() && ! reg_queue_.empty() ){
        
        color_queue_.pop();
        depth_queue_.pop();
        reg_queue_.pop();
        pop_color_queue_ = false;
        pop_depth_queue_ = false;
        pop_reg_queue_ = false;
        
        static FpsCalculator fps_calc( 30 );
        fps_pop_ = fps_calc.fps();
    }

    std::this_thread::sleep_for( std::chrono::milliseconds( 10 ) );
    
    }
}

void KinectRecorder::save(){

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

            int frame_count = 1;

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
                                << color_img_extension_;

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


bool KinectRecorder::saveColor( cv::VideoWriter& video_writer ){

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

bool KinectRecorder::saveColor( const std::string& file_path ){

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

bool KinectRecorder::saveDepth( const std::string& file_path ){

    if( pop_depth_queue_ || pop_reg_queue_ || depth_queue_.empty() || reg_queue_.empty() ){
        std::this_thread::sleep_for( std::chrono::milliseconds( 30 ));
        return false;
    }

    static cloudRGB_t::Ptr cloud( new cloudRGB_t(kDWidth, kDHeight) );

    if( H_.isContinuous() )
        kinect_->getPointCloudRGB( &depth_queue_.front()->at(0),
                                   &reg_queue_.front()->at(0),
                                   *cloud,
                                   H_[0] );
    else throw std::runtime_error( "homogenous transformation matrix data is not continuous" );
    
    pcl::io::savePCDFileBinary<pcl::PointXYZRGB>( file_path, *cloud );
    pop_depth_queue_ = true;
    pop_reg_queue_ = true;
    
    return true;
}


void KinectRecorder::createSceneDir(){
    
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

void KinectRecorder::waitVideoWriterToBeOpened( cv::VideoWriter& video_writer ){
    video_writer_for_main_thread_ = &video_writer;
    // wait for video_writer_color to be opened
    int count = 0;
    while( video_writer_for_main_thread_.load() ){ 
        std::this_thread::sleep_for( std::chrono::milliseconds( 1 ));
        if( count++ > 5000 ){
            std::cerr << "error: failed to open " << scene_dir_ + "/color.avi"
                      << std::endl;
            return;
        }
    }
}

void KinectRecorder::sync(){

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
                                         std::bind(&KinectRecorder::processRecvBuf,
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

void KinectRecorder::processRecvBuf( const std::vector<char>* buf,
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
        if( frame_id == 1 && is( ReadyToRecord ) ){ // start recording 
            if( ! queuesAreEmpty() )
                throw std::runtime_error( "not ready to start recording." );
            set( Recording );
            saveCurrentFrame();
            frame_id_prev = frame_id;
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

bool KinectRecorder::fpsKeepsHigh( const double fps )const{
                        
    static int high_fps_count = 0;
                    
    if( (freq_ < 0 && fps >= 29.7) || (freq_ >= 0 && fps >= freq_ - 0.3) )
        high_fps_count++;
    else
        high_fps_count = 0;
                    
    if( high_fps_count == 3 ){
        high_fps_count = 0;
        return true;
    }
    return false;
}


void KinectRecorder::showImgAndInfo(){

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

void KinectRecorder::putText( cv::Mat& img, const std::string& text, const bool newline,
                             const cv::Scalar& color, const cv::Point& org )const{
    static int x = 10;
    static int y = 40;
    x = org.x >= 0 ? org.x : 10;
    y = org.y >= 0 ? org.y : ( newline ? y + 30 : y ) ; 
    cv::putText( img, text, cv::Point( x, y ), cv::FONT_HERSHEY_SIMPLEX,
                 kFontScale, color, kTextThickness );
}

std::string KinectRecorder::toBinaryString( const float value )const{
    union{
        float v;
        uint32_t n;
    }u;
    u.v = value;
    static std::stringstream sstream;
    sstream.str("");
    for( int i = 8 * sizeof( value ) - 1; i >= 0; i-- )
        sstream << (( u.n >> i ) & 1);
    return sstream.str();
}

float KinectRecorder::toFloat( const std::string& str )const{
    union{
        float v;
        uint32_t n;
    }u;
    u.v = 0.0;
    for( int i = 0; i < 8 * sizeof( float ); i++ )
        if( str[i] == '1')
            u.n |= (1 << (8 * sizeof( float ) - i - 1));
    return u.v;
}

std::string KinectRecorder::toString( const double value )const{
    static std::stringstream sstream;
    sstream.str("");
    sstream << std::fixed << std::setprecision(3) << value;
    return sstream.str();
}

std::string KinectRecorder::toString( const int value )const{
    static std::stringstream sstream;
    sstream.str("");
    sstream << value;
    return sstream.str();
}


const double KinectRecorder::kResizeScale = 0.5;
const double KinectRecorder::kFontScale = 0.6;
const cv::Scalar KinectRecorder::kRed = cv::Scalar(0,0,255);
const cv::Scalar KinectRecorder::kGreen = cv::Scalar(0,255,0);
const cv::Scalar KinectRecorder::kBlue = cv::Scalar(255,0,0);
const cv::Scalar KinectRecorder::kOrange = cv::Scalar(0,100,255);
const cv::Scalar KinectRecorder::kYellow = cv::Scalar(0,255,255);
const cv::Scalar KinectRecorder::kSkyBlue = cv::Scalar(255,255,0);

