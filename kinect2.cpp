#include "kinect2.hpp"



Kinect2::Kinect2( const std::string& name )
    : state_( Good ),
      device_( nullptr ){

    open( name );
}

Kinect2::Kinect2()
    : state_( Good ),
      device_( nullptr ){
}

Kinect2::~Kinect2(){
    close();
}

uint32_t Kinect2::open( const std::string& name ){

    try{
        name_ = name;
        
        log_file_path_ = std::string("./") + name_ + ".log";
        logger_.reset( new KinectLogger( log_file_path_.c_str() ) );

        libfreenect2::setGlobalLogger( libfreenect2::createConsoleLogger( libfreenect2::Logger::Warning ) );
        if( logger_->good() )
            libfreenect2::setGlobalLogger( logger_.get() );
        else
            set( UnspecifiedError );

        if( succeeded() && freenect2_.enumerateDevices() == 0 )
            set( NoKinectFound );

        if( succeeded() )
            device_ = freenect2_.openDevice( freenect2_.getDefaultDeviceSerialNumber() );
        if( device_ == 0 )
            set( FailedToOpenKinect );

        if( succeeded() )
            listener_.reset( new libfreenect2::SyncMultiFrameListener( frame_t::Color |
                                                                       frame_t::Depth |
                                                                       frame_t::Ir ) );

        if( succeeded() ){
            device_->setColorFrameListener( listener_.get() );
            device_->setIrAndDepthFrameListener( listener_.get() );
            if( ! device_->start() )
                set( FailedToStartKinect );
        }

        if( succeeded() ){
            ir_param_ = device_->getIrCameraParams();
            color_param_ = device_->getColorCameraParams();
            registration_.reset( new libfreenect2::Registration( ir_param_, color_param_ ) );
        }
        
    }catch(...){
        set( UnspecifiedError );
    }

    std::this_thread::sleep_for( std::chrono::seconds( 3 ) );
    
    return state_;
}

int Kinect2::saveKinectParams( const std::string& file_path )const{

    if( device_ == nullptr )
        return 1;
    
    std::ofstream fout( file_path, std::ios::binary );
    if( ! fout )
        return 1;
    
    ir_param_t ip = device_->getIrCameraParams();
    float ip_params[9] = { ip.fx,
                           ip.fy,
                           ip.cx,
                           ip.cy,
                           ip.k1,
                           ip.k2,
                           ip.k3,
                           ip.p1,
                           ip.p2 };
    fout.write( (char*)ip_params, 9 * sizeof( float ) );

    color_param_t cp = device_->getColorCameraParams();
    float cp_params[26] = { cp.fx,
                            cp.fy,
                            cp.cx,
                            cp.cy,
                            cp.shift_d,
                            cp.shift_m,
                            cp.mx_x3y0, // xxx
                            cp.mx_x0y3, // yyy
                            cp.mx_x2y1, // xxy
                            cp.mx_x1y2, // yyx
                            cp.mx_x2y0, // xx
                            cp.mx_x0y2, // yy
                            cp.mx_x1y1, // xy
                            cp.mx_x1y0, // x
                            cp.mx_x0y1, // y
                            cp.mx_x0y0, // 1
                            cp.my_x3y0, // xxx
                            cp.my_x0y3, // yyy
                            cp.my_x2y1, // xxy
                            cp.my_x1y2, // yyx
                            cp.my_x2y0, // xx
                            cp.my_x0y2, // yy
                            cp.my_x1y1, // xy
                            cp.my_x1y0, // x
                            cp.my_x0y1, // y
                            cp.my_x0y0 };// 1

    fout.write( (char*)cp_params, 26 * sizeof(float) );
    fout.close();

    if( fout.good() )
        return 0;
    else return 2;
}

int Kinect2::loadKinectParams(  const std::string& file_path ){

    try{
        std::ifstream fin( file_path, std::ios::binary );

        if( ! fin )
            return 1;

        fin.read( (char*)&ir_param_.fx, sizeof( float ));
        fin.read( (char*)&ir_param_.fy, sizeof( float ));
        fin.read( (char*)&ir_param_.cx, sizeof( float ));
        fin.read( (char*)&ir_param_.cy, sizeof( float ));
        fin.read( (char*)&ir_param_.k1, sizeof( float ));
        fin.read( (char*)&ir_param_.k2, sizeof( float ));
        fin.read( (char*)&ir_param_.k3, sizeof( float ));
        fin.read( (char*)&ir_param_.p1, sizeof( float ));
        fin.read( (char*)&ir_param_.p2, sizeof( float ));
        fin.read( (char*)&color_param_.fx, sizeof( float ));
        fin.read( (char*)&color_param_.fy, sizeof( float ));
        fin.read( (char*)&color_param_.cx, sizeof( float ));
        fin.read( (char*)&color_param_.cy, sizeof( float ));
        fin.read( (char*)&color_param_.shift_d, sizeof( float ));
        fin.read( (char*)&color_param_.shift_m, sizeof( float ));
        fin.read( (char*)&color_param_.mx_x3y0, sizeof( float )); // xxx
        fin.read( (char*)&color_param_.mx_x0y3, sizeof( float )); // yyy
        fin.read( (char*)&color_param_.mx_x2y1, sizeof( float )); // xxy
        fin.read( (char*)&color_param_.mx_x1y2, sizeof( float )); // yyx
        fin.read( (char*)&color_param_.mx_x2y0, sizeof( float )); // xx
        fin.read( (char*)&color_param_.mx_x0y2, sizeof( float )); // yy
        fin.read( (char*)&color_param_.mx_x1y1, sizeof( float )); // xy
        fin.read( (char*)&color_param_.mx_x1y0, sizeof( float )); // x
        fin.read( (char*)&color_param_.mx_x0y1, sizeof( float )); // y
        fin.read( (char*)&color_param_.mx_x0y0, sizeof( float )); // 1
        fin.read( (char*)&color_param_.my_x3y0, sizeof( float )); // xxx
        fin.read( (char*)&color_param_.my_x0y3, sizeof( float )); // yyy
        fin.read( (char*)&color_param_.my_x2y1, sizeof( float )); // xxy
        fin.read( (char*)&color_param_.my_x1y2, sizeof( float )); // yyx
        fin.read( (char*)&color_param_.my_x2y0, sizeof( float )); // xx
        fin.read( (char*)&color_param_.my_x0y2, sizeof( float )); // yy
        fin.read( (char*)&color_param_.my_x1y1, sizeof( float )); // xy
        fin.read( (char*)&color_param_.my_x1y0, sizeof( float )); // x
        fin.read( (char*)&color_param_.my_x0y1, sizeof( float )); // y
        fin.read( (char*)&color_param_.my_x0y0, sizeof( float )); // 1

        registration_.reset( new libfreenect2::Registration( ir_param_, color_param_ ) );
        
        if( fin.good() )
            return 0;
        else return 2;
    }catch(...){
        throw;
    }
}

void Kinect2::waitForNewFrame(){
    if( failed() )
        return;
    listener_->waitForNewFrame( frames_ );
    listener_->release( frames_ );    
}

void Kinect2::waitForNewFrame( color_ch_t* const color,
                               depth_ch_t* const depth,
                               color_ch_t* const registered ){

    static frame_t undistorted_dump( kDWidth, kDHeight, kDNumOfBytesPerPixel );
    static frame_t registered_dump( kDWidth, kDHeight, kDNumOfBytesPerPixel );
    
    if( failed() )
        return;
    listener_->waitForNewFrame( frames_ );
    if( color )
        std::copy( frames_[frame_t::Color]->data,
                   frames_[frame_t::Color]->data + kCNumOfChannels, color );
    if( depth && registered ){
        frame_t undistorted( kDWidth, kDHeight, kDNumOfBytesPerPixel, (unsigned char*)depth );
        frame_t registered( kDWidth, kDHeight, kDNumOfBytesPerPixel, (unsigned char*)registered );
        registration_->apply( frames_[frame_t::Color], frames_[frame_t::Depth],
                              &undistorted, &registered );
    }else if( depth ){
        frame_t undistorted( kDWidth, kDHeight, kDNumOfBytesPerPixel, (unsigned char*)depth );
        registration_->apply( frames_[frame_t::Color], frames_[frame_t::Depth],
                              &undistorted, &registered_dump );
    }else if( registered ){
        frame_t registered( kDWidth, kDHeight, kDNumOfBytesPerPixel, (unsigned char*)registered );
        registration_->apply( frames_[frame_t::Color], frames_[frame_t::Depth],
                              &undistorted_dump, &registered );
    }
    
    listener_->release( frames_ );
}

void Kinect2::mapDepthPointToCameraSpace( const int row,
                                                const int col,
                                                const float depth,
                                                float& x,
                                                float& y,
                                                float& z )const{

    const float fx = 1/ir_param_.fx, fy = 1/ir_param_.fy;
    const float bad_point = std::numeric_limits<float>::quiet_NaN();
    const float depth_val = depth/1000.0f; //scaling factor, so that value of 1 is one meter.
    if (_isnan(depth_val) || depth_val <= 0.001){
        //depth value is not valid
        x = y = z = bad_point;
    }
    else{
        x = (col + 0.5 - ir_param_.cx) * fx * depth_val;
        y = - (row + 0.5 - ir_param_.cy) * fy * depth_val;
        z = depth_val;
    }
    return;
}

void Kinect2::close(){
    if( device_ ){
        device_->stop();
        device_->close();
    }
}

void Kinect2::getPointCloudRGB( depth_ch_t const * const depth,
                                color_ch_t const * const registered,
                                pcl::PointCloud<pcl::PointXYZRGB>& cloud,
                                const double* const homogenous_tansformation_mat )const{

    try{
        pcl::PointCloud<pcl::PointXYZRGB> tmp_cloud(kDWidth, kDHeight);
        cloud.clear();
        cloud = tmp_cloud;
    
        for( int row = 0; row < kDHeight; row++ ){
            for( int col = 0; col < kDWidth; col++ ){
                static pcl::PointXYZRGB point;
                static float x, y, z;
                mapDepthPointToCameraSpace( row, col, depth[row * kDWidth + col], x, y, z );
                if( homogenous_tansformation_mat ){
                    point.x = homogenous_tansformation_mat[0] * x
                        + homogenous_tansformation_mat[1] * y
                        + homogenous_tansformation_mat[2] * z
                        + homogenous_tansformation_mat[3];
                    point.y = homogenous_tansformation_mat[4] * x
                        + homogenous_tansformation_mat[5] * y
                        + homogenous_tansformation_mat[6] * z
                        + homogenous_tansformation_mat[7];
                    point.z = homogenous_tansformation_mat[8] * x
                        + homogenous_tansformation_mat[9] * y
                        + homogenous_tansformation_mat[10] * z
                        + homogenous_tansformation_mat[11];
                }else{
                    point.x = x;
                    point.y = y;
                    point.z = z;
                }

                point.rgb = cvtUcharBgrToFloat( registered[ 4* (row * kDWidth + col ) + 0 ],
                                                registered[ 4* (row * kDWidth + col ) + 1 ],
                                                registered[ 4* (row * kDWidth + col ) + 2 ] );
                
                if( cloud.isOrganized() )
                    cloud( col, row ) = point;
                else
                    cloud.push_back( point );
                        
            }
        }
        return;
    }catch(...){
        throw;
    }
}

float Kinect2::cvtUcharBgrToFloat( const uint8_t b, const uint8_t g, const uint8_t r ){
    union{
        uint8_t c[4];
        float f;
    }u;
    u.c[0] = b;
    u.c[1] = g;
    u.c[2] = r;
    u.c[3] = 255;
    return u.f;
}

void Kinect2::cvtFloatBgrToUchar( const float rgb, uint8_t& b, uint8_t& g, uint8_t& r ){
    union{
        uint8_t c[4];
        float f;
    }u;
    u.f = rgb;
    r = u.c[2];
    g = u.c[1];
    b = u.c[0];
    return;
}

    
