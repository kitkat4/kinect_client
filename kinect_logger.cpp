#include "kinect_logger.hpp"

void KinectLogger::log(Level level, const std::string &message){

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
