#include "kinect_recorder.hpp"

#include <string>
#include <fstream>
#include <thread>

int main(){

    try{

        KinectRecorder kinect_recorder( "./kinect_config.yaml" );

        if( ! kinect_recorder ){
            std::cerr << "failed to construct kinect_recorder" << std::endl;
            return 1;
        }

        kinect_recorder.init();
        
        kinect_recorder.enterMainLoop();

        kinect_recorder.stopKinectAndDestroyWindow();
        
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": "<< ex.what() << std::endl;
        return 1;
    }   

    return 0;
}
