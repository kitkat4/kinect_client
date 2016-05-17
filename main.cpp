#include "kinect_recorder.hpp"

#include <string>
#include <fstream>
#include <thread>

int main(){

    try{

        // KinectRecorder kinect_recorder1( "./kinect_config1.yaml" );
        KinectRecorder kinect_recorder2( "./kinect_config2.yaml" );
        // if( ! kinect_recorder1 ){
        //     std::cerr << "failed to construct kinect_recorder1" << std::endl;
        //     return 1;
        // }
        if( ! kinect_recorder2 ){
            std::cerr << "failed to construct kinect_recorder2" << std::endl;
            return 1;
        }

        // std::cout << __LINE__ << std::endl;
        kinect_recorder2.init();
        // std::cout << __LINE__ << std::endl;
        // kinect_recorder1.init();
        // std::cout << __LINE__ << std::endl;

        // std::thread thread1( &KinectRecorder::enterMainLoop, &kinect_recorder1 );

        // std::cout << __LINE__ << std::endl;
        
        kinect_recorder2.enterMainLoop();

        // std::cout << __LINE__ << std::endl;
        // thread1.join();
        // std::cout << __LINE__ << std::endl;

        // kinect_recorder1.stopKinectAndDestroyWindow();
        kinect_recorder2.stopKinectAndDestroyWindow();
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": "<< ex.what() << std::endl;
        return 1;
    }   

    return 0;
}
