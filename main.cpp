#include "kinect_manager.hpp"

#include <string>
#include <fstream>

int main(){

    std::string server_ip = "", server_port = "";
    bool specify_each_frame = false;

    std::ifstream ifs( "kinect_client_settings.tsv" );
    if( ! ifs )
        std::cerr << "warning: no setting file found!" << std::endl;
    else{
        std::string buf;
        std::getline( ifs, buf, '\t' );
        std::getline( ifs, buf, '\n' );
        if( buf == "1" ){ // use a server
            std::getline( ifs, buf, '\t' );
            std::getline( ifs, buf, '\n' );
            if( buf != "" )
                server_ip = buf;
            std::getline( ifs, buf, '\t' );
            std::getline( ifs, buf, '\n' );
            if( buf != "" )
                server_port = buf;
        }else if( buf == "0" ){
            std::getline( ifs, buf );
            std::getline( ifs, buf );
        }else
            std::cerr << "warning: invalid setting file!" << std::endl; 

        std::getline( ifs, buf, '\t' );
        std::getline( ifs, buf, '\n' );
        if( buf == "1" )
            specify_each_frame = true;
        else if( buf != "0" )
            std::cerr << "warning: invalid setting file!" << std::endl;                
    }
    
    try{
        std::string kWorkingDir = "C:/Users/admin/Desktop/kinect_data";
        KinectManager kinect_manager( kWorkingDir, server_ip, server_port, specify_each_frame,
                                      4, 4 );
        kinect_manager.init();
        kinect_manager.startKinectAndCreateWindow();
        kinect_manager.enterMainLoop();
        kinect_manager.stopKinectAndDestroyWindow();
    }catch( std::exception& ex ){
        std::cerr << "error in " << __func__ << ": "<< ex.what() << std::endl;
        return 1;
    }   

    return 0;
}
