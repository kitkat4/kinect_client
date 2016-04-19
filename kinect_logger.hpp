#pragma once

#include <libfreenect2/logger.h>
#include <iostream>
#include <fstream>
#include <ctime>
#include <string>

class KinectLogger: public libfreenect2::Logger{
    
private:
    std::ofstream logfile_;
public:
    KinectLogger(const char *filename){
        open( filename );
        level_ = Debug;

    }
    ~KinectLogger(){
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
