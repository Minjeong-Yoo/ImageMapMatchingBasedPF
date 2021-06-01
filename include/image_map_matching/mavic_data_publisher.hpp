#ifndef __MAVIC_DATA_PUBLISHER_HPP__
#define __MAVIC_DATA_PUBLISHER_HPP__

#include <cstdio>
#include <cstring>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp> 

#include <ros/ros.h>

#include "image_map_matching/sensor_data.hpp"


class MavicDataPublisher
{
    public:
        explicit MavicDataPublisher();
        ~MavicDataPublisher();
        void Run();

    private:
        void GetParameter();
        bool DataParser();
    
    private:
        // Node Init
        ros::NodeHandle m_rosNodeHandler;

        // Publisher

        // Subscriber

        // Member variable
        std::string m_csvFilePath;
        std::string m_imageFolderPath;

        int m_param_total_row = 6978;       // CSV total row
        unsigned int m_param_timestamp_origin_sec  = 1614229200;    // Origin timestamp 2021-02-25 14:00:00 (KMT)
        
        std::vector<SensorData> m_v_sensor_data; 

        // Mode Flag 
        bool _DEBUG_MODE;
    

};




#endif 