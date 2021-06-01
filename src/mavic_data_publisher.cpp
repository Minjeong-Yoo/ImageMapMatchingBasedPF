#include "image_map_matching/mavic_data_publisher.hpp"

#define __APP_NAME__ "mavic_data_publisher"

MavicDataPublisher::MavicDataPublisher()
{
    GetParameter();
    DataParser();
}

MavicDataPublisher::~MavicDataPublisher()
{

}

void MavicDataPublisher::GetParameter()
{
    m_rosNodeHandler.getParam("mavic_data_publisher/DEBUG_MODE", _DEBUG_MODE);
    m_rosNodeHandler.getParam("mavic_data_publisher/csvFilePath", m_csvFilePath);
    m_rosNodeHandler.getParam("mavic_data_publisher/imageFolderPath", m_imageFolderPath);
    
    if(_DEBUG_MODE)
    {
        std::cout << "====================== [MAVIC DATA PUBLISHER] GetParameter start ======================" << std::endl;
        ROS_INFO("[%s] DEBUG_MODE: %d", __APP_NAME__, _DEBUG_MODE);
        ROS_INFO("[%s] CSV File Path: %s", __APP_NAME__, m_csvFilePath.c_str());
        ROS_INFO("[%s] Image Folder Path: %s", __APP_NAME__, m_imageFolderPath.c_str());
    }
}

bool MavicDataPublisher::DataParser()
{
    // Debug
    if(_DEBUG_MODE)
    {
        std::cout <<"====================== [MAVIC DATA PUBLISHER] DataParser start ======================" << std::endl;
    }

    // Read file
    FILE *csv_file = NULL;
    csv_file = fopen(m_csvFilePath.c_str(), "r");

    // Check 
    if (csv_file == NULL)
    {
        ROS_ERROR("ERROR OCCURED DURING OPEN FILE : %s", m_csvFilePath.c_str());
        return 1;
    }

    if(_DEBUG_MODE) { ROS_INFO_STREAM("File read complete"); }
    
    
    // idx for selecting data in csv file 
    int idxTime;
    int idxPhoto;
    int idxLatitude;
    int idxLongitude;
    int idxAltitude; 
    int idxXspeed;
    int idxYspeed;
    int idxZspeed;
    int idxRoll;
    int idxPitch;
    int idxYaw; 

    // idx
    unsigned int idxRow = 0;
    unsigned int idxColumn = 0;

    char *token;

    while(idxRow <= m_param_total_row)
    {
        idxColumn = 0;  //initialization

        // timestamp
        int i_hours;
        int i_minutes;
        int i_seconds;
        int i_nano_seconds;

        int i_diff_hours_timestamp;
        int i_diff_minutes_timestamp;
        int i_diff_seconds_timestamp;

        int i_origin_hours = 14;
        int i_origin_minutes = 0;
        int i_origin_seconds = 0;

        int total_timestamp_sec = 0;
        int total_timestamp_nsec = 0;

        // data variable
        double d_latitude_deg;
        double d_longitude_deg;
        double d_altitude_deg;

        double d_xvel_ms;
        double d_yvel_ms;
        double d_zvel_ms;

        double d_roll_deg;
        double d_pitch_deg;
        double d_yaw_deg;

        // sensor msgs
        std_msgs::Header header;
        SensorData sensor_data;
        std::string image_file_name;
        sensor_msgs::NavSatFix gnss;
        geometry_msgs::TwistStamped vel;
        geometry_msgs::PoseStamped pose;

        unsigned int i_imageNum;

        char str_tmp[8000];
        fgets(str_tmp, 8000, csv_file);
        // std::cout << str_tmp << std::endl;
        token = strtok(str_tmp, " ");

        // std::cout << "first : " << token << std::endl;
        while(token != NULL)
        {
            if (idxRow < 1)
            {
                 // idx for selecting data in csv file 
                idxTime = 0;
                idxPhoto = 1;
                idxLatitude = 12;
                idxLongitude = 13;
                idxAltitude = 15; 
                idxXspeed = 16;
                idxYspeed = 17;
                idxZspeed = 18;
                idxRoll = 70;
                idxPitch = 69;
                idxYaw = 71; 
                
                idxRow++;
                break;
                
            }

            else
            {   
                if (idxColumn == idxTime)
                {
                    // Hours parsing
                    token = strtok(NULL, ":");
                    i_hours = atoi(token) + 9;      // GMT => KMT

                    // minutes parsing
                    token = strtok(NULL, ":");
                    i_minutes = atoi(token);

                    // seconds parsing
                    token = strtok(NULL, ".");
                    i_seconds = atoi(token);

                    // nanoseconds parsing
                    token = strtok(NULL, ",");
                    i_nano_seconds = 1000000 * atoi(token);

                    i_diff_hours_timestamp = (i_hours - i_origin_hours) * 3600;
                    i_diff_minutes_timestamp = (i_minutes - i_origin_minutes) * 60;
                    i_diff_seconds_timestamp = (i_seconds - i_origin_seconds);

                    total_timestamp_sec = m_param_timestamp_origin_sec + i_diff_hours_timestamp + i_diff_minutes_timestamp + i_diff_seconds_timestamp;
                    total_timestamp_nsec = i_nano_seconds;

                    header.stamp.sec = total_timestamp_sec;
                    header.stamp.nsec = total_timestamp_nsec;

                    // Put timestamp in SensorData
                    sensor_data.SetHeaderData(header);
                    // std::cout << "Timestamp: " << header << std::endl;
                }
            

                else if (idxColumn == idxPhoto)
                {
                    i_imageNum = atoi(token);

                    int no_image_data = 0;

                    if (i_imageNum != no_image_data) 
                    {
                        std::string extension = ".jpg";
                        std::string fileName = std::to_string(i_imageNum); 

                        image_file_name = m_imageFolderPath + fileName + extension;
                        std::cout << image_file_name << std::endl;

                        cv::Mat image = cv::imread(image_file_name, 0);     // 0: IMREAD_GRAYSCALE, 1: IMREAD_COLOR
                        
                        // Put image in SensorData
                        sensor_data.SetImageData(image);
                        // cv::imshow("image", image);
                        // cv::waitKey(0);
                    }
                }

                else if (idxColumn == idxLatitude || idxColumn == idxLongitude || idxColumn == idxAltitude)
                {
                    if (idxColumn == idxLatitude) 
                    {
                        d_latitude_deg = atof(token);
                        gnss.latitude = d_latitude_deg;
                    }
                    else if(idxColumn == idxLongitude) 
                    {
                        d_longitude_deg = atof(token);
                        gnss.longitude = d_longitude_deg;
                    }
                    else 
                    {
                        d_altitude_deg = std::atof(token);
                        gnss.altitude = d_altitude_deg;

                        // Put GNSS data in SensorData
                        sensor_data.SetGnssData(gnss);
                        // std::cout << gnss << std::endl;
                    }
                }

                else if (idxColumn == idxXspeed || idxColumn == idxYspeed || idxColumn == idxZspeed)
                {
                    if (idxColumn == idxXspeed) d_xvel_ms = atof(token);
                    else if(idxColumn == idxYspeed) d_yvel_ms = atof(token);
                    else 
                    {
                        d_zvel_ms = atof(token);
                        
                        // Put velocity data into SensorData
                        vel.twist.linear.x = d_xvel_ms;
                        vel.twist.linear.y = d_yvel_ms;
                        vel.twist.linear.z = d_zvel_ms;
                    }
                }

                else if (idxColumn == idxRoll || idxColumn == idxPitch || idxColumn == idxYaw)
                {
                    if (idxColumn == idxRoll) d_roll_deg = atof(token);
                    else if (idxColumn == idxPitch) d_pitch_deg = atof(token);
                    else 
                    {
                        d_yaw_deg = atof(token);

                        // Euler to quaternion 
                        tf::Quaternion q;
                        q.setRPY(d_roll_deg, d_pitch_deg, d_yaw_deg);
                        
                        geometry_msgs::Quaternion quaternion;
                        tf::quaternionTFToMsg(q, quaternion);
                         
                        pose.pose.orientation = quaternion;
                        sensor_data.SetPoseData(pose);

                        std::cout << pose << std::endl;

                    }
                }

                token = strtok(NULL, ",");
                idxColumn++;
            }
        }

        if (idxRow > 0)
        {
            if (_DEBUG_MODE)
            {
                std::cout << "Row: " << idxRow <<", Time(KMT): " << i_hours 
                << ":"<< i_minutes << ":" << i_seconds << "." << i_nano_seconds << ", "
                << "ImageNum: " << i_imageNum << ", "
                << std::setprecision(12) << " Latitude: " << d_latitude_deg << ", "
                << std::setprecision(12) << " longitude: " << d_longitude_deg << ", "
                << std::setprecision(12) << " altitude: " << d_altitude_deg << ", "
                << "XSpeed: " << d_xvel_ms << ", " 
                << "YSpeed: " << d_yvel_ms << ", "
                << "ZSpeed: " << d_zvel_ms << ", "
                << "Roll: " << d_roll_deg << ", "
                << "pitch: " << d_pitch_deg << ", "
                << "Yaw: " << d_yaw_deg << std::endl;
            }



        } 

        idxRow++;
    }
    
    std::cout <<"====================== [MAVIC DATA PUBLISHER] DataParser complete ======================" << std::endl;
    
    return 0;
}

void MavicDataPublisher::Run()
{
    

}

int main(int argc, char **argv){
    ros::init(argc, argv, "mavic_data_publisher");
    
    MavicDataPublisher MavicDataPublisher;

    // Loop Rate
    ros::Rate loop_rate(20);

    while(ros::ok())
    {
        MavicDataPublisher.Run();        
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}