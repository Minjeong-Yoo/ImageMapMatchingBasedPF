#include <std_msgs/Header.h>
#include <sensor_msgs/NavSatFix.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>

#include <tf/tf.h>

#include <opencv2/core/mat.hpp>

class SensorData
{
    public:
        explicit SensorData(std_msgs::Header input_header, sensor_msgs::NavSatFix input_gnss, 
                            geometry_msgs::TwistStamped input_vel, geometry_msgs::PoseStamped input_pose, 
                            cv::Mat input_image)
        {
            m_header_data = input_header;
            m_gnss_data = input_gnss;
            m_vel_data = input_vel;
            m_pose_data = input_pose;
            m_image_data = input_image;
        }
        explicit SensorData()
        {

        }

        ~SensorData()
        {

        }

        void SetHeaderData(std_msgs::Header input_header)
        {
            m_header_data = input_header;
        }

        void SetGnssData(sensor_msgs::NavSatFix input_gnss)
        {
            m_gnss_data = input_gnss;
        }

        void SetGnssData(geometry_msgs::TwistStamped input_vel)
        {
            m_vel_data = input_vel;
        }

        void SetPoseData(geometry_msgs::PoseStamped input_pose)
        {
            m_pose_data = input_pose;
        }

        void SetImageData(cv::Mat input_image)
        {
            m_image_data = input_image;
        }

        std_msgs::Header GetHeaderData()
        {
            return m_header_data;
        }

        sensor_msgs::NavSatFix GetGnssData()
        {
            return m_gnss_data;
        }

        geometry_msgs::TwistStamped GetVelData()
        {
            return m_vel_data;
        }


        geometry_msgs::PoseStamped GetPoseData()
        {
            return m_pose_data;
        }

        cv::Mat GetImageData()
        {
            return m_image_data;
        }


    private:
        std_msgs::Header m_header_data;
        sensor_msgs::NavSatFix m_gnss_data;
        geometry_msgs::TwistStamped m_vel_data;
        geometry_msgs::PoseStamped m_pose_data;
        cv::Mat m_image_data;
};