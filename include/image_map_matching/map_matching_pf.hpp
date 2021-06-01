#ifndef __MAP_MATCHING_PF_HPP__
#define __MAP_MATCHING_PF_HPP__

#include <string>
#include <unistd.h>
#include <random>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <opencv2/core/mat.hpp>
#include <eigen3/Eigen/Core>

#include "image_map_matching/map_data.hpp"

class MapMatchingPF
{
    struct GNSS
    {
        double timestamp;
        double latitude;
        double longitude;
        double altitude;
    };

    struct IMU
    {
        double timestamp;
        double linear_acceleration_x;
        double linear_acceleration_y;
        double linear_acceleration_z;
        double angular_velocity_x;
        double angular_velocity_y;
        double angular_velocity_z;

    };

    public:
        explicit MapMatchingPF();
        ~MapMatchingPF();

    private:
        void GetParameter();
        void MapReader();
        void MapVisualizer();
        
        void SensorInit();
        void ParticleInit();

        void CallBackGNSS(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void CallBackIMU(const sensor_msgs::Imu::ConstPtr &msg);



        // Utils
        inline geometry_msgs::Point llh2enu(double latitude, double longitude, double altitude);
        inline double FnKappaLat(double dRef_Latitude, double dHeight);
        inline double FnKappaLon(double dRef_Latitude, double dHeight);
        inline double GaussianRandomGenerator(double dMean, double dStd);
    


    
    private:
        ros::NodeHandle nh;

        ros::Subscriber m_sub_gnss;
        ros::Subscriber m_sub_imu;

        ros::Publisher m_pub_map_lane;
        ros::Publisher m_pub_map_lane_array;

        // temp
        ros::Publisher m_pub_gnss_points;
        
        // Map data
        std::string m_MapNodePath;
        std::string m_MapWayPath;
        std::vector<Node> m_vec_map_node;
        std::vector<Way> m_vec_map_way;

        int m_cfg_iNumParticle;
        int m_iNumState;
        double m_dStdEast;
        double m_dStdNorth;
        double m_dStdUp;

        const double Geod_a = 6378137.0;//SemiMajorAxis
        const double Geod_e2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2 
        const double DEG2RAD = M_PI / 180;
        const double RAD2DEG = 180 / M_PI;

        GNSS m_gnssInitGnss;
        GNSS m_gnssGnss;
        IMU m_imuInitImu;
        IMU m_imuImu;

        // Flag 
        bool _DEBUG_MODE;
        bool m_bIsInit;
        bool m_bIsInitGNSS;
        bool m_bIsFirstIMUStep;
        bool m_bMapInit;
        bool m_bImuExistFlag;
        bool m_bGnssExistFlag;
        bool m_bIsInitGnss; 

        std::default_random_engine m_RandGenerator;


        geometry_msgs::PoseStamped m_psInitGnssEnu;
        geometry_msgs::PoseStamped m_psGnssEnu;


        // Particle filter variable 
        Eigen::MatrixXd m_MatTimestampStatePF;
        Eigen::MatrixXd m_MatResamplingTimestampStatePF;
        Eigen::MatrixXd m_MatParticleWeight;
        Eigen::MatrixXd m_MatParticleLikelihoodGNSS;
        Eigen::MatrixXd m_MatParticleLikelihoodMapMatching;
        Eigen::MatrixXd m_MatEstRepState;           // representative state
        Eigen::MatrixXd m_MatEstRepStd;





        double m_dMapHeight;





};



#endif 
