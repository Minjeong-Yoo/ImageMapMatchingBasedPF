#ifndef __MAP_MATCHING_PF_HPP__
#define __MAP_MATCHING_PF_HPP__

#include <string>
#include <unistd.h>
#include <random>
#include <chrono>
#include <cmath>

#include <ros/ros.h>

#include <geometry_msgs/Point.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <tf/tf.h>
#include <tf2/LinearMath/Quaternion.h>

#include <opencv2/core/mat.hpp>
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include "image_map_matching/map_data.hpp"

class MapMatchingPF
{
    struct GNSS
    {
        double timestamp;
        double latitude = 0.0;
        double longitude = 0.0;
        double altitude = 0.0;
    };

    struct IMU
    {
        double timestamp;
        tf::Quaternion initial_q;
        double orientation[3] = {0.};
        double linear_acceleration_x = 0.0;
        double linear_acceleration_y = 0.0;
        double linear_acceleration_z = 0.0;
        double angular_velocity_x = 0.0;
        double angular_velocity_y = 0.0;
        double angular_velocity_z = 0.0;
        double magnetic_field_x = 0.0;
        double magnetic_field_y = 0.0;
        double magnetic_field_z = 0.0;

        double magentic_field_x_bias = -4.11705e-06;
        double magentic_field_y_bias = 2.94433e-05;
        double magentic_field_z_bias = -4.15828e-05;


    };

    public:
        explicit MapMatchingPF();
        ~MapMatchingPF();
        
        void Run();

    private:
        void GetParameter();
        void MapReader();
        void MapVisualizer();
        
        void SensorInit();
        void ParticleInit();

        void CallBackGNSS(const sensor_msgs::NavSatFix::ConstPtr &msg);
        void CallBackImu(const sensor_msgs::Imu::ConstPtr &msg);
        void CallBackImuMagnetometer(const sensor_msgs::MagneticField::ConstPtr &msg);

        void PFPrediction();
        void PredictionCVModel(Eigen::Matrix<double, 6, 1> tmpInput, Eigen::Matrix<double, 16, 1> tmpState, double dt);


        void TRIAD(double body2Nav_east, double body2Nav_north, double body2Nav_up);
        

        // Utils
        inline geometry_msgs::Point llh2enu(double latitude, double longitude, double altitude);
        inline double FnKappaLat(double dRef_Latitude, double dHeight);
        inline double FnKappaLon(double dRef_Latitude, double dHeight);
        inline double GaussianRandomGenerator(double dMean, double dStd);
        inline Eigen::Quaterniond euler2quat(double yaw, double pitch, double roll);
        inline Eigen::Quaterniond InitialOrientation();


    
    private:
        ros::NodeHandle nh;

        ros::Subscriber m_sub_gnss;
        ros::Subscriber m_sub_imu;
        ros::Subscriber m_sub_imu_magetometer;

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

        const double Geod_a = 6378137.0;//SemiMajorAxis
        const double Geod_e2 = 0.00669437999014; // FirstEccentricitySquard, e ^ 2 
        const double DEG2RAD = M_PI / 180;
        const double RAD2DEG = 180 / M_PI;
        const double Interval = 10; // Sensor bias compensation during stationary periods(15s)  
        const double Magnetic_declination_rad = 13.78 * DEG2RAD;    // deg, Date : 2021-06-17, changing by 0.08 deg/hour
        const double GRAVITY_MAGNITUDE = 9.79951;    // m/s^2, https://www.sensorsone.com/local-gravity-calculator/

        int m_iGnssCount = 0;
        int m_iImuCount = 0;
        int m_iMagnetometerCount = 0;

        GNSS m_gnssInitGnss;
        GNSS m_gnssGnss;
        GNSS m_gnssGnssSum;
        IMU m_imuInitImu;
        IMU m_imuImu;
        IMU m_imuImuSum;

        double m_dStdGNSS;
        double m_dStdVelocity_ms;
        double m_dStdOrientation;
        double m_dStdGyroBias;
        double m_dStdAccelBias;
        
        double m_dStdInputAccX_mss;
        double m_dStdInputAccY_mss;
        double m_dStdInputAccZ_mss;
        double m_dStdInputRollRate_degs;
        double m_dStdInputPitchRate_degs;
        double m_dStdInputYawRate_degs;

        double m_dStdEast;
        double m_dStdNorth;
        double m_dStdUp;
        
        // Flag 
        bool _DEBUG_MODE;
        bool m_bIsInit;
        bool m_bIsInitGNSS;
        bool m_bIsFirstIMUStep;
        bool m_bMapInit;
        bool m_bImuExistFlag;
        bool m_bGnssExistFlag;
        bool m_bIsInitGnss; 
        bool m_bIsFirstMagnetometerStep;

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

        Eigen::VectorXd m_VecGravityNav;
        // Sensor data Matrix 
        Eigen::MatrixXd m_MatMagnetometer;
        


        double m_dMapHeight;





};



#endif 
