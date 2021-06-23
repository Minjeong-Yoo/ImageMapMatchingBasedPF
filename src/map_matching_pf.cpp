#include "image_map_matching/map_matching_pf.hpp"

#define __APP_NAME__ "map_matching_pf"

MapMatchingPF::MapMatchingPF()
:m_bIsInit(false), m_bIsInitGNSS(false), m_bIsFirstIMUStep(false), m_bIsFirstMagnetometerStep(false), m_bMapInit(false),
m_bGnssExistFlag(false), m_bImuExistFlag(false), 
m_dStdInputAccX_mss(0.0), m_dStdInputAccY_mss(0.0), m_dStdInputAccZ_mss(0.0), m_dStdInputRollRate_degs(0.0), m_dStdInputPitchRate_degs(0.0), m_dStdInputYawRate_degs(0.0),
m_dMapHeight(7.0)
{
    GetParameter();
    int buffer_size = 1;
   
    // Subscriber
    m_sub_gnss = nh.subscribe("/mavros/global_position/global", buffer_size, &MapMatchingPF::CallBackGNSS, this);
    // m_sub_imu = nh.subscribe("/mavros/imu/data_raw", buffer_size, &MapMatchingPF::CallBackImu, this);
    m_sub_imu = nh.subscribe("/mavros/imu/data", buffer_size, &MapMatchingPF::CallBackImu, this);
    m_sub_imu_magetometer = nh.subscribe("/mavros/imu/mag", buffer_size, &MapMatchingPF::CallBackImuMagnetometer, this);

    // Publisher
    m_pub_map_lane = nh.advertise<visualization_msgs::Marker>("/map_lane", 10);
    m_pub_map_lane_array = nh.advertise<visualization_msgs::MarkerArray>("/map_lane_array", 10);

    // temp
    m_pub_gnss_points = nh.advertise<visualization_msgs::Marker>("/gnss_points", 10);

    // ParticleFilter variable 
    m_MatTimestampStatePF = Eigen::MatrixXd::Zero(m_cfg_iNumParticle,m_iNumState);
    m_MatResamplingTimestampStatePF = Eigen::MatrixXd::Zero(m_cfg_iNumParticle,m_iNumState);
    m_MatParticleWeight = Eigen::MatrixXd::Zero(m_cfg_iNumParticle, 1);
    m_MatParticleLikelihoodGNSS = Eigen::MatrixXd::Zero(m_cfg_iNumParticle, 1);
    m_MatParticleLikelihoodMapMatching = Eigen::MatrixXd::Zero(m_cfg_iNumParticle, 1);
    m_MatEstRepState = Eigen::MatrixXd::Zero(m_iNumState, 1);
    m_MatEstRepStd = Eigen::MatrixXd::Zero(m_iNumState, 1);

    m_VecGravityNav = Eigen::Vector3d(0.0, 0.0, -GRAVITY_MAGNITUDE);

    MapReader();    
    SensorInit(); 
    ParticleInit();
}

MapMatchingPF::~MapMatchingPF()
{


}

void MapMatchingPF::GetParameter()
{
    nh.getParam("map_matching_pf/DEBUG_MODE", _DEBUG_MODE);
    nh.getParam("map_matching_pf/m_MapNodePath", m_MapNodePath);
    nh.getParam("map_matching_pf/m_MapWayPath", m_MapWayPath);
    nh.getParam("map_matching_pf/m_cfg_iNumParticle", m_cfg_iNumParticle);
    nh.getParam("map_matching_pf/m_iNumState", m_iNumState);
    nh.getParam("map_matching_pf/m_dStdGNSS", m_dStdGNSS);
    nh.getParam("map_matching_pf/m_dStdVelocity_ms", m_dStdVelocity_ms);
    nh.getParam("map_matching_pf/m_dStdOrientation", m_dStdOrientation);
    nh.getParam("map_matching_pf/m_dStdGyroBias", m_dStdGyroBias);
    nh.getParam("map_matching_pf/m_dStdAccelBias", m_dStdAccelBias);
    nh.getParam("map_matching_pf/m_dStdEast", m_dStdEast);
    nh.getParam("map_matching_pf/m_dStdNorth", m_dStdNorth);
    nh.getParam("map_matching_pf/m_dStdUp", m_dStdUp);
    nh.getParam("map_matching_pf/m_dStdInputAccX_mss", m_dStdInputAccX_mss);
    nh.getParam("map_matching_pf/m_dStdInputAccY_mss", m_dStdInputAccY_mss);
    nh.getParam("map_matching_pf/m_dStdInputAccZ_mss", m_dStdInputAccZ_mss);
    nh.getParam("map_matching_pf/m_dStdInputRollRate_degs", m_dStdInputRollRate_degs);
    nh.getParam("map_matching_pf/m_dStdInputPitchRate_degs", m_dStdInputPitchRate_degs);
    nh.getParam("map_matching_pf/m_dStdInputYawRate_degs", m_dStdInputYawRate_degs);

    

    if(_DEBUG_MODE)
    {
        std::cout << "====================== [map_matching_pf] GetParameter start ======================" << std::endl;
        ROS_INFO("[%s] DEBUG_MODE: %d", __APP_NAME__, _DEBUG_MODE);
        ROS_INFO("[%s] m_MapNodePath: %s", __APP_NAME__, m_MapNodePath.c_str());
        ROS_INFO("[%s] m_MapWayPath: %s", __APP_NAME__, m_MapWayPath.c_str());
        ROS_INFO("[%s] m_cfg_iNumParticle: %d", __APP_NAME__, m_cfg_iNumParticle);
        ROS_INFO("[%s] m_dStdGNSS: %lf", __APP_NAME__, m_dStdGNSS);
        ROS_INFO("[%s] m_dStdVelocity_ms: %lf", __APP_NAME__, m_dStdVelocity_ms);
        ROS_INFO("[%s] m_dStdOrientation: %lf", __APP_NAME__, m_dStdOrientation);
        ROS_INFO("[%s] m_dStdGyroBias: %lf", __APP_NAME__, m_dStdGyroBias);
        ROS_INFO("[%s] m_dStdAccelBias: %lf", __APP_NAME__, m_dStdAccelBias);
        ROS_INFO("[%s] m_cfg_iNumState: %d", __APP_NAME__, m_iNumState);
        ROS_INFO("[%s] m_dStdEast: %lf", __APP_NAME__, m_dStdEast);
        ROS_INFO("[%s] m_dStdNorth: %lf", __APP_NAME__, m_dStdNorth);
        ROS_INFO("[%s] m_dStdUp: %lf\n", __APP_NAME__, m_dStdUp);
        ROS_INFO("[%s] m_dStdInputAccX_mss: %lf\n", __APP_NAME__, m_dStdInputAccX_mss);
        ROS_INFO("[%s] m_dStdInputAccY_mss: %lf\n", __APP_NAME__, m_dStdInputAccY_mss);
        ROS_INFO("[%s] m_dStdInputAccZ_mss: %lf\n", __APP_NAME__, m_dStdInputAccZ_mss);
        ROS_INFO("[%s] m_dStdInputRollRate_degs: %lf\n", __APP_NAME__, m_dStdInputRollRate_degs);
        ROS_INFO("[%s] m_dStdInputPitchRate_degs: %lf\n", __APP_NAME__, m_dStdInputPitchRate_degs);
        ROS_INFO("[%s] m_dStdInputYawRate_degs: %lf\n", __APP_NAME__, m_dStdInputYawRate_degs);
    }
}

void MapMatchingPF::MapReader()
{
    if(_DEBUG_MODE)
    {
        std::cout << "====================== [map_matching_pf] MapReader start ======================" << std::endl;
    }

    FILE* node_csv_file = NULL;
    node_csv_file = fopen(m_MapNodePath.c_str(), "r");

    // Read MapNodeFile
    if(node_csv_file != NULL)
    {
        while(!feof(node_csv_file))
        {
            Node node;
            char str_tmp[2048];
            char* token; 
            int cnt = 0;

            fgets(str_tmp, 1024, node_csv_file);
            token = strtok(str_tmp, ",");
 
            while(token != NULL)
            {
                if(cnt == 0) { node.SetId(atoi(token)); }
                if(cnt == 1) { node.SetLatitude(atof(token)); }
                if(cnt == 2) { node.SetLongitude(atof(token)); }
                cnt++;
                
                token = strtok(NULL, ",");
            }

            m_vec_map_node.push_back(node);
        }

    }

    else if(_DEBUG_MODE)
    {
        ROS_ERROR("ERROR OCCURED DURING OPEN MAP NODE FILE");
    }


    // Read MapWayFile
    FILE* way_csv_file = NULL;
    way_csv_file = fopen(m_MapWayPath.c_str(), "r");

    if(way_csv_file != NULL)
    {
        while(!feof(way_csv_file))
        {
            Way way;
            char str_tmp[2048];
            char* token; 
            int cnt = 0;

            fgets(str_tmp, 2048, way_csv_file);
            token = strtok(str_tmp, ",");

            while(token != NULL)
            {
                if(cnt == 0) { way.SetId(atoi(token)); }
                else { way.SetNodeIds(atoi(token)); }
                cnt++;

                token = strtok(NULL, ",");
            }

            m_vec_map_way.push_back(way);
        }
    }

    else if(_DEBUG_MODE)
    {
        ROS_ERROR("ERROR OCCURED DURING OPEN MAP WAY FILE");
    }

    if(_DEBUG_MODE) ROS_INFO_STREAM("Map file read complete");

    m_bMapInit = true;
    MapVisualizer();
}


void MapMatchingPF::MapVisualizer()
{
    if(_DEBUG_MODE) ROS_INFO_STREAM("MapVisulizer function start");
    visualization_msgs::MarkerArray map_lane_array;

    unsigned int map_lane_size = m_vec_map_way.size();

    for (int lane_idx = 0; lane_idx < map_lane_size; lane_idx++) 
    {
        // ROS_ERROR_STREAM("1");
        visualization_msgs::Marker lane_marker;

        lane_marker.header.frame_id = "/map";
        lane_marker.header.stamp = ros::Time::now();
        lane_marker.ns = "map_lane";
        lane_marker.id = lane_idx;
        lane_marker.type = visualization_msgs::Marker::LINE_STRIP;
        lane_marker.action = visualization_msgs::Marker::ADD;

        unsigned int map_node_size = m_vec_map_way.at(lane_idx).GetNodeIds().size();
        if(map_node_size < 1) continue;

        for (int node_idx = 0; node_idx < map_node_size; node_idx++)
        {
            geometry_msgs::Point point;   

            // Convert to Map coordinate 
            point = llh2enu(m_vec_map_node.at(m_vec_map_way.at(lane_idx).GetNodeIds().at(node_idx)).GetLatitude(),
                            m_vec_map_node.at(m_vec_map_way.at(lane_idx).GetNodeIds().at(node_idx)).GetLongitude(),
                            m_vec_map_node.at(m_vec_map_way.at(lane_idx).GetNodeIds().at(node_idx)).GetAltitude());
            
            lane_marker.points.push_back(point);
            usleep(100);
        }

        lane_marker.pose.orientation.w = 1.0;
        lane_marker.pose.orientation.x = 0.0;
        lane_marker.pose.orientation.y = 0.0;
        lane_marker.pose.orientation.z = 0.0;

        lane_marker.scale.x = 0.1;
        lane_marker.scale.y = 0.1;
        lane_marker.scale.z = 0.1;

        lane_marker.color.r = 0.0f;
        lane_marker.color.g = 1.0f;
        lane_marker.color.b = 0.0f;
        lane_marker.color.a = 1.0;
        lane_marker.lifetime = ros::Duration(100);

        map_lane_array.markers.push_back(lane_marker);
        m_pub_map_lane.publish(lane_marker);
    }

    m_pub_map_lane_array.publish(map_lane_array);

    if(_DEBUG_MODE) ROS_INFO_STREAM("MapVisulizer function finish\n");
}


void MapMatchingPF::SensorInit()
{
    if(_DEBUG_MODE) 
    {
        std::cout << "====================== [map_matching_pf] SensorInit start ======================" << std::endl;
    }

    int loop_hz = 20;
    ros::Rate loop_rate(loop_hz);

    while(!m_bIsInit && ros::ok())
    {
        if(_DEBUG_MODE)
        {
                std::cout << "Initializing\n" << "m_bIsInitGNSS: " << m_bIsInitGNSS 
                << " m_bIsFirstIMUStep: " << m_bIsFirstIMUStep 
                << " m_bIsFirstMagnetometer: " << m_bIsFirstMagnetometerStep
                << " m_bMapInit: " << m_bMapInit << "\n" << std::endl; 
        }

        if(m_bIsInitGNSS && m_bIsFirstIMUStep && m_bIsFirstMagnetometerStep && m_bMapInit)
        {
            m_bIsInit = true;
            return;
        }

        ros::spinOnce();
        loop_rate.sleep();

    }
}
 
void MapMatchingPF::ParticleInit()
{
    if(_DEBUG_MODE) ROS_INFO_STREAM("ParticleInit function start");

    // Initial state
    Eigen::VectorXd InitState(m_iNumState);
    
    // Position
    InitState(0) = m_psInitGnssEnu.pose.position.x;
    InitState(1) = m_psInitGnssEnu.pose.position.y;
    InitState(2) = m_psInitGnssEnu.pose.position.z;
    
    // Velocity
    InitState(3) = 0.0;
    InitState(4) = 0.0;
    InitState(5) = 0.0;

    // Orientation
    Eigen::Quaterniond quat;
    quat = InitialOrientation();

    InitState(6) = quat.w();
    InitState(7) = quat.x();
    InitState(8) = quat.y();
    InitState(9) = quat.z();

    // Bias
    InitState(10) = m_imuInitImu.angular_velocity_x;
    InitState(11) = m_imuInitImu.angular_velocity_y;
    InitState(12) = m_imuInitImu.angular_velocity_z;
    
    InitState(13) = 0.0;
    InitState(14) = 0.0;
    InitState(15) = 0.0;

    if(_DEBUG_MODE)
    {
        std::cout << "ParticleInitState: \n" << "px: " << InitState(0) << "\n"
                                << "py: " << InitState(0) << "\n"
                                << "pz: " << InitState(1) << "\n"
                                << "vx: " << InitState(2) << "\n"
                                << "vy: " << InitState(3) << "\n"
                                << "vz: " << InitState(4) << "\n"
                                << "qx: " << InitState(5) << "\n"
                                << "qy: " << InitState(6) << "\n"
                                << "qz: " << InitState(7) << "\n"
                                << "qw: " << InitState(8) << "\n" 
                                << "b_gyro_x: " << InitState(9) << "\n" 
                                << "b_gyro_y: " << InitState(10) << "\n" 
                                << "b_gyro_z: " << InitState(11) << "\n" 
                                << "b_accel_x: " << InitState(12) << "\n" 
                                << "b_accel_y: " << InitState(13) << "\n" 
                                << "b_accel_z: " << InitState(14) << std::endl;
    }


    Eigen::MatrixXd ParticleInitState(m_cfg_iNumParticle, m_iNumState);

    // Particle Initialization 
    for(int particleIdx = 0; particleIdx < m_cfg_iNumParticle; particleIdx++)
    {
        // Position
        ParticleInitState(particleIdx,0) = InitState(0) + GaussianRandomGenerator(0, m_dStdGNSS);
        ParticleInitState(particleIdx,1) = InitState(1) + GaussianRandomGenerator(0, m_dStdGNSS);
        ParticleInitState(particleIdx,2) = InitState(2) + GaussianRandomGenerator(0, m_dStdGNSS);
  
        // Velocity
        ParticleInitState(particleIdx,3) = InitState(3) + GaussianRandomGenerator(0, m_dStdVelocity_ms);
        ParticleInitState(particleIdx,4) = InitState(4) + GaussianRandomGenerator(0, m_dStdVelocity_ms);
        ParticleInitState(particleIdx,5) = InitState(5) + GaussianRandomGenerator(0, m_dStdVelocity_ms);
        
        // Orientation 
        ParticleInitState(particleIdx,6) = InitState(6) + GaussianRandomGenerator(0, m_dStdOrientation);
        ParticleInitState(particleIdx,7) = InitState(7) + GaussianRandomGenerator(0, m_dStdOrientation);
        ParticleInitState(particleIdx,8) = InitState(8) + GaussianRandomGenerator(0, m_dStdOrientation);
        ParticleInitState(particleIdx,9) = InitState(9) + GaussianRandomGenerator(0, m_dStdOrientation);

        // bias
        // gyroscope bias 
        ParticleInitState(particleIdx, 10) = InitState(10) + GaussianRandomGenerator(0, m_dStdGyroBias);
        ParticleInitState(particleIdx, 11) = InitState(11) + GaussianRandomGenerator(0, m_dStdGyroBias);
        ParticleInitState(particleIdx, 12) = InitState(12) + GaussianRandomGenerator(0, m_dStdGyroBias);

        // acclerometer bias
        ParticleInitState(particleIdx, 13) = InitState(13) + GaussianRandomGenerator(0, m_dStdAccelBias);
        ParticleInitState(particleIdx, 14) = InitState(14) + GaussianRandomGenerator(0, m_dStdAccelBias);
        ParticleInitState(particleIdx, 15) = InitState(15) + GaussianRandomGenerator(0, m_dStdAccelBias);

       
        m_MatParticleWeight(particleIdx) = 1. / (double)m_cfg_iNumParticle;

    }

    m_MatTimestampStatePF = ParticleInitState;
        
}



void MapMatchingPF::CallBackGNSS(const sensor_msgs::NavSatFix::ConstPtr &msg)
{ 
    
    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    static double prev_timestamp = timestamp;

    if(!m_bIsInitGNSS)
    {
        double sample_time = (timestamp - prev_timestamp) / 1e6;
        
        if (sample_time <= Interval)
        {
            GNSS temp_gnss;
            temp_gnss.latitude = msg->latitude;
            temp_gnss.longitude = msg->longitude;
            temp_gnss.altitude = msg->altitude;

            // accumulation 
            m_gnssGnssSum.latitude += temp_gnss.latitude;
            m_gnssGnssSum.longitude += temp_gnss.longitude;
            m_gnssGnssSum.altitude += temp_gnss.altitude;

            m_iGnssCount++;

            return;
        }

        // Average of GNSS during 15s
        m_gnssInitGnss.timestamp = timestamp;
        m_gnssInitGnss.latitude = m_gnssGnssSum.latitude / m_iGnssCount;
        m_gnssInitGnss.longitude = m_gnssGnssSum.longitude / m_iGnssCount;
        m_gnssInitGnss.altitude = m_gnssGnssSum.altitude / m_iGnssCount; 

        ROS_ERROR("Latitude: %lf, Longitude: %lf, Altitude: %lf", m_gnssInitGnss.latitude, m_gnssInitGnss.longitude, m_gnssInitGnss.altitude);

        m_psInitGnssEnu.pose.position = llh2enu(m_gnssInitGnss.latitude, m_gnssInitGnss.longitude, m_gnssInitGnss.altitude);
        ROS_ERROR("X: %lf, Y: %lf, Z: %lf", m_psInitGnssEnu.pose.position.x, m_psInitGnssEnu.pose.position.y, m_psInitGnssEnu.pose.position.z);
        
        
        if(_DEBUG_MODE)
        {
            std::cout << "====================== [map_matching_pf] REF GNSS ======================\n"
            << "ref gnss\n" << "ref_gnss_latitude: " << std::setprecision(12) << m_gnssInitGnss.latitude 
            << " ref_gnss_longitude: " << std::setprecision(12) << m_gnssInitGnss.longitude 
            << " ref_gnss_altitude: " << std::setprecision(12) << m_gnssInitGnss.altitude << "\n" << std::endl;

            std::cout << "====================== [map_matching_pf] REF GNSS (map coordinate) ======================\n"
            << "ref gnss\n" << "ref_x: " << std::setprecision(12) << m_psInitGnssEnu.pose.position.x
            << " ref_y: " << std::setprecision(12) << m_psInitGnssEnu.pose.position.y 
            << " ref_z: " << std::setprecision(12) << m_psInitGnssEnu.pose.position.z << "\n" << std::endl;
        }
    
        // Visualization
        visualization_msgs::Marker gnss_marker;
        geometry_msgs::Point p;

        p.x = m_psInitGnssEnu.pose.position.x;
        p.y = m_psInitGnssEnu.pose.position.y;
        p.z = m_psInitGnssEnu.pose.position.z;

        gnss_marker.header.frame_id = "/map";
        gnss_marker.ns = "initialGnss";
        gnss_marker.id = 1;
        gnss_marker.type = visualization_msgs::Marker::POINTS;
        gnss_marker.scale.x = 10.0;
        gnss_marker.scale.y = 10.0;
        gnss_marker.color.r = 1.f;
        gnss_marker.color.g = 0.f;
        gnss_marker.color.b = 0.f;
        gnss_marker.color.a = 1.0;
        gnss_marker.color.b = 0.0;
        gnss_marker.lifetime = ros::Duration(100);

        gnss_marker.points.push_back(p);

        m_pub_gnss_points.publish(gnss_marker);
        usleep(100);

        ROS_INFO_STREAM("publish gnss initial point");

        m_bIsInitGNSS = true;
        return;
    }

    m_bGnssExistFlag = true;

    m_gnssGnss.timestamp = timestamp;
    m_gnssGnss.latitude = msg->latitude;
    m_gnssGnss.longitude = msg->longitude;
    m_gnssGnss.altitude = msg->altitude;

    m_psGnssEnu.pose.position = llh2enu(m_gnssGnss.latitude, m_gnssGnss.longitude, m_gnssGnss.altitude);

    // Visualization
    visualization_msgs::Marker gnss_marker;
    geometry_msgs::Point p;

    p.x = m_psGnssEnu.pose.position.x;
    p.y = m_psGnssEnu.pose.position.y;
    p.z = m_psGnssEnu.pose.position.z;

    gnss_marker.header.frame_id = "/map";
    gnss_marker.ns = "initialGnss";
    gnss_marker.id = 1;
    gnss_marker.type = visualization_msgs::Marker::POINTS;
    gnss_marker.scale.x = 1.2;
    gnss_marker.scale.y = 1.2;
    gnss_marker.color.r = 1.f;
    gnss_marker.color.g = 0.f;
    gnss_marker.color.b = 0.f;
    gnss_marker.color.a = 1.0;
    gnss_marker.color.b = 0.0;
    gnss_marker.lifetime = ros::Duration(100);

    gnss_marker.points.push_back(p);

    m_pub_gnss_points.publish(gnss_marker);
    // usleep(100);

}

void MapMatchingPF::CallBackImu(const sensor_msgs::Imu::ConstPtr &msg)
{

    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    static double init_timestamp = timestamp;
    static double prev_timestamp = timestamp;

    if(!m_bIsFirstIMUStep) 
    {
        double sample_time = (timestamp - prev_timestamp) / 1e6;

        if (sample_time <= Interval)
        {
            IMU temp_imu;
            temp_imu.linear_acceleration_x = msg->linear_acceleration.x;
            temp_imu.linear_acceleration_y = msg->linear_acceleration.y;
            temp_imu.linear_acceleration_z = msg->linear_acceleration.z;
            temp_imu.angular_velocity_x = msg->angular_velocity.x;
            temp_imu.angular_velocity_y = msg->angular_velocity.y;
            temp_imu.angular_velocity_z = msg->angular_velocity.z;
            temp_imu.initial_q[0] = msg->orientation.x;
            temp_imu.initial_q[1] = msg->orientation.y;
            temp_imu.initial_q[2] = msg->orientation.z;
            temp_imu.initial_q[3] = msg->orientation.w;

            m_imuImuSum.linear_acceleration_x += temp_imu.linear_acceleration_x;
            m_imuImuSum.linear_acceleration_y += temp_imu.linear_acceleration_y;
            m_imuImuSum.linear_acceleration_z += temp_imu.linear_acceleration_z;
            m_imuImuSum.angular_velocity_x += temp_imu.angular_velocity_x;
            m_imuImuSum.angular_velocity_y += temp_imu.angular_velocity_y;
            m_imuImuSum.angular_velocity_z += temp_imu.angular_velocity_z;
            m_imuImuSum.initial_q[0] += temp_imu.initial_q[0];
            m_imuImuSum.initial_q[1] += temp_imu.initial_q[1];
            m_imuImuSum.initial_q[2] += temp_imu.initial_q[2];
            m_imuImuSum.initial_q[3] += temp_imu.initial_q[3];


            m_iImuCount++;
            return;
        }

        m_imuInitImu.timestamp = timestamp;
        m_imuInitImu.linear_acceleration_x = m_imuImuSum.linear_acceleration_x / m_iImuCount;
        m_imuInitImu.linear_acceleration_y = m_imuImuSum.linear_acceleration_y / m_iImuCount;
        m_imuInitImu.linear_acceleration_z = m_imuImuSum.linear_acceleration_z / m_iImuCount;
        m_imuInitImu.angular_velocity_x = m_imuImuSum.angular_velocity_x / m_iImuCount;
        m_imuInitImu.angular_velocity_y = m_imuImuSum.angular_velocity_y / m_iImuCount;
        m_imuInitImu.angular_velocity_z = m_imuImuSum.angular_velocity_z / m_iImuCount;

        std::cout << "gyroscope bias x: " << std::setprecision(12) << m_imuInitImu.angular_velocity_x << ", y: " << std::setprecision(12) 
        <<  m_imuInitImu.angular_velocity_y << ", z: " << std::setprecision(12) << m_imuInitImu.angular_velocity_z << std::endl; 

        // reference 
        m_imuInitImu.initial_q[0] = m_imuImuSum.initial_q[0] / m_iImuCount;
        m_imuInitImu.initial_q[1] = m_imuImuSum.initial_q[1] / m_iImuCount;
        m_imuInitImu.initial_q[2] = m_imuImuSum.initial_q[2] / m_iImuCount;
        m_imuInitImu.initial_q[3] = m_imuImuSum.initial_q[3] / m_iImuCount;

        ROS_ERROR("m_imuInitImu.initial_q[0]: %lf" , m_imuInitImu.initial_q[0]);
        ROS_ERROR("m_imuInitImu.initial_q[1]: %lf" , m_imuInitImu.initial_q[1]);
        ROS_ERROR("m_imuInitImu.initial_q[2]: %lf" , m_imuInitImu.initial_q[2]);
        ROS_ERROR("m_imuInitImu.initial_q[3]: %lf" , m_imuInitImu.initial_q[3]);

        tf::Matrix3x3 m(m_imuInitImu.initial_q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        std::cout << "azimuth check: " << roll*RAD2DEG << ", " << pitch*RAD2DEG << ", " << yaw*RAD2DEG << std::endl;

        m_imuInitImu.linear_acceleration_x = msg->linear_acceleration.x;
        m_imuInitImu.linear_acceleration_y = msg->linear_acceleration.y;
        m_imuInitImu.linear_acceleration_z = msg->linear_acceleration.z;
        m_imuInitImu.angular_velocity_x = msg->angular_velocity.x;
        m_imuInitImu.angular_velocity_y = msg->angular_velocity.y;
        m_imuInitImu.angular_velocity_z = msg->angular_velocity.z;


        if(_DEBUG_MODE)
        {
            std::cout << "====================== [map_matching_pf] STATIONARY IMU DATA ======================\n"
            << "m_imuInitImu.linear_acceleration_x : " << std::setprecision(12) << m_imuInitImu.linear_acceleration_x << "\n" 
            << " m_imuInitImu.linear_acceleration_y : " << std::setprecision(12) << m_imuInitImu.linear_acceleration_y << "\n"
            << " m_imuInitImu.linear_acceleration_z : " << std::setprecision(12) << m_imuInitImu.linear_acceleration_z << "\n"
            << " m_imuInitImu.angular_velocity_x : " << std::setprecision(12) << m_imuInitImu.angular_velocity_x << "\n"
            << " m_imuInitImu.angular_velocity_y : " << std::setprecision(12) << m_imuInitImu.angular_velocity_y << "\n"
            << " m_imuInitImu.angular_velocity_z : " << std::setprecision(12) << m_imuInitImu.angular_velocity_z << "\n" << std::endl;
        }

        prev_timestamp = timestamp;        
        m_bIsFirstIMUStep = true;
        return;
    }

    m_bImuExistFlag = true;

    m_imuImu.timestamp = timestamp;
    m_imuImu.linear_acceleration_x = msg->linear_acceleration.x;
    m_imuImu.linear_acceleration_y = msg->linear_acceleration.y;
    m_imuImu.linear_acceleration_z = msg->linear_acceleration.z;
    m_imuImu.angular_velocity_x = msg->angular_velocity.x;
    m_imuImu.angular_velocity_y = msg->angular_velocity.y;
    m_imuImu.angular_velocity_z = msg->angular_velocity.z;

    prev_timestamp = timestamp;

}

void MapMatchingPF::CallBackImuMagnetometer(const sensor_msgs::MagneticField::ConstPtr &msg)
{

    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    static double prev_timestamp = timestamp;

    if(!m_bIsFirstMagnetometerStep)
    {
        double sample_time = (timestamp - prev_timestamp) / 1e6;

        if(sample_time <= Interval)
        {    
            IMU temp_imu;
            temp_imu.magnetic_field_x = msg->magnetic_field.x;
            temp_imu.magnetic_field_y = msg->magnetic_field.y;
            temp_imu.magnetic_field_z = msg->magnetic_field.z;

            m_imuImuSum.magnetic_field_x += temp_imu.magnetic_field_x;
            m_imuImuSum.magnetic_field_y += temp_imu.magnetic_field_y;
            m_imuImuSum.magnetic_field_z += temp_imu.magnetic_field_z;
            

            m_iMagnetometerCount++;
            return;
        }

        m_imuInitImu.magnetic_field_x = m_imuImuSum.magnetic_field_x / m_iMagnetometerCount;
        m_imuInitImu.magnetic_field_y = m_imuImuSum.magnetic_field_y / m_iMagnetometerCount;
        m_imuInitImu.magnetic_field_z = m_imuImuSum.magnetic_field_z / m_iMagnetometerCount;

        m_imuInitImu.magnetic_field_x = msg->magnetic_field.x;
        m_imuInitImu.magnetic_field_y = msg->magnetic_field.y;
        m_imuInitImu.magnetic_field_z = msg->magnetic_field.z;


        if(_DEBUG_MODE)
        {
            std::cout << "====================== [map_matching_pf] STATIONARY MAGNETOMETER DATA ======================\n"
            << "m_imuInitImu.magnetic_field_x: " << std::setprecision(12) << m_imuInitImu.magnetic_field_x << "\n"
            << "m_imuInitImu.magnetic_field_y: " << std::setprecision(12) << m_imuInitImu.magnetic_field_y << "\n"
            << "m_imuInitImu.magnetic_field_z: " << std::setprecision(12) << m_imuInitImu.magnetic_field_z << "\n" << std::endl;
        }

        m_bIsFirstMagnetometerStep = true;
        return;
    } 

    m_imuImu.magnetic_field_x = msg->magnetic_field.x;
    m_imuImu.magnetic_field_y = msg->magnetic_field.y;
    m_imuImu.magnetic_field_z = msg->magnetic_field.z;


}


void MapMatchingPF::Run()
{
    if(!m_bIsInitGNSS || !m_bIsFirstIMUStep || !m_bIsFirstMagnetometerStep || !m_bMapInit)
    {
        return;
    }

    if(m_bImuExistFlag)
    {
        auto t1 = std::chrono::high_resolution_clock::now();
        // Prediction CV Model
        PFPrediction();
        auto t2 = std::chrono::high_resolution_clock::now();

    }




}

void MapMatchingPF::PFPrediction()
{
    Eigen::Matrix<double, 6, 1> tmpInput;
    Eigen::Matrix<double, 16, 1> tmpState;
    Eigen::MatrixXd prestate(m_cfg_iNumParticle, m_iNumState);

    static double prevTimestamp = m_imuImu.timestamp;

    double dt = (m_imuImu.timestamp - prevTimestamp) / 1e6;
    prevTimestamp = m_imuImu.timestamp;

    if(_DEBUG_MODE)
    {
        std::cout << "====================== [map_matching_pf] PFPrediction start ======================" << std::endl;
    }

    for(int particleIdx = 0; particleIdx < m_cfg_iNumParticle; particleIdx++)
    {
        // Input
        const Eigen::Quaterniond q_nb(
            m_MatTimestampStatePF(particleIdx, 6),
            m_MatTimestampStatePF(particleIdx, 7),
            m_MatTimestampStatePF(particleIdx, 8),
            m_MatTimestampStatePF(particleIdx, 9)
        );

        // quaternion to euler angle
        const Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();
        // std::cout << "=================\n C_nb" << C_nb << std::endl;

        double gyro_bias_x = m_MatTimestampStatePF(particleIdx, 10);
        double gyro_bias_y = m_MatTimestampStatePF(particleIdx, 11);
        double gyro_bias_z = m_MatTimestampStatePF(particleIdx, 12);

        double accel_bias_x = m_MatTimestampStatePF(particleIdx, 13);
        double accel_bias_y = m_MatTimestampStatePF(particleIdx, 14);
        double accel_bias_z = m_MatTimestampStatePF(particleIdx, 15);

        // IMU Model
        // Raw data from accelerometer and gyroscope 
        // Gyroscope measures the angular velocities related to each axis with respect to the global frame
        tmpInput[0] = m_imuImu.linear_acceleration_x - accel_bias_x - GaussianRandomGenerator(0, m_dStdInputAccX_mss);
        tmpInput[1] = m_imuImu.linear_acceleration_y - accel_bias_y - GaussianRandomGenerator(0, m_dStdInputAccY_mss);
        tmpInput[2] = m_imuImu.linear_acceleration_z - accel_bias_z - GaussianRandomGenerator(0, m_dStdInputAccZ_mss);

        tmpInput.block<3,1>(0,0) = C_nb * tmpInput.block<3,1>(0,0) + m_VecGravityNav;
        // std::cout << "accelerometer in navigation frame: \n" << tmpInput.block<3,1>(0,0) << std::endl;

        // Gyroscope measures the angular velocities related to each axis with respect to the global frame
        tmpInput[3] = m_imuImu.angular_velocity_x - gyro_bias_x - GaussianRandomGenerator(0, m_dStdInputRollRate_degs * DEG2RAD);
        tmpInput[4] = m_imuImu.angular_velocity_y - gyro_bias_y - GaussianRandomGenerator(0, m_dStdInputPitchRate_degs * DEG2RAD);
        tmpInput[5] = m_imuImu.angular_velocity_z - gyro_bias_z - GaussianRandomGenerator(0, m_dStdInputYawRate_degs * DEG2RAD);

        tmpState = m_MatTimestampStatePF<1,16>(particleIdx, 0);

        PredictionCVModel(tmpInput, tmpState, dt);

    }

}

void MapMatchingPF::PredictionCVModel(Eigen::Matrix<double, 6, 1> tmpInput, Eigen::Matrix<double, 16, 1> tmpState, double dt)
{
    // Previous state
    Eigen::Vector3d prevPos_m = tmpState.block<3,1>(0,0);
    Eigen::Vector3d prevVel_ms = tmpState.block<3,1>(3,0);
    Eigen::Vector4d prevQuat = tmpState.block<4,1>(6,0);
    Eigen::Vector3d prevGyroBias_deg_s = tmpState.block<3,1>(10,0);
    Eigen::Vector3d prevAccelBias_mss = tmpState.block<3,1>(13,0);
   
    // quaternion to euler angle
    const Eigen::Matrix3d C_nb = q_nb.toRotationMatrix();


    // Prediction 
    Eigen::Vector3d currPos_m = prevPos_m + prevVel_ms * dt; 
    Eigen::Vector3d currVel_ms = prevVel_ms + tmpInput.block<3,1>(0,0) * dt;
    // Eigen::Vector4d currQuat = 
    Eigen::Vector3d currGyroBias_deg_s = prevGyroBias_deg_s;
    Eigen::Vector3d currAccelBias_mss = prevAccelBias_mss;

    // std 

}



// Utils
inline geometry_msgs::Point MapMatchingPF::llh2enu(double latitude, double longitude, double altitude)
{
    double dKappaLat = 0;
    double dKappaLon = 0;  

    // reference point is new engineering building in Konkuk Univ.
    double m_dRefLatitude_deg = 37.54071565952212;
    double m_dRefLongitude_deg = 127.07938078518049;

    altitude = m_dMapHeight;

    dKappaLat = FnKappaLat( m_dRefLatitude_deg , altitude );
    dKappaLon = FnKappaLon( m_dRefLatitude_deg , altitude );

    geometry_msgs::Point pose;

    pose.x = (longitude-m_dRefLongitude_deg)/dKappaLon;
    pose.y = (latitude-m_dRefLatitude_deg)/dKappaLat;
    pose.z = altitude;

    return (pose);

}

inline double MapMatchingPF::FnKappaLat(double dRef_Latitude, double dHeight)
{
	double dKappaLat = 0;
	double Denominator = 0;
	double dM = 0;

	// estimate the meridional radius
	Denominator = sqrt(1 - Geod_e2 * pow(sin(dRef_Latitude * DEG2RAD), 2));
	dM = Geod_a * (1 - Geod_e2) / pow(Denominator, 3);

	// Curvature for the meridian
	dKappaLat = 1 / (dM + dHeight) * RAD2DEG;

	return dKappaLat;
}

inline double MapMatchingPF::FnKappaLon(double dRef_Latitude, double dHeight)
{
	double dKappaLon = 0;
	double Denominator = 0;
	double dN = 0;

	// estimate the normal radius
	Denominator = sqrt(1 - Geod_e2 * pow(sin(dRef_Latitude * DEG2RAD), 2));
	dN = Geod_a / Denominator;

	// Curvature for the meridian
	dKappaLon = 1 / ((dN + dHeight) * cos(dRef_Latitude * DEG2RAD)) * RAD2DEG;

	return dKappaLon;
}

inline double MapMatchingPF::GaussianRandomGenerator(double dMean, double dStd)
{
	if (dStd > FLT_MIN)
	{
		std::normal_distribution<double> normalDistribution(dMean, dStd);
		return normalDistribution(m_RandGenerator);
	}
	else
	{
		return dMean;
	}
}

// inline 


inline Eigen::Quaterniond MapMatchingPF::euler2quat(double yaw, double pitch, double roll)
{
    // Convert Euler angle to quaternion 
    // Roatation sequence = zyx

    // halve the Euler angles
    double hpsi = 0.5 * yaw;
    double hth = 0.5 * pitch;
    double hphi = 0.5 * roll;

    // Precalculate the trigonometric values
    double hcpsi = cos(hpsi);
    double hspsi = sin(hpsi);
    double hcth = cos(hth);
    double hsth = sin(hth);
    double hcphi = cos(hphi);
    double hsphi = sin(hphi);

    Eigen::Quaterniond quat;
    quat.w() = hcphi * hcth * hcpsi + hsphi * hsth * hspsi;
    quat.x() = hsphi * hcth * hcpsi - hcphi * hsth * hspsi;
    quat.y() = hcphi * hsth * hcpsi + hsphi * hcth * hspsi;
    quat.z() = hcphi * hcth * hspsi - hsphi * hsth * hcpsi;

    return quat;

}

inline Eigen::Quaterniond MapMatchingPF::InitialOrientation()
{
    // Method 1. Calculate roll, pitch, yaw using IMU data
    double initial_roll_rad;
    double initial_pitch_rad;
    double initial_yaw_rad;
    
    double initial_roll_deg;
    double initial_pitch_deg;
    double initial_yaw_deg;

    // Roll: Rotation around the X-axis 
    //               y
    // roll = atan2(---)
    //               z 
    initial_roll_rad = atan2f(m_imuInitImu.linear_acceleration_y, m_imuInitImu.linear_acceleration_z);

    // pitch : Rotation around the Y-axis
    //               -x
    // pitch = atan(----)
    //                g
    //
    initial_pitch_rad = atanf(-m_imuInitImu.linear_acceleration_x / sqrt(pow(m_imuInitImu.linear_acceleration_x, 2) 
                        + pow(m_imuInitImu.linear_acceleration_y,2) + pow(m_imuInitImu.linear_acceleration_z, 2)));

    // yaw : Rotation around the Z-axis
    //                                     -m_y * cos(roll) + m_z * sin(roll)
    // heading = atan2(---------------------------------------------------------------------------------)
    //                  m_x * cos(pitch) + m_y * sin(pitch) * sin(roll) + m_z * cos(roll) * sin(pitch)
    //
    double mag_norm = sqrt(pow(m_imuInitImu.magnetic_field_x, 2) 
                    + pow(m_imuInitImu.magnetic_field_y, 2) 
                    + pow(m_imuInitImu.magnetic_field_z, 2));

    double mag_x = (m_imuInitImu.magnetic_field_x) / mag_norm;
    double mag_y = (m_imuInitImu.magnetic_field_y) / mag_norm;
    double mag_z = (m_imuInitImu.magnetic_field_z) / mag_norm;

    // Magnetometer heading
    initial_yaw_rad = atan2f(-mag_y * cos(initial_roll_rad) + mag_z * sin(initial_roll_rad)
                        , mag_x * cos(initial_pitch_rad) + mag_y * sin(initial_pitch_rad) * sin(initial_roll_rad) 
                        + mag_z * cos(initial_roll_rad) * sin(initial_pitch_rad));    

    // Magnetometer heading 
    // initial_yaw_rad = atan2f(-m_imuInitImu.magnetic_field_y * cos(initial_roll_rad) + m_imuInitImu.magnetic_field_z * sin(initial_roll_rad)
    //                 , m_imuInitImu.magnetic_field_x * cos(initial_pitch_rad) + m_imuInitImu.magnetic_field_y * sin(initial_roll_rad) * sin(initial_pitch_rad) 
    //                 + m_imuInitImu.magnetic_field_z * cos(initial_roll_rad) * sin(initial_pitch_rad));

    // true heading (true north) + declination 
    initial_yaw_rad = initial_yaw_rad + Magnetic_declination_rad + 90 * DEG2RAD;

    // Convert angular data to degree
    initial_roll_deg = initial_roll_rad * RAD2DEG;
    initial_pitch_deg = initial_pitch_rad * RAD2DEG;
    initial_yaw_deg = initial_yaw_rad * RAD2DEG;

    if(_DEBUG_MODE)
    {
        std::cout << "Initial Orientation:\n" << "initial roll[deg]: " << initial_roll_deg  
                                              << " initial pitch[deg]" << initial_pitch_deg
                                              << " initial yaw[deg]" << initial_yaw_deg << std::endl;
    }

    // Convert euler angle to quaternion
    Eigen::Quaterniond initial_quat;
    initial_quat = euler2quat(initial_yaw_rad, initial_pitch_rad, initial_roll_rad);

    std::cout << "initial_quat: " << initial_quat.coeffs() << std::endl;

    return initial_quat;


    // // Method 2. TRIAD
    // double body2Nav_east;
    // double body2Nav_north;
    // double body2Nav_up;

    // TRIAD(Tri-Axial Attitude Determination): estimate an attitude represented as a Direction Cosine Matrix from two orthogonal vector observations 
    // We identify two main reference vectors: gravity vector and magnetic field vector 
    // TRIAD(body2Nav_east, body2Nav_north, body2Nav_up);        // body frame 2 navigation frame 
}

void MapMatchingPF::TRIAD(double body2Nav_east, double body2Nav_north, double body2Nav_up)
{
    // Setup navigation frame vector
    double gravity_unit_vec_n[3] = {0. , 0. , 1.};          // v1N
    double magnetic_field_unit_vec_n[3] = {0 , 1. , 0.};        // v2N

    // Setup the measured attitude states
    double gravity_vec_b[3] = {0. , 0. , 0.};       
    double magnetic_field_vec_b[3] = {0. , 0. , 0.};        

    gravity_vec_b[0] = m_imuInitImu.linear_acceleration_x;
    gravity_vec_b[1] = m_imuInitImu.linear_acceleration_y;
    gravity_vec_b[2] = m_imuInitImu.linear_acceleration_z;

    magnetic_field_vec_b[0] = m_imuInitImu.magnetic_field_x - m_imuInitImu.magentic_field_x_bias;
    magnetic_field_vec_b[1] = m_imuInitImu.magnetic_field_y - m_imuInitImu.magentic_field_y_bias;
    magnetic_field_vec_b[2] = m_imuInitImu.magnetic_field_z - m_imuInitImu.magentic_field_z_bias;

    // normalize vector 
    double gravity_vec_norm = sqrt(gravity_vec_b[0]*gravity_vec_b[0] + gravity_vec_b[1] * gravity_vec_b[1] + gravity_vec_b[2] * gravity_vec_b[2]);
    double magnetic_field_vec_norm = sqrt(magnetic_field_vec_b[0] * magnetic_field_vec_b[0] + magnetic_field_vec_b[1] * magnetic_field_vec_b[1] 
                                    + magnetic_field_vec_b[2] * magnetic_field_vec_b[2]);

    double gravity_unit_vec_b[3] = {0. , 0. , 0.};      // v1B
    double magnetic_field_unit_vec_b[3] = {0. , 0. , 0.};       //v2B

    gravity_unit_vec_b[0] = gravity_vec_b[0] / gravity_vec_norm;
    gravity_unit_vec_b[1] = gravity_vec_b[1] / gravity_vec_norm;
    gravity_unit_vec_b[2] = gravity_vec_b[2] / gravity_vec_norm;

    magnetic_field_unit_vec_b[0] = magnetic_field_vec_b[0] / magnetic_field_vec_norm;
    magnetic_field_unit_vec_b[1] = magnetic_field_vec_b[1] / magnetic_field_vec_norm;
    magnetic_field_unit_vec_b[2] = magnetic_field_vec_b[2] / magnetic_field_vec_norm;

    // Develop Triad frame
    // Body frame Triad vectors
    double t1B[3] = {0. , 0. , 0.};
    double t2B[3] = {0. , 0. , 0.};
    double t3B[3] = {0. , 0. , 0.};

    t1B[0] = gravity_unit_vec_b[0];
    t1B[1] = gravity_unit_vec_b[1];
    t1B[2] = gravity_unit_vec_b[2];

    double cross_vec[3] = {0. , 0. , 0.};

    cross_vec[0] = gravity_unit_vec_b[1] * magnetic_field_unit_vec_b[2] - gravity_unit_vec_b[2] * magnetic_field_unit_vec_b[1]; 
    cross_vec[1] = gravity_unit_vec_b[2] * magnetic_field_unit_vec_b[0] - gravity_unit_vec_b[0] * magnetic_field_unit_vec_b[2];
    cross_vec[2] = gravity_unit_vec_b[0] * magnetic_field_unit_vec_b[1] - gravity_unit_vec_b[1] * magnetic_field_unit_vec_b[0];

    double cross_vec_norm = sqrt(cross_vec[0] * cross_vec[0] + cross_vec[1] * cross_vec[1] + cross_vec[2] * cross_vec[2]);
    
    t2B[0] = cross_vec[0] / cross_vec_norm;
    t2B[1] = cross_vec[1] / cross_vec_norm;
    t2B[2] = cross_vec[2] / cross_vec_norm;

    cross_vec[0] = t1B[1] * t2B[2] - t1B[2] * t2B[1]; 
    cross_vec[1] = t1B[2] * t2B[0] - t1B[0] * t2B[2];
    cross_vec[2] = t1B[0] * t2B[1] - t1B[1] * t2B[0];

    t3B[0] = cross_vec[0];
    t3B[1] = cross_vec[1];
    t3B[2] = cross_vec[2];

    if( _DEBUG_MODE)
    {
        std::cout << "t1B: " << t1B[0] << ", " << t1B[1] << ", " << t1B[2] << std::endl;
        std::cout << "t2B: " << t2B[0] << ", " << t2B[1] << ", " << t2B[2] << std::endl;
        std::cout << "t3B: " << t3B[0] << ", " << t3B[1] << ", " << t3B[2] << std::endl;
    }

    // Inertial Frame Triad vectors 
    double t1N[3] = {0. , 0. , 0.};
    double t2N[3] = {0. , 0. , 0.};
    double t3N[3] = {0. , 0. , 0.};

    t1N[0] = gravity_unit_vec_n[0];
    t1N[1] = gravity_unit_vec_n[1];
    t1N[2] = gravity_unit_vec_n[2];

    cross_vec[0] = gravity_unit_vec_n[1] * magnetic_field_unit_vec_n[2] - gravity_unit_vec_n[2] * magnetic_field_unit_vec_n[1]; 
    cross_vec[1] = gravity_unit_vec_n[2] * magnetic_field_unit_vec_n[0] - gravity_unit_vec_n[0] * magnetic_field_unit_vec_n[2];
    cross_vec[2] = gravity_unit_vec_n[0] * magnetic_field_unit_vec_n[1] - gravity_unit_vec_n[1] * magnetic_field_unit_vec_n[0];

    cross_vec_norm = sqrt(cross_vec[0] * cross_vec[0] + cross_vec[1] * cross_vec[1] + cross_vec[2] * cross_vec[2]);

    t2N[0] = cross_vec[0] / cross_vec_norm;
    t2N[1] = cross_vec[1] / cross_vec_norm;
    t2N[2] = cross_vec[2] / cross_vec_norm;

    cross_vec[0] = t1N[1] * t2N[2] - t1N[2] * t2N[1]; 
    cross_vec[1] = t1N[2] * t2N[0] - t1N[0] * t2N[2];
    cross_vec[2] = t1N[0] * t2N[1] - t1N[1] * t2N[0];

    t3N[0] = cross_vec[0];
    t3N[1] = cross_vec[1];
    t3N[2] = cross_vec[2];

    if( _DEBUG_MODE)
    {
        std::cout << "t1N: " << t1N[0] << ", " << t1N[1] << ", " << t1N[2] << std::endl;
        std::cout << "t2N: " << t2N[0] << ", " << t2N[1] << ", " << t2N[2] << std::endl;
        std::cout << "t3N: " << t3N[0] << ", " << t3N[1] << ", " << t3N[2] << std::endl;
    }

    // Find Estimated Attitude
    Eigen::Matrix3d matBody2Triad = Eigen::Matrix3d::Zero();
    Eigen::Matrix3d matNav2Triad = Eigen::Matrix3d::Zero();

    matBody2Triad << t1B[0], t2B[0], t3B[0],
                     t1B[1], t2B[1], t3B[1],
                     t1B[2], t2B[2], t3B[2];

    matNav2Triad << t1N[0], t2N[0], t3N[0],
                    t1N[1], t2N[1], t3N[1],
                    t1N[2], t2N[2], t3N[2];
    

    Eigen::Matrix3d matBody2Nav = Eigen::Matrix3d::Zero();
    matBody2Nav = matBody2Triad * matNav2Triad.transpose();

    double roll = atan2(matBody2Nav(1,2), matBody2Nav(2,2)) * RAD2DEG;
    double pitch = asin(-matBody2Nav(0,2)) * RAD2DEG;
    double yaw = atan2(matBody2Nav(0,1), matBody2Nav(0,0)) * RAD2DEG;

        // std::cout << "TRIAD roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;


    if(_DEBUG_MODE)
    {
        std::cout << "TRIAD roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << std::endl;
    }

}



int main(int argc, char **argv){
    
    ros::init(argc, argv, "map_matching_pf");

    MapMatchingPF MapMatchingPF;

    int loop_hz = 10;
    ros::Rate loop_rate(loop_hz);

    while(ros::ok())
    {
        MapMatchingPF.Run();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}