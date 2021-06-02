#include "image_map_matching/map_matching_pf.hpp"

#define __APP_NAME__ "map_matching_pf"

MapMatchingPF::MapMatchingPF()
:m_bIsInit(false), m_bIsInitGNSS(false), m_bIsFirstIMUStep(false), m_bIsFirstMagnetometerStep(false), m_bMapInit(false),
m_bGnssExistFlag(false), m_bImuExistFlag(false),
m_dMapHeight(7.0)
{
    GetParameter();
    int buffer_size = 1;
   
    // Subscriber
    m_sub_gnss = nh.subscribe("/mavros/global_position/global", buffer_size, &MapMatchingPF::CallBackGnss, this);
    m_sub_imu = nh.subscribe("/mavros/imu/data_raw", buffer_size, &MapMatchingPF::CallBackImu, this);
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
    nh.getParam("map_matching_pf/m_dStdEast", m_dStdEast);
    nh.getParam("map_matching_pf/m_dStdNorth", m_dStdNorth);
    nh.getParam("map_matching_pf/m_dStdUp", m_dStdUp);

    if(_DEBUG_MODE)
    {
        std::cout << "====================== [map_matching_pf] GetParameter start ======================" << std::endl;
        ROS_INFO("[%s] DEBUG_MODE: %d", __APP_NAME__, _DEBUG_MODE);
        ROS_INFO("[%s] m_MapNodePath: %s", __APP_NAME__, m_MapNodePath.c_str());
        ROS_INFO("[%s] m_MapWayPath: %s", __APP_NAME__, m_MapWayPath.c_str());
        ROS_INFO("[%s] m_cfg_iNumParticle: %d", __APP_NAME__, m_cfg_iNumParticle);
        ROS_INFO("[%s] m_cfg_iNumState: %d", __APP_NAME__, m_iNumState);
        ROS_INFO("[%s] m_cfg_iNumState: %lf", __APP_NAME__, m_dStdEast);
        ROS_INFO("[%s] m_cfg_iNumState: %lf", __APP_NAME__, m_dStdNorth);
        ROS_INFO("[%s] m_cfg_iNumState: %lf\n", __APP_NAME__, m_dStdUp);
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
    
    Eigen::MatrixXd initState(m_cfg_iNumParticle, m_iNumState);

    // GPS based initialization
    for(int particleIdx = 0; particleIdx < m_cfg_iNumParticle; particleIdx++)
    {
        // Position
        initState(particleIdx,0) = m_psInitGnssEnu.pose.position.x;
        initState(particleIdx,1) = m_psInitGnssEnu.pose.position.y;
        initState(particleIdx,2) = m_psInitGnssEnu.pose.position.z;
  
        // Velocity
        initState(particleIdx,3) = 0.0;
        initState(particleIdx,4) = 0.0;
        initState(particleIdx,5) = 0.0;
        
        // Orientation 
        



        m_MatParticleWeight(particleIdx) = 1. / (double)m_cfg_iNumParticle;
    }

}



void MapMatchingPF::CallBackGnss(const sensor_msgs::NavSatFix::ConstPtr &msg)
{ 
    
    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
    static double prev_timestamp = timestamp;

    if(!m_bIsInitGNSS)
    {
        double sample_time = (timestamp - prev_timestamp) / 1e6;
        
        // average of GNSS 
        if (sample_time <= Interval)
        {
            GNSS temp_gnss;
            temp_gnss.latitude = msg->latitude;
            temp_gnss.longitude = msg->longitude;
            temp_gnss.altitude = msg->altitude;
            temp_gnss.latitude_std = msg->position_covariance[0];
            temp_gnss.longitude_std = msg->position_covariance[4];
            temp_gnss.altitude_std = msg->position_covariance[8];


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

        m_psInitGnssEnu.pose.position = llh2enu(m_gnssInitGnss.latitude, m_gnssInitGnss.longitude, m_gnssInitGnss.altitude);
        
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
    
        // // Visualization
        // visualization_msgs::Marker gnss_marker;
        // geometry_msgs::Point p;

        // p.x = m_psInitGnssEnu.pose.position.x;
        // p.y = m_psInitGnssEnu.pose.position.y;
        // p.z = m_psInitGnssEnu.pose.position.z;

        // gnss_marker.header.frame_id = "/map";
        // gnss_marker.ns = "initialGnss";
        // gnss_marker.id = 1;
        // gnss_marker.type = visualization_msgs::Marker::POINTS;
        // gnss_marker.scale.x = 1.2;
        // gnss_marker.scale.y = 1.2;
        // gnss_marker.color.r = 1.f;
        // gnss_marker.color.g = 0.f;
        // gnss_marker.color.b = 0.f;
        // gnss_marker.color.a = 1.0;
        // gnss_marker.color.b = 0.0;
        // gnss_marker.lifetime = ros::Duration(100);

        // gnss_marker.points.push_back(p);

        // m_pub_gnss_points.publish(gnss_marker);
        // usleep(100);

        // ROS_INFO_STREAM("publish gnss initial point");

        // ParticleInit(m_psInitGnssEnu);

        m_bIsInitGNSS = true;
        return;
    }

    m_bGnssExistFlag = true;

    m_gnssGnss.timestamp = timestamp;
    m_gnssGnss.latitude = msg->latitude;
    m_gnssGnss.longitude = msg->longitude;
    m_gnssGnss.altitude = msg->altitude;

    m_psGnssEnu.pose.position = llh2enu(m_gnssGnss.latitude, m_gnssGnss.longitude, m_gnssGnss.altitude);

    // // Visualization
    // visualization_msgs::Marker gnss_marker;
    // geometry_msgs::Point p;

    // p.x = m_psGnssEnu.pose.position.x;
    // p.y = m_psGnssEnu.pose.position.y;
    // p.z = m_psGnssEnu.pose.position.z;

    // gnss_marker.header.frame_id = "/map";
    // gnss_marker.ns = "initialGnss";
    // gnss_marker.id = 1;
    // gnss_marker.type = visualization_msgs::Marker::POINTS;
    // gnss_marker.scale.x = 1.2;
    // gnss_marker.scale.y = 1.2;
    // gnss_marker.color.r = 1.f;
    // gnss_marker.color.g = 0.f;
    // gnss_marker.color.b = 0.f;
    // gnss_marker.color.a = 1.0;
    // gnss_marker.color.b = 0.0;
    // gnss_marker.lifetime = ros::Duration(100);

    // gnss_marker.points.push_back(p);

    // m_pub_gnss_points.publish(gnss_marker);
    // usleep(100);

}

void MapMatchingPF::CallBackImu(const sensor_msgs::Imu::ConstPtr &msg)
{

    double timestamp = (double)msg->header.stamp.sec * 1e6 + (double)msg->header.stamp.nsec / 1e3;
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

            m_imuImuSum.linear_acceleration_x += temp_imu.linear_acceleration_x;
            m_imuImuSum.linear_acceleration_y += temp_imu.linear_acceleration_y;
            m_imuImuSum.linear_acceleration_z += temp_imu.linear_acceleration_z;
            m_imuImuSum.angular_velocity_x += temp_imu.angular_velocity_x;
            m_imuImuSum.angular_velocity_y += temp_imu.angular_velocity_y;
            m_imuImuSum.angular_velocity_z += temp_imu.angular_velocity_z;


            m_iImuCount++;
            return;
        }

        m_imuInitImu.timestamp = timestamp;
        m_imuInitImu.linear_acceleration_x = m_imuImuSum.linear_acceleration_x / m_iImuCount;
        m_imuInitImu.linear_acceleration_y = m_imuImuSum.linear_acceleration_y / m_iImuCount;
        m_imuInitImu.linear_acceleration_z = m_imuImuSum.linear_acceleration_z / m_iImuCount;
        m_imuInitImu.angular_velocity_x = m_imuInitImu.angular_velocity_x / m_iImuCount;
        m_imuInitImu.angular_velocity_y = m_imuInitImu.angular_velocity_y / m_iImuCount;
        m_imuInitImu.angular_velocity_z = m_imuInitImu.angular_velocity_z / m_iImuCount;

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

        m_bIsFirstIMUStep = true;
        return;
    }

    // m_bImuExistFlag = true;
    // m_imuImu.timestamp = timestamp;
    // m_imuImu.linear_acceleration_x = msg->linear_acceleration.x;
    // m_imuImu.linear_acceleration_y = msg->linear_acceleration.y;
    // m_imuImu.linear_acceleration_z = msg->linear_acceleration.z;
    // m_imuImu.angular_velocity_x = msg->angular_velocity.x;
    // m_imuImu.angular_velocity_y = msg->angular_velocity.y;
    // m_imuImu.angular_velocity_z = msg->angular_velocity.z;

    // if(_DEBUG_MODE)
    // {
    //     std::cout << "====================== IMU ======================\n" 
    //     << "m"
         
    // }

    // prev_timestamp = timestamp;

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

    // m_imuImu.magnetic_field_x = msg->magnetic_field.x;
    // m_imuImu.magnetic_field_y = msg->magnetic_field.y;
    // m_imuImu.magnetic_field_z = msg->magnetic_field.z;


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



int main(int argc, char **argv){
    ros::init(argc, argv, "map_matching_pf");

    MapMatchingPF MapMatchingPF;

    ros::spin();

    return 0;
}