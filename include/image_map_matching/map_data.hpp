#ifndef __MAP_DATA_HPP__
#define __MAP_DATA_HPP__

#include <vector>

class Node 
{
    public:
        Node()
        {
            m_node_id = 0;
            m_latitude = 0;
            m_longitude = 0;
        }

        Node(unsigned int node_id, double latitude, double longitude)
        {
            m_node_id = node_id;
            m_latitude = latitude;
            m_longitude = longitude;
        }

        ~Node()
        {

        }

        // Set Map node data
        void SetId(unsigned int node_id) {m_node_id = node_id; }
        void SetLatitude(double latitude) { m_latitude = latitude; }
        void SetLongitude(double longitude) { m_longitude = longitude; }
        
        // Get Map node data
        unsigned int GetId() { return m_node_id; }
        double GetLatitude() { return m_latitude; }
        double GetLongitude() { return m_longitude; }
        double GetAltitude() { return m_altitude; }

    private:
        unsigned int m_node_id;
        double m_latitude;
        double m_longitude;
        double m_altitude = 83.5;       // [m]

};

class Way
{
    public:
        Way()
        {
            m_way_id = 0;
        }

        ~Way()
        {

        }

        // Set Map way data
        void SetId(unsigned int way_id) { m_way_id = way_id; }
        void SetNodeIds(unsigned int node_id) { m_vec_node_ids.push_back(node_id); }

        // Get Map way data
        unsigned int GetId() { return m_way_id; }
        std::vector<unsigned int> GetNodeIds() { return m_vec_node_ids; }
    

    private:
        unsigned int m_way_id;
        std::vector<unsigned int> m_vec_node_ids;

};


#endif