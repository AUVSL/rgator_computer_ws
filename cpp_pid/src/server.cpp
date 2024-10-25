#include "auvsl_motion_controller/server.h"
#include <iomanip>
#include <fstream>
#include "eigen/Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"

// hi
namespace auvsl
{
    // constructor definition
    Server::Server()
    {
    }

    // destructor definition
    Server::~Server()
    {
    }
    
    // store the global yaw angle, x position, and y position off the vehicle
    void Server::odomCallback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg1)
    {
        odomPos(0) = -(msg1->yawpitchroll.x) * M_PI/180.0; // convert degrees to radians
        double lat  = (msg1->position.x) * M_PI/180.0;
        double lon  = (msg1->position.y) * M_PI/180.0;
        double alt  = msg1->position.z;
        
        double lat0 = 40.11383423222608 * M_PI/180.0;
        double lon0 = -88.30940375075072 * M_PI/180.0;
        double alt0 = 0.0;
        
        double throwAway = 0.0;

        // compute the north east down position in meters from the latitude, longitude, and altitude in degrees
        cppmap3d::geodetic2enu(lat, lon, alt, lat0, lon0, alt0, odomPos(1), odomPos(2), throwAway);
        
        odomPos(0) -= 90;

    }	
}