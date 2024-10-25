#include <cmath>
#include <math.h>
#include <iomanip>
#include <fstream>
#include "eigen/Eigen/Dense"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp/logging.hpp"
#include "dbw_msgs/msg/dbw.hpp"  
#include "vectornav_msgs/msg/common_group.hpp"
#include "cpp_pid/server.h"
#include "cpp_pid/pid.h"
#include "cpp_pid/environment.h"

using std::placeholders::_1;

// define the format you want, you only need one instance of this...
const static Eigen::IOFormat CSVFormat(Eigen::StreamPrecision, Eigen::DontAlignCols, ", ", "\n");

std::string getCurrentDateTime() {
    // Get current time
    std::time_t now = std::time(nullptr);
    std::tm* localTime = std::localtime(&now);

    // Create a string stream
    std::ostringstream oss;

    // Format the time and date
    oss << std::put_time(localTime, "%Y-%m-%d_%H-%M-%S");

    return oss.str();
}

template<class T>
void pushBack(T &mtrx, const double &x_position, const double &y_position, const double &theta, const double &input, const double &error) {
    auto n = mtrx.rows();
    mtrx.conservativeResize(n + 1, 9);
    mtrx(n, 0) = x_position;
    mtrx(n, 1) = y_position;
    mtrx(n, 2) = theta;
    mtrx(n, 3) = input;
    mtrx(n, 4) = error;
}

void writeToCSVfile(std::string name, Eigen::MatrixXd matrix)
{
    std::ofstream file(name.c_str());
    file << matrix.format(CSVFormat);
}

int main(int argc, char **argv)
{   
    double T = 50, dt = 1/T, max = 70, min = -70, Kp = 10, Ki = 2, Kd = 5;

    //make a server object that callsback odomentry information, an object to analysis the relevant waypoints, if the vehcile should stop, path errors fed into the controller, and a controller object
    auvsl::Server*     srvrObj = new auvsl::Server;
    auvsl::Environment* envObj = new auvsl::Environment;
    auvsl::PID*         pidObj = new auvsl::PID(dt, max, min, Kp, Ki, Kd);
    
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("fuzzyControl");

    // create publisher to topic /cmd_vel with queue size of 1 message
    auto cmdPub =  node->create_publisher<dbw_msgs::msg::Dbw>("/control_cmd", 1);

    // create publisher to topic /odometry/filtered which provides position and orientation information
    auto odomSub = node->create_subscription<vectornav_msgs::msg::CommonGroup>("/vectornav/raw/common", 1, std::bind(&auvsl::Server::odomCallback, srvrObj, std::placeholders::_1));
    
    // set the desired path 
    Eigen::MatrixXd path {{0,0}, {1, 0}, {2, 0}, {3, 0}};
    // Eigen::MatrixXd path {{0,0}, {1,  1}, {2,  2}, {3, 3}};
    //Eigen::MatrixXd path {{0,0}, {1, -1}, {2, -2}, {3, -3}};

    Eigen::MatrixXd dataXYWypts; 
  
    int pathcount = 0; int pathlength = path.rows(); double breakingDist0To1 = 0.0;

    rclcpp::Rate rate(T); // ROS Rate at 50Hz
    while (rclcpp::ok()){
        // setup the output message varables
        dbw_msgs::msg::Dbw msg2;

        // set a decrease throttle and steering and increase breaking on the last path segment using breakingDist0To1
        breakingDist0To1 = 0.0;
        if(srvrObj->odomPos(0) != 0.0) {
            // get the relavent waypoints
            auvsl::wyptsStopStruct waypointsStop = envObj->pathProgress(srvrObj->odomPos, path, pathcount, pathlength, breakingDist0To1); 

            // if the stop flag is set to false then use the set vehicle speed. Otherwise, set the velocities to 0 
            if ((waypointsStop.stop == false)) {
                // determine the position/ orientation inputs and store the velocity outputs of the controller
                double input = envObj->controllerInput(srvrObj->odomPos, waypointsStop.waypoints);
                double output = pidObj->calculate(input); 

                msg2.parkbrake = 0;               // parking break false
                msg2.gear      = 1;               // forward gear
                msg2.throttle  = 50;              // full throttle is 100
                msg2.steering  = output; // full steering is 100

                pushBack(dataXYWypts, srvrObj->odomPos(1), srvrObj->odomPos(2), srvrObj->odomPos(0), input, output);
            } else {
                msg2.parkbrake = 1;   // parking break false
                msg2.gear      = 0;   // neutral gear
                msg2.throttle  = 0.0; // zero pecent throttle (   0, 100)
                msg2.steering  = 0.0; // zero pecent steering (-100, 100)
            }
            RCLCPP_INFO_STREAM(node->get_logger(),  srvrObj->odomPos(0) << ", "  <<  srvrObj->odomPos(1) << ", " << srvrObj->odomPos(2) << path(0,0) << ", " << path(0,1) << ", " << path(1,0) << ", " << path(1,1) << ", " << path(2,0) << ", " << path(2,1) << ", " << path(3,0) << ", " << path(3,1));
           
        } else{
            msg2.parkbrake = 1;   // parking break false
            msg2.gear      = 0;   // neutral gear
            msg2.throttle  = 0.0; // zero pecent throttle (   0, 100)
            msg2.steering  = 0.0; // zero pecent steering (-100, 100)
        }

        RCLCPP_INFO_STREAM(node->get_logger(), " prk, gr, thtl, str" << ", " << msg2.parkbrake   << ", " << msg2.gear << ", " << msg2.throttle << ", " << msg2.steering);
        
        // send velocity command to the cmd_vel topic
        cmdPub->publish(msg2); 

        rclcpp::spin_some(node); // process all callbacks waiting to be called in time. 
        rate.sleep();            // sleep to achieve the set rate (50hz)
    }
    
    // date and time used to name file where data saved
    std::string dateTime = getCurrentDateTime();
    
    // save data
    writeToCSVfile(dateTime, dataXYWypts);
    
    delete srvrObj;
    delete envObj;
    delete pidObj;
  
    return 0;    
}