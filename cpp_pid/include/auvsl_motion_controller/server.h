#ifndef SERVER_H
#define SERVER_H

#include <math.h>
#include "cppmap3d.hh"
#include "eigen/Eigen/Dense"
#include "vectornav_msgs/msg/common_group.hpp"

namespace auvsl
{
    // a class for recording odometry information and calbacks more broadly
    class Server {
        public:
            Server();
            ~Server();
            Eigen::Vector<double, 3> odomPos;
            void odomCallback(const vectornav_msgs::msg::CommonGroup::SharedPtr msg1);
    };
}
#endif