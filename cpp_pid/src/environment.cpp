#include "cpp_pid/environment.h"

namespace auvsl
{
    // constructor definition
    Environment::Environment()
    {
    }

    // destructor definition
    Environment::~Environment()
    {
    }

    // wrap to angle in between +/-pi
    double Environment::angSat(double angle_in){
        if ((angle_in < -M_PI) or (M_PI < angle_in)) {
            int sgn = (angle_in > 0) - (angle_in < 0);
            return std::fmod(angle_in + sgn * M_PI, M_PI) - sgn* M_PI;
        } else {
            return angle_in;
        }
    }

  // orentient the path in line with the vehicle's initial orientation 
    void Environment::initRotation(Eigen::VectorXd odomPos, Eigen::MatrixXd &path){
        // unpack the position and orentiation odometry information
        double theta = Environment::angSat(odomPos[0]);
        double x     = odomPos[1];
        double y     = odomPos[2];

        // position matrix
        Eigen::Matrix3d T {
            {1, 0, x},
            {0, 1, y},
            {0, 0, 1}
        };

        // rotational matrix
        Eigen::Matrix3d R {
            {std::cos(theta),  -std::sin(theta), 0},
            {std::sin(theta),   std::cos(theta), 0},
            {               0,                0, 1}
        };

        // make a 3x3 transformation matrix
        Eigen::Matrix3d transform = T * R;

        // store the inverse tranform for plotting/ analysis purposes
        inverseTransform = transform.inverse();

        // add an extra ones to the end of the path to make it n x 3 ... and thus can interact with the transformation matrix
        path.conservativeResize(path.rows(), 3);
        for (int i = 0; i < path.rows(); i++) {path(i, 2) = 1;}

        // transform the path to global coordinates
        path = (transform * path.transpose()).transpose();

        // drop the ones two steps back so the path is n x 2 again
        path.conservativeResize(path.rows(), 2);
    }

    // determine which of the waypoints are of interest and whether the vehicle needs to stop
    wyptsStopStruct Environment::pathProgress(Eigen::VectorXd odomPos, Eigen::MatrixXd &path, int &pathcount, int pathlength, double &breakingDist0To1){
        // setup variables associated with the output
        wyptsStopStruct output;
        Eigen::Vector<double, 6> waypoints;
        bool stop;

        // if this is the first time entering this function...
        if (pathcount == 0) {
            // increment the count so the if statement isn't entered again
            pathcount = 1;

            // orient the path so the vehicle's starting postion is effectively theta = 0, x = 0, and y = 0.
            Environment::initRotation(odomPos, path);

            // get the vector point from the second to last point to the last point in the path
            double diffX = path(pathlength - 1, 0) - path(pathlength-2, 0);
            double diffY = path(pathlength - 1, 1) - path(pathlength-2, 1);
            Eigen::Vector2d diff{{diffX, diffY}};
            
            // make that differnce vector into a unit vector (same direction but length of one)
            Eigen::Vector2d unit_diff = diff/ diff.norm();

            // make the path array larger by two entries
            path.conservativeResize(pathlength + 2, 2);

            // add two points beyond the end of the path so the Environment don't jitter when the vehicle goes past the "last" point but the stop command has not registed yet
            // multiplying by 1 and 2 was an arbitary choice. Feel free to make these number bigger if you have a larger platform.
            path(pathlength  , 0) = path(pathlength-1, 0) + unit_diff(0);
            path(pathlength  , 1) = path(pathlength-1, 1) + unit_diff(1);
            path(pathlength+1, 0) = path(pathlength-1, 0) + unit_diff(0) * 2;
            path(pathlength+1, 1) = path(pathlength-1, 1) + unit_diff(1) * 2;
        }

        // load in the current position, the last waypoint passed, and the current waypoint being targeted
        Eigen::Vector2d currentPos(odomPos(1), odomPos(2));
        Eigen::Vector2d    past1(path(pathcount - 1, 0), path(pathcount - 1, 1));
        Eigen::Vector2d present1(path(pathcount    , 0), path(pathcount    , 1));

        // make an 'a' and 'b' vector to see how far along the vehicle is along the path segment
        Eigen::Vector2d a = currentPos - past1;
        Eigen::Vector2d b =   present1 - past1;
    
        // the scalar projection length of the vehicle onto the path segment
        double projLen = a.dot(b/b.norm()) / b.norm();

        // if true the vehicle is past the target waypoint. So, increment the path count
        if(projLen > 1) { 
            pathcount = std::min(pathcount + 1, pathlength);
        }

        // a vehicle speed slow down proportional to the distance from the end of the path on the last segment
        if((pathcount+1) == pathlength) {
            breakingDist0To1 = std::min(std::max(projLen, 0.0), 0.80);
        }

        // if the path length, established before extending the path, is the same as the path count the vehicle is at/past the "last" waypoint and should stop 
        stop = false;                 
        if(pathcount == pathlength) {
            stop = true;             
        }

        // update the last waypoint the vehicle past, the target waypoint, and the next target waypoint 
        Eigen::Vector2d    past2(path(pathcount - 1, 0), path(pathcount - 1, 1));
        Eigen::Vector2d present2(path(pathcount    , 0), path(pathcount    , 1));
        Eigen::Vector2d  future2(path(pathcount + 1, 0), path(pathcount + 1, 1));
    
        waypoints(0) =    past2(0); waypoints(1) =   past2(1); waypoints(2) = present2(0); 
        waypoints(3) = present2(1); waypoints(4) = future2(0); waypoints(5) =  future2(1);   

        // update and return the output variable
        output.stop      = stop;
        output.waypoints = waypoints;

        return output;
    }

    // generate inputs to Environment based on the vehicle's postion relative to relavent waypoints 
    double Environment::controllerInput(Eigen::VectorXd odomPos, Eigen::VectorXd waypoints){
        // load in the current position, the last waypoint passed, and the current waypoint being targeted, and the next waypoint to be targeted
        double currentAng = Environment::angSat(odomPos(0));
        Eigen::Vector2d past{   waypoints(0), waypoints(1)};
        Eigen::Vector2d present{waypoints(2), waypoints(3)};

        double th1 = std::atan2(  present(1) -    past(1), present(0) -    past(0));

        // determine the vehicle's orientation difference relative the the current path segement and next path segment
        double error = Environment::angSat(th1-currentAng);

        return error;
    }
}