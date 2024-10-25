#ifndef ENVIRONMENT_H
#define ENVIRONMENT_H

#include <math.h>
#include "eigen/Eigen/Dense"

namespace auvsl
{
    struct wyptsStopStruct {
        Eigen::Vector<double, 6> waypoints;
        bool stop;
    };
    class Environment {
        public:
            Environment();
            ~Environment();
            Eigen::Matrix3d inverseTransform;
            double          angSat(         double angle_in);
            void            initRotation(   Eigen::VectorXd odomPos, Eigen::MatrixXd &path);
            double          controllerInput(Eigen::VectorXd odomPos, Eigen::VectorXd waypoints);
            wyptsStopStruct pathProgress(   Eigen::VectorXd odomPos, Eigen::MatrixXd &path, int &pathcount, int pathlength, double &breakingDist0To1);
    };
}

#endif