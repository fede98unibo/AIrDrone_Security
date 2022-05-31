#include <cmath>
#include <functional>
#include <unistd.h>
#include <memory>
#include <thread>
#include <chrono>
#include <iostream>
#include <stdint.h>
#include <math.h>

#include <geometry_msgs/msg/point.hpp>

using Pnt = geometry_msgs::msg::Point;
using namespace std;

class TrajectoryGenerator
{
    public:

    bool cubic_poly_xyz(const Pnt& source,const Pnt& speed_init,const Pnt& destination,array<array<double,4>,3>& coefficents, double mean_speed = 0.5)
    {
        const double MEAN_SPEED = mean_speed; // [m/s]
        double T;

        T = abs(source.x - destination.x)/MEAN_SPEED + 2.0;

        coefficents[0][0] = source.x;
        coefficents[0][1] = speed_init.x;
        coefficents[0][2] = (-3*(source.x-destination.x) - 2*speed_init.x*T)/pow(T,2);
        coefficents[0][3] = (2*(source.x-destination.x) + speed_init.x*T)/pow(T,3);
        
        T = abs(source.y + destination.y)/MEAN_SPEED + 2.0;

        coefficents[1][0] = source.y;
        coefficents[1][1] = speed_init.y;
        coefficents[1][2] = (-3*(source.y-destination.y) - 2*speed_init.y*T)/pow(T,2);
        coefficents[1][3] = (2*(source.y -destination.y) + speed_init.y*T)/pow(T,3);

        T = abs(source.z - destination.z)/MEAN_SPEED + 2.0;

        coefficents[2][0] = source.z;
        coefficents[2][1] = speed_init.z;
        coefficents[2][2] = (-3*(source.z-destination.z) - 2*speed_init.z*T)/pow(T,2);
        coefficents[2][3] = (2*(source.z-destination.z) + speed_init.z*T)/pow(T,3);
        
        return true;

    } 

};

// int main(int argc, char** argv)
// {

//     auto TJ = std::make_shared<TrajectoryGenerator>();

//     array<array<double,4>,3> cubic_poly;
//     array<array<double,4>,3> cubic_poly2;
//     Pnt speed;
//     speed.x = 1.0;
//     speed.y = 0.0;
//     speed.z = 4.0;
//     Pnt dest;
//     dest.x = 10.0;
//     dest.y = 10.0;
//     dest.z = 10.0;

//     Pnt speed2;
//     speed2.x = 0.0;
//     speed2.y = 0.0;
//     speed2.z = 0.0;

//     Pnt dest2;
//     dest.x = 0.0;
//     dest.y = 0.0;
//     dest.z = 0.0;

//     TJ->cubic_poly_xyz(speed,dest,cubic_poly);
//     TJ->cubic_poly_xyz(speed2,dest2,cubic_poly2);

//     double tt = 0.0;

//     while(tt<10.0)
//     {   
//         if
//         double x = cubic_poly[0][0]+cubic_poly[0][1]*tt+cubic_poly[0][2]*pow(tt,2)+cubic_poly[0][3]*pow(tt,3);
//         double y = cubic_poly[1][0]+cubic_poly[1][1]*tt+cubic_poly[1][2]*pow(tt,2)+cubic_poly[1][3]*pow(tt,3);
//         double z = cubic_poly[2][0]+cubic_poly[2][1]*tt+cubic_poly[2][2]*pow(tt,2)+cubic_poly[2][3]*pow(tt,3);

//         tt += 0.1;

//         cout << "tarjectory: " << x <<","<<y<<","<<z<< endl; 
//     }

//     return 0;
// }