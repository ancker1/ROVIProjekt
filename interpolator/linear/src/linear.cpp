#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <fstream>


/*const int N = 2;
rw::math::VectorND<N,float> cubicSpline(float t, float ts, float tf, rw::math::VectorND<N,float> Ps, rw::math::VectorND<N,float> Pf,  rw::math::VectorND<N,float> Vs, rw::math::VectorND<N,float> Vf)
{ //M N-dimension points and velocity profile as input
    rw::math::VectorND<N,float> C = (-2*(Pf-Ps) + (tf-ts)*(Vs+Vf))*pow((t-ts)/(tf-ts),3)
                                   +(3*(Pf-Ps)-(tf-ts)*(2*Vs+Vf))*pow((t-ts)/(tf-ts),2)
                                   +Vs*(t-ts)+Ps;
    return C;
}*/

std::tuple<std::vector<rw::math::Transform3D<>>,std::vector<float>> getPointTimes()
{   // Read/define frames...
    rw::math::Transform3D<> Grasp;
    float time0 = 0.0f;

    rw::math::Transform3D<> GraspTranslatedUpZ;
    float time1 = 1.0f;

    rw::math::Transform3D<> IntermediateP1;
    float time2 = 2.0f;
    rw::math::Transform3D<> IntermediateP2;
    float time3 = 3.0f;

    rw::math::Transform3D<> PlaceTranslatedUpZ;
    float time4 = 4.0f;
    rw::math::Transform3D<> Place;
    float time5 = 5.0f;

    std::vector<rw::math::Transform3D<>> points = {Grasp, GraspTranslatedUpZ, IntermediateP1, IntermediateP2, PlaceTranslatedUpZ, Place};
    std::vector<float> times = {time0, time1, time2, time3, time4, time5};

    return std::make_tuple(points, times);
}

rw::math::Transform3D<> linerInterpolator(float t, float ts, float tf, rw::math::Transform3D<> P1, rw::math::Transform3D<> P2)
{   // t: time,  ts: time start,   tf: time finish,   P1: start point,    P2: end point
    rw::math::Transform3D<> P;
    // Something.....
    return P;
}

std::vector<rw::math::Transform3D<>> linearInterpolator(std::vector<rw::math::Transform3D<>> Points, std::vector<float> Times)
{
    std::vector<rw::math::Transform3D<>> tcpPath;

    // Something.....

    return tcpPath;
}

int main(int argc, char** argv) {

   /**** Define 6 frames for linear interpolator ****/
    auto PointTimes = getPointTimes();
    std::vector<rw::math::Transform3D<>> Points = std::get<0>(PointTimes);
    std::vector<float> Times = std::get<1>(PointTimes);

 
   linearInterpolate(t, Points, times);


   /**** Define 6 frames for linear interpolator ****/
    float ts = 0;
    float tf = 1;
    rw::math::VectorND<N,float> P1;
    P1[0] = 2;
    P1[1] = 2;
    rw::math::VectorND<N,float> P2;
    P2[0] = 6;
    P2[1] = 4;
    rw::math::VectorND<N,float> P3;
    P3[0] = 15;
    P3[1] = 12;
    rw::math::VectorND<N,float> V1;
    V1[0] = 1.0;
    V1[1] = 0.1;
    rw::math::VectorND<N,float> V2;
    V2[0] = 0.0;
    V2[1] = 5.0;
    rw::math::VectorND<N,float> V3;
    V3[0] = 2.0;
    V3[1] = 2.0;

    ofstream outfile;
    outfile.open("/home/emil/Dropbox/UNI/MSc/Robotics/exercises/lab9Data.txt");
    for (float t = 0; t <= 1; t += 0.1)
    {
        rw::math::VectorND<N,float> Xi = cubicSpline(t, ts, tf, P1, P2, V1, V2);
        outfile << Xi[0] << " " << Xi[1] << std::endl;
    }
    for (float t = 0; t <= 1; t += 0.1)
    {
        rw::math::VectorND<N,float> Xi = cubicSpline(t, ts, tf, P2, P3, V2, V3);
        outfile << Xi[0] << " " << Xi[1] << std::endl;
    }
    //std::cout << Xi << std::endl;
    outfile.close();
	return 0;

    // Plot of results: Dropbox/Uni/Msc/Robotics/MATLAB/lecture9.m
}
