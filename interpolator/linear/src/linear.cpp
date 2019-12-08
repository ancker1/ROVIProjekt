#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <fstream>

#include "interpolator.hpp"


/*const int N = 2;
rw::math::VectorND<N,float> cubicSpline(float t, float ts, float tf, rw::math::VectorND<N,float> Ps, rw::math::VectorND<N,float> Pf,  rw::math::VectorND<N,float> Vs, rw::math::VectorND<N,float> Vf)
{ //M N-dimension points and velocity profile as input
    rw::math::VectorND<N,float> C = (-2*(Pf-Ps) + (tf-ts)*(Vs+Vf))*pow((t-ts)/(tf-ts),3)
                                   +(3*(Pf-Ps)-(tf-ts)*(2*Vs+Vf))*pow((t-ts)/(tf-ts),2)
                                   +Vs*(t-ts)+Ps;
    return C;
}*/


int main(int argc, char** argv) {

   /**** Define 6 frames for linear interpolator ****/
    static const std::string wcPath = "/home/emil/Dropbox/UNI/MSc/ROVIProjekt/ROVIProjekt/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcPath);
    if ( wc.isNull() )
    {
        RW_THROW("Error loading workcell");
        return -1;
    }
    // Loading tool frame
    rw::kinematics::Frame *toolFrame = wc->findFrame<rw::kinematics::Frame>("GraspTCP");
    if ( toolFrame == nullptr ){
        RW_THROW("Error finding frame: Tool");
        return -1;
    }
    //Load Cylinder Frame
    rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
    if ( cylinderFrame == nullptr ){
        RW_THROW("Error finding frame: Cylinder");
        return -1;
    }
    // Loading target
    rw::kinematics::MovableFrame *targetFrame = wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");
    if ( targetFrame == nullptr ){
        RW_THROW("Error finding frame: GraspTarget");
        return -1;
    }
    // Loading UR
    const std::string deviceName = "UR-6-85-5-A";
    rw::models::SerialDevice::Ptr UR6 =wc->findDevice<rw::models::SerialDevice>(deviceName);
    if ( UR6 == nullptr ){
        RW_THROW("Device UR6 not found.");
        return -1;
    }
    // Get state
    rw::kinematics::State state = wc->getDefaultState();
    rw::math::Transform3D<> T_Table_URbase = rw::math::Transform3D<>(rw::math::Vector3D<>(-0.0081, -0.0403, 0.110));
    rw::math::Vector3D<> placePos(0.30, -0.50, 0.150);
    //rw::math::RPY<> placeRPY(-90, 0, 180);
    rw::math::Transform3D<> placeFrame(placePos, cylinderFrame->getTransform(state).R());



    // Move Cylinder Frame for different pick locations
    cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.250, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.000, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.250, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);

    auto PointTimes = interpolator::util::getPointTimes(cylinderFrame->getTransform(state), UR6->baseTend(state), placeFrame);
    std::vector<rw::math::Transform3D<>> Points = std::get<0>(PointTimes);
    std::vector<float> Times = std::get<1>(PointTimes);

    float t = 0;

    // Testing
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::math::Transform3D<> T_World_Table = rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.1));

    std::vector<rw::math::Q> configurationPath;
    std::vector<rw::math::Transform3D<>> Tfs = Points;
    for ( unsigned int i = 0; i < Tfs.size(); i++ )
        Tfs[i] = T_World_Table*Tfs[i];

    for ( rw::math::Transform3D<> Tf : Tfs )
    {
        //rw::math::Transform3D<> T_World_Target = T_World_Table*Tf;
        targetFrame->moveTo(Tf, state);
        //std::cout << Tf << std::endl;
        std::vector<rw::math::Q> solutions = interpolator::util::getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
        rw::math::Q configuration = interpolator::util::getCollisionFreeSolution(UR6, state, detector, solutions);
        configurationPath.push_back(configuration);
        //std::cout << configuration << std::endl;
    }

    /*******************************************************************
     *  Time test of interpolation methods                             *
     *******************************************************************/
    std::vector<int> LItime; //Linear interpolate time
    std::vector<int> PBtime; //Parabolic blend time
    std::ofstream LIfile, PBfile;
    for (unsigned int i = 0; i < 60; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();

        interpolator::linearInterpolate(Tfs, Times);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        int dur_ms = duration.count();
        LItime.push_back(dur_ms);
        std::cout << "(" << i <<  ") Linear interpolation execution time: " << dur_ms << " [micros]" << std::endl;
    }
    LIfile.open("/home/emil/Documents/LIexetime_micros_1.txt");
    for ( int msTime : LItime )
        LIfile << msTime << std::endl;

    for (unsigned int i = 0; i < 60; i++)
    {
        auto start = std::chrono::high_resolution_clock::now();

        interpolator::parabolicBlend(Tfs, Times);

        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
        int dur_ms = duration.count();
        PBtime.push_back(dur_ms);
        std::cout << "(" << i <<  ") Parabolic blend interpolation execution time: " << dur_ms << " [micros]" << std::endl;
    }
    PBfile.open("/home/emil/Documents/PBexetime_micros_1.txt");
    for ( int msTime : PBtime )
        PBfile << msTime << std::endl;



    /*******************************************************************
     *  Show solution                                                  *
     *******************************************************************/
    //  Interpolate in Cartesian space      linearInterpolate
    //  Interpolate in joint space          linearInterpolateQ
    //  Interpolate using Parabolic blend   parabolicBlend
    auto paths = interpolator::linearInterpolate(Tfs, Times, targetFrame, UR6, wc, state, detector);
    std::vector<rw::math::Transform3D<>> blendPath = interpolator::parabolicBlend(Tfs, Times);
    std::vector<rw::math::Q> jointPath = std::get<0>(paths);
    std::vector<rw::math::Transform3D<>> cartPath = std::get<1>(paths);
    std::cout << "Size of blend path: " << blendPath.size() << std::endl;
    // Write to file

    std::ofstream blendQFile;
    blendQFile.open("/home/emil/Documents/blendQ.txt");
    std::vector<rw::math::Q> BlendQPath = interpolator::util::mapCartesianToJoint(blendPath, targetFrame, UR6, wc, state, detector);
    for ( rw::math::Q jointQ : BlendQPath )
        blendQFile << jointQ[0] << " " << jointQ[1] << " " << jointQ[2] << " " << jointQ[3] << " " << jointQ[4] << " " << jointQ[5] << std::endl;
    blendQFile.close();

    std::ofstream tfFile, qFile, blendFile;
    tfFile.open("/home/emil/Documents/LinIntTF_1.txt");
    qFile.open("/home/emil/Documents/LinIntQ_1.txt");
    blendFile.open("/home/emil/Documents/blend_tau25_1.txt");
    for ( rw::math::Q jointQ : jointPath )
    {
        qFile << jointQ[0] << " " << jointQ[1] << " " << jointQ[2] << " " << jointQ[3] << " " << jointQ[4] << " " << jointQ[5] << std::endl;
    }
    qFile.close();
    for ( rw::math::Transform3D<> tf : cartPath )
    {
        tfFile << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
        tfFile << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
        tfFile << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
        tfFile <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
    }
    tfFile.close();
    for ( rw::math::Transform3D<> tf : blendPath )
    {
        //blendFile << p[0] << " " << p[1] << " " << p[2] << std::endl;
        blendFile << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
        blendFile << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
        blendFile << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
        blendFile <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
    }
    blendFile.close();

    rw::trajectory::TimedStatePath statePath;
    double time = 0;
    double dur = 0.1;
    for ( rw::math::Q configuration : jointPath ) // configurationPath
    {
        UR6->setQ(configuration, state);
        //rw::kinematics::Kinematics::gr
        statePath.push_back(rw::trajectory::TimedState(time, state));
        time += dur/double(1);
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath, "../../Project_WorkCell_Cam/Project_WorkCell/visu.rwplay");

    /*
    ofstream outfile;
    outfile.open("/home/emil/Dropbox/UNI/MSc/Robotics/exercises/lab9Data.txt");
    for (float t = 0; t <= 1; t += 0.1)
    {
        rw::math::VectorND<N,float> Xi = cubicSpline(t, ts, tf, P1, P2, V1, V2);
        outfile << Xi[0] << " " << Xi[1] << std::endl;
    }
    outfile.close();
    */
	return 0;
}
