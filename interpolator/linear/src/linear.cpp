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

std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    /******************************************************************************************
     * This function was provided in Robotics, Lab Assignment 5                               *
     ******************************************************************************************/
    // Get, make and print name of frames
    const std::string robotName = robot->getName();
    const std::string nameRobotBase = robotName + "." + "Base";
    const std::string nameRobotTcp = robotName + "." + "TCP";

    // Find frames and check for existence
    rw::kinematics::Frame* frameGoal = wc->findFrame(nameGoal);
    rw::kinematics::Frame* frameTcp = wc->findFrame(nameTcp);
    rw::kinematics::Frame* frameRobotBase = wc->findFrame(nameRobotBase);
    rw::kinematics::Frame* frameRobotTcp = wc->findFrame(nameRobotTcp);
    if(frameGoal==nullptr || frameTcp==nullptr || frameRobotBase==nullptr || frameRobotTcp==nullptr)
    {
        std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==nullptr ? "NO!" : "YES!") << std::endl;
        std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==nullptr ? "NO!" : "YES!") << std::endl;
    }

    // Make "helper" transformations
    rw::math::Transform3D<> frameBaseTGoal = rw::kinematics::Kinematics::frameTframe(frameRobotBase, frameGoal, state);
    rw::math::Transform3D<> frameTcpTRobotTcp = rw::kinematics::Kinematics::frameTframe(frameTcp, frameRobotTcp, state);

    // get grasp frame in robot tool frame
    rw::math::Transform3D<> targetAt = frameBaseTGoal * frameTcpTRobotTcp;

    rw::invkin::ClosedFormIKSolverUR::Ptr closedFormSovler = rw::common::ownedPtr( new rw::invkin::ClosedFormIKSolverUR(robot, state) );
    return closedFormSovler->solve(targetAt, state);
}

rw::math::Q getCollisionFreeSolution(rw::models::SerialDevice::Ptr UR6, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector, std::vector<rw::math::Q> solutions)
{
    rw::math::Q configuration;
    for ( unsigned int i = 0; i < solutions.size(); i++ )
    {
        UR6->setQ(solutions[i], state);
        if ( !detector->inCollision(state, NULL, true) )
        {
            configuration = solutions[i];
            break;
        }
    }
    return configuration;
}



std::tuple<std::vector<rw::math::Transform3D<>>,std::vector<float>> getPointTimes(rw::math::Transform3D<> pickFrame, rw::math::Transform3D<> homeFrame, rw::math::Transform3D<> placeFrame)
{   // Read/define frames...
    // start point: home
     rw::math::Transform3D<> P0 = homeFrame;
     float t0 = 0.0f;
    // 1st point: above target
    rw::math::Transform3D<> P1 = pickFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t1 = 1.0f;
    // 2nd point: target
    rw::math::Transform3D<> P2 = pickFrame;
    float t2 = 2.0f;
    // 3rd point: above target
    rw::math::Transform3D<> P3 = pickFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t3 = 1.0f;
    // 4th point: home pos
    rw::math::Transform3D<> P4 = homeFrame;
    float t4 = 3.0f;
    // 5th point: above place
    rw::math::Transform3D<> P5 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t5 = 4.0f;
    // 6th point: place
    rw::math::Transform3D<> P6 = placeFrame;
    float t6 = 5.0f;
    // 7th point: above place
    rw::math::Transform3D<> P7 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t7 = 6.0f;
    // 8th point: home
    rw::math::Transform3D<> P8 = homeFrame;
    float t8 = 7.0f;

    std::vector<rw::math::Transform3D<>> points = {P0, P1, P2, P3, P4, P5, P6, P7, P8};
    std::vector<float> times = {t0, t1, t2, t3, t4, t5, t6, t7, t8};

    return std::make_tuple(points, times);
}

std::vector<rw::math::Q> linerInterpolateQ(std::vector<rw::math::Transform3D<>> Ts, std::vector<float> Times, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector)
{   // Linear in cartesian space
    std::vector<rw::math::Q> path;
    std::vector<rw::math::Q> TsQ;
    for ( unsigned int i = 0; i < Ts.size(); i++ )
    {
        targetFrame->moveTo(Ts[i], state);
        std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
        rw::math::Q configuration = getCollisionFreeSolution(UR6, state, detector, solutions);
        TsQ.push_back(configuration);
    }


    for ( unsigned int i = 1; i < TsQ.size(); i++ )
    {
        for ( float t = 0.0f; t < 1.0f; t += 0.1f )
        {

            // Interpolate configurations...
            rw::math::Q Ximin1 = TsQ[i-1];
            rw::math::Q Xi = TsQ[i];
            rw::math::Q Xit = Ximin1 + ( Xi - Ximin1 ) * t;
            path.push_back(Xit);
            std::cout << Xit << std::endl;
        }
    }
    return path;
}

std::vector<rw::math::Q> linerInterpolate(std::vector<rw::math::Transform3D<>> Ts, std::vector<float> Times, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector)
{   // Linear in cartesian space
    std::vector<rw::math::Q> path;
    for ( unsigned int i = 1; i < Ts.size(); i++ )
    {
        for ( float t = 0.0f; t < 1.0f; t += 0.01f )
        {

            // Interpolate transforms...
            rw::math::Transform3D<> Ximin1 = Ts[i-1];
            rw::math::Transform3D<> Xi = Ts[i];


            rw::math::Rotation3D<> rotdiff = Xi.R() * Ximin1.R().inverse();
            rw::math::EAA<> rotdiffEAA(rotdiff);
            rw::math::EAA<> rotEAAt(rotdiffEAA.axis(), rotdiffEAA.angle()*t);

            rw::math::Rotation3D<> rotdiffback = rotEAAt.toRotation3D();
            rw::math::Rotation3D<> linrot = rotdiffback*Ximin1.R();

            rw::math::Transform3D<> Xit(Ximin1.P() + (Xi.P() - Ximin1.P())*t, linrot);

            targetFrame->moveTo(Xit, state);

            std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
            rw::math::Q configuration = getCollisionFreeSolution(UR6, state, detector, solutions);
            path.push_back(configuration);
        }
    }
    return path;
}

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
    rw::kinematics::Frame *cylinderFrame = wc->findFrame<rw::kinematics::Frame>("Cylinder");
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


    auto PointTimes = getPointTimes(cylinderFrame->getTransform(state), UR6->baseTend(state), placeFrame);
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
        std::cout << Tf << std::endl;
        std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
        rw::math::Q configuration = getCollisionFreeSolution(UR6, state, detector, solutions);
        configurationPath.push_back(configuration);
        std::cout << configuration << std::endl;
    }



    /*******************************************************************
     *  Show solution
     *******************************************************************/
    //  Interpolate in Cartesian space: linerInterpolate
    //  Interpolate in joint space:     linerInterpolateQ
    std::vector<rw::math::Q> path = linerInterpolate(Tfs, Times, targetFrame, UR6, wc, state, detector);

    rw::trajectory::TimedStatePath statePath;
    double time = 0;
    double dur = 0.1;
    for ( rw::math::Q configuration : path ) // configurationPath
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
