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
    float t3 = 3.0f;
    // 4th point: home pos
    rw::math::Transform3D<> P4 = homeFrame;
    float t4 = 4.0f;
    // 5th point: above place
    rw::math::Transform3D<> P5 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t5 = 5.0f;
    // 6th point: place
    rw::math::Transform3D<> P6 = placeFrame;
    float t6 = 6.0f;
    // 7th point: above place
    rw::math::Transform3D<> P7 = placeFrame*rw::math::Transform3D<>(rw::math::Vector3D<>(0,0,-0.20));
    float t7 = 7.0f;
    // 8th point: home
    rw::math::Transform3D<> P8 = homeFrame;
    float t8 = 8.0f;

    std::vector<rw::math::Transform3D<>> points = {P0, P1, P2, P3, P4, P5, P6, P7, P8};
    std::vector<float> times = {t0, t1, t2, t3, t4, t5, t6, t7, t8};

    return std::make_tuple(points, times);
}

std::vector<rw::math::Q> linerInterpolateQ(std::vector<rw::math::Transform3D<>> Ts, std::vector<float> Times, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector)
{   // Linear in configuration space
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

std::vector<rw::math::Transform3D<>> parabolicBlend(std::vector<rw::math::Transform3D<>> P, std::vector<float> T)
{   // we wish to limit acceleration during blend (puts lower limit on tau)
    // distance largest at: d = 1/4*(v2 - v1)*tau (puts upper limit on tau)

    //(v2 - v1)/(4*tau)*pow(t + tau, 2)+ v1*t+Pi;
    std::vector<rw::math::Transform3D<>> Path;
    std::vector<rw::math::Vector3D<>> Ps;
    float tau = 0.25;
    for ( float t = 0; t < T[1]; t += 0.01f )
        if ( t <= T[0] + tau )
        {
            rw::math::Vector3D<> Pi = P[0].P() + (P[1].P() - P[0].P())*(t - T[0])/(T[1] - T[0]);
            Ps.push_back(Pi);
            rw::math::EAA<> eaaDiff(P[1].R() * P[0].R().inverse());
            rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[0])/(T[1]-T[0]));
            Path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[0].R() ));
        }

    for ( unsigned int i = 1; i < P.size(); i++ )
    {
        // Create linear intepolation between all P's
        for ( float t = T[i-1]; t < T[i+1]; t += 0.01f )
        {
            if ( T[i-1] + tau <= t && t <= T[i] - tau )
            {
                rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
                Ps.push_back(Pi);
                rw::math::EAA<> eaaDiff(P[i-1].R() * P[i].R().inverse());
                rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i])/(T[i-1]-T[i]));
                Path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[i].R() ));
            }
            else if ( T[i] - tau <= t && t <= T[i] + tau ) // If within blend interval
            {
                rw::math::Vector3D<> X = P[i].P();
                rw::math::Vector3D<> v1 = (P[i-1].P() - P[i].P())/(T[i-1] - T[i]); // (P[i-1]-P[i])/(T[i-1]-T[i]) = P[i]-P[i-1], when all t = 1
                rw::math::Vector3D<> v2 = (P[i+1].P() - P[i].P())/(T[i+1] - T[i]); // since all t = 1
                rw::math::Vector3D<> Pblend = (v2 - v1)/(4 * tau) * pow(t - T[i] + tau, 2) + v1*(t-T[i]) + X;
                Ps.push_back(Pblend);

                rw::math::EAA<> vR1eaa(P[i-1].R() * P[i].R().inverse());
                rw::math::Vector3D<> vR1 = rw::math::Vector3D<>(vR1eaa.axis()*vR1eaa.angle() / (T[i-1] - T[i]));

                rw::math::EAA<> vR2eaa(P[i+1].R() * P[i].R().inverse());
                rw::math::Vector3D<> vR2 = rw::math::Vector3D<>(vR2eaa.axis()*vR2eaa.angle() / (T[i+1] - T[i]));

                rw::math::Vector3D <> vBlend = (vR2 - vR1)/(4*tau)*pow(t - T[i] + tau, 2) + vR1*(t-T[i]); // + (X=0)

                rw::math::Rotation3D<> Rblend = rw::math::EAA<>(vBlend).toRotation3D()*P[i].R();

                //rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i-1])/(T[i]-T[i-1]));
                Path.push_back(rw::math::Transform3D<>( Pblend, Rblend ));
            }

        }

    }

    for ( float t = T[T.size()-2]; t < T[T.size() - 1]; t += 0.01f )
        if ( t >= T[T.size()-2] + tau )
        {
            rw::math::Vector3D<> Pi = P[P.size()-2].P() + (P[P.size()-1].P() - P[P.size()-2].P())*(t - T[T.size()-2])/(T[T.size()-1] - T[T.size()-2]);
            Ps.push_back(Pi);
            rw::math::EAA<> eaaDiff(P[P.size()-1].R() * P[P.size()-2].R().inverse());
            rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[T.size()-2])/(T[T.size()-1]-T[T.size()-2]));
            Path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[P.size()-2].R() ));
        }

    return Path;
}

std::vector<rw::math::Transform3D<>> linearInterpolate(std::vector<rw::math::Transform3D<>> P, std::vector<float> T)
{
    std::vector<rw::math::Transform3D<>> path;
    for ( unsigned int i = 1; i < P.size(); i++ )
    {
        for ( float t = T[i-1]; t < T[i]; t += 0.01f )
        {
            // Interpolate transforms...
            rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
            rw::math::EAA<> eaaDiff(P[i-1].R() * P[i].R().inverse());
            rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i])/(T[i-1]-T[i]));
            path.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[i].R() ));
        }
    }
    return path;
}

std::tuple<std::vector<rw::math::Q>,std::vector<rw::math::Transform3D<>>> linearInterpolate(std::vector<rw::math::Transform3D<>> P, std::vector<float> T, rw::kinematics::MovableFrame *targetFrame, rw::models::SerialDevice::Ptr UR6, rw::models::WorkCell::Ptr wc, rw::kinematics::State state, rw::proximity::CollisionDetector::Ptr detector)
{   // Linear in cartesian space
    std::vector<rw::math::Q> path;
    std::vector<rw::math::Transform3D<>> xits;
    for ( unsigned int i = 1; i < P.size(); i++ )
    {
        for ( float t = T[i-1]; t < T[i]; t += 0.01f )
        {
            // Interpolate transforms...

            rw::math::Vector3D<> Pi = P[i-1].P() + (P[i].P() - P[i-1].P())*(t - T[i-1])/(T[i] - T[i-1]);
            rw::math::EAA<> eaaDiff(P[i-1].R() * P[i].R().inverse());
            rw::math::EAA<> eaaChange(eaaDiff.axis(), eaaDiff.angle()*(t - T[i])/(T[i-1]-T[i]));
            xits.push_back(rw::math::Transform3D<>( Pi, eaaChange.toRotation3D()*P[i].R() ));

            targetFrame->moveTo(xits.back(), state);

            std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
            rw::math::Q configuration = getCollisionFreeSolution(UR6, state, detector, solutions);
            path.push_back(configuration);
        }
    }
    return std::make_tuple(path, xits);
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
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(-0.250, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.000, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);
    //cylinderFrame->moveTo(rw::math::Transform3D<>(rw::math::Vector3D<>(0.250, 0.474, 0.150),cylinderFrame->getTransform(state).R()), state);

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
        //std::cout << Tf << std::endl;
        std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", UR6, wc, state);
        rw::math::Q configuration = getCollisionFreeSolution(UR6, state, detector, solutions);
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

        linearInterpolate(Tfs, Times);

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

        parabolicBlend(Tfs, Times);

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
    auto paths = linearInterpolate(Tfs, Times, targetFrame, UR6, wc, state, detector);
    std::vector<rw::math::Transform3D<>> blendPath = parabolicBlend(Tfs, Times);
    std::vector<rw::math::Q> jointPath = std::get<0>(paths);
    std::vector<rw::math::Transform3D<>> cartPath = std::get<1>(paths);
    std::cout << "Size of blend path: " << blendPath.size() << std::endl;
    // Write to file
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
