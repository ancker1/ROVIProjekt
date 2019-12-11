#pragma once
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <iostream>

// Function below is from template used in lab6 //
bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) {
    rw::kinematics::State testState;
    rw::proximity::CollisionDetector::QueryResult data;
    bool colFrom;

    testState = state;
    device->setQ(q,testState);
    colFrom = detector.inCollision(testState,&data);
    if (colFrom) {
        std::cerr << "Configuration in collision: " << q << std::endl;
        std::cerr << "Colliding frames: " << std::endl;
        rw::kinematics::FramePairSet fps = data.collidingFrames;
        for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
            std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
        }
        return false;
    }
    return true;
}

// Function below is from template used in lab5 //
std::vector<rw::math::Q> getConfigurations(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
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

rw::math::Q get_collision_free_configuration(const std::string nameGoal, const std::string nameTcp, rw::models::SerialDevice::Ptr robot, rw::models::WorkCell::Ptr wc, rw::kinematics::State state)
{
    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    std::vector<rw::math::Q> solutions = getConfigurations(nameGoal, nameTcp, robot, wc, state);
    for ( unsigned int i = 0; i < solutions.size(); i++ ){
        robot->setQ(solutions[i], state);
        if ( !detector->inCollision(state, NULL, true) ){
            return solutions[i]; // Returns the first collision free solutions
        }
    }
    return rw::math::Q();
}

// The two functions below has inspiration from https://www.robwork.dk//manual/kinematics/?highlight=forward%20kinematics#forward-kinematics //
rw::math::Transform3D<> fw_kinematics_pos_world_to_mFrame(rw::models::WorkCell::Ptr workcell, rw::math::Q configuration, rw::models::Device::Ptr robot, rw::kinematics::Frame* mframe)
{
    rw::math::Transform3D<> worldTmFramePos;
    rw::kinematics::State state = workcell->getDefaultState();
    // Setting configuration of robot
    robot->setQ( configuration , state );

    // Calculate Tranformation from world to mFrame
    rw::math::Transform3D<> bTmf = rw::kinematics::Kinematics::worldTframe(mframe, state);
    worldTmFramePos = bTmf;

    return worldTmFramePos;
}

rw::math::Transform3D<> fw_kinematics_sFrame_to_gFrame(rw::models::WorkCell::Ptr workcell, rw::math::Q configuration, rw::models::Device::Ptr robot, rw::kinematics::Frame* sframe, rw::kinematics::Frame* gframe)
{
    rw::math::Transform3D<> sFrameTgFrame;
    rw::kinematics::State state = workcell->getDefaultState();
    // Setting configuration of robot
    robot->setQ( configuration , state );

    // Calculate Tranformation from world to mFrame
    rw::math::Transform3D<> sfTgf = rw::kinematics::Kinematics::frameTframe(sframe, gframe, state);
    sFrameTgFrame = sfTgf;

    return sFrameTgFrame;
}
