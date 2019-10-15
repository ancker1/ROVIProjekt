#include <rw/rw.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <iostream>


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


int main(int argc, char** argv)
{

    std::cout << "-- Begin --" << std::endl;

    /*******************************************************************
     *  Load workcell, frames and device
     *******************************************************************/
        // Load workcell
    static const std::string wc_path = "../../Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( wc.isNull() ){
        RW_THROW("Error loading workcell");
        return -1;
    }

    const std::string deviceName = "UR-6-85-5-A";
    rw::kinematics::Frame *toolFrame = wc->findFrame<rw::kinematics::Frame>("GraspTCP");
    if ( toolFrame == nullptr ){
        RW_THROW("Error finding frame: Tool");
        return -1;
    }

    rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
    if( cylinderFrame == nullptr ){
        RW_THROW("Error finding frame: Cylinder");
        return -1;
    }

    rw::models::SerialDevice::Ptr robotUR6 = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if ( robotUR6 == nullptr ){
        RW_THROW("Device UR6 not found.");
        return -1;
    }

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    /*******************************************************************
     *  Inverse kinematics: Q from pose
     *******************************************************************/

    rw::kinematics::State state = wc->getDefaultState();
    rw::kinematics::MovableFrame *target = wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");
    std::vector<rw::math::Q> collisionFreeSolutions;
    if ( target == nullptr ){
        RW_THROW("Error finding frame: GraspTarget");
        return -1;
    }
    for (double roll = 0; roll < 2*rw::math::Pi; roll += rw::math::Deg2Rad*1) {
        rw::math::RPY<> rotTarget(roll, 0, 0);
        rw::math::Vector3D<> posTarget = cylinderFrame->getTransform(state).P();
        posTarget[2] = 0.07;
        rw::math::Transform3D<> newTarget (posTarget, cylinderFrame->getTransform(state).R()*rotTarget.toRotation3D());
        target->moveTo(newTarget, state);
        std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR6, wc, state);
        for ( unsigned int i = 0; i < solutions.size(); i++ ){
            robotUR6->setQ(solutions[i], state);
            if ( !detector->inCollision(state, NULL, true) ){   // Take first solution without collision
                collisionFreeSolutions.push_back(solutions[i]);
                break;
            }
        }
    }
    std::cout << "Amount of collision free solutions: " << collisionFreeSolutions.size() << std::endl;


  //  std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robotUR6, wc, state);
    // for solutions
    // if not collision -> add
  //  if ( solutions.size() >= 1 )
    //    robotUR6->setQ(solutions[0], state);

    // Q from
    // Q to



    /*******************************************************************
     *  Show solution
     *******************************************************************/
    rw::trajectory::TimedStatePath statePath;
    double time = 0;
    double dur = 5;
    for (unsigned int i = 0; i < collisionFreeSolutions.size(); i++) {
        robotUR6->setQ(collisionFreeSolutions[i], state);
        statePath.push_back(rw::trajectory::TimedState(time, state));
        time += dur/double(collisionFreeSolutions.size());
    }
    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath, "../../Project_WorkCell_Cam/Project_WorkCell/visu.rwplay");


    std::cout << "-- Done --" << std::endl;
    return 0;
}
