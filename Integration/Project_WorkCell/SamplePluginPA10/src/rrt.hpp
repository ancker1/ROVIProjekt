#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/rw.hpp>
#include <string>
#include <vector>

#define ESTEPSIZE 0.4 // Step size of RRT-connect.

namespace RRT { namespace util {
    // Utility functions
bool checkCollisions(rw::models::Device::Ptr device, const rw::kinematics::State &state, const rw::proximity::CollisionDetector &detector, const rw::math::Q &q) 
{
rw::kinematics::State testState;
rw::proximity::CollisionDetector::QueryResult data;
bool colFrom;

testState = state;
device->setQ(q,testState);
colFrom = detector.inCollision(testState,&data);
if (colFrom) {
    //std::cerr << "Configuration in collision: " << q << std::endl;
    //std::cerr << "Colliding frames: " << std::endl;
    rw::kinematics::FramePairSet fps = data.collidingFrames;
    for (rw::kinematics::FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
        //std::cerr << (*it).first->getName() << " " << (*it).second->getName() << std::endl;
    }
    return false;
}
return true;
}

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
        //std::cout << " ALL FRAMES NOT FOUND:" << std::endl;
        //std::cout << " Found \"" << nameGoal << "\": " << (frameGoal==nullptr ? "NO!" : "YES!") << std::endl;
        //std::cout << " Found \"" << nameTcp << "\": " << (frameTcp==nullptr ? "NO!" : "YES!") << std::endl;
        //std::cout << " Found \"" << nameRobotBase << "\": " << (frameRobotBase==nullptr ? "NO!" : "YES!") << std::endl;
        //std::cout << " Found \"" << nameRobotTcp << "\": " << (frameRobotTcp==nullptr ? "NO!" : "YES!") << std::endl;
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
    std::vector<rw::math::Q> solutions = RRT::util::getConfigurations(nameGoal, nameTcp, robot, wc, state);
    for ( unsigned int i = 0; i < solutions.size(); i++ ){
        robot->setQ(solutions[i], state);
        if ( !detector->inCollision(state, NULL, true) ){
            return solutions[i]; // Returns the first collision free solutions
        }
    }

    return rw::math::Q(6, 0, 0, 0, 0, 0, 0);
}

}
    // RRT functions
    rw::trajectory::QPath calculate_path_rrt(rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame, rw::math::Q from, rw::math::Q to)
{
    rw::trajectory::QPath path;

    //Set Q to the initial state and grip the bottle frame
    robot->setQ(from, state); // sets initial state


    rw::proximity::CollisionDetector detector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector, robot, state);

    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robot),constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    // check if collision in state
    if (!RRT::util::checkCollisions(robot, state, detector, from))
        return path;
    if (!RRT::util::checkCollisions(robot, state, detector, to))
        return path;

    //Use the planner to find a trajectory between the configurations
    planner->query(from, to, path);
    planner->make(constraint);


    return path;
}

rw::trajectory::QPath calculate_whole_path_rrt(std::vector<rw::math::Q> confi_along_path, rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame)
{
    rw::trajectory::QPath whole_path;
    rw::trajectory::QPath sub_path;
    rw::math::Q from;
    rw::math::Q to;

    for(unsigned int i = 0; i < confi_along_path.size() - 1; i++)
    {
        from = confi_along_path[i];
        to = confi_along_path[i+1];
        //std::cout << "Planning from " << from << " to " << to << std::endl;
        sub_path = RRT::calculate_path_rrt(workcell, state, robot, tool_frame, object_frame, from, to);
        for(unsigned int j = 0; j < sub_path.size(); j++)
        {
            whole_path.push_back(sub_path[j]);
        }
    }

    return whole_path;
}

rw::trajectory::QPath rrt_path_calculate(rw::math::Transform3D<> obj_place, rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::Device::Ptr robot, rw::models::SerialDevice::Ptr robot_serial, rw::kinematics::Frame* tool_frame, rw::kinematics::MovableFrame* object_frame, unsigned int &index_pick, unsigned int &index_place)
{
    rw::trajectory::QPath path;

    rw::math::Q from = RRT::util::get_collision_free_configuration(object_frame->getName(), "GraspTCP", robot_serial, workcell, state); // Pick area
    // Moving place frame to correct pos
    object_frame->moveTo(obj_place, state);

    rw::math::Q to = RRT::util::get_collision_free_configuration(object_frame->getName(), "GraspTCP", robot_serial, workcell, state); // Place area

    rw::math::Q home = robot->getQ(state);
    std::vector<rw::math::Q> confi_along_path = {home}; // Home position
    confi_along_path.push_back(from);
    confi_along_path.push_back(to);
    confi_along_path.push_back(home);

    path = RRT::calculate_whole_path_rrt(confi_along_path, workcell, state, robot, tool_frame, object_frame);

    // Finding index of pick and place configuration
    for (unsigned int i = 0; i < path.size(); i++)
    {
        //std::cout << path[i] << std::endl;
        if( from == path[i])
        {
            index_pick = i;
            //std::cout<< "pick " << i << std::endl;
        }
        if( to == path[i])
        {
            index_place = i;
            //std::cout << "place " << i << std::endl;
        }
    }

    return path;
}
}