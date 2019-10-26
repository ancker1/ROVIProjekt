#include <rw/rw.hpp>
#include <rw/invkin/ClosedFormIKSolverUR.hpp>
#include <rw/loaders/WorkCellLoader.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include <rw/math/Random.hpp>
#include <iostream>
#include <fstream>

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

std::vector<rw::math::Q> collision_free_from_object(bool from_side, const rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::SerialDevice::Ptr robot, rw::kinematics::MovableFrame::Ptr object_frame)
{
    rw::kinematics::MovableFrame *target = workcell->findFrame<rw::kinematics::MovableFrame>("GraspTarget"); // using to grasp to. Linked to object
    std::vector<rw::math::Q> collisionFreeSolutions;

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));

    if ( target == nullptr ){
        RW_THROW("Error finding frame: GraspTarget");
        return collisionFreeSolutions;
    }
    else if (from_side){
    // Finds solutions from side of object
        for (double yaw = 0; yaw < 2*rw::math::Pi; yaw += rw::math::Deg2Rad*1) {
            rw::math::RPY<> rotTarget_side(0, yaw, 0);
            // Rotates axis so following grasp target from the side
            rw::math::RPY<> rotyaw90(0, rw::math::Deg2Rad*90, 0);
            rw::math::RPY<> rotroll90(-rw::math::Deg2Rad*90, 0, 0);
            rw::math::Vector3D<> posTarget = object_frame->getTransform(state).P(); // Get position from object and uses it for targetframe
            posTarget[2] = 0.07;
            rw::math::Transform3D<> newTarget (posTarget, rotyaw90.toRotation3D()*rotroll90.toRotation3D()*rotTarget_side.toRotation3D());
            target->moveTo(newTarget, state);
            std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robot, workcell, state);
            for ( unsigned int i = 0; i < solutions.size(); i++ ){
                robot->setQ(solutions[i], state);
                if ( !detector->inCollision(state, NULL, true) ){   // Take first solution without collision
                    collisionFreeSolutions.push_back(solutions[i]);
                    break;
                }
            }
        }
    }
    else if(!from_side){
    // Finds solutions from top of object
        for (double roll = 0; roll < 2*rw::math::Pi; roll += rw::math::Deg2Rad*1) {
            // Rotates axis so following grasp target from the top
            rw::math::RPY<> rotTarget_up(roll, 0, 0);
            rw::math::Vector3D<> posTarget = object_frame->getTransform(state).P();
            posTarget[2] = 0.07;
            rw::math::Transform3D<> newTarget (posTarget, object_frame->getTransform(state).R()*rotTarget_up.toRotation3D());
            target->moveTo(newTarget, state);
            std::vector<rw::math::Q> solutions = getConfigurations("GraspTarget", "GraspTCP", robot, workcell, state);
            for ( unsigned int i = 0; i < solutions.size(); i++ ){
                robot->setQ(solutions[i], state);
                if ( !detector->inCollision(state, NULL, true) ){   // Take first solution without collision
                    collisionFreeSolutions.push_back(solutions[i]);
                    break;
                }
            }
        }
    }



    return collisionFreeSolutions;
}

std::vector<rw::math::Vector3D<double>> position_base_frame_gen_rand(int iterations)
{
    std::vector<rw::math::Vector3D<double>> pos_base_check; // x,y,collision_free
    int i = 0;
    while(i < iterations) {
        double randx = rw::math::Random::ran(-0.325, 0.325);
        double randy = rw::math::Random::ran(-0.525, 0.225);

        if(randy < -0.325){
            if(randx > 0.125)
                continue;
            else{
                rw::math::Vector3D<double> p(randx, randy, 0);
                pos_base_check.push_back(p);
                i++;
            }
        }
        else {
            rw::math::Vector3D<double> p(randx, randy, 0);
            pos_base_check.push_back(p);
            i++;
        }

    }


    return pos_base_check;
}

std::vector<rw::math::Vector3D<double>> position_base_frame_gen()
{
    double radius_base = 0.075; // 7.5 cm

    // First area coordinates
    std::vector<rw::math::Vector3D<double>> pos_base_check; // x,y,collision_free
    for (double x = 0; x < 4; x++) {
        rw::math::Vector3D<double> p(0.2 - (radius_base + 0.15*x), -0.5, 0);
        pos_base_check.push_back(p);
    }
    // Second area coordinates
    // offset with y 5cm and x 2.5cm
    for (double x = 0; x < 5; x++) {
        for (double y = 0; y < 4; y++) {
            rw::math::Vector3D<double> p(0.4 - 0.025 - (radius_base + 0.15*x), 0.3 - 0.05 - (radius_base + 0.15*y), 0);
            pos_base_check.push_back(p);
        }
    }

    return pos_base_check;
}

std::vector<rw::math::Vector3D<double>> best_robot_position(std::vector<rw::math::Vector3D<double>> &positions_base, rw::math::Vector3D<double> object_pos, bool from_side, const rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::SerialDevice::Ptr robot, rw::kinematics::MovableFrame::Ptr object_frame)
{
    if(from_side)
        std::cout << "Finding collision free solutions grasping from side " << std::endl;
    else
        std::cout << "Finding collision free solutions grasping from top " << std::endl;


    rw::kinematics::MovableFrame *base = workcell->findFrame<rw::kinematics::MovableFrame>("URReference");

    // Moving robot base
    for (unsigned int i = 0; i < positions_base.size(); i++) {
        rw::kinematics::State state = workcell->getDefaultState();
        // Placing robot
        rw::math::Vector3D<> posBase;
        posBase[0] = positions_base[i][0]; // x - coordinate of base frame
        posBase[1] = positions_base[i][1]; // y - coordinate of base frame

        rw::math::Transform3D<> newBase (posBase, base->getTransform(state).R());
        base->moveTo(newBase, state);

        // Placing object
        rw::math::Transform3D<> newObj (object_pos, object_frame->getTransform(state).R());
        object_frame->moveTo(newObj, state);

        std::vector<rw::math::Q> collisionFreeSolutions = collision_free_from_object(from_side, workcell, state, robot, object_frame);
        positions_base[i][2] = collisionFreeSolutions.size();

        if( i % 50 == 0)
            std::cout << "Trying to find solution at " << i << " out of " << positions_base.size() << std::endl;
    }

    return positions_base;
}

void write_pos_to_file(std::string file_name, std::vector<rw::math::Vector3D<double>> &positions)
{
    std::ofstream myfile;
    myfile.open(file_name);
    for (unsigned int i = 0; i < positions.size(); i++) {
       myfile << positions[i][0] << " " << positions[i][1] << " " << positions[i][2] << std::endl;
    }

    myfile.close();

    std::cout << "Done writing" << std::endl;
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
    // Loading toolframe
    const std::string deviceName = "UR-6-85-5-A";
    rw::kinematics::Frame *toolFrame = wc->findFrame<rw::kinematics::Frame>("GraspTCP");
    if ( toolFrame == nullptr ){
        RW_THROW("Error finding frame: Tool");
        return -1;
    }
    // Loading cylinder frame
    rw::kinematics::MovableFrame *cylinderFrame = wc->findFrame<rw::kinematics::MovableFrame>("Cylinder");
    if( cylinderFrame == nullptr ){
        RW_THROW("Error finding frame: Cylinder");
        return -1;
    }
    // Loading UR robot
    rw::models::SerialDevice::Ptr robotUR6 = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if ( robotUR6 == nullptr ){
        RW_THROW("Device UR6 not found.");
        return -1;
    }

    rw::proximity::CollisionDetector::Ptr detector = rw::common::ownedPtr(new rw::proximity::CollisionDetector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy()));
    rw::kinematics::State state = wc->getDefaultState();

    /*******************************************************************
     * Moving robot around to get best position of base and get collsion free for up/side of object
     *******************************************************************/
    std::vector<rw::math::Vector3D<double>> base_frame_positions_side_pick_right = position_base_frame_gen_rand(1000); // gen 1000 random point
    std::vector<rw::math::Vector3D<double>> base_frame_positions_side_pick_mid = base_frame_positions_side_pick_right;
    std::vector<rw::math::Vector3D<double>> base_frame_positions_side_pick_left = base_frame_positions_side_pick_right;
    std::vector<rw::math::Vector3D<double>> base_frame_positions_side_place = base_frame_positions_side_pick_right;
    //std::vector<rw::math::Vector3D<double>> base_frame_positions_top = position_base_frame_gen();


    // Cylinder pos right
    rw::math::Vector3D<> cylinderPos(-0.25, 0.474, 0.150);
    base_frame_positions_side_pick_right = best_robot_position(base_frame_positions_side_pick_right, cylinderPos, true, wc, state, robotUR6, cylinderFrame); // Moves all robot position to one object position

    // Cylinder pos mid
    cylinderPos[0] = 0.0;
    base_frame_positions_side_pick_mid = best_robot_position(base_frame_positions_side_pick_mid, cylinderPos, true, wc, state, robotUR6, cylinderFrame);

    // Cylinder pos left
    cylinderPos[0] = 0.25;
    base_frame_positions_side_pick_left = best_robot_position(base_frame_positions_side_pick_left, cylinderPos, true, wc, state, robotUR6, cylinderFrame);

    // Cylinder in place area
    cylinderPos[0] = 0.3;
    cylinderPos[1] = -0.5;
    base_frame_positions_side_place = best_robot_position(base_frame_positions_side_place, cylinderPos, true, wc, state, robotUR6, cylinderFrame);
    //base_frame_positions_top = best_robot_position(base_frame_positions_top, false, wc, state, robotUR6, cylinderFrame);

    /*******************************************************************
     * Writing to file
     *******************************************************************/
     write_pos_to_file("base_pos_side_pick_right.txt", base_frame_positions_side_pick_right);
     write_pos_to_file("base_pos_side_pick_mid.txt", base_frame_positions_side_pick_mid);
     write_pos_to_file("base_pos_side_pick_left.txt", base_frame_positions_side_pick_left);
     write_pos_to_file("base_pos_side_place.txt", base_frame_positions_side_place);



//    rw::kinematics::MovableFrame *base = wc->findFrame<rw::kinematics::MovableFrame>("URReference");
//    rw::math::Vector3D<> posBase;
//    posBase[0] = 0.125; // x - coordinate of base frame
//    posBase[1] = -0.5; // y - coordinate of base frame

//    rw::math::Transform3D<> newBase (posBase, base->getTransform(state).R());
//    base->moveTo(newBase, state);

//    std::vector<rw::math::Q> collisionFreeSolutions = collision_free_from_object(true, wc, state, robotUR6, cylinderFrame);
//    /*******************************************************************
//     *  Show solution
//     *******************************************************************/
//    rw::trajectory::TimedStatePath statePath;
//    double time = 0;
//    double dur = 5;
//    for (unsigned int i = 0; i < collisionFreeSolutions.size(); i++) {
//        robotUR6->setQ(collisionFreeSolutions[i], state);
//        statePath.push_back(rw::trajectory::TimedState(time, state));
//        time += dur/double(collisionFreeSolutions.size());
//    }
//    rw::loaders::PathLoader::storeTimedStatePath(*wc, statePath, "../../Project_WorkCell_Cam/Project_WorkCell/visu.rwplay");


    std::cout << "-- Done --" << std::endl;
    return 0;
}
