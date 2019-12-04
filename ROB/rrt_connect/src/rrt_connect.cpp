#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <thread>

#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#define MAXTIME 60. // Max time trying to find solution.
#define ESTEPSIZE 0.4 // Step size of RRT-connect.

struct rrt_connect {
  std::vector<float> stepsizes;
  std::vector<rw::trajectory::QPath> paths;
  std::vector<float> times;
} ;

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

    return rw::math::Q(6, 0, 0, 0, 0, 0, 0);
}

rw::trajectory::QPath calculate_path_rrt(rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame, rw::math::Q from, rw::math::Q to)
{
    rw::trajectory::QPath path;

    rw::kinematics::State state = workcell->getDefaultState();
    //Set Q to the initial state and grip the bottle frame
    robot->setQ(from, state); // sets initial state
    rw::kinematics::Kinematics::gripFrame(object_frame, tool_frame, state); // Grip the bottle

    rw::proximity::CollisionDetector detector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector, robot, state);

    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robot),constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    // check if collision in state
    if (!checkCollisions(robot, state, detector, from))
        return path;
    if (!checkCollisions(robot, state, detector, to))
        return path;

    //Use the planner to find a trajectory between the configurations
    planner->query(from, to, path);
    planner->make(constraint);


    return path;
}

void calculate_path_from_stepsize_thread( rrt_connect &rrt_info, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame, rw::math::Q from, rw::math::Q to)
{
    for (unsigned int i = 0; i < rrt_info.stepsizes.size(); i++) {
        // Calculation of path and time for one stepsize
        rw::trajectory::QPath path;
        float time;

        rw::kinematics::State state = workcell->getDefaultState();
        //Set Q to the initial state and grip the bottle frame
        robot->setQ(from, state); // sets initial state
        rw::kinematics::Kinematics::gripFrame(object_frame, tool_frame, state); // Grip the bottle

        rw::proximity::CollisionDetector detector(workcell, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
        rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector, robot, state);

        rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(robot),constraint.getQConstraintPtr());
        rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
        rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, rrt_info.stepsizes[i], rwlibs::pathplanners::RRTPlanner::RRTConnect);

        // check if collision in state
        if (!checkCollisions(robot, state, detector, from))
            std::cout << "Error Collision between device in \"from\" configuration. Please stop program and make another configuration" << std::endl;
        if (!checkCollisions(robot, state, detector, to))
            std::cout << "Error Collision between device in \"to\" configuration. Please stop program and make another configuration" << std::endl;

        // time to plan rrt, used for statistics.
        auto start = std::chrono::high_resolution_clock::now();


        //Use the planner to find a trajectory between the configurations
        planner->query(from, to, path);
        auto stop = std::chrono::high_resolution_clock::now();

        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
        int dur = duration.count();

        planner->make(constraint);


        rrt_info.paths.push_back(path);
        rrt_info.times.push_back(dur);

    }
}

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

void print_cartesian_dist_statistics(bool append, rrt_connect rrt_connect_info, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr robot, rw::kinematics::Frame* mframe)
{
    std::ofstream file;
    float dist = 0;
    rw::trajectory::QPath path;
    rw::math::Q currentQ;
    rw::math::Q prevQ;
    rw::math::Vector3D<> currentPos;
    rw::math::Vector3D<> prevPos;

    std::cout << "Begining writing cartesian dist statistics to file" << std::endl;


    if ( append )
        file.open("stepsize_vs_cartesian_distance.txt",  std::ios_base::app);
    else
        file.open("stepsize_vs_cartesian_distance.txt");

     // Cartesian space distance for TCP from world
    for (unsigned int i = 0; i < rrt_connect_info.stepsizes.size(); i++) {
        //std::cout << "Writing stepsize " << i << " out of " << stepsizes.size() << std::endl;
        path = rrt_connect_info.paths[i];

        for (unsigned int j = 1; j < path.size(); j++){
            currentPos = fw_kinematics_pos_world_to_mFrame(workcell, path.at(j), robot, mframe).P(); // Only calculating based on position
            prevPos = fw_kinematics_pos_world_to_mFrame(workcell, path.at(j-1), robot, mframe).P(); // Only calculating base on position

            dist += sqrt(pow((currentPos[0]-prevPos[0]),2)+pow((currentPos[1]-prevPos[1]),2)+pow((currentPos[2]-prevPos[2]),2)); // L2 distance
        }
        file << rrt_connect_info.stepsizes[i] << " " << dist << std::endl;
        dist = 0;
    }
    file.close();

}

void print_configuration_dist_statistics(bool append, rrt_connect rrt_connect_info)
{
    std::ofstream file;
    float dist = 0;
    rw::trajectory::QPath path;

    std::cout << "Begining writing configuration dist statistics to file" << std::endl;


    if ( append )
        file.open("stepsize_vs_configuration_distance.txt", std::ios_base::app);
    else
        file.open("stepsize_vs_configuration_distance.txt");

    // Configuration space distance
    for (unsigned int i = 0; i < rrt_connect_info.stepsizes.size(); i++) {
        //std::cout << "Writing stepsize " << i << " out of " << stepsizes.size() << std::endl;
        path = rrt_connect_info.paths[i];
        for (unsigned int j = 1; j < path.size(); j++){
            dist += sqrt(pow((path.at(j)(0)-path.at(j-1)(0)),2)+pow((path.at(j)(1)-path.at(j-1)(1)),2)+pow((path.at(j)(2)-path.at(j-1)(2)),2)+pow((path.at(j)(3)-path.at(j-1)(3)),2)+pow((path.at(j)(4)-path.at(j-1)(4)),2)+pow((path.at(j)(5)-path.at(j-1)(5)),2)); // L2 distance
        }
        file << rrt_connect_info.stepsizes[i] << " " << dist << std::endl;
        dist = 0;
    }
    file.close();
}

void print_configuration_num_statistics(bool append, rrt_connect rrt_connect_info)
{
    std::ofstream file;
    rw::trajectory::QPath path;

    std::cout << "Begining writing configuration number statistics to file" << std::endl;

    if ( append )
        file.open("stepsize_vs_configuration_number.txt", std::ios_base::app);
    else
        file.open("stepsize_vs_configuration_number.txt");

    // Number of configurations
    for (unsigned int i = 0; i < rrt_connect_info.stepsizes.size(); i++) {
        //std::cout << "Writing stepsize " << i << " out of " << stepsizes.size() << std::endl;
        path = rrt_connect_info.paths[i];
        file << rrt_connect_info.stepsizes[i] << " " << path.size() << std::endl;
    }
    file.close();

}

void print_path_time_statistics(bool append, rrt_connect rrt_connect_info)
{
    std::ofstream file;
    rw::trajectory::QPath path;

    std::cout << "Begining writing path time statistics to file" << std::endl;

    if ( append )
        file.open("stepsize_vs_path_time.txt", std::ios_base::app);
    else
        file.open("stepsize_vs_path_time.txt");

    // Time calculating paths
    for (unsigned int i = 0; i < rrt_connect_info.stepsizes.size(); i++) {
        //std::cout << "Writing stepsize " << i << " out of " << stepsizes.size() << std::endl;
        file << rrt_connect_info.stepsizes[i] << " " << rrt_connect_info.times[i] << std::endl;
    }
    file.close();
}

void print_trajectory_transform(std::string file_name, std::vector<rw::trajectory::QPath> paths, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr robot, rw::kinematics::Frame* mframe)
{
    std::ofstream file;
    std::vector<rw::math::Transform3D<>> transformations;

    file.open(file_name);
    rw::math::Transform3D<> tf;
    //Calculating transformation based on configurations in paths for TCP
    for(unsigned int j = 0; j < paths.size(); j++) //take one out of the 60 iterations
    {
        for(unsigned int i = 0; i < paths[j].size(); i++)
        {
            rw::math::Q path(6, paths[j][i][0], paths[j][i][1], paths[j][i][2], paths[j][i][3], paths[j][i][4], paths[j][i][5]);
            tf = fw_kinematics_pos_world_to_mFrame(workcell, path, robot, mframe);
            // Write a homogen transformation to file
            file << j << " " << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << std::endl;
            file << j << " " << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << std::endl;
            file << j << " " << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << std::endl;
            file << j << " " <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << std::endl;
        }

    }
    file.close();
}

void print_trajectory_configuration(std::string file_name, std::vector<rw::trajectory::QPath> paths)
{
    std::ofstream file;
    file.open(file_name);
    // Write a homogen transformation to file
    for(unsigned int j = 0; j < paths.size(); j++) //take one out of the 60 iterations
    {
        for(unsigned int i = 0; i < paths[j].size(); i++){
            file << j << " " << paths[j][i][0] << " " << paths[j][i][1] << " " << paths[j][i][2] << " " << paths[j][i][3] << " " << paths[j][i][4] << " " << paths[j][i][5] << std::endl;
        }
    }
}

void calc_and_print_path_treaded(bool append, std::vector<float> stepsizes, rw::models::WorkCell::Ptr workcell, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame, rw::math::Q from, rw::math::Q to)
{
    // Running 4 threads when writing to file
    rrt_connect rrt_total, rrt1, rrt2, rrt3, rrt4, rrt5, rrt6;

    rrt1.stepsizes = stepsizes;
    rrt2.stepsizes = stepsizes;
    rrt3.stepsizes = stepsizes;
    //rrt4.stepsizes = stepsizes;
    //rrt5.stepsizes = stepsizes;
    //rrt6.stepsizes = stepsizes;


       // calculate_path_from_stepsize_thread(rrt1, workcell, robot, tool_frame, object_frame, from, to);



    std::thread t1( calculate_path_from_stepsize_thread, std::ref(rrt1), workcell, robot, tool_frame, object_frame, from, to);
    std::thread t2( calculate_path_from_stepsize_thread, std::ref(rrt2), workcell, robot, tool_frame, object_frame, from, to);
    std::thread t3( calculate_path_from_stepsize_thread, std::ref(rrt3), workcell, robot, tool_frame, object_frame, from, to);
    //std::thread t4( calculate_path_from_stepsize_thread, std::ref(rrt4), workcell, robot, tool_frame, object_frame, from, to);
    //std::thread t5( calculate_path_from_stepsize_thread, std::ref(rrt5), workcell, robot, tool_frame, object_frame, from, to);
    //std::thread t6( calculate_path_from_stepsize_thread, std::ref(rrt6), workcell, robot, tool_frame, object_frame, from, to);
    t1.join();
    t2.join();
    t3.join();
    //t4.join();
    //t5.join();
    //t6.join();

    // Joining all the data to write to file
    for(unsigned int i = 0; i < rrt1.stepsizes.size(); i++)
    {
        // Stepsizes
        rrt_total.stepsizes.push_back(rrt1.stepsizes[i]);
        rrt_total.stepsizes.push_back(rrt2.stepsizes[i]);
        rrt_total.stepsizes.push_back(rrt3.stepsizes[i]);
        //rrt_total.stepsizes.push_back(rrt4.stepsizes[i]);
        //rrt_total.stepsizes.push_back(rrt5.stepsizes[i]);
        //rrt_total.stepsizes.push_back(rrt6.stepsizes[i]);
        // Paths
        rrt_total.paths.push_back(rrt1.paths[i]);
        rrt_total.paths.push_back(rrt2.paths[i]);
        rrt_total.paths.push_back(rrt3.paths[i]);
        //rrt_total.paths.push_back(rrt4.paths[i]);
        //rrt_total.paths.push_back(rrt5.paths[i]);
        //rrt_total.paths.push_back(rrt6.paths[i]);
        // Times
        rrt_total.times.push_back(rrt1.times[i]);
        rrt_total.times.push_back(rrt2.times[i]);
        rrt_total.times.push_back(rrt3.times[i]);
        //rrt_total.times.push_back(rrt4.times[i]);
        //rrt_total.times.push_back(rrt5.times[i]);
        //rrt_total.times.push_back(rrt6.times[i]);
    }


    print_path_time_statistics(append, rrt_total);
    print_configuration_num_statistics(append, rrt_total);
    print_configuration_dist_statistics(append, rrt_total);
    print_cartesian_dist_statistics(append, rrt_total, workcell, robot, tool_frame);
}



int main(int argc, char** argv) {

    /****************************************************************************************
    **                            RobWork setup for workcell                               **
    ****************************************************************************************/
    const std::string wcFile = "/home/mikkel/Desktop/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml"; //"../../Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const std::string deviceName = "UR-6-85-5-A";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;
    std::ofstream myfile;
    myfile.open("path.lua");
    rw::math::Math::seed();     //Sets the random seed


    rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wcFile);
    //Find the tool and bottle frame
    const std::string name_tool_frame = "WSG50TCP";
    const std::string name_cylinder_frame = "Cylinder";
    const std::string name_table_frame = "Table";
    const std::string name_place_area_frame = "placeArea";

    rw::kinematics::Frame *tool_frame = wc->findFrame(name_tool_frame);
    rw::kinematics::Frame *cylinder_frame = wc->findFrame(name_cylinder_frame);
    rw::kinematics::Frame *Table_frame = wc->findFrame(name_table_frame);

    rw::kinematics::MovableFrame *cylinder_frame_moveable = wc->findFrame<rw::kinematics::MovableFrame>("GraspTarget");

    rw::models::Device::Ptr device = wc->findDevice(deviceName);
    rw::models::SerialDevice::Ptr device1 = wc->findDevice<rw::models::SerialDevice>(deviceName);
    if (device == NULL)
    {
        std::cerr << "Device: " << deviceName << " not found!" << std::endl;
        return 0;
    }

    /****************************************************************************************
    **                         Print on trajectory to LUA file                             **
    ****************************************************************************************/
    // The code below to "Calculating optimal parameters multiThreading" is from template used in lab6 //

    //Get the state
    rw::kinematics::State state = wc->getDefaultState();

    // Moving the object to correct position
    rw::math::Vector3D<> cylinder;
    cylinder[0] = -0.25; // x - coordinate of base frame
    cylinder[1] = 0.475; // y - coordinate of base frame

    rw::math::Transform3D<> newCylinder (cylinder, cylinder_frame_moveable->getTransform(state).R());
    cylinder_frame_moveable->moveTo(newCylinder, state);

    //Getting start and end configuration from pick and place area
    rw::math::Q from = get_collision_free_configuration(name_cylinder_frame, "GraspTCP", device1, wc, state); // Pick area
    rw::math::Q to = get_collision_free_configuration(name_place_area_frame, "GraspTCP", device1, wc, state); // Place area

    //Set Q to the initial state and grip the bottle frame
    device->setQ(from, state); // sets initial state
    rw::kinematics::Kinematics::gripFrame(cylinder_frame, tool_frame, state); // Grip the bottle

    rw::proximity::CollisionDetector detector(wc, rwlibs::proximitystrategies::ProximityStrategyFactory::makeDefaultCollisionStrategy());
    rw::pathplanning::PlannerConstraint constraint = rw::pathplanning::PlannerConstraint::make(&detector,device,state);

    rw::pathplanning::QSampler::Ptr sampler = rw::pathplanning::QSampler::makeConstrained(rw::pathplanning::QSampler::makeUniform(device), constraint.getQConstraintPtr());
    rw::math::QMetric::Ptr metric = rw::math::MetricFactory::makeEuclidean<rw::math::Q>();
    rw::pathplanning::QToQPlanner::Ptr planner = rwlibs::pathplanners::RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, rwlibs::pathplanners::RRTPlanner::RRTConnect);

    if (!checkCollisions(device, state, detector, from))
        return 0;
    if (!checkCollisions(device, state, detector, to))
        return 0;

    //Creates the functions for the LUA script and initializes the position and state of the robot
    myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n"
              <<"state = wc:getDefaultState()"
              <<"\ndevice = wc:findDevice(\"UR-6-85-5-A\")"
              <<"\ngripper = wc:findFrame(\"GraspTCP\")"
              <<"\ncylinder = wc:findFrame(\"Cylinder\")\n"
              <<"table = wc:findFrame(\"Table\")\n\n"
              <<"function setQ(q)\n"
              <<"qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
              <<"device:setQ(qq,state)\n"
              <<"rws.getRobWorkStudio():setState(state)\n"
              <<"rw.sleep(0.1)\n"
              <<"end\n\n"
              <<"function attach(obj, tool)\n"
              <<"rw.gripFrame(obj, tool, state)\n"
              <<"rws.getRobWorkStudio():setState(state)\n"
              <<"rw.sleep(0.1)\n"
              <<"end\n\n";

    std::cout << "Planning from " << from << " to " << to << std::endl;
    rw::trajectory::QPath path;
    //Use the planner to find a trajectory between the configurations
    planner->query(from, to, path);
    planner->make(constraint);

    double distance = 0;

    //Appends the path to the LUA script. This file can be "played" in RobWorkStudio.
    for (unsigned int i = 0; i< path.size(); i++)
    {
        if(i == 1)
            myfile << "attach(cylinder, gripper)\n";
        if(i >= 1)
            distance += sqrt(pow((path.at(i)(0)-path.at(i-1)(0)),2)+pow((path.at(i)(1)-path.at(i-1)(1)),2)+pow((path.at(i)(2)-path.at(i-1)(2)),2)+pow((path.at(i)(3)-path.at(i-1)(3)),2)+pow((path.at(i)(4)-path.at(i-1)(4)),2)+pow((path.at(i)(5)-path.at(i-1)(5)),2)); // L2 distance
        //cout << path.at(i)(0) << endl;
        std::cout << distance << std::endl;
        myfile <<"setQ({" << path.at(i)(0) << "," << path.at(i)(1) << "," << path.at(i)(2) << "," << path.at(i)(3) << "," << path.at(i)(4) << "," << path.at(i)(5) << "})" << "\n";
    }

    myfile.close();

    /****************************************************************************************
    **              Calculating optimal parameters multiThreading                          **
    ****************************************************************************************/

//    rw::math::Q from(6, 1.77843, -2.10011, -1.47274, -4.28113, -1.5708, -1.36316); // Pick configuration, Middle
//    rw::math::Q to(6, -0.782077, -2.24428, -1.23335, -4.37635, -1.5708, 2.35952);  // Place configuration, Middle
//    std::vector<float> stepsizes;


//    // Initialize stepsize values
//    for (float i = 0.1; i <= 3; i += 0.1) {
//        stepsizes.push_back(i);
//    }

//    int number_of_threads = 3;
//    int number_of_data = 60/number_of_threads; // Divide by number of threads running in function below

//    for (unsigned int i = 0; i < number_of_data; i++)
//    {
//        std::cout << "Calculating iteration " << i*number_of_threads << " out of " << number_of_data*number_of_threads << " for rrt-connect" << std::endl;
//        if (i == 0) // Do not append at the start to file
//            calc_and_print_path_treaded(false, stepsizes, wc, device, tool_frame, cylinder_frame, from, to); // Running 3 threads when generating data
//        else
//            calc_and_print_path_treaded(true, stepsizes, wc, device, tool_frame, cylinder_frame, from, to); // Running 3 threads when generating data
//    }


    /****************************************************************************************
    **                         Running RRT with optimal stepsize                           **
    ****************************************************************************************/
/*
    std::vector<rw::trajectory::QPath> paths;
    rw::math::Q to(6, -0.842337, -2.31799, -1.10501, -4.43098, -1.5708, 2.29926); // Place configuration
    std::vector<rw::math::Transform3D<>> transforms;
    // ******************************* PICK PLACE 1 *********************************** //
    rw::math::Q from(6, 2.26097, -2.2029, -1.3037, -4.34737, -1.5708, -0.880619); // Pick configuration right corner
    for(unsigned int i = 0; i < 60; i++)
    {
            paths.push_back(calculate_path_rrt(wc, device, tool_frame, cylinder_frame, from, to));
    }
    print_trajectory_transform("rrtConnect_transform_pickplace_right.txt", paths, wc, device, tool_frame);
    print_trajectory_configuration("rrtConnect_configuration_pickplace_right.txt", paths);
    // ******************************* PICK PLACE 2 *********************************** //
    from = rw::math::Q(6, 2.26097, -2.2029, -1.3037, -4.34737, -1.5708, -0.880619); // Pick configuration middle
    paths.clear();
    for(unsigned int i = 0; i < 60; i++)
    {
            paths.push_back(calculate_path_rrt(wc, device, tool_frame, cylinder_frame, from, to));
    }
    print_trajectory_transform("rrtConnect_transform_pickplace_middle.txt", paths, wc, device, tool_frame);
    print_trajectory_configuration("rrtConnect_configuration_pickplace_middle.txt", paths);

    // ******************************* PICK PLACE 3 *********************************** //
    from = rw::math::Q(6, 2.26097, -2.2029, -1.3037, -4.34737, -1.5708, -0.880619); // Pick configuration left corner
    paths.clear();
    for(unsigned int i = 0; i < 60; i++)
    {
            paths.push_back(calculate_path_rrt(wc, device, tool_frame, cylinder_frame, from, to));
    }
    print_trajectory_transform("rrtConnect_transform_pickplace_left.txt", paths, wc, device, tool_frame);
    print_trajectory_configuration("rrtConnect_configuration_pickplace_left.txt", paths);
*/
    return 0;
}
