#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
#include <thread>

#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

using namespace std;
using namespace rw::common;
using namespace rw::math;
using namespace rw::kinematics;
using namespace rw::loaders;
using namespace rw::models;
using namespace rw::pathplanning;
using namespace rw::proximity;
using namespace rw::trajectory;
using namespace rwlibs::pathplanners;
using namespace rwlibs::proximitystrategies;

#define MAXTIME 60. // Max time trying to find solution.
#define ESTEPSIZE 0.65 // Step size of RRT-connect.

struct rrt_connect {
  std::vector<float> stepsizes;
  std::vector<QPath> paths;
  std::vector<float> times;
} ;


bool checkCollisions(Device::Ptr device, const State &state, const CollisionDetector &detector, const Q &q) {
	State testState;
	CollisionDetector::QueryResult data;
	bool colFrom;

	testState = state;
	device->setQ(q,testState);
	colFrom = detector.inCollision(testState,&data);
	if (colFrom) {
		cerr << "Configuration in collision: " << q << endl;
		cerr << "Colliding frames: " << endl;
		FramePairSet fps = data.collidingFrames;
		for (FramePairSet::iterator it = fps.begin(); it != fps.end(); it++) {
			cerr << (*it).first->getName() << " " << (*it).second->getName() << endl;
		}
		return false;
	}
    return true;
}

std::tuple<float, QPath> calculate_path_from_stepsize(WorkCell::Ptr workcell, Device::Ptr robot, Frame* tool_frame, Frame* object_frame, Q from, Q to, float stepsize)
{
    QPath path;
    float time;

    State state = workcell->getDefaultState();
    //Set Q to the initial state and grip the bottle frame
    robot->setQ(from, state); // sets initial state
    Kinematics::gripFrame(object_frame, tool_frame, state); // Grip the bottle

    CollisionDetector detector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector, robot, state);

    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(robot),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, stepsize, RRTPlanner::RRTConnect);

    // check if collision in state
    if (!checkCollisions(robot, state, detector, from))
        return {0.0, path};
    if (!checkCollisions(robot, state, detector, to))
        return {0.0, path};

    // time to plan rrt, used for statistics.
    Timer t;
    t.resetAndResume();

    //Use the planner to find a trajectory between the configurations
    planner->query(from, to, path);
    planner->make(constraint);

    t.pause();
    time = t.getTime();

    return {time, path};
}

void calculate_path_from_stepsize_thread( rrt_connect &rrt_info, WorkCell::Ptr workcell, Device::Ptr robot, Frame* tool_frame, Frame* object_frame, Q from, Q to)
{
    for (unsigned int i = 0; i < rrt_info.stepsizes.size(); i++) {
        // Calculation of path and time for one stepsize
        QPath path;
        float time;

        State state = workcell->getDefaultState();
        //Set Q to the initial state and grip the bottle frame
        robot->setQ(from, state); // sets initial state
        Kinematics::gripFrame(object_frame, tool_frame, state); // Grip the bottle

        CollisionDetector detector(workcell, ProximityStrategyFactory::makeDefaultCollisionStrategy());
        PlannerConstraint constraint = PlannerConstraint::make(&detector, robot, state);

        QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(robot),constraint.getQConstraintPtr());
        QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
        QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, rrt_info.stepsizes[i], RRTPlanner::RRTConnect);

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

Vector3D<> fw_kinematics_pos_world_to_mFrame(WorkCell::Ptr workcell, Q configuration, Device::Ptr robot, Frame* mframe)
{
    Vector3D<> baseTmFramePos;
    State state = workcell->getDefaultState();
    // Setting configuration of robot
    robot->setQ( configuration , state );

    // Calculate Tranformation from world to mFrame
    Transform3D<> bTmf = Kinematics::worldTframe(mframe, state);
    baseTmFramePos = bTmf.P();

    return baseTmFramePos;
}

void print_cartesian_dist_statistics(bool append, rrt_connect rrt_connect_info, WorkCell::Ptr workcell, Device::Ptr robot, Frame* mframe)
{
    ofstream file;
    float dist = 0;
    QPath path;
    Q currentQ;
    Q prevQ;
    Vector3D<> currentPos;
    Vector3D<> prevPos;

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
            currentPos = fw_kinematics_pos_world_to_mFrame(workcell, path.at(j), robot, mframe);
            prevPos = fw_kinematics_pos_world_to_mFrame(workcell, path.at(j-1), robot, mframe);

            dist += sqrt(pow((currentPos[0]-prevPos[0]),2)+pow((currentPos[1]-prevPos[1]),2)+pow((currentPos[2]-prevPos[2]),2)); // L2 distance
        }
        file << rrt_connect_info.stepsizes[i] << " " << dist << std::endl;
        dist = 0;
    }
    file.close();

}

void print_configuration_dist_statistics(bool append, rrt_connect rrt_connect_info)
{
    ofstream file;
    float dist = 0;
    QPath path;

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
    ofstream file;
    QPath path;

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
    ofstream file;
    QPath path;

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

void calc_and_print_path_treaded(bool append, std::vector<float> stepsizes, WorkCell::Ptr workcell, Device::Ptr robot, Frame* tool_frame, Frame* object_frame, Q from, Q to)
{
    // Running 4 threads when writing to file
    rrt_connect rrt_total, rrt1, rrt2, rrt3, rrt4, rrt5, rrt6;

    rrt1.stepsizes = stepsizes;
    rrt2.stepsizes = stepsizes;
    rrt3.stepsizes = stepsizes;
    //rrt4.stepsizes = stepsizes;
    //rrt5.stepsizes = stepsizes;
    //rrt6.stepsizes = stepsizes;


        calculate_path_from_stepsize_thread(rrt1, workcell, robot, tool_frame, object_frame, from, to);



//    std::thread t1( calculate_path_from_stepsize_thread, std::ref(rrt1), workcell, robot, tool_frame, object_frame, from, to);
//    std::thread t2( calculate_path_from_stepsize_thread, std::ref(rrt2), workcell, robot, tool_frame, object_frame, from, to);
//    std::thread t3( calculate_path_from_stepsize_thread, std::ref(rrt3), workcell, robot, tool_frame, object_frame, from, to);
    //std::thread t4( calculate_path_from_stepsize_thread, std::ref(rrt4), workcell, robot, tool_frame, object_frame, from, to);
    //std::thread t5( calculate_path_from_stepsize_thread, std::ref(rrt5), workcell, robot, tool_frame, object_frame, from, to);
    //std::thread t6( calculate_path_from_stepsize_thread, std::ref(rrt6), workcell, robot, tool_frame, object_frame, from, to);
//    t1.join();
//    t2.join();
//    t3.join();
    //t4.join();
    //t5.join();
    //t6.join();

    // Joining all the data to write to file
    for(unsigned int i = 0; i < rrt1.stepsizes.size(); i++)
    {
        // Stepsizes
        rrt_total.stepsizes.push_back(rrt1.stepsizes[i]);
//        rrt_total.stepsizes.push_back(rrt2.stepsizes[i]);
//        rrt_total.stepsizes.push_back(rrt3.stepsizes[i]);
        //rrt_total.stepsizes.push_back(rrt4.stepsizes[i]);
        //rrt_total.stepsizes.push_back(rrt5.stepsizes[i]);
        //rrt_total.stepsizes.push_back(rrt6.stepsizes[i]);
        // Paths
        rrt_total.paths.push_back(rrt1.paths[i]);
//        rrt_total.paths.push_back(rrt2.paths[i]);
//        rrt_total.paths.push_back(rrt3.paths[i]);
        //rrt_total.paths.push_back(rrt4.paths[i]);
        //rrt_total.paths.push_back(rrt5.paths[i]);
        //rrt_total.paths.push_back(rrt6.paths[i]);
        // Times
        rrt_total.times.push_back(rrt1.times[i]);
//        rrt_total.times.push_back(rrt2.times[i]);
//        rrt_total.times.push_back(rrt3.times[i]);
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

    const string wcFile = "/home/mikkel/Desktop/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml"; //"../../Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const string deviceName = "UR-6-85-5-A";
    cout << "Trying to use workcell " << wcFile << " and device " << deviceName << endl;
    ofstream myfile;
    myfile.open("path.lua");
    rw::math::Math::seed();     //Sets the random seed


    WorkCell::Ptr wc = WorkCellLoader::Factory::load(wcFile);
    //Find the tool and bottle frame
    const std::string name_tool_frame = "WSG50TCP";
    const std::string name_cylinder_frame = "Cylinder";

    Frame *tool_frame = wc->findFrame(name_tool_frame);
    Frame *cylinder_frame = wc->findFrame(name_cylinder_frame);

    Device::Ptr device = wc->findDevice(deviceName);
    if (device == NULL)
    {
        cerr << "Device: " << deviceName << " not found!" << endl;
        return 0;
    }


//    //Get the state
//    State state = wc->getDefaultState();

//    //These Q's contains the start and end configurations
//    Q from(6, 2.26097, -2.2029, -1.3037, -4.34737, -1.5708, -0.880619); // Pick configuration Q{2.30356, -2.9901, -0.152361, -3.14073, 2.80971, -3.14159} (from side right corner)
//    Q to(6, -0.842337, -2.31799, -1.10501, -4.43098, -1.5708, 2.29926); // Place configuration Q{0.900888, -0.897058, 1.64999, -0.752929, -0.669908, -3.14159} (middle in place area)

//    //Set Q to the initial state and grip the bottle frame
//    device->setQ(from, state); // sets initial state
//    Kinematics::gripFrame(cylinder_frame, tool_frame, state); // Grip the bottle

//    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
//    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

//    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
//    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
//    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTConnect);

//    if (!checkCollisions(device, state, detector, from))
//        return 0;
//    if (!checkCollisions(device, state, detector, to))
//        return 0;


//    //Creates the functions for the LUA script and initializes the position and state of the robot
//    myfile << "wc = rws.getRobWorkStudio():getWorkCell()\n"
//              <<"state = wc:getDefaultState()"
//              <<"\ndevice = wc:findDevice(\"UR-6-85-5-A\")"
//              <<"\ngripper = wc:findFrame(\"GraspTCP\")"
//              <<"\ncylinder = wc:findFrame(\"Cylinder\")\n"
//              <<"table = wc:findFrame(\"Table\")\n\n"
//              <<"function setQ(q)\n"
//              <<"qq = rw.Q(#q,q[1],q[2],q[3],q[4],q[5],q[6])\n"
//              <<"device:setQ(qq,state)\n"
//              <<"rws.getRobWorkStudio():setState(state)\n"
//              <<"rw.sleep(0.1)\n"
//              <<"end\n\n"
//              <<"function attach(obj, tool)\n"
//              <<"rw.gripFrame(obj, tool, state)\n"
//              <<"rws.getRobWorkStudio():setState(state)\n"
//              <<"rw.sleep(0.1)\n"
//              <<"end\n\n";

//    cout << "Planning from " << from << " to " << to << endl;
//    QPath path;
//    Timer t;
//    t.resetAndResume();

//    //Use the planner to find a trajectory between the configurations
//    planner->query(from, to, path);
//    planner->make(constraint);

//    t.pause();
//    double distance = 0;

//    cout << "Took secounds to calculate path: " << t.getTime()<< endl;

//    if (t.getTime() >= MAXTIME)
//    {
//        cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
//    }

//    //Appends the path to the LUA script. This file can be "played" in RobWorkStudio.
//    for (unsigned int i = 0; i< path.size(); i++)
//    {
//        if(i == 1)
//            myfile << "attach(cylinder, gripper)\n";
//        if(i >= 1)
//            distance += sqrt(pow((path.at(i)(0)-path.at(i-1)(0)),2)+pow((path.at(i)(1)-path.at(i-1)(1)),2)+pow((path.at(i)(2)-path.at(i-1)(2)),2)+pow((path.at(i)(3)-path.at(i-1)(3)),2)+pow((path.at(i)(4)-path.at(i-1)(4)),2)+pow((path.at(i)(5)-path.at(i-1)(5)),2)); // L2 distance
//        //cout << path.at(i)(0) << endl;
//        cout << distance << endl;
//        myfile <<"setQ({" << path.at(i)(0) << "," << path.at(i)(1) << "," << path.at(i)(2) << "," << path.at(i)(3) << "," << path.at(i)(4) << "," << path.at(i)(5) << "})" << "\n";
//    }

//    myfile.close();

    /****************************************************************************************
    **                          Calculating optimal parameters                             **
    ****************************************************************************************/
    /*
    Q from(6, 2.26097, -2.2029, -1.3037, -4.34737, -1.5708, -0.880619); // Pick configuration
    Q to(6, -0.842337, -2.31799, -1.10501, -4.43098, -1.5708, 2.29926); // Place configuration
    QPath path;
    float time;
    std::vector<QPath> paths;
    std::vector<float> times;
    std::vector<float> stepsizes;

    int count = 0;
    for (float i = 0.005; i <= 3; i += 0.005) {
        std::cout << "Calculating paths with stepsize " << ++count << " out of " << 3/0.005 << std::endl;
        stepsizes.push_back(i);
        std::tie(time, path) = calculate_path_from_stepsize(wc, device, tool_frame, cylinder_frame, from, to, i);
        paths.push_back(path);
        times.push_back(time);
    }

    print_path_time_statistics(stepsizes, times);
    print_configuration_num_statistics(stepsizes, paths);
    print_configuration_dist_statistics(stepsizes, paths);
    print_cartesian_dist_statistics(wc, device, tool_frame, stepsizes, paths);

    */
    /****************************************************************************************
    **                                 MultiThreading                                      **
    ****************************************************************************************/

    Q from(6, 2.26097, -2.2029, -1.3037, -4.34737, -1.5708, -0.880619); // Pick configuration
    Q to(6, -0.842337, -2.31799, -1.10501, -4.43098, -1.5708, 2.29926); // Place configuration
    std::vector<float> stepsizes;


    // Initialize stepsize values
    for (float i = 0.1; i <= 3; i += 0.1) {
        stepsizes.push_back(i);
    }

    int number_of_threads = 3;
    int number_of_data = 30;//number_of_threads; // Divide by number of threads running in function below

    for (unsigned int i = 0; i < number_of_data; i++)
    {
        std::cout << "Calculating iteration " << i*number_of_threads << " out of " << number_of_data*number_of_threads << " for rrt-connect" << std::endl;
        if (i == 0) // Do not append at the start to file
            calc_and_print_path_treaded(false, stepsizes, wc, device, tool_frame, cylinder_frame, from, to); // Running 3 threads when generating data
        else
            calc_and_print_path_treaded(true, stepsizes, wc, device, tool_frame, cylinder_frame, from, to); // Running 3 threads when generating data
    }
	return 0;
}
