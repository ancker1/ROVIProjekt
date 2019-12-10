#pragma once
#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <iostream>
#include <fstream>
#include <thread>

#include "util.hpp"
#include "rrt_connect_methods.hpp"


struct rrt_connect {
  std::vector<float> stepsizes;
  std::vector<rw::trajectory::QPath> paths;
  std::vector<float> times;
} ;


/****************************************************************************************
**                                Printing functions                                   **
****************************************************************************************/
// The function below to write to a lua file is from template used in lab6 //
void write_to_lua_file(const std::string lua_file_name, std::vector<rw::math::Q> confi_along_path, rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame){

    rw::trajectory::QPath path = calculate_between_configurations_path_rrt(confi_along_path, workcell, state, robot, tool_frame, object_frame);

    std::ofstream myfile;
    myfile.open(lua_file_name);
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

void print_trajectory_planning_time(std::string file_name, std::vector<int> times)
{
    std::ofstream file;
    file.open(file_name);
    for(unsigned int i = 0; i < times.size(); i++)
    {
        file << times[i] << std::endl;
    }
}


/****************************************************************************************
**            Finding optimal stepsize via threads and printing                        **
****************************************************************************************/

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
