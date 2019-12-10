#include <iostream>
#include <fstream>
#include <thread>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>

#include "util.hpp"
#include "performance_evaluation.hpp"
#include "rrt_connect_methods.hpp"

#define ESTEPSIZE 0.4 // Step size of RRT-connect.


int main(int argc, char** argv) {
    /* Setup below has inspiration from lab6 robotics */
    /****************************************************************************************
    **                            RobWork setup for workcell                               **
    ****************************************************************************************/
    const std::string wcFile = "/home/mikkel/Desktop/Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const std::string deviceName = "UR-6-85-5-A";
    std::cout << "Trying to use workcell " << wcFile << " and device " << deviceName << std::endl;
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

    //Get the state
    rw::kinematics::State state = wc->getDefaultState();

    /****************************************************************************************
    **                         Print on trajectory to LUA file                             **
    ****************************************************************************************/

    const std::string path_lua = "path.lua";

    // Moving the object to correct position
    rw::math::Vector3D<> cylinder;
    cylinder[0] = -0.25; // x - coordinate of base frame
    cylinder[1] = 0.475; // y - coordinate of base frame

    rw::math::Transform3D<> newCylinder (cylinder, cylinder_frame_moveable->getTransform(state).R());
    cylinder_frame_moveable->moveTo(newCylinder, state);

    //Getting start and end configuration from pick and place area
    rw::math::Q from = get_collision_free_configuration(name_cylinder_frame, "GraspTCP", device1, wc, state); // Pick area
    rw::math::Q to = get_collision_free_configuration(name_place_area_frame, "GraspTCP", device1, wc, state); // Place area

    rw::math::Q home = device->getQ(state);
    std::vector<rw::math::Q> confi_along_path = {home}; // Home position
    confi_along_path.push_back(from);
    confi_along_path.push_back(to);
    confi_along_path.push_back(home);

    write_to_lua_file(path_lua, confi_along_path, wc, state, device, tool_frame, cylinder_frame);

    /****************************************************************************************
    **              Calculating optimal stepsize from multiThreading                       **
    ****************************************************************************************/

//    // Moving the object to middle position
//    rw::math::Vector3D<> cylinder;
//    cylinder[0] = 0; // x - coordinate of base frame
//    cylinder[1] = 0.45; // y - coordinate of base frame

//    rw::math::Transform3D<> newCylinder (cylinder, cylinder_frame_moveable->getTransform(state).R());
//    cylinder_frame_moveable->moveTo(newCylinder, state);

//    //Getting start and end configuration from pick and place area
//    rw::math::Q from = get_collision_free_configuration(name_cylinder_frame, "GraspTCP", device1, wc, state); // Pick area
//    rw::math::Q to = get_collision_free_configuration(name_place_area_frame, "GraspTCP", device1, wc, state); // Place area

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
    **           Running performance testing for 3 locations with optimal stepsize         **
    ****************************************************************************************/

//    //Getting start and end configuration from pick and place area
//    rw::math::Q home = device->getQ(state);
//    std::vector<rw::math::Q> confi_along_path = {device->getQ(state)}; // Configuration that needs visited
//    rw::math::Q to = get_collision_free_configuration(name_place_area_frame, "GraspTCP", device1, wc, state); // Place area
//    std::vector<rw::trajectory::QPath> paths;
//    rw::trajectory::QPath path;
//    std::vector<int> path_planning_time;
//    // ******************************* PICK PLACE 1 *********************************** //
//    // Moving the object to correct position
//    rw::math::Vector3D<> cylinder;
//    cylinder[0] = -0.25; // x - coordinate of base frame
//    cylinder[1] = 0.475; // y - coordinate of base frame

//    rw::math::Transform3D<> newCylinder (cylinder, cylinder_frame_moveable->getTransform(state).R());
//    cylinder_frame_moveable->moveTo(newCylinder, state);
//    rw::math::Q from = get_collision_free_configuration(name_cylinder_frame, "GraspTCP", device1, wc, state); // Pick area right corner

//    // Inserts all configuration that needs to be visited
//    confi_along_path.push_back(from);
//    confi_along_path.push_back(to);
//    confi_along_path.push_back(home);
//    for(unsigned int i = 0; i < 60; i++)
//    {
//        // Time logging
//        auto start = std::chrono::high_resolution_clock::now();

//        path = calculate_between_configurations_path_rrt(confi_along_path, wc, state, device, tool_frame, cylinder_frame);

//        auto stop = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
//        path_planning_time.push_back(duration.count());

//        paths.push_back(path);
//        state = wc->getDefaultState();
//        cylinder_frame_moveable->moveTo(newCylinder, state);
//    }
//    print_trajectory_transform("rrtConnect_transform_pickplace_right.txt", paths, wc, device, tool_frame);
//    print_trajectory_configuration("rrtConnect_configuration_pickplace_right.txt", paths);
//    print_trajectory_planning_time("rrtConnect_planning_times_pickplace_right.txt", path_planning_time);

//    // ******************************* PICK PLACE 2 *********************************** //

//    // Resetting to get ready for next pick place
//    state = wc->getDefaultState();
//    paths.clear();
//    path_planning_time.clear();
//    confi_along_path.clear();

//    // Moving the object to correct position
//    cylinder[0] = 0; // x - coordinate of base frame

//    newCylinder = rw::math::Transform3D<>(cylinder, cylinder_frame_moveable->getTransform(state).R());
//    cylinder_frame_moveable->moveTo(newCylinder, state);
//    from = get_collision_free_configuration(name_cylinder_frame, "GraspTCP", device1, wc, state); // Pick configuration middle

//    // Inserts all configuration that needs to be visited
//    confi_along_path.push_back(home);
//    confi_along_path.push_back(from);
//    confi_along_path.push_back(to);
//    confi_along_path.push_back(home);

//    for(unsigned int i = 0; i < 60; i++)
//    {
//        // Time logging
//        auto start = std::chrono::high_resolution_clock::now();

//        path = calculate_between_configurations_path_rrt(confi_along_path, wc, state, device, tool_frame, cylinder_frame);

//        auto stop = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
//        path_planning_time.push_back(duration.count());

//        paths.push_back(path);
//        state = wc->getDefaultState();
//        cylinder_frame_moveable->moveTo(newCylinder, state);
//    }
//    print_trajectory_transform("rrtConnect_transform_pickplace_middle.txt", paths, wc, device, tool_frame);
//    print_trajectory_configuration("rrtConnect_configuration_pickplace_middle.txt", paths);
//    print_trajectory_planning_time("rrtConnect_planning_times_pickplace_middle.txt", path_planning_time);

//    // ******************************* PICK PLACE 3 *********************************** //

//    // Resetting to get ready for next pick place
//    state = wc->getDefaultState();
//    paths.clear();
//    path_planning_time.clear();
//    confi_along_path.clear();

//    // Moving the object to correct position
//    cylinder[0] = 0.25; // x - coordinate of base frame

//    newCylinder = rw::math::Transform3D<>(cylinder, cylinder_frame_moveable->getTransform(state).R());
//    cylinder_frame_moveable->moveTo(newCylinder, state);
//    from = get_collision_free_configuration(name_cylinder_frame, "GraspTCP", device1, wc, state);// Pick configuration left corner

//    // Inserts all configuration that needs to be visited
//    confi_along_path.push_back(home);
//    confi_along_path.push_back(from);
//    confi_along_path.push_back(to);
//    confi_along_path.push_back(home);

//    for(unsigned int i = 0; i < 60; i++)
//    {
//        // Time logging
//        auto start = std::chrono::high_resolution_clock::now();

//        path = calculate_between_configurations_path_rrt(confi_along_path, wc, state, device, tool_frame, cylinder_frame);

//        auto stop = std::chrono::high_resolution_clock::now();
//        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop-start);
//        path_planning_time.push_back(duration.count());

//        paths.push_back(path);
//        state = wc->getDefaultState();
//        cylinder_frame_moveable->moveTo(newCylinder, state);
//    }
//    print_trajectory_transform("rrtConnect_transform_pickplace_left.txt", paths, wc, device, tool_frame);
//    print_trajectory_configuration("rrtConnect_configuration_pickplace_left.txt", paths);
//    print_trajectory_planning_time("rrtConnect_planning_times_pickplace_left.txt", path_planning_time);

    return 0;
}


