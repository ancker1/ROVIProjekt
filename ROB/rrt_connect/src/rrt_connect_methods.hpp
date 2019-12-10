#pragma once

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>


#include "util.hpp"

#define ESTEPSIZE 0.4 // Chosen optimal step size of RRT-connect.

rw::trajectory::QPath calculate_between_two_config_rrt(rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame, rw::math::Q from, rw::math::Q to)
{
    rw::trajectory::QPath path;

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

rw::trajectory::QPath calculate_between_configurations_path_rrt(std::vector<rw::math::Q> confi_along_path, rw::models::WorkCell::Ptr workcell, rw::kinematics::State state, rw::models::Device::Ptr robot, rw::kinematics::Frame* tool_frame, rw::kinematics::Frame* object_frame)
{
    rw::trajectory::QPath whole_path;
    rw::trajectory::QPath sub_path;
    rw::math::Q from;
    rw::math::Q to;

    for(unsigned int i = 0; i < confi_along_path.size() - 1; i++)
    {
        from = confi_along_path[i];
        to = confi_along_path[i+1];
        std::cout << "Planning from " << from << " to " << to << std::endl;
        sub_path = calculate_between_two_config_rrt(workcell, state, robot, tool_frame, object_frame, from, to);
        for(unsigned int j = 0; j < sub_path.size(); j++)
        {
            whole_path.push_back(sub_path[j]);
        }
    }

    return whole_path;
}
