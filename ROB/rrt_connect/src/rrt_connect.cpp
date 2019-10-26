#include <iostream>
#include <fstream>
#include <cstdio>
#include <cstdlib>
#include <rw/rw.hpp>
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
#define ESTEPSIZE 0.0000001 // Step size of RRT-connect.

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

int main(int argc, char** argv) {

    const string wcFile = "../../Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
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


    //Get the state
    State state = wc->getDefaultState();

    //These Q's contains the start and end configurations
    Q from(6, 2.30356, -2.9901, -0.152361, -3.14073, 2.80971, -3.14159); // Pick configuration Q{2.30356, -2.9901, -0.152361, -3.14073, 2.80971, -3.14159} (from side right corner)
    Q to(6, 0.900888, -0.897058, 1.64999, -0.752929, -0.669908, -3.14159); // Place configuration Q{0.900888, -0.897058, 1.64999, -0.752929, -0.669908, -3.14159} (middle in place area)

    //Set Q to the initial state and grip the bottle frame
    device->setQ(from, state); // sets initial state
    Kinematics::gripFrame(cylinder_frame, tool_frame, state); // Grip the bottle

    CollisionDetector detector(wc, ProximityStrategyFactory::makeDefaultCollisionStrategy());
    PlannerConstraint constraint = PlannerConstraint::make(&detector,device,state);

    QSampler::Ptr sampler = QSampler::makeConstrained(QSampler::makeUniform(device),constraint.getQConstraintPtr());
    QMetric::Ptr metric = MetricFactory::makeEuclidean<Q>();
    QToQPlanner::Ptr planner = RRTPlanner::makeQToQPlanner(constraint, sampler, metric, ESTEPSIZE, RRTPlanner::RRTConnect);

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

    cout << "Planning from " << from << " to " << to << endl;
    QPath path;
    Timer t;
    t.resetAndResume();

    //Use the planner to find a trajectory between the configurations
    planner->query(from, to, path);
    planner->make(constraint);

    t.pause();
    double distance = 0;

    cout << "Took secounds to calculate path: " << t.getTime()<< endl;

    if (t.getTime() >= MAXTIME)
    {
        cout << "Notice: max time of " << MAXTIME << " seconds reached." << endl;
    }

    //Appends the path to the LUA script. This file can be "played" in RobWorkStudio.
    for (unsigned int i = 0; i< path.size(); i++)
    {
        if(i == 1)
            myfile << "attach(cylinder, gripper)\n";
        if(i >= 1)
            distance += sqrt(pow((path.at(i)(0)-path.at(i-1)(0)),2)+pow((path.at(i)(1)-path.at(i-1)(1)),2)+pow((path.at(i)(2)-path.at(i-1)(2)),2)+pow((path.at(i)(3)-path.at(i-1)(3)),2)+pow((path.at(i)(4)-path.at(i-1)(4)),2)+pow((path.at(i)(5)-path.at(i-1)(5)),2)); // L2 distance
        //cout << path.at(i)(0) << endl;
        cout << distance << endl;
        myfile <<"setQ({" << path.at(i)(0) << "," << path.at(i)(1) << "," << path.at(i)(2) << "," << path.at(i)(3) << "," << path.at(i)(4) << "," << path.at(i)(5) << "})" << "\n";
    }

    myfile.close();
	return 0;
}
