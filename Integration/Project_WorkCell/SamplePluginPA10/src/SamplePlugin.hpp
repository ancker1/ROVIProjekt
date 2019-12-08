#ifndef SAMPLEPLUGIN_HPP
#define SAMPLEPLUGIN_HPP

// Qt
#include <QTimer>

// RobWork includes
#include <rw/models/WorkCell.hpp>
#include <rw/kinematics/State.hpp>
#include <rwlibs/opengl/RenderImage.hpp>
#include <rwlibs/simulation/GLFrameGrabber.hpp>
#include <rwlibs/simulation/GLFrameGrabber25D.hpp>

#include <rw/rw.hpp>
#include <rwlibs/pathplanners/rrt/RRTPlanner.hpp>
#include <rwlibs/pathplanners/rrt/RRTQToQPlanner.hpp>
#include <rwlibs/proximitystrategies/ProximityStrategyFactory.hpp>
#include <rw/trajectory/LinearInterpolator.hpp>

// RobWorkStudio includes
#include <RobWorkStudioConfig.hpp> // For RWS_USE_QT5 definition
#include <rws/RobWorkStudioPlugin.hpp>

// OpenCV 3
#include <opencv2/opencv.hpp>

#include "ui_SamplePlugin.h"

#include <rws/RobWorkStudio.hpp>

#include <QPushButton>

#include <rw/loaders/ImageLoader.hpp>
#include <rw/loaders/WorkCellFactory.hpp>

#include <functional>

// PCL
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

// STD
#include <random>

class SamplePlugin: public rws::RobWorkStudioPlugin, private Ui::SamplePlugin {
Q_OBJECT
Q_INTERFACES( rws::RobWorkStudioPlugin )
Q_PLUGIN_METADATA(IID "dk.sdu.mip.Robwork.RobWorkStudioPlugin/0.1" FILE "plugin.json")
public:
    SamplePlugin();
    virtual ~SamplePlugin();
    virtual void open(rw::models::WorkCell* workcell);
    virtual void close();
    virtual void initialize();

private slots:
    void btnPressed();
    void timer();
    void getImage();
    void get25DImage();
    void poseEstimateM2();
    void showPoseEstimate();
    void runRRT();
    void moveObjectRandomly();
    void combinedSolution();
    void resetRobotAndObject();
    std::string get25DImage(std::string);
    void runLinearInterpolation();
    void stateChangedListener(const rw::kinematics::State& state);

private:
    static cv::Mat toOpenCVImage(const rw::sensor::Image& img);
    QTimer* _timer;
    QTimer* _timer25D;
    rw::models::WorkCell::Ptr _wc;
    rw::kinematics::State _state;
    rwlibs::opengl::RenderImage *_textureRender, *_bgRender;
    rwlibs::simulation::GLFrameGrabber* _framegrabber;
    rwlibs::simulation::GLFrameGrabber25D* _framegrabber25D;    
    std::vector<std::string> _cameras;
    std::vector<std::string> _cameras25D;
    rw::models::Device::Ptr _device;
    rw::trajectory::QPath _path;
    rw::kinematics::MovableFrame* _duck;
    rw::kinematics::Frame* _table;
    rw::kinematics::Frame* _tcp;
    unsigned int _step;
    unsigned int _stepGrasp;
    unsigned int _stepRelease;
    rw::math::Transform3D<> _pose;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _scene;
    pcl::PointCloud<pcl::PointXYZ>::Ptr _object;
    rw::models::SerialDevice::Ptr _UR6;
    rw::kinematics::MovableFrame * _scanner25D;
    rw::kinematics::MovableFrame * _target;
    rw::math::Transform3D<> _place;
    rw::math::Q _home;
    rw::math::Transform3D<> _duckPos;
    rw::math::Transform3D<> _duckHomePos;

    //pcl::common::UniformGenerator<int> x_gen(-300, 300, 42); // mm
    //pcl::common::UniformGenerator<int> y_gen(350, 550, 42); // mm
    //pcl::common::UniformGenerator<int> zrot_gen(-3141, 3141, 42); // mm
    std::default_random_engine _generator;
    std::uniform_int_distribution<int> _xdist{-300,300};
    std::uniform_int_distribution<int> _ydist{350,550};
    std::uniform_int_distribution<int> _zrotdist{-3141, 3141};


};

#endif /*RINGONHOOKPLUGIN_HPP_*/
