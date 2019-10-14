#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>


#include <iostream>


cv::Mat calculateDisparityMap( cv::Mat imageLeft, cv::Mat imageRight, int numDisparities, int blockSize )
{
    /*
     *  Calculate disparity map using Block Matching
     */
    cv::Mat disparity;
    cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create( numDisparities, blockSize );
    sbm->compute(imageLeft, imageRight, disparity);
    return disparity;
}

void visualizeDisparity( cv::Mat disparity )
{
    cv::Mat normalizedDisparity;
    cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);    // normalize to [0, 255] to visualize
    cv::imshow("Disparity", normalizedDisparity);
    cv::waitKey(0);
}

void show_cloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> color_g(cloud, 0, 255, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, color_g, "cloud");
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    //viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    //viewer->setCameraClipDistances(0.244446, 0.824412);
    //viewer->setCameraPosition(-0.0562254,-0.0666346,-0.529442,-0.00165773, -0.0633305, -0.167617,0.982829,0.108549,-0.149214);
    //viewer->setPosition(662,137);

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce (100);
      boost::this_thread::sleep (boost::posix_time::microseconds (100000));
    }
}

int main(int argc, char** argv)
{
    if ( argc != 3 ){
        std::cout << "Usage:" << std::endl;
        std::cout << "leftImgPath.png rightImgPath.png" << std::endl;
        return -1;
    }

    std::cout << "-- Begin --" << std::endl;

    /*******************************************************************
     *  RobWork Camera
     *******************************************************************/
        // Load workcell
    static const std::string wc_path = "../../Project_WorkCell_Cam/Project_WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( wc.isNull() ){
        RW_THROW("Error loading workcell");
        return -1;
    }
        // Load camera frames
    rw::kinematics::Frame* const cam_left = wc->findFrame("Camera_Left");
    rw::kinematics::Frame* const cam_right = wc->findFrame("Camera_Right");
    if ( cam_left == nullptr || cam_right == nullptr ) {
        RW_THROW("Error finding cameras");
        return -1;
    }
        // Load parameters: same for both cameras
    double fovy;
    int width, height;
    const rw::common::PropertyMap& properties = cam_left->getPropertyMap();
    if ( !properties.has("Camera") ){
        RW_THROW("Camera does not have camera property.");
        return -1;
    }
    const std::string params = properties.get<std::string>("Camera");
    std::istringstream stringstream (params, std::istringstream::in);
    stringstream >> fovy >> width >> height;
    std::cout << "Camera properties: fov " << fovy << " width " << width << " height " << height << std::endl;

    const std::string arg_rw = "";
    rws::RobWorkStudioApp rwapp("");
    /*
    rwapp.start();
    while( rwapp.getRobWorkStudio() == nullptr ){
        if( !rwapp.isRunning() ){
            RW_THROW("Could not start RobworkStudio app");
            return -1;
        }
        rw::common::TimerUtil::sleepMs(100);
    }
    rws::RobWorkStudio* const rwstudio = rwapp.getRobWorkStudio();
    rwstudio->postOpenWorkCell(wc_path);
*/


    /*******************************************************************
     *      Load images
     *******************************************************************/
    cv::Mat leftImage   = cv::imread(argv[1]);
    cv::Mat rightImage  = cv::imread(argv[2]);
    if ( leftImage.empty() || rightImage.empty() ){
        std::cout << "Error loading images" << std::endl;
        return -1;
    }

    /*******************************************************************
     *  Save colors and cast to gray scale
    *******************************************************************/
    cv::Mat colors = leftImage;                         // Save colors from left image
    cv::cvtColor(leftImage, leftImage, CV_BGR2GRAY);
    cv::cvtColor(rightImage, rightImage, CV_BGR2GRAY);

    /*******************************************************************
     *  Calculate disparity and display
    *******************************************************************/
    int numDisparity = 16, blockSize = 11;
    cv::Mat disparityMap = calculateDisparityMap(leftImage, rightImage, numDisparity, blockSize);
    visualizeDisparity( disparityMap );

    /*******************************************************************
     *  If it is a rectified system:
     *  - Images are from same image plane.
     *  - Depth can be computed from disparity.
    *******************************************************************/
    // Q is recieved from rectification by openCV

//    cv::Mat Q;      // Camera calibrations is needed to define Q.

//    cv::Mat depthMat;
//    cv::reprojectImageTo3D(disparityMap, depthMat, Q);

    // get pointcloud from depthMat

    /*******************************************************************
     *  Load and show point cloud
     *******************************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>); // Point cloud with XYZ

    pcl::PCDReader reader;  // Read cloud data from PCD files
    reader.read("../../ObjectFiles/cylinder_voxel.pcd", *cloud_object);
    show_cloud(cloud_object);


    std::cout << "-- Done --" << std::endl;
    return 0;
}
