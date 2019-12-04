#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/features/spin_image.h>


#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/visualization/cloud_viewer.h>


#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include "alignment.hpp"

#include <iostream>


cv::Mat calculateDisparityMap( cv::Mat imageLeft, cv::Mat imageRight, int numDisparities, int blockSize )
{
    /*
     *  Calculate disparity map using Block Matching
     */
    cv::Mat disparity;

    //cv::Ptr<cv::StereoBM> sbm = cv::StereoBM::create( numDisparities, blockSize );
    //sbm->compute(imageLeft, imageRight, disparity);

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create(0, numDisparities, blockSize);
    sgbm->compute(imageLeft, imageRight, disparity);

    return disparity;
}

void visualizeDisparity( cv::Mat disparity )
{
    cv::Mat normalizedDisparity;
    cv::normalize(disparity, normalizedDisparity, 0, 255, cv::NORM_MINMAX, CV_8UC1);    // normalize to [0, 255] to visualize
    cv::imshow("Disparity", normalizedDisparity);
    cv::waitKey(0);
}

cv::Mat defineQ(int imgWidth, int imgHeight, double Tx, double f)
{
    cv::Mat Q = (cv::Mat_<double>(4,4) << 1, 0, 0, double(-imgWidth)/2.0,0,1,0,double(-imgHeight)/2.0,0,0,0,f,0,0,double(1.0/Tx),0);
    return Q;
}

void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // might be better to show it with colors
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);
    viewer->addPointCloud<pcl::PointXYZ>(cloud, "cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::milliseconds(100));
    }
}

pcl::PointCloud<pcl::PointXYZ>::Ptr depthToCloud(cv::Mat depthMap, float z_threshold)
{
    pcl::PointXYZ defaultPoint;
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene( new pcl::PointCloud<pcl::PointXYZ>(depthMap.rows, depthMap.cols, defaultPoint) );
    for (unsigned int i = 0; i < depthMap.rows; i++) {
        for (unsigned int j = 0; j < depthMap.cols; j++){
            pcl::PointXYZ point;
            cv::Vec3f pos = depthMap.at<cv::Vec3f>(i,j);
            if ( fabs(pos[2]) < z_threshold ){
                point.x = pos[0];
                point.y = pos[1];
                point.z = pos[2];
                scene->at(i,j) = point;
            }
        }
    }
    return scene;
}

int DenseStereo(cv::Mat leftImage, cv::Mat rightImage)
{
    std::cout << "-- Begin Dense Stereo --" << std::endl;

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



    /*******************************************************************
     *      Load images
     *******************************************************************/
    if ( leftImage.empty() || rightImage.empty() ){
        std::cout << "Error loading images" << std::endl;
        return -1;
    }

    cv::imshow("Left image", leftImage);
    cv::waitKey(0);
    cv::imshow("Right image", rightImage);
    cv::waitKey(0);

    /*******************************************************************
     *  Save colors and cast to gray scale
    *******************************************************************/
    cv::Mat colors = leftImage;                         // Save colors from left image
    cv::cvtColor(leftImage, leftImage, CV_BGR2GRAY);
    cv::cvtColor(rightImage, rightImage, CV_BGR2GRAY);

    cv::imshow("Left image", leftImage);
    cv::waitKey(0);
    cv::imshow("Right image", rightImage);
    cv::waitKey(0);



    /*******************************************************************
     *  Calculate disparity and display
    *******************************************************************/
    int numDisparity = 32, blockSize = 11;
    cv::Mat disparityMap = calculateDisparityMap(leftImage, rightImage, numDisparity, blockSize);
    visualizeDisparity( disparityMap );

    /*******************************************************************
     *  Assuming it is a rectified system
    *******************************************************************/
    float Tx = 100, f = 500, z_threshold = 500;  // What is focal length???
    cv::Mat Q = defineQ(leftImage.cols, leftImage.rows, Tx, f);

    cv::Mat depthMap;
    cv::reprojectImageTo3D(disparityMap, depthMap, Q);

    pcl::PointCloud<pcl::PointXYZ>::Ptr scene = depthToCloud(depthMap, z_threshold);

    showCloud(scene);
    pcl::io::savePCDFileASCII("scene.pcd", *scene);
    /*******************************************************************
     *  Load and show point cloud
     *******************************************************************/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>); // Point cloud with XYZ

    pcl::PCDReader reader;  // Read cloud data from PCD files
    reader.read("../../ObjectFiles/cylinder_voxel.pcd", *cloud_object);
    showCloud(cloud_object);


    std::cout << "-- Done --" << std::endl;
}

void thresholdZ( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float lowThreshold, float highTreshold )
{
    pcl::PassThrough<pcl::PointXYZ> spatialFilter;
    spatialFilter.setInputCloud (input_cloud);
    spatialFilter.setFilterFieldName ("z");
    spatialFilter.setFilterLimits (lowThreshold, highTreshold); //  prev: -2.0, 0
    spatialFilter.filter(*output_cloud);
}

void voxelGrid( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float leafSize )
{
    pcl::VoxelGrid<pcl::PointXYZ> vxlGrid;
    vxlGrid.setInputCloud(input_cloud);
    vxlGrid.setLeafSize(leafSize, leafSize, leafSize);
    vxlGrid.filter(*output_cloud);
}

void poseEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene)
{
    std::cout << "[Start] Object count: " << cloud_object->points.size() << std::endl;
    voxelGrid(cloud_object, cloud_object, 0.005f);
    std::cout << "[Voxel] Object count: " << cloud_object->points.size() << std::endl;

    /*
     * Filter scene
     * by depth
    */
    std::cout << "[Start] Scene count: " << cloud_scene->points.size() << std::endl;
    thresholdZ(cloud_scene, cloud_scene, -2.0f, 0.0f);
    std::cout << "[Threshold z] Scene count: " << cloud_scene->points.size() << std::endl;
    voxelGrid(cloud_scene, cloud_scene, 0.005f);
    std::cout << "[Voxel] Scene count: " << cloud_scene->points.size() << std::endl;

    align::showTwoPointClouds(cloud_scene, cloud_object);

    pcl::PointCloud<pcl::Normal>::Ptr normals_scene = align::global::calculateNormals(cloud_scene, 10);
    pcl::PointCloud<pcl::Normal>::Ptr normals_object = align::global::calculateNormals(cloud_object,10);


    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_scene = align::global::calculateSpinImage(cloud_scene, normals_scene, 0.5);
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_object = align::global::calculateSpinImage(cloud_object, normals_object, 0.5);

    std::vector<float> featureDists;
    std::vector<int> nearest_indices = align::global::findNearestFeatures(spinImage_scene, spinImage_object);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = align::global::RANSAC_transform(cloud_scene, cloud_object, nearest_indices, 0.005f);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_tfed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_object, *object_tfed, transform);

    align::showTwoPointClouds(cloud_scene, object_tfed);
}

void preprocess_scene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
{
    thresholdZ(scene, scene, -2.0f, 0.0f); // keep points, where z in [-2.0 , 0]
    voxelGrid(scene, scene, 0.005f);       // Apply Voxel Grid with leaf size 5 [mm]

    //showCloud(scene);

    // Find largest plane in point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(scene);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(scene);
    extract.setIndices(planeIndices);
    extract.setNegative(true);
    extract.filter(*scene);

    //showCloud(cloud);
}

int main(int argc, char** argv)
{
    if ( argc != 3 ){
        std::cout << "Usage:" << std::endl;
        std::cout << "leftImgPath.png rightImgPath.png" << std::endl;
        return -1;
    }
    //cv::Mat leftImage   = cv::imread(argv[1]);
    //cv::Mat rightImage  = cv::imread(argv[2]);
    //DenseStereo(leftImage, rightImage);
    //return 0;
    /*
     * Read PCD files
    */

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>); // Point cloud with XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene (new pcl::PointCloud<pcl::PointXYZ>); // Point cloud with XYZ

    pcl::PCDReader reader;  // Read cloud data from PCD files
    reader.read("../../ObjectFiles/duck_voxel.pcd", *cloud_object);
    reader.read("../../ObjectFiles/scene_duck.pcd", *cloud_scene);
    //reader.read("/home/emil/Dropbox/UNI/MSc/Vision/exercises/lecture6/ex2/object-global.pcd", *cloud_object);
    //reader.read("/home/emil/Dropbox/UNI/MSc/Vision/exercises/lecture6/ex2/scene.pcd", *cloud_scene);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr objects(new pcl::PointCloud<pcl::PointXYZ>);
    reader.read("../../ObjectFiles/scene_duck.pcd", *cloud);

    /*
     *  Find plane -> remove points
     */

    std::cout << "[Start] Scene count: " << cloud->points.size() << std::endl;
    thresholdZ(cloud, cloud, -2.0f, 0.0f);
    std::cout << "[Threshold z] Scene count: " << cloud->points.size() << std::endl;
    voxelGrid(cloud, cloud, 0.005f);
    std::cout << "[Voxel] Scene count: " << cloud->points.size() << std::endl;

    showCloud(cloud);

    // Get the plane model, if present.
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::SACSegmentation<pcl::PointXYZ> segmentation;
    segmentation.setInputCloud(cloud);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setDistanceThreshold(0.01);
    segmentation.setOptimizeCoefficients(true);
    pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
    segmentation.segment(*planeIndices, *coefficients);

    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(planeIndices);
    extract.setNegative(true);
    extract.filter(*cloud);

    showCloud(cloud);



    return 0;
}
