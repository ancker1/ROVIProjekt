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
#include <pcl/features/spin_image.h>

#include <pcl/visualization/cloud_viewer.h>


#include <rw/rw.hpp>
#include <rw/loaders/WorkCellLoader.hpp>

#include <rwslibs/rwstudioapp/RobWorkStudioApp.hpp>

#include "alignment.hpp"
#include "preprocess.hpp"

#include <iostream>
#include <fstream>


void showCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
{
    // might be better to show it with colors
    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer ("3D Viewer"));
    viewer->setBackgroundColor(0, 0, 0);

    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(cloud, 0, 255, 0);

    viewer->addPointCloud<pcl::PointXYZ>(cloud, green, "cloud");
    //viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cloud");

    viewer->initCameraParameters();

    while (!viewer->wasStopped ())
    {
      viewer->spinOnce(100);
      boost::this_thread::sleep (boost::posix_time::milliseconds(100));
    }
}

pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 poseEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene)
{

    std::cout << "[Start] Object count: " << cloud_object->points.size() << std::endl;
    preprocess::PointCloud::voxelGrid(cloud_object, cloud_object, 0.005f); // 5[mm] leaf size
    std::cout << "[Voxel] Object count: " << cloud_object->points.size() << std::endl;

    std::cout << "[Start] Scene count: " << cloud_scene->points.size() << std::endl;
    //preprocess::PointCloud::thresholdAxis(cloud_scene, cloud_scene, "z", -2.0f, 0.0f);
    //std::cout << "[Threshold z] Scene count: " << cloud_scene->points.size() << std::endl;
    //preprocess::PointCloud::voxelGrid(cloud_scene, cloud_scene, 0.005f);
    //std::cout << "[Voxel] Scene count: " << cloud_scene->points.size() << std::endl;

    align::showTwoPointClouds(cloud_scene, cloud_object);

    pcl::PointCloud<pcl::Normal>::Ptr normals_scene = align::global::calculateNormals(cloud_scene, 10);
    pcl::PointCloud<pcl::Normal>::Ptr normals_object = align::global::calculateNormals(cloud_object,10);


    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_scene = align::global::calculateSpinImage(cloud_scene, normals_scene, 0.05); // 0.05
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_object = align::global::calculateSpinImage(cloud_object, normals_object, 0.05);

    std::vector<int> nearest_indices = align::global::findNearestFeatures(spinImage_scene, spinImage_object);


    //pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = align::global::RANSAC_transform(cloud_scene, cloud_object, nearest_indices, 0.005f);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = align::global::RANSAC(cloud_scene, cloud_object, nearest_indices);
    return transform;
}

void TEST_RANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr object, pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
{
    preprocess::PointCloud::voxelGrid(object, object, 0.005f); // 5[mm] leaf size
    // run 30 times
    pcl::PointCloud<pcl::Normal>::Ptr normals_scene = align::global::calculateNormals(scene, 10);
    pcl::PointCloud<pcl::Normal>::Ptr normals_object = align::global::calculateNormals(object,10);


    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_scene = align::global::calculateSpinImage(scene, normals_scene, 0.05); // 0.05
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_object = align::global::calculateSpinImage(object, normals_object, 0.05);

    std::vector<int> nearest_indices = align::global::findNearestFeatures(spinImage_scene, spinImage_object);

    std::cout << "Object points: " << object->points.size() << std::endl;
    std::cout << "Scene points: " << scene->points.size() << std::endl;
    std::cout << "----- RANSAC execution time test started -----" << std::endl;

    std::vector<int> optimizedRANSAC;
    for (unsigned int i = 0; i < 30; i++) {
        auto start = std::chrono::high_resolution_clock::now();
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = align::global::RANSAC(scene, object, nearest_indices);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        int dur_ms = duration.count();
        optimizedRANSAC.push_back(dur_ms);
        std::cout << "RANSAC execution time: " << dur_ms << " [ms]" << std::endl;
    }

    ofstream outfile;
    outfile.open("/home/emil/Documents/RansacTest/ransacOptimizedTime.txt");
    for (int dur : optimizedRANSAC)
        outfile << dur << endl;
    outfile.close();

    std::vector<int> timeRANSAC;
    for (unsigned int i = 0; i < 30; i++) {
        auto start = std::chrono::high_resolution_clock::now();
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = align::global::RANSAC_transform(scene, object, nearest_indices, 0.005f);
        auto stop = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(stop - start);
        int dur_ms = duration.count();
        timeRANSAC.push_back(dur_ms);
        std::cout << "RANSAC execution time: " << dur_ms << " [ms]" << std::endl;
    }

    outfile.open("/home/emil/Documents/RansacTest/ransacTime.txt");
    for (int dur : timeRANSAC)
        outfile << dur << endl;
    outfile.close();
}

rw::math::Transform3D<double> getGroundTruthPose()
{
    static const std::string wc_path = "/home/emil/Desktop/Project_WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( wc.isNull() ){
        RW_THROW("Error loading workcell");
    }

    rw::kinematics::MovableFrame *duckFrame = wc->findFrame<rw::kinematics::MovableFrame>("duck");
    if( duckFrame == nullptr ){
        RW_THROW("Error finding frame: duck");
    }
    rw::kinematics::State state = wc->getDefaultState();

    return duckFrame->getTransform(state);
}

void genRandomPoses()
{
    static const std::string wc_path = "/home/emil/Desktop/Project_WorkCell/Scene.wc.xml";
    const rw::models::WorkCell::Ptr wc = rw::loaders::WorkCellLoader::Factory::load(wc_path);
    if ( wc.isNull() ){
        RW_THROW("Error loading workcell");
    }

    rw::kinematics::MovableFrame *duckFrame = wc->findFrame<rw::kinematics::MovableFrame>("duck");
    if( duckFrame == nullptr ){
        RW_THROW("Error finding frame: duck");
    }
    rw::kinematics::State state = wc->getDefaultState();

    pcl::common::UniformGenerator<int> x_gen(-300, 300, 42); // mm
    pcl::common::UniformGenerator<int> y_gen(350, 550, 42); // mm
    pcl::common::UniformGenerator<int> zrot_gen(-3141, 3141, 42); // mm



    ofstream outfile, outfile2;
    outfile.open("/home/emil/Documents/posesRW.txt");
    outfile2.open("/home/emil/Documents/posesMATLAB.txt");
    for (unsigned int i = 0; i < 30; i++)
    {
        rw::math::RPY<> rotz(double(zrot_gen.run())/1000.0,0,0);
        rw::math::Transform3D<> tf(rw::math::Vector3D<>(double(x_gen.run())/1000.0, double(y_gen.run())/1000.0, duckFrame->getTransform(state).P()[2]),rotz.toRotation3D());
        rw::math::RPY<> rot(tf.R());
        outfile << "<RPY> " << rot[0]*180.0/M_PI << " 0 0 </RPY> <Pos> " << tf.P()[0] << " " << tf.P()[1] << " " << tf.P()[2] << " </Pos>" << endl;

        outfile2 << tf.R().getRow(0)[0] << " " << tf.R().getRow(0)[1] << " " << tf.R().getRow(0)[2] << " " << tf.P()[0] << endl;
        outfile2 << tf.R().getRow(1)[0] << " " << tf.R().getRow(1)[1] << " " << tf.R().getRow(1)[2] << " " << tf.P()[1] << endl;
        outfile2 << tf.R().getRow(2)[0] << " " << tf.R().getRow(2)[1] << " " << tf.R().getRow(2)[2] << " " << tf.P()[2] << endl;
        outfile2 <<          0          << " " <<          0          << " " <<          0          << " " <<      1    << endl;
    }
    outfile.close();
    outfile2.close();
}

void testPoseEstimation()
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);  // Point cloud with XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene  (new pcl::PointCloud<pcl::PointXYZ>);   // Point cloud with XYZ
    pcl::PCDReader reader;  // Read cloud data from PCD files
    reader.read("../../ObjectFiles/duck_voxel.pcd", *cloud_object);
    std::string path = "/home/emil/Documents/M2ErrorEval/pcds/c";
    std::string typ = ".pcd";
    std::vector<pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4> tfs;
    for(unsigned int i = 1; i < 31; i++)
    {
        std::string full_path = path+std::to_string(i)+typ;
        std::cout << "(" << i << "/30) " << full_path << std::endl;
        reader.read(full_path, *cloud_scene );
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sceneObjects = preprocess::PointCloud::preprocessScene(cloud_scene);
        std::cout << "A3" << std::endl;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = poseEstimate(cloud_object, cloud_sceneObjects);
        std::cout << "A4" << std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr object_tfed (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::transformPointCloud(*cloud_object, *object_tfed, transform);
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_ICP = align::local::ICP(cloud_sceneObjects, object_tfed, 50);
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 combinedTransform = transform * T_ICP;
        std::cout << "A11" << std::endl;
        tfs.push_back(combinedTransform);
        std::cout << "B11" << std::endl;
    }
    ofstream outfile;
    outfile.open("/home/emil/Documents/M2ErrorEval/poseEstTestCombined.txt");
    for(pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 tf : tfs)
        outfile << tf << endl;
    outfile.close();
}

int main(int argc, char** argv)
{
   /* if ( argc != 3 ){
        std::cout << "Usage:" << std::endl;
        std::cout << "leftImgPath.png rightImgPath.png" << std::endl;
        return -1;
    } */
   // testPoseEstimation();
   // return 0;

    /**** Read point clouds ****/
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_object (new pcl::PointCloud<pcl::PointXYZ>);  // Point cloud with XYZ
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_scene  (new pcl::PointCloud<pcl::PointXYZ>);   // Point cloud with XYZ

    pcl::PCDReader reader;  // Read cloud data from PCD files
    reader.read("../../ObjectFiles/duck_voxel.pcd", *cloud_object);
    reader.read("../../ObjectFiles/scene_duck.pcd", *cloud_scene );

    /**** Pose Estimation ****/
   // showCloud(cloud_scene);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sceneObjects = preprocess::PointCloud::preprocessScene(cloud_scene);
   // showCloud(cloud_sceneObjects);

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = poseEstimate(cloud_object, cloud_sceneObjects);
    std::cout << "Pose: " <<transform << std::endl;
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_tfed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*cloud_object, *object_tfed, transform);

   // align::showTwoPointClouds(cloud_sceneObjects, object_tfed);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_ICP = align::local::ICP(cloud_sceneObjects, object_tfed, 50);
    std::cout << "(ICP) Pose: " << T_ICP << std::endl;
   // align::showTwoPointClouds(cloud_sceneObjects, object_tfed);

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 combinedTransform = transform * T_ICP;
    std::cout << "(Combined) Pose: " << combinedTransform << std::endl;

//    ofstream outfile;
//    outfile.open("/home/emil/Documents/M2ErrorEval/combinedTransform.txt");
//        outfile << combinedTransform << endl;
//    outfile.close();
    return 0;

    /**** RANSAC execution time test ****/
    //pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sceneObjectsa = preprocess::PointCloud::preprocessScene(cloud_scene);
    //TEST_RANSAC(cloud_object, cloud_sceneObjectsa);
    //return 0;
    /**** Generate 30 poses for pose estimation test ****/
    //genRandomPoses();
    //return 0;
    /**** Get (default) ground truth ****/
 //   rw::math::Transform3D<> groundTruth = getGroundTruthPose();
 //   std::cout << groundTruth << std::endl;


}
