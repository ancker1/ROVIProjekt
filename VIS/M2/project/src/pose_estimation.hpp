#include "alignment.hpp"
#include "preprocess.hpp"


namespace poseEstimate {

pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 poseEstimateGlobal(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
    preprocess::PointCloud::voxelGrid(object, object, 0.005f);              // 5[mm] leaf size
    pcl::PointCloud<pcl::Normal>::Ptr normals_scene = align::global::calculateNormals(scene, 10);
    pcl::PointCloud<pcl::Normal>::Ptr normals_object = align::global::calculateNormals(object,10);
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_scene = align::global::calculateSpinImage(scene, normals_scene, 0.05); // 0.05
    pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImage_object = align::global::calculateSpinImage(object, normals_object, 0.05);
    std::vector<int> nearest_indices = align::global::findNearestFeatures(spinImage_scene, spinImage_object);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = align::global::RANSAC(scene, object, nearest_indices);
    return transform;
}

pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 poseEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sceneObjects = preprocess::PointCloud::preprocessScene(scene);
    preprocess::PointCloud::voxelGrid(scene, scene, 0.005f);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = poseEstimateGlobal(cloud_sceneObjects, object);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_tfed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*object, *object_tfed, transform);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_ICP = align::local::ICP(cloud_sceneObjects, object_tfed, 50);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 combinedTransform = transform * T_ICP;
    return combinedTransform;
}

pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 poseEstimate(pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object, float sceneLeafSize)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_sceneObjects = preprocess::PointCloud::preprocessScene(scene);
    preprocess::PointCloud::voxelGrid(scene, scene, sceneLeafSize);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = poseEstimateGlobal(cloud_sceneObjects, object);
    pcl::PointCloud<pcl::PointXYZ>::Ptr object_tfed (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud(*object, *object_tfed, transform);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 T_ICP = align::local::ICP(cloud_sceneObjects, object_tfed, 50);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 combinedTransform = transform * T_ICP;
    return combinedTransform;
}

namespace visualize {
    // Add poseEstimation versions with visualizations
}

}
