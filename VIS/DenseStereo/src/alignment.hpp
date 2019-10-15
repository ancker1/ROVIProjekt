#pragma once

#include "local_alignment.hpp"
#include "global_alignment.hpp"

namespace align {

    void showTwoPointClouds( pcl::PointCloud<pcl::PointXYZ>::Ptr scene, pcl::PointCloud<pcl::PointXYZ>::Ptr object )
    {
        pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->setBackgroundColor(0, 0, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> green(scene, 0, 255, 0);
        pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> red(object, 255, 0, 0);
        viewer->addPointCloud<pcl::PointXYZ>(scene, green, "Scene");
        viewer->addPointCloud<pcl::PointXYZ>(object, red, "Object");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Scene");
        viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "Object");

        viewer->initCameraParameters();
        // viewer->setCameraClipDistances
        // viewer->setCameraPosition
        // viewer->setPosition

        while(!viewer->wasStopped()){
            viewer->spinOnce(100);
            boost::this_thread::sleep( boost::posix_time::millisec(100) );
        }
    }

}
