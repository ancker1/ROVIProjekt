#pragma once
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/visualization/cloud_viewer.h>

namespace preprocess { namespace PointCloud{

    void voxelGrid( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, float leafSize )
    {
        pcl::VoxelGrid<pcl::PointXYZ> vxlGrid;
        vxlGrid.setInputCloud(input_cloud);
        vxlGrid.setLeafSize(leafSize, leafSize, leafSize);
        vxlGrid.filter(*output_cloud);
    }

    void thresholdAxis( pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud, pcl::PointCloud<pcl::PointXYZ>::Ptr &output_cloud, std::string axis, float lowThreshold, float highTreshold )
    {
        pcl::PassThrough<pcl::PointXYZ> spatialFilter;
        spatialFilter.setInputCloud (input_cloud);
        spatialFilter.setFilterFieldName (axis);
        spatialFilter.setFilterLimits (lowThreshold, highTreshold); //  prev: -2.0, 0
        spatialFilter.filter(*output_cloud);
    }

    void removeIndicesFromCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointIndices::Ptr indices)
    {
        pcl::ExtractIndices<pcl::PointXYZ> extract;
        extract.setInputCloud(cloud);
        extract.setIndices(indices);
        extract.setNegative(true);
        extract.filter(*cloud);
    }

    std::tuple<pcl::PointIndices::Ptr, pcl::ModelCoefficients::Ptr> findPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
    {
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        pcl::SACSegmentation<pcl::PointXYZ> segmentation;
        segmentation.setInputCloud(cloud);
        segmentation.setModelType(pcl::SACMODEL_PLANE);
        segmentation.setMethodType(pcl::SAC_RANSAC);
        segmentation.setDistanceThreshold(0.01);
        segmentation.setOptimizeCoefficients(true);
        pcl::PointIndices::Ptr planeIndices(new pcl::PointIndices);
        segmentation.segment(*planeIndices, *coefficients);
        //std::cout << coefficients->values.size() << std::endl;
        //plane_normal = (coefficients[0], coefficients[1], coefficients[2])
        return std::make_tuple(planeIndices, coefficients);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
    {
        preprocess::PointCloud::thresholdAxis(scene, scene, "z", -2.0f, 0.0f);   // keep points, where z in [-2.0 , 0]
        //preprocess::PointCloud::voxelGrid(scene, scene, 0.005f);                // Apply Voxel Grid with leaf size 5 [mm]

        //showCloud(scene);
        //std::cout << "Size of cloud" << scene->points.size() << std::endl;

        // Find largest plane in point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane       (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects     (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull  (new pcl::PointCloud<pcl::PointXYZ>);

        /**** Method: ConvexHull ****/
        auto planeInfo = findPlane(scene);
        pcl::PointIndices::Ptr planeIndices = std::get<0>(planeInfo);
        pcl::ModelCoefficients::Ptr planeCoeffs = std::get<1>(planeInfo);

        if (planeIndices->indices.size() == 0)
                std::cout << "No plane found" << std::endl;
        else
        {


            pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
            extractIndices.setInputCloud(scene);
            extractIndices.setIndices(planeIndices);
            extractIndices.filter(*plane);

            /****** Below/Above plane test *******/
            removeIndicesFromCloud(scene, planeIndices);
            std::vector<int> belowIndices;
            pcl::PointXYZ planePoint = scene->points[planeIndices->indices[0]]; // Random points on plane
            for ( unsigned int i = 0; i < scene->points.size(); i++ )
            {
                float dotProd = (scene->points[i].x - planePoint.x) * planeCoeffs->values[0] + (scene->points[i].y - planePoint.y) * planeCoeffs->values[1] + (scene->points[i].z - planePoint.z) * planeCoeffs->values[2];
                //std::cout << dotProd << std::endl;
                if ( dotProd < 0 )
                    belowIndices.push_back(i);
            }
            //boost::shared_ptr<std::vector<int>> below (new std::vector<int> (belowIndices));
            boost::shared_ptr<std::vector<int>> below_ptr = boost::make_shared<std::vector<int>>(belowIndices);
            //pcl::PointIndices::Ptr bealow;
            //bealow->indices(belowIndices);
            pcl::ExtractIndices<pcl::PointXYZ> extractBelowIndices;
            extractIndices.setInputCloud(scene);
            extractIndices.setIndices(below_ptr);
            extractIndices.setNegative(true);
            extractIndices.filter(*scene);

            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (scene);
            sor.setMeanK (20);
            sor.setStddevMulThresh (1.0);
            sor.filter (*scene);

            //removeIndicesFromCloud(scene, below);
            return scene;

            /* Work-around not using convex hull */
            removeIndicesFromCloud(scene, planeIndices);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sort;
            sor.setInputCloud (scene);
            sor.setMeanK (20);
            sor.setStddevMulThresh (1.0);
            sor.filter (*scene);
            preprocess::PointCloud::voxelGrid(scene, scene, 0.005f);
            return scene;
            /*
            pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
            prism.setInputCloud(scene);
            prism.setInputPlanarHull(plane);
            prism.setHeightLimits(0.01, 0.2);
            pcl::PointIndices::Ptr objsI(new pcl::PointIndices);
            prism.segment(*objsI);
            extractIndices.setIndices(objsI);
            extractIndices.filter(*objects);
            preprocess::PointCloud::voxelGrid(objects, objects, 0.005f);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud (objects);
            sor.setMeanK (20);
            sor.setStddevMulThresh (1.0);
            sor.filter (*objects);
            return objects;*/
            /* Work-around done */
            /*
            pcl::ConvexHull<pcl::PointXYZ> hull;         
            hull.setInputCloud(plane);
            hull.setDimension(2);
            std::cout << "(Begin) reconstruct, plane size: " << plane->points.size() << std::endl;
            hull.reconstruct(*convexHull);
            std::cout << "(Done) reconstruct" << std::endl;
            if (hull.getDimension() == 2)
            {
                // Prism object.
                pcl::ExtractPolygonalPrismData<pcl::PointXYZ> prism;
                prism.setInputCloud(scene);
                prism.setInputPlanarHull(convexHull);
                prism.setHeightLimits(0.01, 0.2); // Only keep objects with which are table + [1, 20] cm in height
                pcl::PointIndices::Ptr objectIndices(new pcl::PointIndices);
                prism.segment(*objectIndices);
                extractIndices.setIndices(objectIndices);
                extractIndices.filter(*objects);            // Get all points retreived by hull
            }
            */
        }
        return objects;
    }


    }
}
