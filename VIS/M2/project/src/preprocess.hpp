#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/segmentation/extract_polygonal_prism_data.h>
#include <pcl/surface/convex_hull.h>

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

    pcl::PointIndices::Ptr findPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud)
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
        return planeIndices;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr preprocessScene(pcl::PointCloud<pcl::PointXYZ>::Ptr scene)
    {
        preprocess::PointCloud::thresholdAxis(scene, scene, "z", -2.0f, 0.0f);   // keep points, where z in [-2.0 , 0]
        preprocess::PointCloud::voxelGrid(scene, scene, 0.005f);                // Apply Voxel Grid with leaf size 5 [mm]

        //showCloud(scene);
        //std::cout << "Size of cloud" << scene->points.size() << std::endl;

        // Find largest plane in point cloud
        pcl::PointCloud<pcl::PointXYZ>::Ptr plane       (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objects     (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull  (new pcl::PointCloud<pcl::PointXYZ>);


        pcl::PointIndices::Ptr planeIndices = findPlane(scene);


        if (planeIndices->indices.size() == 0)
                std::cout << "No plane found" << std::endl;
        else
        {

            pcl::ExtractIndices<pcl::PointXYZ> extractIndices;
            extractIndices.setInputCloud(scene);
            extractIndices.setIndices(planeIndices);
            extractIndices.filter(*plane);

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

        }
        return objects;
    }


    }
}
