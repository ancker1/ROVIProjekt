#pragma once

#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>

namespace align { namespace local {

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 ICP ( pcl::PointCloud<pcl::PointXYZ>::Ptr &scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &object )
    {
        pcl::search::KdTree<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(scene);

        int k = 1;
        std::vector<int> k_indices(k);
        std::vector<float> k_sqr_dist(k);

        pcl::PointCloud<pcl::PointXYZ>::Ptr a(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr b(new pcl::PointCloud<pcl::PointXYZ>);

        a->width = object->points.size();
        a->height = 1;
        a->is_dense = false;
        a->resize(a->height * a->width);

        b->width = object->points.size();
        b->height = 1;
        b->is_dense = false;
        b->resize(b->height * b->width);

        for (unsigned int i = 0; i < object->points.size(); i++)
        {
            kdTree.nearestKSearch(object->points[i], k, k_indices, k_sqr_dist);
            for (unsigned int j = 0; j < k; j++)
                if ( k_sqr_dist[j] < 0.0001f ) // squared distance threshold
                {
                    a->points[i] = object->points[i];
                    b->points[i] = scene->points[k_indices[j]];
                }
        }
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> ESTSVD;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform;

        ESTSVD.estimateRigidTransformation(*a, *b, transform);
        pcl::transformPointCloud(*object, *object, transform);
        return transform;
    }

    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 ICP( pcl::PointCloud<pcl::PointXYZ>::Ptr &scene, pcl::PointCloud<pcl::PointXYZ>::Ptr &object, int iterations )
    {
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform = ICP(scene, object);
        for (unsigned int i = 0; i < iterations - 1; i++) {
            transform = transform * ICP(scene, object);
        }
        return transform;
    }

}}
