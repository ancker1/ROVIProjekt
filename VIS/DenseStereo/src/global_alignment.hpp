#pragma once

#include <iostream>
#include <algorithm>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/search/kdtree.h>
#include <pcl/registration/transformation_estimation_svd.h>
#include <pcl/common/transforms.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/spin_image.h>
#include <pcl/common/random.h>

namespace align { namespace global {

    pcl::PointCloud<pcl::Normal>::Ptr calculateNormals( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, int kNearest)
    {
        pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> ne;
        ne.setInputCloud(cloud);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>() );
        ne.setSearchMethod(tree);
        pcl::PointCloud<pcl::Normal>::Ptr normals( new pcl::PointCloud<pcl::Normal> );
        ne.setKSearch( kNearest );
        ne.compute(*normals);
        return normals;
    }

    pcl::PointCloud<pcl::Histogram<153>>::Ptr calculateSpinImage( pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, pcl::PointCloud<pcl::Normal>::Ptr normals, double searchRadius )
    {   // Find parameters for spinImageEst
        pcl::SpinImageEstimation<pcl::PointXYZ, pcl::Normal, pcl::Histogram<153>> spinImageEst(8, 0.5, 4); // prev (8, 0.5, 4)
        spinImageEst.setInputCloud(cloud);
        spinImageEst.setInputNormals(normals);
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree( new pcl::search::KdTree<pcl::PointXYZ>() );
        spinImageEst.setSearchMethod(tree);
        spinImageEst.setRadiusSearch(searchRadius);
        pcl::PointCloud<pcl::Histogram<153>>::Ptr spinImages( new pcl::PointCloud<pcl::Histogram<153>> );
        spinImageEst.compute(*spinImages);
        return spinImages;
    }

    float l2Dist ( const pcl::Histogram<153> &histA, const pcl::Histogram<153> &histB )
    {
        float result = 0;
        for ( unsigned int i = 0; i < histA.descriptorSize(); i++ )
            result += std::sqrt( std::pow(histA.histogram[i] - histB.histogram[i], 2) );
        return result;
    }

    std::vector<int> findNearestFeatures( const pcl::PointCloud<pcl::Histogram<153>>::Ptr &scene, const pcl::PointCloud<pcl::Histogram<153>>::Ptr &object )
    {   // Finds nearest point in spinB for every point in spinA
        std::vector<int> nearestIndices;
        for (unsigned int i = 0; i < object->size(); i++) {
            float minDist = std::numeric_limits<float>::max();
            int indx = 0;
            for (unsigned int j = 0; j < scene->size(); j++) {
                float dist = l2Dist( object->points[i], scene->points[j] );
                if ( dist < minDist ){
                    minDist = dist;
                    indx = j;
                }
            }
            nearestIndices.push_back( indx );
        }
        return nearestIndices;
    }


    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 RANSAC_transform( const pcl::PointCloud<pcl::PointXYZ>::Ptr &scene, const pcl::PointCloud<pcl::PointXYZ>::Ptr &object, const std::vector<int> &nearestIndices, float inlierThreshold)
    {
        pcl::common::UniformGenerator<int> generator(0, nearestIndices.size()-1);
        pcl::PointCloud<pcl::PointXYZ>::Ptr objectTransformed( new pcl::PointCloud<pcl::PointXYZ> );

        pcl::PointCloud<pcl::PointXYZ>::Ptr a(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr b(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> ESTSVD;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 transform;
        pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 bestTransform;

        pcl::search::KdTree<pcl::PointXYZ> kdTree;
        kdTree.setInputCloud(scene);

        float epsilon = std::pow(inlierThreshold,2); // 5mm threshold

        a->width = 3;
        a->height = 1;
        a->is_dense = false;
        a->resize(a->height * a->width);

        b->width = 3;
        b->height = 1;
        b->is_dense = false;
        b->resize(b->height * b->width);

        const int num_it = 10000;
        unsigned int k = 1, max_inliers = 0;
        std::vector<int> k_indices(k);
        std::vector<float> k_sqr_dist(k);

        /*************************************************
         *  Optimize by adding preliminary check
         *************************************************/

        for (unsigned int it = 0; it < num_it; it++){
            unsigned int count_inliers = 0;

            for (unsigned int i = 0; i < 3; i++){
                int uni_int = generator.run();
                a->points[i] = object->points[uni_int];
                b->points[i] = scene->points[nearestIndices[uni_int]];
             }

            ESTSVD.estimateRigidTransformation(*a, *b, transform);
            pcl::transformPointCloud(*object, *objectTransformed, transform);


            for (unsigned int i = 0; i < objectTransformed->points.size(); i++){
                kdTree.nearestKSearch(objectTransformed->points[i], k, k_indices, k_sqr_dist);

                if ( k_sqr_dist[0] < epsilon ) {
                    count_inliers++;
                }
            }

            if ( count_inliers > max_inliers ){
                max_inliers = count_inliers;
                bestTransform = transform;
                std::cout << "New model: iteration = " << it << ", inliers = " << count_inliers << std::endl;
            }
            if ( it % 500 == 0)
                std::cout << "Ransac iteration: " << it << "/" << num_it << std::endl;
        }
        return bestTransform;

    }

}}
