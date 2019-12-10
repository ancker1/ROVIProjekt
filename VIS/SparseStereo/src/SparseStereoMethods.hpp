#pragma once

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl/registration/transformation_estimation_svd.h>


/*
 * Contains two methods for finding either ball pose or rubber duck pose.
 * Ball pose only finds position of the center using primarly triangulate.
 * Rubber duck pose finds both position and oriantation using a defined known pose/transformation of the duck and then using that as base.
 *
 */


/****************************************************************************************
 **                              Ball pose finding method                              **
 ****************************************************************************************/
cv::Mat color_threshold_ball(cv::Mat pic){
    cv::Mat image_HSV, mask, mask_yellow;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(23, 25, 0), cv::Scalar(75, 255, 255), mask_yellow);
    mask = mask_yellow;

    // Opening and closing to get whole ball
    cv::Mat two_by_two( 5, 5, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, two_by_two);
    cv::Mat five_by_five( 5, 5, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, five_by_five);

    return mask;
}

cv::Point2d find_circle_center(cv::Mat ball_pic_binary){
    cv::Point2d center;

    std::vector<std::vector<cv::Point>> contours_points;
    cv::findContours(ball_pic_binary, contours_points, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

    if(contours_points.size() == 0)
        return center = cv::Point2d(0, 0);

    cv::Point2d upper = contours_points[0][0];
    cv::Point2d lower = contours_points[0][0];
    cv::Point2d left = contours_points[0][0];
    cv::Point2d right = contours_points[0][0];

    for(int i = 0; i < contours_points.size(); i++)
    {
        for(int j = 0; j < contours_points[i].size(); j++)
        {
            if(upper.y > contours_points[i][j].y)
                upper = contours_points[i][j];
            if(lower.y < contours_points[i][j].y)
                lower = contours_points[i][j];
            if(left.x > contours_points[i][j].x)
                left = contours_points[i][j];
            if(right.x < contours_points[i][j].x)
                right = contours_points[i][j];
            //std::cout << "contour #" << i << " , Point " << contours_points[i][j] << std::endl;
        }
    }

    center.x = right.x - (right.x - left.x)/2.0;
    center.y = upper.y + (lower.y - upper.y)/2.0;

    cv::cvtColor(ball_pic_binary, ball_pic_binary, cv::COLOR_GRAY2BGR);
    cv::circle(ball_pic_binary, center, 4, cv::Scalar(0,0,255), 2);

    return center;
}

cv::Mat find_ball_pose(cv::Mat left_img, cv::Mat right_img, cv::Mat proj_mat_left, cv::Mat proj_mat_right){
    cv::Mat triangulate_point(1, 1, CV_64FC4);
    cv::Mat left_point(1, 1, CV_64FC2);
    cv::Mat right_point(1, 1, CV_64FC2);

    // Thresholding only ball left in image
    left_img = color_threshold_ball(left_img);
    right_img = color_threshold_ball(right_img);

    // Finding center of ball using contour
    left_point.at<cv::Vec2d>(0) = find_circle_center(left_img);
    right_point.at<cv::Vec2d>(0) = find_circle_center(right_img);

    // Triangulate the ball center features
    cv::triangulatePoints(proj_mat_left, proj_mat_right, left_point, right_point, triangulate_point);
    triangulate_point =  triangulate_point / triangulate_point.at<double>(0, 3); // Normalizing the 3D-point

    // Transform from world to table
    cv::Mat TF_TABLE = (cv::Mat_<double>(4, 4) << 1, 0, 0, 0,
                                                  0, 1, 0, 0,
                                                  0, 0, 1, 0.1,
                                                  0, 0, 0, 1);

    return (TF_TABLE*triangulate_point);
}

/****************************************************************************************
 **                             6D - Rubber duck finding method                        **
 ****************************************************************************************/
cv::Mat color_threshold_duck(cv::Mat pic){

    cv::Mat image_HSV, mask, mask_body, mask_beak, duck;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(15, 120, 65), cv::Scalar(20, 255, 255), mask_body);
    cv::inRange(image_HSV, cv::Scalar(20, 150, 55), cv::Scalar(36, 255, 255), mask_beak);
    mask = mask_body + mask_beak;

    // Opening and closing to get whole duck
    cv::Mat five_by_five( 2, 2, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, five_by_five);
    cv::Mat fifteen_by_fifteen( 8, 8, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, fifteen_by_fifteen);

    //cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    //cv::bitwise_and(pic, mask, duck);
    return mask;
}

/****************************************************************************************
 **                             Rubber duck dot finding method                         **
 ****************************************************************************************/
cv::Mat color_threshold_dot_duck(cv::Mat pic){

    cv::Mat image_HSV, mask, duck;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(0, 190, 70), cv::Scalar(154, 255, 255), mask);

    // Opening and closing to get whole duck
    // Taking from function colorFiltering in TestingMethod
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    cv::bitwise_and(pic, mask, duck);

    return duck;
}

cv::Mat color_threshold_duck_hsv(cv::Mat pic, cv::Scalar lower, cv::Scalar upper)
{
    cv::Mat image_HSV, mask, duck;
    pic.copyTo(image_HSV);
    cv::cvtColor(image_HSV, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, lower, upper, mask);

    // Opening and closing to get whole duck
    // Taking from function colorFiltering in TestingMethod
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::dilate(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );
    cv::erode(mask, mask, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)) );

//    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);
//    cv::bitwise_and(pic, mask, duck);
    return mask;
}

void find_circle_center_from_dots(cv::Mat duck_pic, cv::Point2d &green_point, cv::Point2d &purple_point, cv::Point2d &red_point, cv::Point2d &blue_point)
{
     cv::Mat copy_duck, mask_green, mask_purple, mask_red, mask_blue;

     duck_pic.copyTo(copy_duck);
     copy_duck=color_threshold_dot_duck(copy_duck);

     cv::Scalar red_l(0, 135, 0);
     cv::Scalar red_u(26, 255, 255);
     cv::Scalar blue_l(29, 0, 0);
     cv::Scalar blue_u(137, 255, 226);
     cv::Scalar green_l(57, 0, 0);
     cv::Scalar green_u(107, 255, 255);
     cv::Scalar purple_l(137, 0, 0);
     cv::Scalar purple_u(179, 255, 255);

     mask_green = color_threshold_duck_hsv(copy_duck, green_l, green_u);
     mask_purple = color_threshold_duck_hsv(copy_duck, purple_l, purple_u);
     mask_red = color_threshold_duck_hsv(copy_duck, red_l, red_u);
     mask_blue = color_threshold_duck_hsv(copy_duck, blue_l, blue_u);

     green_point = find_circle_center(mask_green);
     purple_point = find_circle_center(mask_purple);
     red_point = find_circle_center(mask_red);
     blue_point = find_circle_center(mask_blue);
     if (green_point == cv::Point2d(0,0) || purple_point == cv::Point2d(0,0) || red_point == cv::Point2d(0,0)|| blue_point == cv::Point2d(0,0))
     {
         //std::cout << "Not all dots on the duck found, please check thresholding" << std::endl;
     }

}

std::vector<cv::Point3d> triangulate_duck(cv::Mat image_left, cv::Mat image_right, cv::Mat proj_mat_left, cv::Mat proj_mat_right)
{
    cv::Mat triangulate_points(1, 4, CV_64FC4);
    cv::Mat mat_triang_norm;
    std::vector<cv::Point3d> triangulate_points_norm;
    cv::Mat left_points(1, 4, CV_64FC2);
    cv::Mat right_points(1, 4, CV_64FC2);

    // Thresholding and finding cirlce centers MISSING FOR RIGHT OR LEFT IMAGE
    cv::Mat mask_green, mask_purple, mask_red, mask_blue, copy_duck;
    cv::Point2d green_p_l, purple_p_l, red_p_l, blue_p_l, green_p_r, purple_p_r, red_p_r, blue_p_r;

    find_circle_center_from_dots(image_left, green_p_l, purple_p_l, red_p_l, blue_p_l);
    find_circle_center_from_dots(image_right, green_p_r, purple_p_r, red_p_r, blue_p_r);

    left_points.at<cv::Vec2d>(0) = cv::Vec2d(green_p_l);
    left_points.at<cv::Vec2d>(1) = cv::Vec2d(purple_p_l);
    left_points.at<cv::Vec2d>(2) = cv::Vec2d(red_p_l);
    left_points.at<cv::Vec2d>(3) = cv::Vec2d(blue_p_l);

    right_points.at<cv::Vec2d>(0) = cv::Vec2d(green_p_r);
    right_points.at<cv::Vec2d>(1) = cv::Vec2d(purple_p_r);
    right_points.at<cv::Vec2d>(2) = cv::Vec2d(red_p_r);
    right_points.at<cv::Vec2d>(3) = cv::Vec2d(blue_p_r);

    cv::triangulatePoints(proj_mat_left, proj_mat_right, left_points, right_points, triangulate_points);
    for (unsigned int i = 0; i < 4; i++) {
        mat_triang_norm = triangulate_points.col(i).rowRange(0, 3) / triangulate_points.at<double>(3, i); // Normalizes the 3d points
        triangulate_points_norm.push_back(cv::Point3d(mat_triang_norm.at<double>(0, 0), mat_triang_norm.at<double>(0, 1), mat_triang_norm.at<double>(0, 2))); // Normalizes the 3d points
    }

    return triangulate_points_norm;
}

Eigen::Matrix4f estimate_6D_pose_dot_duck(cv::Mat init_image_left, cv::Mat init_image_right, cv::Mat new_image_left, cv::Mat new_image_right, cv::Mat proj_mat_left, cv::Mat proj_mat_right)
{
    // Gotten from scene file for init pose of duck
    Eigen::Matrix4f result;
    Eigen::Matrix4f initTransform;
    initTransform <<-1.0, 0.0, 0.0, 0.0,
                    0.0, 0.0, 1.0, 0.45,
                    0.0, 1.0, 0.0, 0.188,
                    0.0, 0.0, 0.0, 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr initPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr newPointCloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ> ESTSVD;
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZ, pcl::PointXYZ>::Matrix4 newTransform;
    initPointCloud->width = 4;
    initPointCloud->height = 1;
    initPointCloud->is_dense = false;
    initPointCloud->resize(initPointCloud->height * initPointCloud->width);

    newPointCloud->width = 4;
    newPointCloud->height = 1;
    newPointCloud->is_dense = false;
    newPointCloud->resize(newPointCloud->height * newPointCloud->width);

    // Triangulates the points
    std::vector<cv::Point3d> initPoints =  triangulate_duck(init_image_left, init_image_right, proj_mat_left, proj_mat_right);
    std::vector<cv::Point3d> newPoints = triangulate_duck(new_image_left, new_image_right, proj_mat_left, proj_mat_right);;

    std::vector<int> indices;
    for ( unsigned int i = 0; i < 4; i++ )
    {
        if ( initPoints[i].x > 100 || initPoints[i].y > 100 || initPoints[i].z > 100 || newPointCloud->points[i].x > 100 || newPointCloud->points[i].y > 100 || newPointCloud->points[i].z > 100)
            continue;
        initPointCloud->points[i].x = initPoints[i].x;
        initPointCloud->points[i].y = initPoints[i].y;
        initPointCloud->points[i].z = initPoints[i].z;

        newPointCloud->points[i].x = newPoints[i].x;
        newPointCloud->points[i].y = newPoints[i].y;
        newPointCloud->points[i].z = newPoints[i].z;

        indices.push_back(i);
    }
    if ( indices.size() > 2 )
    {
        ESTSVD.estimateRigidTransformation(*initPointCloud, indices, *newPointCloud, indices, newTransform);
        result = initTransform * newTransform;
    }
    else
    {
        result << -1, -1, -1, -1,
                  -1, -1, -1, -1,
                 -1, -1, -1, -1,
                 -1, -1, -1, -1;
        // cout << result
    }
    return result;
}
