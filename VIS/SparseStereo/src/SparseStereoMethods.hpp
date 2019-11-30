#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>


/*
 * Contains two methods for finding either ball pose or rubber duck pose.
 * Ball pose only finds position of the center using primarly triangulate.
 * Rubber duck pose finds both position and oriantation using sift and solvePnP.
 * ( Right now solvePnP not give a correct solution )
 *
 *
 */


/****************************************************************************************
 **                              Ball pose finding method                              **
 ****************************************************************************************/
cv::Mat color_threshold_smiley(cv::Mat pic){
    cv::Mat image_HSV, mask, mask_yellow, mask_blue, mask_red, smiley;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(25, 102, 153), cv::Scalar(28, 255, 255), mask_yellow);
    cv::inRange(image_HSV, cv::Scalar(118, 250, 153), cv::Scalar(122, 255, 255), mask_blue);
    cv::inRange(image_HSV, cv::Scalar(0, 250, 153), cv::Scalar(4, 255, 255), mask_red);
    mask = mask_yellow + mask_red + mask_blue;
    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    cv::bitwise_and(pic, mask, smiley);
    return smiley;
}

cv::Mat color_threshold_ball(cv::Mat pic){
    cv::Mat image_HSV, mask, mask_yellow;
    cv::cvtColor(pic, image_HSV, cv::COLOR_BGR2HSV);
    cv::inRange(image_HSV, cv::Scalar(25, 102, 153), cv::Scalar(28, 255, 255), mask_yellow);
    mask = mask_yellow;

    // Opening and closing to get whole duck
    cv::Mat two_by_two( 2, 2, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, two_by_two);
    cv::Mat five_by_five( 5, 5, CV_8U, cv::Scalar(1) );
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, five_by_five);

    return mask;
}

cv::Point2d find_ball_center(cv::Mat ball_pic_binary){
    cv::Point2d center;

    std::vector<std::vector<cv::Point>> contours_points;
    cv::findContours(ball_pic_binary, contours_points, cv::RETR_LIST, cv::CHAIN_APPROX_SIMPLE);

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
    left_point.at<cv::Vec2d>(0) = find_ball_center(left_img);
    right_point.at<cv::Vec2d>(0) = find_ball_center(right_img);

    // Triangulate the ball center features
    cv::triangulatePoints(proj_mat_left, proj_mat_right, left_point, right_point, triangulate_point);
    triangulate_point =  triangulate_point / triangulate_point.at<double>(0, 3); // Normalizing the 3D-point

    return triangulate_point;
}

/****************************************************************************************
 **                             Rubber duck finding method                             **
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

    cv::cvtColor(mask, mask, cv::COLOR_GRAY2BGR);

    cv::bitwise_and(pic, mask, duck);
    return duck;
}
